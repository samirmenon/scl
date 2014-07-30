/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file DatabaseUtils.cpp
 *
 *  Created on: Aug 6, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/util/DatabaseUtils.hpp>
#include <scl/Singletons.hpp>
#include <scl/data_structs/SRobotParsed.hpp>

#include <vector>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdexcept>

#ifdef DEBUG
//Asserts appear in debug mode when the ifdef is true
#include <cassert>
#endif

using namespace scl;

namespace scl_util
{

  void printRobotLinkTree(const SRigidBody &link, int depth)
  {
    std::cout<<"\n Link: "<<link.name_<<" Robot: <"<<link.robot_name_<<">";
    if(S_NULL==link.parent_addr_)
    { std::cout<<" Parent: <None>";  }
    else
    { std::cout<<" Parent: <"<<link.parent_addr_->name_<<">"; }
    std::cout<<", Id: "<<link.link_id_
              <<", Depth: "<<depth
              <<", Com: "<<(link.com_).transpose()
              <<", Mass: "<<link.mass_
              <<", Inertia: \n"<<link.inertia_
              <<",\nPos in par: "<<link.pos_in_parent_.transpose()
              <<",\nOri in par: \n"<<link.ori_parent_quat_.toRotationMatrix()
              <<std::flush;

    std::vector<SRigidBody*>::const_iterator clink, clinke;
    for(clink = link.child_addrs_.begin(),
        clinke = link.child_addrs_.end();
        clink != clinke; ++clink)
    { printRobotLinkTree( (**clink),depth+1); }

    if(link.child_addrs_.begin() == link.child_addrs_.end())
    { std::cout<<"\nLEAF NODE."<<std::flush; }
  }

  /** Initializes a dynamic tree given a static tree for a robot. */
  bool initDynRobotFromParsedRobot(sutil::CMappedTree<std::string, scl::SRigidBodyDyn>& arg_rbd_tree,
      const sutil::CMappedTree<std::string, scl::SRigidBody>& arg_rb_tree)
  {
    bool flag;
    try
    {
      const int ndof = arg_rb_tree.size()-1;
      arg_rbd_tree.clear();

      sutil::CMappedTree<std::string, SRigidBody>::const_iterator it,ite;
      for(it = arg_rb_tree.begin(), ite = arg_rb_tree.end();
          it!=ite; ++it)
      {
        const SRigidBody& rb = *it;
        SRigidBodyDyn *rbd = arg_rbd_tree.create(rb.name_,rb.is_root_);
        if(NULL == rbd)
        { throw(std::runtime_error( std::string("Could not create dyn node: ")+ rb.name_+std::string("for robot: ")+rb.robot_name_ )); }

        rbd->name_ = rb.name_;
        rbd->parent_name_ = rb.parent_name_;
        rbd->link_ds_ = &rb;

        rbd->J_com_.setZero(6, ndof);

        if(rb.is_root_)
        {//The root node doesn't move, so we can already compute the translations.
          rbd->T_o_lnk_.setIdentity();
          rbd->T_o_lnk_.translate(rbd->link_ds_->pos_in_parent_);
          rbd->T_o_lnk_.rotate(rbd->link_ds_->ori_parent_quat_);
          rbd->T_lnk_ = rbd->T_o_lnk_;

          //Default is NaN, which indicates that these values weren't initialized.
          //Set to zero to indicate that they are now initialized. (actual value has no
          //meaning since the root node never moves).
          rbd->q_T_ = 0.0;
        }
        else
        {
          rbd->T_o_lnk_.setIdentity();
          rbd->T_lnk_.setIdentity();
        }
      }

      flag = arg_rbd_tree.linkNodes();
      if(false == flag)
      { throw(std::runtime_error( "Could not link the dynamic nodes into a tree" )); }
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nDatabaseUtils::initDynRobotFromParsedRobot() : "<<ee.what();
      return false;
    }
    return true;
  }

  /** Checks if a muscle system is compatible with a given robot. Looks for
     * both in the database */
    sBool isMuscleCompatWithRobot(const std::string& arg_msys,
        const std::string& arg_robot)
    {
      try
      {
        SDatabase * db = CDatabase::getData();
        if(S_NULL == db) { throw(std::runtime_error("Database not initialized"));  }

        SMuscleSetParsed *msys = db->s_parser_.muscle_systems_.at(arg_msys);
        if(S_NULL == msys) { throw(std::runtime_error("Could not find muscle system"));  }

        SRobotParsed *rob = db->s_parser_.robots_.at(arg_robot);
        if(S_NULL == rob) { throw(std::runtime_error("Could not find robot"));  }

        //NOTE TODO : Implement this.
      }
      catch(std::exception& ee)
      {
        std::cerr<<"\nDatabaseUtils::isMuscleCompatWithRobot() : "<<ee.what();
        return false;
      }
      return true;
    }
}
