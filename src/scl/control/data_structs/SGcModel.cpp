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
/*
 * \file SGcModel.cpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/control/data_structs/SGcModel.hpp>
#include <stdexcept>
#include <iostream>

namespace scl
{
  SGcModel::SGcModel() : mass_(-1)
  { }

  sBool SGcModel::init(const SRobotParsedData& arg_robot_data)
  {
    try
    {
      if(false == arg_robot_data.has_been_init_)
      { throw(std::runtime_error("Passed uninitialized robot data structure")); }

      int ndof = arg_robot_data.dof_;
      if(0==ndof)
      { throw(std::runtime_error("Can not initialize for a parent-robot with 0 dof")); }

      A_.setIdentity(ndof,ndof);
      Ainv_.setIdentity(ndof,ndof);
      b_.setZero(ndof);
      g_.setZero(ndof);
      q_.setZero(ndof);
      dq_.setZero(ndof);
      pos_com_.setZero(3);

      sutil::CMappedTree<std::string, SRigidBody>::const_iterator it,ite;
      for(it = arg_robot_data.robot_br_rep_.begin(), ite = arg_robot_data.robot_br_rep_.end();
          it!=ite; ++it)
      {
        const SRigidBody& rb = *it;
        SRigidBodyDyn *com = link_ds_.create(rb.name_,rb.is_root_);
        if(NULL == com)
        { throw(std::runtime_error( std::string("Could not create dyn node: ")+ rb.name_+std::string("for robot: ")+rb.robot_name_ )); }

        com->name_ = rb.name_;
        com->parent_name_ = rb.parent_name_;
        com->link_ds_ = &rb;
        com->link_dynamic_id_ = NULL;

        com->J_com_.setZero(6, ndof);

        if(rb.is_root_)
        {//The root node doesn't move, so we can already compute the translations.
          com->T_o_lnk_.setIdentity();
          com->T_o_lnk_.translation() = com->link_ds_->pos_in_parent_;
          com->T_o_lnk_.rotate(com->link_ds_->ori_parent_quat_);
          com->T_lnk_ = com->T_o_lnk_;

          //Default is NaN, which indicates that these values weren't initialized.
          //Set to zero to indicate that they are now initialized. (actual value has no
          //meaning since the root node never moves).
          com->q_J_ = 0.0;
          com->q_T_ = 0.0;
          com->q_T_o_ = 0.0;
        }
        else
        {
          com->T_o_lnk_.setIdentity();
          com->T_lnk_.setIdentity();
        }
      }

      bool flag = link_ds_.linkNodes();
      if(false == flag)
      { throw(std::runtime_error( "Could not link the dynamic nodes into a tree" )); }

      // Sort the underlying list for the new tree
      std::vector<std::string> tmp_sort_order;
      flag = arg_robot_data.robot_br_rep_.sort_get_order(tmp_sort_order);
      if(false == flag)
      { throw(std::runtime_error( "Could not obtain sort order from passed robot parsed data" )); }

      // Sort the underlying list for the new tree
      flag = link_ds_.sort(tmp_sort_order);
      if(false==flag)
      { throw(std::runtime_error( "Could not sort dynamic node tree" )); }
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSGcModel::init() : "<<e.what();
      return false;
    }

    return true;
  }
}
