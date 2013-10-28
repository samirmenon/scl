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

#include <vector>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdexcept>

#ifdef DEBUG
//Asserts appear in debug mode when the ifdef is true
#include <cassert>
#endif

#include <scl/Singletons.hpp>
#include <scl/data_structs/SRobotParsed.hpp>

using namespace scl;

namespace scl_util
{

  void printRobotLinkTree(const SRigidBody &link, int depth)
  {
    if(S_NULL==link.parent_addr_)
    { std::cout<<"\nParent: <None>"<<" Robot: <"<<link.robot_name_<<">";  }
    else
    {
      std::cout<<"\nParent: <"<<link.parent_addr_->name_<<">"
               <<" Robot: <"<<link.robot_name_<<">";
    }
    std::cout<<" Link: "<<link.name_
              <<", Id: "<<link.link_id_
              <<", Depth: "<<depth
              <<", Com: "<<(link.com_).transpose()
              <<", Mass: "<<link.mass_
              <<", \nInertia: "<<link.inertia_
              <<std::flush;

    std::vector<SRigidBody*>::const_iterator clink, clinke;
    for(clink = link.child_addrs_.begin(),
        clinke = link.child_addrs_.end();
        clink != clinke; ++clink)
    { printRobotLinkTree( (**clink),depth+1); }

    if(link.child_addrs_.begin() == link.child_addrs_.end())
    { std::cout<<". LEAF NODE."<<std::flush; }
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

        SMuscleSystemParsed *msys = db->s_parser_.muscle_systems_.at(arg_msys);
        if(S_NULL == msys) { throw(std::runtime_error("Could not find muscle system"));  }

        SRobotParsed *rob = db->s_parser_.robots_.at(arg_robot);
        if(S_NULL == rob) { throw(std::runtime_error("Could not find robot"));  }

        //NOTE TODO : Implement this.
      }
      catch(std::exception& ee)
      {
        std::cerr<<"\nCChaiGraphics::addMeshToRender() : "<<ee.what();
        return false;
      }
      return true;
    }
}
