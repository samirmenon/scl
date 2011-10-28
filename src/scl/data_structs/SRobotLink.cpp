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
/* \file SRobotLink.cpp
 *
 *  Created on: Mar 15, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 

#include <scl/data_structs/SRobotLink.hpp>

#include <iostream>

namespace scl
{

  SRobotLink::SRobotLink() : SObject(std::string("SRobotLink") )
  {
    init();
    has_been_init_ = false;
  }

  /** Sets the default parameter values */
  void SRobotLink::init()
  {
    is_root_ = false;
    link_id_ = -2; //(root starts at -1)
    name_ = "not_assigned";
    parent_name_ = "not_assigned";
    robot_name_ = "not_assigned";
    joint_name_ = "not_assigned";
    joint_limit_lower_ = -3.14;
    joint_limit_upper_ = 3.14;
    joint_default_pos_ = 0.0;

    //Tree structure information:
    parent_addr_ = NULL;
    child_addrs_.clear();

    //Link's Physical Properties
    com_<<0, 0, 0;
    inertia_<<1,1,1;
    rot_axis_<<0,0,0;

    mass_ = 1.0;
    rot_angle_ = 0;
    link_is_fixed_ = 0;
    joint_type_ = JOINT_TYPE_NOTASSIGNED;
    pos_in_parent_<<0,0,0;
    ori_parent_quat_ = Eigen::Quaternion<sFloat>::Identity();

    graphics_obj_vec_.clear();
    render_type_ = RENDER_TYPE_SPHERE;
  }
}
