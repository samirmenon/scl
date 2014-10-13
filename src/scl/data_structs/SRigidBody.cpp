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
/* \file SRigidBody.cpp
 *
 *  Created on: Mar 15, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 

#include <scl/data_structs/SRigidBody.hpp>

#include <iostream>

namespace scl
{

  SRigidBody::SRigidBody() : SObject(std::string("SRigidBody") )
  {
    init();
    has_been_init_ = false;
  }

  /** Sets the default parameter values */
  void SRigidBody::init()
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

    stiction_gc_force_lower_=0.0;
    stiction_gc_force_upper_=0.0;
    stiction_gc_vel_lower_=0.0;
    stiction_gc_vel_upper_=0.0;
    friction_gc_kv_=0.0;

    force_gc_lim_lower_=0.0;
    force_gc_lim_upper_=0.0;

    //Tree structure information:
    parent_addr_ = NULL;
    child_addrs_.clear();

    //Link's Physical Properties
    com_<<0, 0, 0;
    inertia_ = Eigen::Matrix3d::Identity();
    inertia_gc_ = 0.0;

    mass_ = 1.0;
    link_is_fixed_ = 0;
    joint_type_ = JOINT_TYPE_NOTASSIGNED;
    pos_in_parent_<<0,0,0;
    ori_parent_quat_ = Eigen::Quaternion<sFloat>::Identity();

    graphics_obj_vec_.clear();
    render_type_ = RENDER_TYPE_SPHERE;
  }
}
