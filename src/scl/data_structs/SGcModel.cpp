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

#include <scl/data_structs/SGcModel.hpp>
#include <scl/util/DatabaseUtils.hpp>

#include <stdexcept>
#include <iostream>

namespace scl
{
  SGcModel::SGcModel() : SObject("SGcModel"), mass_(-1)
  { }

  sBool SGcModel::init(const SRobotParsed& arg_robot_data)
  {
    bool flag;
    try
    {
      if(false == arg_robot_data.has_been_init_)
      { throw(std::runtime_error("Passed uninitialized robot data structure")); }

      int ndof = arg_robot_data.dof_;
      if(0==ndof)
      { throw(std::runtime_error("Can not initialize for a parent-robot with 0 dof")); }

      M_gc_.setIdentity(ndof,ndof);
      M_gc_inv_.setIdentity(ndof,ndof);
      force_gc_cc_.setZero(ndof);
      force_gc_grav_.setZero(ndof);
      q_.setZero(ndof);
      dq_.setZero(ndof);
      pos_com_.setZero(3);

      computed_spatial_transformation_and_inertia_ = false;

      flag = scl_util::initDynRobotFromParsedRobot(rbdyn_tree_,arg_robot_data.rb_tree_);
      if(false==flag)
      { throw(std::runtime_error("Could not initialize dynamic tree from the static tree for a robot.")); }

      // Sort the underlying list for the new tree
      std::vector<std::string> tmp_sort_order;
      flag = arg_robot_data.rb_tree_.sort_get_order(tmp_sort_order);
      if(false == flag)
      { throw(std::runtime_error( "Could not obtain sort order from passed robot parsed data" )); }

      // Sort the underlying list for the new tree
      flag = rbdyn_tree_.sort(tmp_sort_order);
      if(false==flag)
      { throw(std::runtime_error( "Could not sort dynamic node tree" )); }

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSGcModel::init() : "<<e.what();
      return false;
    }

    return true;
  }
}
