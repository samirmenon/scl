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
/* \file SServo.cpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
/** 
 *  A data structure to store the main servo loop's data.
 *  The main servo loop accesses the current torques to be
 *  applied from each task, computes the range spaces of the
 *  tasks and projects the torques through them to obtain the
 *  combined robot torque.
 */

#include "SServo.hpp"

#include <stdexcept>
#include <iostream>

namespace scl
{

  SServo::SServo() : SObject("SServo"), task_data_(S_NULL)
  {}

  sBool SServo::init(const SRobotParsedData* arg_robot_ds,
      sutil::CMappedMultiLevelList<std::string, STaskBase*>* arg_task_ds)
  {
    try
    {
      if(S_NULL==arg_robot_ds)
      { throw(std::runtime_error("Passed a NULL parent-robot data structure")); }

      if(0==arg_robot_ds->dof_)
      { throw(std::runtime_error("Passed a parent-robot data structure with 0 dof")); }

      if(1>arg_robot_ds->name_.size())
      { throw(std::runtime_error("Passed parent-robot's name is too short")); }

      if(S_NULL==arg_task_ds)
      { throw(std::runtime_error("Passed a NULL task data structure")); }

      robot_ = arg_robot_ds;
      force_gc_.setZero(arg_robot_ds->dof_);
      task_data_ = arg_task_ds;

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSControllerBase::init() : "<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }
}
