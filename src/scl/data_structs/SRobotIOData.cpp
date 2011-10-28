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
/* \file SRobotIOData.cpp
 *
 *  Created on: Dec 25, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/data_structs/SRobotIOData.hpp>

#include <stdexcept>
#include <iostream>

namespace scl
{

SRobotIOData::SRobotIOData() : SObject("SRobotIOData")
{
  dof_ = 0;
  has_been_init_ = true;
}

sBool SRobotIOData::init(const std::string& arg_robot_name, const sUInt arg_robot_dof)
{
  try
  {
    if(1 > arg_robot_name.size())
    { throw(std::runtime_error("Robot's name is too short."));  }

    if(0 == arg_robot_dof)
    { throw(std::runtime_error("Can't add a robot with 0 degrees of freedom"));  }

    name_ = arg_robot_name;
    dof_ = arg_robot_dof;

    //Initialize sensors
    sensors_.q_.setZero(dof_);
    sensors_.dq_.setZero(dof_);
    sensors_.ddq_.setZero(dof_);
    sensors_.force_gc_measured_.setZero(dof_);
    sensors_.forces_external_.clear();

    actuators_.force_gc_commanded_.setZero(dof_);

    has_been_init_ = true;
  }
  catch(std::exception& e)
  {
    std::cerr<<"\nSRobotIOData::init() : "<<e.what();
    //Will set init to false since state might be invalid.
    //Will not reset data to enable error handling and debugging.
    has_been_init_ = false;
  }

  return has_been_init_;
}

sBool SRobotIOData::printInfo()
{
  if(false == has_been_init_)
  {
    std::cout<<"\nSRobotIOData::printInfo() : Not initialized. Can't print data. Ptr = "<<this;
    return false;
  }

  std::cout<<"\n-------SRobotIOData::printInfo()-------"
      <<"\nName : "<<name_
      <<"\n Dof : "<<dof_
      <<"\n   Q : "<<sensors_.q_.transpose()
      <<"\n  dQ : "<<sensors_.dq_.transpose()
      <<"\n ddQ : "<<sensors_.ddq_.transpose()
      <<"\n F_s : "<<sensors_.force_gc_measured_.transpose()
      <<"\n F_c : "<<actuators_.force_gc_commanded_.transpose();

  return true;
}

}
