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
/* \file SRobotIO.cpp
 *
 *  Created on: Dec 25, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/data_structs/SRobotIO.hpp>

#include <stdexcept>
#include <iostream>

namespace scl
{

SRobotIO::SRobotIO() : SObject("SRobotIO")
{
  name_robot_ = "";
  dof_ = 0;
  has_been_init_ = true;
}

sBool SRobotIO::init(const SRobotParsed& arg_rds)
{
  try
  {
    if(false == arg_rds.has_been_init_)
    { throw(std::runtime_error("Passed uninitialized robot data structure."));  }

    if(1 > arg_rds.name_.size())
    { throw(std::runtime_error("Robot's name is too short."));  }

    if(0 == arg_rds.dof_)
    { throw(std::runtime_error("Can't add a robot with 0 degrees of freedom"));  }

    name_robot_ = arg_rds.name_;
    name_ = arg_rds.name_ + "_io_ds";
    dof_ = arg_rds.dof_;

    //Initialize sensors
    sensors_.q_.setZero(dof_);
    sensors_.dq_.setZero(dof_);
    sensors_.ddq_.setZero(dof_);
    sensors_.force_gc_measured_.setZero(dof_);
    sensors_.forces_external_.clear();

    // Set defaults
    for(auto it:arg_rds.rb_tree_)
    {
      if(0 < it.link_id_ && static_cast<sInt>(arg_rds.dof_) > it.link_id_)
      { sensors_.q_(it.link_id_) = it.joint_default_pos_; }
    }

    actuators_.force_gc_commanded_.setZero(dof_);

    has_been_init_ = true;
  }
  catch(std::exception& e)
  {
    std::cerr<<"\nSRobotIO::init() : "<<e.what();
    //Will set init to false since state might be invalid.
    //Will not reset data to enable error handling and debugging.
    has_been_init_ = false;
  }

  return has_been_init_;
}

sBool SRobotIO::printInfo()
{
  if(false == has_been_init_)
  {
    std::cout<<"\nSRobotIO::printInfo() : Not initialized. Can't print data. Ptr = "<<this;
    return false;
  }

  std::cout<<"\n-------SRobotIO::printInfo()-------"
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
