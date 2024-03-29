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
 * SControllerBase.cpp
 *
 *  Created on: Dec 29, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
#include <scl/control/data_structs/SControllerBase.hpp>

#include <string>
#include <stdexcept>
#include <iostream>

namespace scl
{
  SControllerBase::SControllerBase() : SObject("SControllerBase")
  {
    robot_ = S_NULL;
    io_data_ = S_NULL;
    gc_model_ = S_NULL;
    robot_name_ = "";
  }

  SControllerBase::~SControllerBase() {}

  sBool SControllerBase::init(const std::string & arg_controller_name,
      const SRobotParsed* arg_robot_ds,
      SRobotIO* arg_io_data,
      SGcModel* arg_gc_model)
  {
    bool flag=false;
    try
    {
      if(1>arg_controller_name.size())
      { throw(std::runtime_error("Controller name is too short")); }

      if(S_NULL==arg_robot_ds)
      { throw(std::runtime_error("Passed a NULL parent-robot data structure")); }

      if(0==arg_robot_ds->dof_)
      { throw(std::runtime_error("Passed a parent-robot data structure with 0 dof")); }

      if(1>arg_robot_ds->name_.size())
      { throw(std::runtime_error("Passed parent-robot's name is too short")); }

      if(S_NULL==arg_io_data)
      { throw(std::runtime_error("Passed a NULL I/O data structure")); }

      name_ = arg_controller_name;
      robot_name_ = arg_robot_ds->name_;
      robot_ = arg_robot_ds;
      io_data_ = arg_io_data;

      if(S_NULL == arg_gc_model)
      { gc_model_ = new SGcModel(); }
      else
      { gc_model_ = arg_gc_model; }

      if(false == gc_model_->hasBeenInit())
      {
        flag = gc_model_->init(*arg_robot_ds);
        if(false == gc_model_->hasBeenInit() || false == flag)
        { throw(std::runtime_error("Could not initialize generalized coordinate dynamic matrices")); }
      }

      //NOTE : We will not set has_been_init_ to true. The actual controller's data struct
      //implementation should do that.
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSControllerBase::init() : "<<e.what();

      //The data was half-initialized; it might be in an invalid state now.
      //So, we will keep the past values for error handling / debugging (instead of clearing them).
      return false;
    }

    return true;
  }
}
