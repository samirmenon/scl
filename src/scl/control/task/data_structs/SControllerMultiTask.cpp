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
/* \file SControllerMultiTask.cpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

/**  A singleton to unify all the disparate data
 *  sources in scl. Similar to a very fast database.
 *
 *  The data structure is universally available for
 *  any subcomponent of Scl to use.
 */

#include "SControllerMultiTask.hpp"

#include <stdexcept>
#include <iostream>

namespace scl
{
  SControllerMultiTask::SControllerMultiTask() : SControllerBase(),
      servo_to_model_rate_(1)
  {
    type_ = "SControllerMultiTask";
  }

  SControllerMultiTask::~SControllerMultiTask()
  {}

  /** Initializes the data structure */
  sBool SControllerMultiTask::init(const std::string & arg_ctrl_name,
      const SRobotParsed* arg_robot_ds,
      SRobotIO* arg_io_data,
      SGcModel* arg_gc_model,
      const std::string &arg_must_use_robot,
      const sUInt arg_servo_to_model_rate)
  {
    bool flag;
    try
    {
      if(arg_must_use_robot!="")
        if(arg_robot_ds->name_ != arg_must_use_robot)
        {
          throw (std::runtime_error(std::string("Must use robot: ") + arg_must_use_robot +
            std::string(". Passed robot ds for: ") + arg_robot_ds->name_));
        }

      flag = SControllerBase::init(arg_ctrl_name,arg_robot_ds,arg_io_data,arg_gc_model);
      if(false == flag)
      {throw (std::runtime_error("Couldn't initialize the controller base data structure"));}

      flag = servo_.init(arg_robot_ds, & tasks_);
      if(false == flag)
      {throw (std::runtime_error("Couldn't initialize the servo"));}

      servo_to_model_rate_ = arg_servo_to_model_rate;

      tasks_.clear();
      tasks_non_ctrl_.clear();

      has_been_init_ = true;
    }
    catch (std::exception & e)
    {
      std::cout<<"\nSControllerMultiTask::init() : Failed "<<e.what();
      has_been_init_ = false;
    }

    return has_been_init_;
  }

}
