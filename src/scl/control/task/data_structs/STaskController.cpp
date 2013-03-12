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
/* \file STaskController.cpp
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

#include "STaskController.hpp"

#include <stdexcept>
#include <iostream>

namespace scl
{
  STaskController::STaskController() : SControllerBase()
  {
    type_ = "STaskController";
  }

  STaskController::~STaskController()
  {}

  /** Initializes the data structure */
  sBool STaskController::init(const std::string & arg_ctrl_name,
      const SRobotParsedData* arg_robot_ds,
      SRobotIOData* arg_io_data)
  {
    bool flag;
    try
    {
      flag = SControllerBase::init(arg_ctrl_name,arg_robot_ds,arg_io_data);
      if(false == flag)
      {throw (std::runtime_error("Couldn't initialize the task controller"));}

      flag = servo_.init(arg_robot_ds, & tasks_);
      if(false == flag)
      {throw (std::runtime_error("Couldn't initialize the servo"));}

      tasks_.clear();

      has_been_init_ = true;
    }
    catch (std::exception & e)
    {
      std::cout<<"\nparseGraphics() : Failed "<<e.what();
      has_been_init_ = false;
    }

    return has_been_init_;
  }

}
