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
 * \file CTaskGcEmpty.hpp
 *
 *  Created on: Oct 20, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTASKGCEMPTY_HPP_
#define CTASKGCEMPTY_HPP_

#include <scl/control/task/CTaskBase.hpp>

#include "data_structs/STaskGcEmpty.hpp"

namespace scl_app
{

  /** Function to register task with the database and enable
   * dynamic typing. Call this function if you want your
   * task to be initialized by specifying stuff in a file. */
  scl::sBool registerType_TaskGcEmpty();

  /** This is my cool new task. */
  class CTaskGcEmpty : public scl::CTaskBase
  {
  public:
    /*******************************************
     * CTaskBase API : TODO : IMPLEMENT THESE!!
     *******************************************/
    /** Computes the gc forces that resist gc velocity */
    virtual bool computeServo(
        const scl::SRobotSensorData* arg_sensors);

    /** Does nothing. GC tasks don't require OSC models */
    virtual bool computeModel();

    /** Return this task controller's task data structure.*/
    virtual scl::STaskBase* getTaskData();

    /** Initializes the task object. Required to set output
     * gc force dofs. */
    virtual bool init(scl::STaskBase* arg_task_data,
        scl::CDynamicsBase* arg_dynamics);

    /** Resets the task by removing its data.
     * NOTE : Does not deallocate its data structure */
    virtual void reset();

  public:
    CTaskGcEmpty();
    virtual ~CTaskGcEmpty();

  protected:
    //This will be filled in from the file
    STaskGcEmpty* data_;

    //TODO : Add what you want.
    //myCoolDataType data2_;

    const void * link_dynamic_id_;
  };
}

#endif /* CTaskGcEmpty_HPP_ */
