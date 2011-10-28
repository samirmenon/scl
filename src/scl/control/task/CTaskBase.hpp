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
/* \file CTaskBase.hpp
 *  Encapsulates a task servo and a task model
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTASKBASE_HPP_
#define CTASKBASE_HPP_

#include <scl/DataTypes.hpp>

#include <scl/control/task/data_structs/STaskBase.hpp>
#include <scl/data_structs/SRobotIOData.hpp>

#include <scl/dynamics/CDynamicsBase.hpp>

namespace scl {

/**
 * Container class to encapsulate a task model and a
 * task servo.
 *
 * NOTE : Virtual class. Subclass and implement functions
 * that compute the task forces.
 */
class CTaskBase {
public:
  /** Computes the task torques */
  virtual bool computeServo(const SRobotSensorData* arg_sensors)=0;

  /** Computes the dynamics (task model) */
  virtual bool computeModel()=0;

  /** Constructor does nothing */
  CTaskBase(): has_been_init_(false), dynamics_(S_NULL) {}

  /** Destructor does nothing */
  virtual ~CTaskBase(){}

  /** Initializes the task object. Create a subclass of
   * STaskBase if your task requires more data than the defaults
   * in STaskBase provide.
   * This function should set has_been_init_ to true*/
  virtual bool init(STaskBase* arg_task_data,
      CDynamicsBase* arg_dynamics)=0;

  /** Return this task controller's task data structure.
   *   Use it responsibly!
   * NOTE : Use dynamic casts whenever you downcast the data.
   *        And try not to downcast very often. Perf hit. */
  virtual STaskBase* getTaskData()=0;

  /** Resets the task by removing its data. */
  virtual void reset()=0;

  virtual sBool hasBeenInit() { return has_been_init_;  }

protected:
  sBool has_been_init_;

  /** A Dynamics model required to compute the task's dynamics */
  CDynamicsBase* dynamics_;
};

}

#endif /* CTASKBASE_HPP_ */
