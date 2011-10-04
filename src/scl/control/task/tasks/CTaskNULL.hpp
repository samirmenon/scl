/* This file is part of Busylizzy, a control and simulation library
for robots and biomechanical models.

Busylizzy is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

Busylizzy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
Busylizzy. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file CTaskNULL.hpp
 *
 *  Created on: Aug 19, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTASKNULL_HPP_
#define CTASKNULL_HPP_

#include <scl/DataTypes.hpp>

#include <scl/control/task/CTaskBase.hpp>

#include <scl/dynamics/CDynamicsBase.hpp>

namespace scl
{

/**
 * A null task. Does nothing. Yes it really does nothing.
 *
 * Q) Why does it exist?
 * A) For debugging.
 */
class CTaskNULL: public scl::CTaskBase
{
public:
  /** Constructor sets initialization state to false */
  CTaskNULL() : data_(S_NULL) {}

  /** Destructor does nothing. */
  virtual ~CTaskNULL(){}

  /********************************
   * CTaskBase API
   *********************************/

  /** Initializes the task object. Required to set output
   * gc force dofs */
  virtual bool init(STaskBase* arg_task_data,
      CDynamicsBase* arg_dynamics);

  /** Return this task controller's task data structure.*/
  virtual STaskBase* getTaskData();

  /** Resets the task by removing its data.
   * NOTE : Does not deallocate its data structure*/
  virtual void reset();

  /** Computes the task torques */
  virtual bool computeServo(const SRobotSensorData* arg_sensors);

  /** Computes the dynamics (task model) */
  virtual bool computeModel();

private:
  /** The kinematic and dynamic data required to compute any arbitrary task. */
  STaskBase* data_;
};

}

#endif /* CTASKNULL_HPP_ */
