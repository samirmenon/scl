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
/* \file CTaskOpPosNoGravity.hpp
 *
 *  Created on: Aug 19, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTASKOPPOSNOGRAVITY_HPP_
#define CTASKOPPOSNOGRAVITY_HPP_

#include <string>
#include <vector>

#include <scl/DataTypes.hpp>

#include <scl/control/task/tasks/data_structs/STaskOpPosNoGravity.hpp>

#include <scl/control/task/tasks/CTaskOpPos.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>


namespace scl
{
  /**
   * Computes the operational space  forces for a single
   * 3-d (x,y,z) goal point Euclidean task
   *
   * It computes:
   *
   * 1. The task model (computes, mass, jacobian, inv jacobian,
   * coriolis, centrifugal and gravity matrices/vectors).
   *
   * 2. The task servo (computes the dynamically decoupled task
   * forces and the torques. uses the task model to do so).
   */
class CTaskOpPosNoGravity : public scl::CTaskOpPos
{
public:
  /********************************
   * CTaskBase API
   *********************************/
  /** Initializes the task object. Required to set output
   * gc force dofs */
  virtual bool init(STaskBase* arg_task_data,
      CDynamicsBase* arg_dynamics)
  { return scl::CTaskOpPos::init(arg_task_data, arg_dynamics);  }

  /** Return this task controller's task data structure.*/
  virtual STaskBase* getTaskData()
  { return scl::CTaskOpPos::getTaskData();  }

  /** Resets the task by removing its data.
   * NOTE : Does not deallocate its data structure*/
  virtual void reset()
  { return scl::CTaskOpPos::reset();  }

  /** Computes the task torques */
  virtual bool computeServo(const SRobotSensors* arg_sensors)
  { return scl::CTaskOpPos::computeServo(arg_sensors);  }

  /** Computes the dynamics (task model)
   * Assumes that the data_->model_.gc_model_ has been updated. */
  virtual bool computeModel(const SRobotSensors* arg_sensors)
  { return scl::CTaskOpPos::computeModel(arg_sensors);  }

  /********************************
   * CTaskOpPosNoGravity specific functions
   *********************************/
  /** Default constructor : Does nothing   */
  CTaskOpPosNoGravity() : CTaskOpPos()
  { data_->flag_compute_op_gravity_ = false; }

  /** Default destructor : Does nothing.   */
  virtual ~CTaskOpPosNoGravity(){}
};

}

#endif /* CMARKERTRACKTASK_HPP_ */
