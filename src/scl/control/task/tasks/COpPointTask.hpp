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
/* \file COpPointTask.hpp
 *
 *  Created on: Aug 19, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef COPPOINTTASK_HPP_
#define COPPOINTTASK_HPP_

#include <string>
#include <vector>

#include <scl/DataTypes.hpp>

#include <scl/control/task/tasks/data_structs/SOpPointTask.hpp>

#include <scl/control/task/CTaskBase.hpp>

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
class COpPointTask : public scl::CTaskBase
{
public:
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

  /** Computes the dynamics (task model)
   * Assumes that the data_->model_.gc_model_ has been updated. */
  virtual bool computeModel();

  /********************************
   * COpPointTask specific functions
   *********************************/
  /** Default constructor : Does nothing   */
  COpPointTask();

  /** Default destructor : Does nothing.   */
  virtual ~COpPointTask(){}

  /** Sets the current goal position */
  inline void setGoal(const Eigen::VectorXd & arg_goal)
  { data_->x_goal_ = arg_goal;  }

  /** Sets the current goal velocity */
  inline void setGoalVel(const Eigen::VectorXd & arg_goal)
  { data_->dx_goal_ = arg_goal;  }

  /** Sets the current goal acceleration */
  inline void setGoalAcc(const Eigen::VectorXd & arg_goal)
  { data_->ddx_goal_ = arg_goal;  }

  /** Whether the task has achieved its goal position. */
  sBool achievedGoalPos();

private:
  /** The actual data structure for this computational object */
  SOpPointTask* data_;

  /** Temporary variables */
  Eigen::VectorXd tmp1, tmp2;

  /** For inverting the lambda matrix (when it gets singular) */
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_;

  /** True when the lambda_inv matrix turns singular. */
  sBool lambda_inv_singular_;

  /** For inverting the operational space inertia matrix
   * near singularities. 3x3 for operational point tasks. */
  Eigen::JacobiSVD<Eigen::Matrix3d > svd_;
  Eigen::Matrix3d singular_values_;
};

}

#endif /* CMARKERTRACKTASK_HPP_ */
