
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
/* \file * COpRotationQuatTask.hpp
 *
 *  Created on: Sep 14, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef COPROTATIONTASK_HPP_
#define COPROTATIONTASK_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/CTaskBase.hpp>
#include <scl/control/task/tasks/data_structs/SOpRotationQuatTask.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace scl
{
  /** Computes the operational space  forces for a single
   * 3-d (x,y,z) rotation task.
   *
   * This task uses "Quaternions" internally to compute
   * rotations.
   *
   * It computes:
   *
   * 1. The task model (computes, mass, jacobian, inv jacobian,
   * coriolis, centrifugal and gravity matrices/vectors).
   *
   * 2. The task servo (computes the dynamically decoupled task
   * forces and the torques. uses the task model to do so).
   */
  class COpRotationQuatTask : public scl::CTaskBase
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
     * COpRotationQuatTask specific functions
     *********************************/
    /** Default constructor : Does nothing   */
    COpRotationQuatTask();

    /** Default destructor : Does nothing.   */
    virtual ~COpRotationQuatTask(){}

    /** Sets the current goal position, when given Euler angles */
    inline void setGoal(const Eigen::Vector3d & arg_goal)
    { data_->ori_eulerang_goal_ = arg_goal;  }

    /** Whether the task has achieved its goal position. */
    inline sBool achievedGoalPos();

  protected:
    /** The actual data structure for this computational object */
    SOpRotationQuatTask* data_;

    /** Temporary variables */
    Eigen::VectorXd tmp1, tmp2;

    /** For inverting the lambda matrix (when it gets singular) */
    Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr_;

    /** True when the lambda_inv matrix turns singular. */
    sBool lambda_inv_singular_;

    /** For inverting the operational space inertia matrix
     * near singularities. 3x3 for operational point tasks. */
    Eigen::JacobiSVD<Eigen::Matrix3d > svd_;
    Eigen::Matrix3d singular_values_;
  };

} /* namespace scl */

#endif /* COPROTATIONTASK_HPP_ */
