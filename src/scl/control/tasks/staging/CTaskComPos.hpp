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
/* \file CTaskComPos.hpp
 *
 *  Created on: Sep 2, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCL_CONTROL_TASKS_CCOMPOSTASK_HPP_
#define SCL_CONTROL_TASKS_CCOMPOSTASK_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/tasks/data_structs/STaskComPos.hpp>
#include <scl/control/task/CTaskBase.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <string>
#include <vector>

namespace scl
{
  /** Computes the operational space  forces for a single
   * 3-d (x,y,z) center of mass goal point Euclidean task
   *
   * It computes:
   *
   * 1. The task model (computes, mass, jacobian, inv jacobian,
   * coriolis, centrifugal and gravity matrices/vectors).
   *
   * 2. The task servo (computes the dynamically decoupled task
   * forces and the torques. uses the task model to do so). */
  class CTaskComPos : public scl::CTaskBase
  {
  public:
    /********************************
     * CTaskBase API
     *********************************/
    /** Computes the task torques */
    virtual bool computeServo(const SRobotSensors* arg_sensors);

    /** Computes the dynamics (task model)
     * Assumes that the data_->model_.gc_model_ has been updated. */
    virtual bool computeModel(const SRobotSensors* arg_sensors);

    /* **************************************************************
     *                   Status Get/Set Functions
     * ************************************************************** */
    /** Return this task controller's task data structure.*/
    virtual STaskBase* getTaskData();

    /** Sets the current goal position */
    virtual bool setGoalPos(const Eigen::VectorXd & arg_goal);

    /** Sets the current goal velocity */
    virtual bool setGoalVel(const Eigen::VectorXd & arg_goal);

    /** Sets the current goal acceleration */
    virtual bool setGoalAcc(const Eigen::VectorXd & arg_goal);

    /** Gets the current goal position. Returns false if not supported by task. */
    virtual bool getGoalPos(Eigen::VectorXd & arg_goal) const
    { arg_goal = data_->x_goal_; return true; }

    /** Gets the current goal velocity. Returns false if not supported by task. */
    virtual bool getGoalVel(Eigen::VectorXd & arg_goal) const
    { arg_goal = data_->dx_goal_; return true; }

    /** Gets the current goal acceleration. Returns false if not supported by task. */
    virtual bool getGoalAcc(Eigen::VectorXd & arg_goal) const
    { arg_goal = data_->ddx_goal_; return true; }

    /** Gets the current position. Returns false if not supported by task. */
    virtual bool getPos(Eigen::VectorXd & arg_pos) const
    { arg_pos = data_->x_; return true; }

    /** Gets the current velocity. Returns false if not supported by task. */
    virtual bool getVel(Eigen::VectorXd & arg_vel) const
    { arg_vel = data_->dx_; return true; }

    /** Gets the current acceleration. Returns false if not supported by task. */
    virtual bool getAcc(Eigen::VectorXd & arg_acc) const
    { arg_acc = data_->ddx_; return true; }

    /* *******************************
     * CTaskComPos specific functions
     ******************************** */
    /** Whether the task has achieved its goal position. */
    sBool achievedGoalPos();

    /********************************
     * Initialization specific functions
     *********************************/
    /** Default constructor : Sets stuff to zero.   */
    CTaskComPos();

    /** Default destructor : Does nothing.   */
    virtual ~CTaskComPos(){}

    /** Initializes the task object. Required to set output
     * gc force dofs */
    virtual bool init(STaskBase* arg_task_data,
        CDynamicsBase* arg_dynamics);

    /** Resets the task by removing its data.
     * NOTE : Does not deallocate its data structure*/
    virtual void reset();

  private:
    /** The actual data structure for this computational object */
    STaskComPos* data_;

    /** Temporary variables */
    Eigen::VectorXd tmp1, tmp2;

    /** For normalizing the Jacobian */
    Eigen::Matrix3d J_premultiplier_;

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
#endif /* SCL_CONTROL_TASKS_CCOMPOSTASK_HPP_ */
