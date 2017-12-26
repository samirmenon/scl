/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

This file is released under the MIT license.
See COPYING.MIT in the scl base directory.
*/
/* \file CTaskCustom.hpp
 *
 *  Created on: Oct 20, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTASKCUSTOM_HPP_
#define CTASKCUSTOM_HPP_

#include "STaskCustom.hpp"
#include <scl/DataTypes.hpp>
#include <scl/control/task/CTaskBase.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <string>
#include <vector>

namespace scl_app
{
  /** Function to register task with the database and enable
   * dynamic typing. Call this function if you want your
   * task to be initialized by specifying stuff in a file. */
  scl::sBool registerType_TaskCustom();

  /** Computes the operational space  forces for a single
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
  class CTaskCustom : public scl::CTaskBase
  {
  public:
    /********************************
     * CTaskBase API
     *********************************/
    /** Computes the task torques */
    virtual bool computeServo(const scl::SRobotSensors* arg_sensors);

    /** Computes the dynamics (task model)
     * Assumes that the data_->model_.gc_model_ has been updated. */
    virtual bool computeModel(const scl::SRobotSensors* arg_sensors);

    /* **************************************************************
     *                   Status Get/Set Functions
     * ************************************************************** */
    /** Return this task controller's task data structure.*/
    virtual scl::STaskBase* getTaskData();

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
     * CTaskCustom specific functions
     ******************************** */
    /** Whether the task has achieved its goal position. */
    scl::sBool achievedGoalPos();

    void setFlagComputeOpPosGravity(scl::sBool arg_compute_grav)
    { flag_compute_gravity_ = arg_compute_grav; }

    /* *******************************
     * Initialization specific functions
     ******************************** */
    /** Default constructor : Does nothing   */
    CTaskCustom();

    /** Default destructor : Does nothing.   */
    virtual ~CTaskCustom(){}

    /** Initializes the task object. Required to set output
     * gc force dofs */
    virtual bool init(scl::STaskBase* arg_task_data,
        scl::CDynamicsBase* arg_dynamics);

    /** Resets the task by removing its data.
     * NOTE : Does not deallocate its data structure*/
    virtual void reset();

  protected:
    /** The actual data structure for this computational object */
    STaskCustom* data_;

    /** Temporary variables */
    Eigen::VectorXd tmp1, tmp2;

    /** For inverting the lambda matrix (when it gets singular) */
    Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr_;

    /** True when the lambda_inv matrix turns singular. */
    scl::sBool use_svd_for_lambda_inv_;

    /** For inverting the operational space inertia matrix
     * near singularities. 3x3 for operational point tasks. */
    Eigen::JacobiSVD<Eigen::Matrix3d > svd_;
    Eigen::Matrix3d singular_values_;

    scl::sBool flag_compute_gravity_;
  };

}

#endif /* CTASKCUSTOM_HPP_ */
