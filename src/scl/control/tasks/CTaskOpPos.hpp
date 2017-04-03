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
/* \file CTaskOpPos.hpp
 *
 *  Created on: Aug 19, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SRC_SCL_CONTROL_TASKS_CTASKOPPOS_HPP_
#define SRC_SCL_CONTROL_TASKS_CTASKOPPOS_HPP_

#include <scl/DataTypes.hpp>

#include "data_structs/STaskOpPos.hpp"
#include "CTaskBase.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <string>
#include <vector>

namespace scl
{
  namespace tasks
  {
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
    class CTaskOpPos : public scl::tasks::CTaskBase
    {
    public:
      /********************************
       * CTaskBase API
       *********************************/
      /** Computes the task torques :
       *
       * This is essentially a function of the type:
       *   Fgc_task = PDA ( x, xgoal, dx, dxgoal, ddxgoal )
       *
       * Fgc_task is stored locally in this object's data structure. */
      virtual bool computeControl(
          const SRobotSensors &arg_sensors,
          const SGcModel &arg_gcm,
          const CDynamicsBase &arg_dyn);

      /** Computes the dynamics (task model, inertias, gravity etc.)
       *
       * The model matrices etc. are stored locally in this object's
       * data structure
       * */
      virtual bool computeModel(
          const SRobotSensors &arg_sensors,
          const SGcModel &arg_gcm,
          const CDynamicsBase &arg_dyn);

      /* **************************************************************
       *                   Status Get/Set Functions
       * ************************************************************** */
      /** Return this task controller's task data structure.*/
      virtual STaskBase* getTaskData() { return &data_;  }

      /** Return this task controller's task data structure (const).*/
      virtual const STaskBase* getTaskDataConst() const { return static_cast<const STaskBase*>(&data_);  }

      /** Sets the current goal position, velocity, and acceleration.
       * Leave vector pointers NULL if you don't want to set them. */
      bool setStateGoal(const Eigen::VectorXd * arg_xgoal,
          const Eigen::VectorXd * arg_dxgoal=NULL,
          const Eigen::VectorXd * arg_ddxgoal=NULL);

      /** Gets the current goal position, velocity and acceleration.
       * If a passed pointer is NULL, nothing is returned.. */
      bool getStateGoal(Eigen::VectorXd * ret_xgoal,
          Eigen::VectorXd * ret_dxgoal=NULL,
          Eigen::VectorXd * ret_ddxgoal=NULL) const;

      /** Gets the current position, velocity and acceleration.
       * If a passed pointer is NULL, nothing is returned.. */
      bool getState(Eigen::VectorXd * ret_x,
          Eigen::VectorXd * ret_dx=NULL,
          Eigen::VectorXd * ret_ddx=NULL) const;

      /* *******************************
       * CTaskOpPos specific functions
       ******************************** */
      /** Whether the task has achieved its goal position. */
      sBool achievedGoalPos();

      void setFlagComputeOpGravity(sBool arg_compute_grav)
      { data_.flag_compute_op_gravity_ = !arg_compute_grav; }

      void setFlagComputeOpCCForces(sBool arg_compute_cc_forces)
      { data_.flag_compute_op_cc_forces_ = !arg_compute_cc_forces; }

      void setFlagComputeOpInertia(sBool arg_compute_inertia)
      { data_.flag_compute_op_inertia_ = !arg_compute_inertia; }

      /* *******************************
       * Initialization specific functions
       ******************************** */
      /** Default constructor : Does nothing   */
      CTaskOpPos() : CTaskBase("CTaskOpPos"){}

      /** Default destructor : Does nothing.   */
      virtual ~CTaskOpPos(){}

      /** Initializes the task object. Required to set output
       * gc force dofs
       *
       *  Input : A JSON string that contains all the data required to
       *          initialize the data structure (STaskOpPos). */
      bool init(const std::string &arg_json_ds_string);

      /** Resets the task by removing its data.
       * NOTE : Does not deallocate its data structure*/
      virtual void reset()
      {
        data_.has_been_init_ = false;
        has_been_init_ = false;
      }

    protected:
      /** The actual data structure for this computational object */
      STaskOpPos data_;

      /** Temporary variables */
      Eigen::VectorXd tmp1, tmp2;

      /** For inverting the lambda matrix (when it gets singular) */
      Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr_;

      /** True when the lambda_inv matrix turns singular. */
      sBool use_svd_for_lambda_inv_=false;

      /** For inverting the operational space inertia matrix
       * near singularities. 3x3 for operational point tasks. */
      Eigen::JacobiSVD<Eigen::Matrix3d > svd_;
      Eigen::Matrix3d singular_values_;
    };
  }
}
#endif /* SRC_SCL_CONTROL_TASKS_CTASKOPPOS_HPP_ */
