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
/* \file COpRotationQuatTask.cpp
 *
 *  Created on: Sep 14, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "COpRotationQuatTask.hpp"
#include <scl/Singletons.hpp>
#include <scl/util/RobotMath.hpp>
#include <sutil/CRegisteredDynamicTypes.hpp>

#include <stdio.h>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <string>

#ifdef DEBUG
#include <cassert>
#endif

namespace scl
{

  COpRotationQuatTask::COpRotationQuatTask() : CTaskBase(), data_(S_NULL), lambda_inv_singular_(false)
  { }

  //************************
  // Inherited stuff
  //************************
  bool COpRotationQuatTask::init(STaskBase* arg_task_data,
      CDynamicsBase* arg_dynamics)
  {
    try
    {
      if(S_NULL == arg_task_data)
      { throw(std::runtime_error("Passed a null task data structure"));  }

      if(false == arg_task_data->has_been_init_)
      { throw(std::runtime_error("Passed an uninitialized task data structure"));  }

      if(S_NULL == arg_dynamics)
      { throw(std::runtime_error("Passed a null dynamics object"));  }

      if(false == arg_dynamics->hasBeenInit())
      { throw(std::runtime_error("Passed an uninitialized dynamics object"));  }

      data_ = dynamic_cast<scl::SOpRotationQuatTask*>(arg_task_data);

      dynamics_ = arg_dynamics;
      data_->link_dynamic_id_ = dynamics_->getIdForLink(data_->link_name_);
      if(S_NULL == data_->link_dynamic_id_)
      { throw(std::runtime_error("Couldn't find link in dynamics object")); }

      //Defaults
      singular_values_.setZero();

      //Try to use the householder qr instead of the svd in general
      //Computing this once here initializes memory and resizes qr_
      //It will be used later.
      qr_.compute(data_->lambda_);

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCOpRotationQuatTask::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  STaskBase* COpRotationQuatTask::getTaskData()
  { return data_; }

  void COpRotationQuatTask::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


  bool COpRotationQuatTask::computeServo(const SRobotSensorData* arg_sensors)
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(S_NULL!=data_->link_dynamic_id_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      //Step 1: Find current orientation
      Eigen::Affine3d T;
      dynamics_->calculateTransformationMatrix(data_->link_dynamic_id_,T);
      data_->ori_quat_ = T.rotation();

      //Step 2: Find desired orientation.
      //NOTE : These are considered absolute euler angles wrt. the origin
      scl::eulerAngleXYZToQuat(data_->ori_eulerang_goal_, data_->ori_quat_goal_);

      //If the goal has already been achieved, exit.
      if(achievedGoalPos())
      {
        data_->force_gc_.setZero(data_->robot_->dof_);
        return true;
      }

      //Step 3: Find angular error between the current and goal quaternions:
      // (a) Slerp the quaternions.
      // (b) Get next step quaternion
      // (c) Get the difference between the current and next step in euler angles
      // (d) Apply the euler angle difference to the Jacobian pd control loop
      Eigen::Vector3d tmp_delta_angle;
      scl::quatDiffToEulerAngleXYZ(data_->ori_quat_goal_,data_->ori_quat_,tmp_delta_angle);

      //Step 4: Find angular velocity
      Eigen::Vector3d tmp_delta_angular_velocity;
      tmp_delta_angular_velocity = data_->J_ * arg_sensors->dq_;

      //Step 5: compute unit-mass force(=acceleration), due to rotation
      Eigen::Vector3d F_star;
      F_star = - data_->kp_.array() * tmp_delta_angle.array() - data_->kv_.array() * tmp_delta_angular_velocity.array();

      //Step 6: scale with Mass matrix
      // NOTE : We ignore the gravitational and centrifugal/coriolis forces here
      data_->force_task_ = data_->lambda_ * F_star;

      //Step 7: Compute GC forces, which will be used later.
      data_->force_gc_ = (data_->J_).transpose() * data_->force_task_;

      return true;
    }
    else
    { return false; }
  }

  /** Computes the dynamics (task model)
   * Assumes that the data_->model_.gc_model_ has been updated. */
  bool COpRotationQuatTask::computeModel()
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(data_->has_been_init_);
    assert(S_NULL!=data_->link_dynamic_id_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      bool flag = true;
      const SGcModel* gcm = data_->gc_model_;

      Eigen::Affine3d T;
      dynamics_->calculateTransformationMatrix(data_->link_dynamic_id_,T);
      Eigen::Vector3d pos = T * data_->pos_in_parent_;

      flag = flag && dynamics_->calculateJacobian(data_->link_dynamic_id_,pos,data_->jacobian_);

      //angluar velocity Jacobian
      data_->J_ = data_->jacobian_.block(3,0,3,data_->robot_->dof_);

      //Operational space mass/KE matrix:
      //Lambda = (J * Ainv * J')^-1
      data_->lambda_inv_ = data_->J_ * gcm->Ainv_ * data_->J_.transpose();

      if(!lambda_inv_singular_)
      {
        //The general inverse function works very well for op-point controllers.
        //3x3 matrix inversion behaves quite well. Even near singularities where
        //singular values go down to ~0.001. If the model is coarse, use a n-k rank
        //approximation with the SVD for a k rank loss in a singularity.
        qr_.compute(data_->lambda_inv_);
        if(qr_.isInvertible())
        { data_->lambda_ = qr_.inverse();  }
        else
        {
          std::cout<<"\nCOpRotationQuatTask::computeModel() : Warning. Lambda_inv is rank deficient. Using svd. Rank = "<<qr_.rank();
          lambda_inv_singular_ = true;
        }
      }

      if(lambda_inv_singular_)
      {
        //Use a Jacobi svd. No preconditioner is required coz lambda inv is square.
        //NOTE : This is slower and generally performs worse than the simple inversion
        //for small (3x3) matrices that are usually used in op-space controllers.
        svd_.compute(data_->lambda_inv_,
            Eigen::ComputeFullU | Eigen::ComputeFullV | Eigen::ColPivHouseholderQRPreconditioner);

#ifdef DEBUG
        std::cout<<"\n Singular values : "<<svd_.singularValues().transpose();
#endif

        //NOTE : A threshold of .005 works quite well for most robots.
        //Experimentally determined: Take the robot to a singularity
        //and observe the response as you allow the min singular values
        //to decrease. Stop when the robot starts to go unstable.
        //NOTE : This also strongly depends on how good your model is
        //and how fast you update it. A bad model will require higher
        //thresholds and will result in coarse motions. A better model
        //will allow much lower thresholds and will result in smooth
        //motions.
        if(svd_.singularValues()(0) > 0.005)
        { singular_values_(0,0) = 1.0/svd_.singularValues()(0);  }
        else { singular_values_(0,0) = 0.0; }
        if(svd_.singularValues()(1) > 0.005)
        { singular_values_(1,1) = 1.0/svd_.singularValues()(1);  }
        else { singular_values_(1,1) = 0.0; }
        if(svd_.singularValues()(2) > 0.005)
        { singular_values_(2,2) = 1.0/svd_.singularValues()(2);  }
        else { singular_values_(2,2) = 0.0; }

        data_->lambda_ = svd_.matrixU() * singular_values_ * svd_.matrixV().transpose();

        //Turn off the svd after 20 iterations
        //Don't worry, the qr will pop back to svd if it is still singular
        static sInt svd_ctr = 0; svd_ctr++;
        if(20>=svd_ctr)
        { svd_ctr = 0; lambda_inv_singular_ = false;  }
      }

      //Compute the Jacobian dynamically consistent generalized inverse :
      //J_dyn_inv = Ainv * J' (J * Ainv * J')^-1
      data_->jacobian_dyn_inv_ = gcm->Ainv_ * data_->J_.transpose() * data_->lambda_;

      //J' * J_dyn_inv'
      sUInt dof = data_->robot_->dof_;

      // Set up the null space
      data_->null_space_ = Eigen::MatrixXd::Identity(dof, dof) -data_->J_.transpose() * data_->jacobian_dyn_inv_.transpose();

      // We do not use the centrifugal/coriolis forces. They can cause instabilities.
      data_->mu_.setZero(data_->dof_task_,1);

      // J' * J_dyn_inv' * g(q)
      //data_->p_ =  data_->jacobian_dyn_inv_.transpose() * gcm->g_;
      data_->p_.setZero(data_->dof_task_,1);

      return flag;
    }
    else
    { return false; }
  }


  /* Achieved Goal Orientation or not */
  bool COpRotationQuatTask::achievedGoalPos()
  { return data_->ori_quat_.isApprox(data_->ori_quat_goal_); }

}
