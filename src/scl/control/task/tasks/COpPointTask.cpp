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
/* \file COpPointTask.cpp
 *
 *  Created on: Aug 19, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/control/task/tasks/COpPointTask.hpp>

#include <scl/control/task/tasks/data_structs/SOpPointTask.hpp>

#include <stdio.h>
#include <iostream>
#include <stdexcept>
#include <sstream>

#ifdef W_TESTING
#include <cassert>
#endif

#include <Eigen/Dense>

#ifdef W_USE_SVD_TO_INVERT_LAMBDA
//Don't always use it. Read comments in the model update function
#include <Eigen/SVD>
#endif

namespace scl
{

  COpPointTask::COpPointTask() : CTaskBase(), data_(S_NULL)
  { }

  //************************
  // Inherited stuff
  //************************
  bool COpPointTask::init(STaskBase* arg_task_data,
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

      data_ = dynamic_cast<SOpPointTask*>(arg_task_data);

      dynamics_ = arg_dynamics;
      data_->link_dynamic_id_ = dynamics_->getIdForLink(data_->link_name_);
      if(S_NULL == data_->link_dynamic_id_)
      { throw(std::runtime_error("Couldn't find link in dynamics object")); }

#ifdef W_USE_SVD_TO_INVERT_LAMBDA
      singular_values_.setZero();
#endif

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCOpPointTask::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  STaskBase* COpPointTask::getTaskData()
  { return data_; }

  void COpPointTask::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


bool COpPointTask::computeServo(const SRobotSensorData* arg_sensors)
{
#ifdef W_TESTING
  assert(has_been_init_);
  assert(S_NULL!=data_->link_dynamic_id_);
  assert(S_NULL!=dynamics_);
#endif
  if(data_->has_been_init_)
  {
    //Step 1: Find position of the op_point
    Eigen::Affine3d T;
    dynamics_->calculateTransformationMatrix(data_->link_dynamic_id_,T);
    data_->x_ = T * data_->pos_in_parent_;

    Eigen::MatrixXd &tmp_J = data_->jacobian_;
    //Global coordinates : dx = J . dq
    data_->dx_ = tmp_J * arg_sensors->dq_;

    //Compute the servo torques
    tmp1 = (data_->x_goal_ - data_->x_);
    tmp1 =  data_->kp_.array() * tmp1.array();

    tmp2 = (data_->dx_goal_ - data_->dx_);
    tmp2 = data_->kv_.array() * tmp2.array();

    //Obtain force to be applied to a unit mass floating about
    //in space (ie. A dynamically decoupled mass).
    data_->ddx_ = data_->ddx_goal_ + tmp2 + tmp1;

    data_->ddx_ = data_->ddx_.array().min(data_->force_task_max_.array());//Min of self and max
    data_->ddx_ = data_->ddx_.array().max(data_->force_task_min_.array());//Max of self and min

    data_->force_task_ = data_->lambda_ * data_->ddx_ + data_->p_;

    // T = J' ( M x F* + p)
    // We do not use the centrifugal/coriolis forces. They can cause instabilities.
    data_->force_gc_ = data_->jacobian_.transpose() * data_->force_task_;

    return true;
  }
  else
  { return false; }
}

/** Computes the dynamics (task model)
 * Assumes that the data_->model_.gc_model_ has been updated. */
bool COpPointTask::computeModel()
{
#ifdef W_TESTING
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

    flag = flag && dynamics_->calculateJacobian(
        data_->link_dynamic_id_,pos,data_->jacobian_);

    //Use the position jacobian only. This is an op-point task.
    data_->jacobian_ = data_->jacobian_.block(0,0,3,data_->robot_->dof_);

    //Operational space mass/KE matrix:
    //Lambda = (J * Ainv * J')^-1
    data_->lambda_inv_ = data_->jacobian_ * gcm->Ainv_ * data_->jacobian_.transpose();

#ifndef W_USE_SVD_TO_INVERT_LAMBDA
    //The general inverse function works very well for op-point controllers.
    //3x3 matrix inversion behaves quite well. Even near singularities where
    //singular values go down to ~0.001. If the model is coarse, use a n-k rank
    //approximation with the SVD for a k rank loss in a singularity.
    data_->lambda_ = data_->lambda_inv_.inverse();
#else
    //Use a Jacobi svd. No preconditioner is required coz lambda inv is square.
    //NOTE : This is slower and generally performs worse than the simple inversion
    //for small (3x3) matrices that are usually used in op-space controllers.
    svd_.compute(data_->lambda_inv_,
        Eigen::ComputeFullU | Eigen::ComputeFullV | Eigen::NoQRPreconditioner);

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
#endif

    //Compute the Jacobian dynamically consistent generalized inverse :
    //J_dyn_inv = Ainv * J' (J * Ainv * J')^-1
    data_->jacobian_dyn_inv_ = gcm->Ainv_ * data_->jacobian_.transpose() * data_->lambda_;

    //J' * J_dyn_inv'
    sUInt dof = data_->robot_->dof_;
    data_->null_space_ = Eigen::MatrixXd::Identity(dof, dof) -
        data_->jacobian_.transpose() * data_->jacobian_dyn_inv_.transpose();

    // We do not use the centrifugal/coriolis forces. They can cause instabilities.
    data_->mu_.setZero(data_->dof_task_,1);

    // J' * J_dyn_inv' * g(q)
    data_->p_ =  data_->jacobian_dyn_inv_.transpose() * gcm->g_;

    return flag;
  }
  else
  { return false; }
}


//************************
// Task specific stuff
//************************

bool COpPointTask::achievedGoalPos()
{
  sFloat dist;
  dist = fabs((data_->x_goal_ - data_->x_).norm());

  if(dist > data_->spatial_resolution_)
  { return false; }
  else
  { return true;  }
}

}
