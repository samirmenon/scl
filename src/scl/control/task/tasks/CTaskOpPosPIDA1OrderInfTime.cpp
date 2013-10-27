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
/* \file CTaskOpPosPIDA1OrderInfTime.cpp
 *
 *  Created on: Aug 9, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/control/task/tasks/CTaskOpPosPIDA1OrderInfTime.hpp>

#include <sutil/CSystemClock.hpp>

#include <stdio.h>
#include <iostream>
#include <stdexcept>
#include <sstream>

#ifdef DEBUG
#include <cassert>
#endif

#include <Eigen/Dense>

//Don't always use it. Read comments in the model update function
#include <Eigen/SVD>

namespace scl
{

  CTaskOpPosPIDA1OrderInfTime::CTaskOpPosPIDA1OrderInfTime() :
      CTaskBase(),
      data_(S_NULL),
      lambda_inv_singular_(false),
      flag_compute_gravity_(true),
      flag_integral_gain_active_(true)
  { }

  //************************
  // Inherited stuff
  //************************
  bool CTaskOpPosPIDA1OrderInfTime::init(STaskBase* arg_task_data,
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

      data_ = dynamic_cast<STaskOpPosPIDA1OrderInfTime*>(arg_task_data);

      dynamics_ = arg_dynamics;
      data_->link_dynamic_id_ = dynamics_->getIdForLink(data_->link_name_);
      if(S_NULL == data_->link_dynamic_id_)
      { throw(std::runtime_error("Couldn't find link in dynamics object")); }

      //Defaults
      singular_values_.setZero();

      //Try to use the householder qr instead of the svd in general
      //Computing this once here initializes memory and resizes qr_
      //It will be used later.
      qr_.compute(data_->M_task_);

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCTaskOpPosPIDA1OrderInfTime::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  STaskBase* CTaskOpPosPIDA1OrderInfTime::getTaskData()
  { return data_; }

  /** Sets the current goal position */
  bool CTaskOpPosPIDA1OrderInfTime::setGoalPos(const Eigen::VectorXd & arg_goal)
  {
    if((data_->dof_task_ == arg_goal.cols() && 1 == arg_goal.rows()) ||
        (1 == arg_goal.cols() && data_->dof_task_ == arg_goal.rows()) )
    {
      data_->x_goal_ = arg_goal;
      return true;
    }
#ifdef DEBUG
    else
    {
      std::cerr<<"\nCTaskOpPosPIDA1OrderInfTime::setGoalPos() : Error : Goal vector's size != data_->dof_task_"<<std::flush;
      assert(false);
    }
#endif
    return false;
  }

  /** Sets the current goal velocity */
  bool CTaskOpPosPIDA1OrderInfTime::setGoalVel(const Eigen::VectorXd & arg_goal)
  {
    if((data_->dof_task_ == arg_goal.cols() && 1 == arg_goal.rows()) ||
        (1 == arg_goal.cols() && data_->dof_task_ == arg_goal.rows()) )
    {
      data_->dx_goal_ = arg_goal;
      return true;
    }
#ifdef DEBUG
    else
    {
      std::cerr<<"\nCTaskOpPosPIDA1OrderInfTime::setGoalVel() : Error : Goal vector's size != data_->dof_task_"<<std::flush;
      assert(false);
    }
#endif
    return false;
  }

  /** Sets the current goal acceleration */
  bool CTaskOpPosPIDA1OrderInfTime::setGoalAcc(const Eigen::VectorXd & arg_goal)
  {
    if((data_->dof_task_ == arg_goal.cols() && 1 == arg_goal.rows()) ||
        (1 == arg_goal.cols() && data_->dof_task_ == arg_goal.rows()) )
    {
      data_->ddx_goal_ = arg_goal;
      return true;
    }
#ifdef DEBUG
    else
    {
      std::cerr<<"\nCTaskOpPosPIDA1OrderInfTime::setGoalAcc() : Error : Goal vector's size != data_->dof_task_"<<std::flush;
      assert(false);
    }
#endif
    return false;
  }

  void CTaskOpPosPIDA1OrderInfTime::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


bool CTaskOpPosPIDA1OrderInfTime::computeServo(const SRobotSensorData* arg_sensors)
{
#ifdef DEBUG
  assert(has_been_init_);
  assert(S_NULL!=data_->link_dynamic_id_);
  assert(S_NULL!=dynamics_);
#endif
  if(data_->has_been_init_)
  {
    // Take a time step
    data_->integral_gain_time_pre_ = data_->integral_gain_time_curr_;
    data_->integral_gain_time_curr_ = sutil::CSystemClock::getSysTime();

    //Step 1: Find position of the op_point
    Eigen::Affine3d T;
    dynamics_->computeTransform_Depracated(data_->link_dynamic_id_,T);
    data_->x_ = T * data_->pos_in_parent_;

    Eigen::MatrixXd &tmp_J = data_->J_;
    //Global coordinates : dx = J . dq
    data_->dx_ = tmp_J * arg_sensors->dq_;

    //Compute the servo torques
    tmp1 = (data_->x_goal_ - data_->x_);
    tmp1 =  data_->kp_.array() * tmp1.array();

    tmp2 = (data_->dx_goal_ - data_->dx_);
    tmp2 = data_->kv_.array() * tmp2.array();

    //Obtain force to be applied to a unit mass floating about
    //in space (ie. A dynamically decoupled mass).
    data_->ddx_ = data_->ka_.array() * data_->ddx_goal_.array();
    data_->ddx_ += tmp2 + tmp1;

    // Compute the integral force
    double tmp_int_dt = data_->integral_gain_time_curr_ - data_->integral_gain_time_pre_;
    if(data_->integral_gain_time_max_ <= tmp_int_dt)
    {// Reset the integral gain, since the system has lost sync with real-time
      data_->integral_force_.setZero(data_->dof_task_);
    }
    else
    {
      tmp_int_dt /= data_->integral_gain_time_constt_;
      // All the array() casts are for element wise operations.
      data_->integral_force_ = data_->integral_force_.array() +
          data_->ki_.array() * (data_->x_goal_ - data_->x_).array() * tmp_int_dt;

      //Add the integral force.
      data_->ddx_ += data_->integral_force_;
    }

    data_->ddx_ = data_->ddx_.array().min(data_->force_task_max_.array());//Min of self and max
    data_->ddx_ = data_->ddx_.array().max(data_->force_task_min_.array());//Max of self and min

    if(flag_compute_gravity_)
    { data_->force_task_ = data_->M_task_ * data_->ddx_ + data_->force_task_grav_;  }
    else
    { data_->force_task_ = data_->M_task_ * data_->ddx_;  }

    // T = J' ( M x F* + p)
    // We do not use the centrifugal/coriolis forces. They can cause instabilities.
    data_->force_gc_ = data_->J_.transpose() * data_->force_task_;

    return true;
  }
  else
  { return false; }
}

/** Computes the dynamics (task model)
 * Assumes that the data_->model_.gc_model_ has been updated. */
bool CTaskOpPosPIDA1OrderInfTime::computeModel()
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
    dynamics_->computeTransform_Depracated(data_->link_dynamic_id_,T);
    Eigen::Vector3d pos = T * data_->pos_in_parent_;

    flag = flag && dynamics_->computeJacobian_Depracated(
        data_->link_dynamic_id_,pos,data_->J_);

    //Use the position jacobian only. This is an op-point task.
    data_->J_ = data_->J_.block(0,0,3,data_->robot_->dof_);

    //Operational space mass/KE matrix:
    //Lambda = (J * Ainv * J')^-1
    data_->M_task_inv_ = data_->J_ * gcm->M_gc_inv_ * data_->J_.transpose();

    if(!lambda_inv_singular_)
    {
      //The general inverse function works very well for op-point controllers.
      //3x3 matrix inversion behaves quite well. Even near singularities where
      //singular values go down to ~0.001. If the model is coarse, use a n-k rank
      //approximation with the SVD for a k rank loss in a singularity.
      qr_.compute(data_->M_task_inv_);
      if(qr_.isInvertible())
      { data_->M_task_ = qr_.inverse();  }
      else
      {
        std::cout<<"\nCTaskOpPosPIDA1OrderInfTime::computeModel() : Warning. Lambda_inv is rank deficient. Using svd. Rank = "<<qr_.rank();
        lambda_inv_singular_ = true;
      }
    }

    if(lambda_inv_singular_)
    {
      //Use a Jacobi svd. No preconditioner is required coz lambda inv is square.
      //NOTE : This is slower and generally performs worse than the simple inversion
      //for small (3x3) matrices that are usually used in op-space controllers.
      svd_.compute(data_->M_task_inv_,
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

      data_->M_task_ = svd_.matrixV() * singular_values_ * svd_.matrixU().transpose();

      //Turn off the svd after 20 iterations
      //Don't worry, the qr will pop back to svd if it is still singular
      static sInt svd_ctr = 0; svd_ctr++;
      if(20>=svd_ctr)
      { svd_ctr = 0; lambda_inv_singular_ = false;  }
    }

    //Compute the Jacobian dynamically consistent generalized inverse :
    //J_dyn_inv = Ainv * J' (J * Ainv * J')^-1
    data_->J_dyn_inv_ = gcm->M_gc_inv_ * data_->J_.transpose() * data_->M_task_;

    //J' * J_dyn_inv'
    sUInt dof = data_->robot_->dof_;
    data_->null_space_ = Eigen::MatrixXd::Identity(dof, dof) -
        data_->J_.transpose() * data_->J_dyn_inv_.transpose();

    // We do not use the centrifugal/coriolis forces. They can cause instabilities.
    data_->force_task_cc_.setZero(data_->dof_task_,1);

    // J' * J_dyn_inv' * g(q)
    if(flag_compute_gravity_)
    { data_->force_task_grav_ =  data_->J_dyn_inv_.transpose() * gcm->g_;  }

    return flag;
  }
  else
  { return false; }
}


//************************
// Task specific stuff
//************************

bool CTaskOpPosPIDA1OrderInfTime::achievedGoalPos()
{
  sFloat dist;
  dist = fabs((data_->x_goal_ - data_->x_).norm());

  if(dist > data_->spatial_resolution_)
  { return false; }
  else
  { return true;  }
}

}
