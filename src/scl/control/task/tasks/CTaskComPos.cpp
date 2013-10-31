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
/* \file CTaskComPos.cpp
 *
 *  Created on: Sep 2, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/control/task/tasks/CTaskComPos.hpp>

#include <stdio.h>
#include <iostream>
#include <stdexcept>
#include <sstream>

#ifdef DEBUG
#include <cassert>
#endif

#include <Eigen/Dense>
//Only used in J singularities. Read comments in the model update function
#include <Eigen/SVD>

namespace scl
{

  CTaskComPos::CTaskComPos() : CTaskBase(), data_(S_NULL), lambda_inv_singular_(false)
  { }

  //************************
  // Inherited stuff
  //************************
  bool CTaskComPos::init(STaskBase* arg_task_data,
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

      data_ = dynamic_cast<STaskComPos*>(arg_task_data);
      data_->J_.setZero(3, data_->robot_->dof_);
      data_->J_dyn_inv_.setZero(data_->robot_->dof_, 3);
      data_->M_task_.setZero(3,3);
      data_->M_task_inv_.setZero(3,3);

      dynamics_ = arg_dynamics;

      //Defaults
      singular_values_.setZero();

      J_premultiplier_ = Eigen::Matrix3d::Identity();
      J_premultiplier_ = J_premultiplier_.array() / data_->robot_->dof_;

      //Try to use the householder qr instead of the svd in general
      //Computing this once here initializes memory and resizes qr_
      //It will be used later.
      qr_.compute(data_->M_task_);

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCTaskComPos::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  STaskBase* CTaskComPos::getTaskData()
  { return data_; }

  /** Sets the current goal position */
  bool CTaskComPos::setGoalPos(const Eigen::VectorXd & arg_goal)
  {
    if((3 == arg_goal.cols() && 1 == arg_goal.rows()) ||
        (1 == arg_goal.cols() && 3 == arg_goal.rows()) )
    {
      data_->x_goal_ = arg_goal;
      return true;
    }
#ifdef DEBUG
    else
    {
      std::cerr<<"\nCTaskComPos::setGoalPos() : Error : Goal vector's size !=3"<<std::flush;
      assert(false);
    }
#endif
    return false;
  }

  /** Sets the current goal velocity */
  bool CTaskComPos::setGoalVel(const Eigen::VectorXd & arg_goal)
  {
    if((3 == arg_goal.cols() && 1 == arg_goal.rows()) ||
        (1 == arg_goal.cols() && 3 == arg_goal.rows()) )
    {
      data_->dx_goal_ = arg_goal;
      return true;
    }
#ifdef DEBUG
    else
    {
      std::cerr<<"\nCTaskComPos::setGoalVel() : Error : Goal vector's size !=3"<<std::flush;
      assert(false);
    }
#endif
    return false;
  }

  /** Sets the current goal acceleration */
  bool CTaskComPos::setGoalAcc(const Eigen::VectorXd & arg_goal)
  {
    if((3 == arg_goal.cols() && 1 == arg_goal.rows()) ||
        (1 == arg_goal.cols() && 3 == arg_goal.rows()) )
    {
      data_->ddx_goal_ = arg_goal;
      return true;
    }
#ifdef DEBUG
    else
    {
      std::cerr<<"\nCTaskComPos::setGoalAcc() : Error : Goal vector's size !=3"<<std::flush;
      assert(false);
    }
#endif
    return false;
  }


  void CTaskComPos::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


  bool CTaskComPos::computeServo(const SRobotSensors* arg_sensors)
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
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
      data_->ddx_ = data_->ka_.array() * (data_->ddx_goal_ - data_->ddx_).array();
      data_->ddx_ += tmp2 + tmp1;

      data_->ddx_ = data_->ddx_.array().min(data_->force_task_max_.array());//Min of self and max
      data_->ddx_ = data_->ddx_.array().max(data_->force_task_min_.array());//Max of self and min

      data_->force_task_ = data_->M_task_ * data_->ddx_ + data_->force_task_grav_;

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
  bool CTaskComPos::computeModel(const SRobotSensors* arg_sensors)
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(data_->has_been_init_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      data_->x_ = data_->gc_model_->pos_com_;

      //Compute the COM Jacobian : sum over all the link com jacobians
      data_->J_.setZero(3, data_->robot_->dof_);
      sutil::CMappedTree<std::string, SRigidBodyDyn>::const_iterator it,ite;
      for(it = data_->gc_model_->rbdyn_tree_.begin(), ite = data_->gc_model_->rbdyn_tree_.end(); it!=ite; ++it)
      { data_->J_ += it->J_com_.block(0,0,3,data_->robot_->dof_);  }

      data_->J_ = J_premultiplier_ * data_->J_;

      //Operational space mass/KE matrix:
      //Lambda = (J * Ainv * J')^-1
      data_->M_task_inv_ = data_->J_ * data_->gc_model_->M_gc_inv_ * data_->J_.transpose();

      if(!lambda_inv_singular_)
      {
        //The general inverse function works very well for op-point controllers.
        //3x3 matrix inversion behaves quite well. Even near singularities where
        //singular values go down to ~0.001. If the model is updated at a much lower
        //rate than the servo (aka. 1/20 or lesser), use an n-k rank
        //approximation with the SVD for stable inverses at the cost of a k-rank loss
        //of control.
        qr_.compute(data_->M_task_inv_);
        if(qr_.isInvertible())
        { data_->M_task_ = qr_.inverse();  }
        else
        {
          std::cout<<"\nCTaskComPos::computeModel() : Warning. Lambda_inv is rank deficient. Using svd. Rank = "<<qr_.rank();
          lambda_inv_singular_ = true;
        }
      }

      if(lambda_inv_singular_)
      {
        //Use a Jacobi svd. No preconditioner is required coz lambda inv is square.
        //NOTE : This is slower and generally performs worse than the simple inversion
        //for small (3x3) matrices that are usually used in op-space controllers.
        svd_.compute(data_->M_task_inv_,
            Eigen::ComputeThinU | Eigen::ComputeThinV | Eigen::ColPivHouseholderQRPreconditioner);

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
      data_->J_dyn_inv_ = data_->gc_model_->M_gc_inv_ * data_->J_.transpose() * data_->M_task_;

      //J' * J_dyn_inv'
      sUInt dof = data_->robot_->dof_;
      data_->null_space_ = Eigen::MatrixXd::Identity(dof, dof) -
          data_->J_.transpose() * data_->J_dyn_inv_.transpose();

      // We do not use the centrifugal/coriolis forces. They can cause instabilities.
      data_->force_task_cc_.setZero(data_->dof_task_,1);

      // J' * J_dyn_inv' * g(q)
      data_->force_task_grav_ =  data_->J_dyn_inv_.transpose() * data_->gc_model_->force_gc_grav_;

      return true;
    }
    return false;
  }


  //************************
  // Task specific stuff
  //************************

  bool CTaskComPos::achievedGoalPos()
  {
    sFloat dist;
    dist = fabs((data_->x_goal_ - data_->x_).norm());

    if(dist > data_->spatial_resolution_)
    { return false; }
    else
    { return true;  }
  }

}
