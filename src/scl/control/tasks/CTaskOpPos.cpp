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
/* \file CTaskOpPos.cpp
 *
 *  Created on: Aug 19, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CTaskOpPos.hpp"

#include <scl/serialization/SerializationJSON.hpp>

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
  namespace tasks
  {
    /********************************
     * CTaskBase API
     *********************************/
    /** Computes the task torques :
     *
     * This is essentially a function of the type:
     *   Fgc_task = PDA ( x, xgoal, dx, dxgoal, ddxgoal )
     *
     * Fgc_task is stored locally in this object's data structure. */
    bool CTaskOpPos::computeControl(
        const SRobotSensors &arg_sensors,
        const SGcModel &arg_gcm,
        const CDynamicsBase &arg_dyn)
    {
      bool flag = true;
#ifdef DEBUG
      assert(has_been_init_);
#endif
      if(data_->has_been_init_)
      {
        // Get the link at which the Jacobian is to be computed...
        const SRigidBodyDyn *rbd = arg_gcm.rbdyn_tree_.at_const(data_->link_name_);
        if(NULL == rbd){  data_->error_state_ = ERROR_STATE::BadDynamics_CouldNotFindLinkInRBDynTree; return false; }

        //Step 1: Find position of the op_point
        data_->x_ = rbd->T_o_lnk_ * data_->pos_in_parent_;

        //Global coordinates : dx = J . dq
        data_->dx_ = data_->J_ * arg_sensors.dq_;

        //Compute the servo torques
        tmp1 = (data_->x_goal_ - data_->x_);
        tmp1 =  data_->kp_.array() * tmp1.array();

        tmp2 = (data_->dx_goal_ - data_->dx_);
        tmp2 = data_->kv_.array() * tmp2.array();

        //Obtain force to be applied to a unit mass floating about
        //in space (ie. A dynamically decoupled mass).
        data_->ddx_ = data_->ka_.array() * (data_->ddx_goal_ - data_->ddx_).array();
        data_->ddx_ += tmp2 + tmp1;

        // NOTE : We apply the force limits in "task space". This is the whole point of
        // using operational space control. Since the control point is effectively a unit
        // inertia object floating in space, it is very easy to specify force limits. Moreover,
        // one can usually specify isotropic limits instead of hand-tuning different limits
        // for different directions.
        data_->ddx_ = data_->ddx_.array().min(data_->force_task_max_.array());//Min of self and max
        data_->ddx_ = data_->ddx_.array().max(data_->force_task_min_.array());//Max of self and min

        if(data_->flag_compute_op_inertia_)
        { data_->force_task_ = data_->M_task_ * data_->ddx_;  }
        else
        { data_->force_task_ = data_->ddx_;  }

        if(data_->flag_compute_op_cc_forces_)
        { data_->force_task_ += data_->force_task_cc_;  }

        // NOTE : We subtract gravity (since we want to apply an equal and opposite force
        if(data_->flag_compute_op_gravity_)
        { data_->force_task_ -= data_->force_task_grav_;  }

        // T = J' ( M x F* + p)
        // We do not use the centrifugal/coriolis forces. They can cause instabilities.
        data_->force_gc_ = data_->J_.transpose() * data_->force_task_;
      }
      else
      { return false; }

      return flag;
    }



    /** Computes the dynamics (task model, inertias, gravity etc.)
     *
     * The model matrices etc. are stored locally in this object's
     * data structure
     * */
    bool CTaskOpPos::computeModel(
        const SRobotSensors &arg_sensors,
        const SGcModel &arg_gcm,
        const CDynamicsBase &arg_dyn)
    {
#ifdef DEBUG
      assert(has_been_init_);
      assert(data_->has_been_init_);
#endif
      if(data_->has_been_init_)
      {
        bool flag = true;
        sUInt dof = arg_gcm.dof_robot_;

        // Get the link at which the Jacobian is to be computed...
        const SRigidBodyDyn *rbd = arg_gcm.rbdyn_tree_.at_const(data_->link_name_);
        if(NULL == rbd){  data_->error_state_ = ERROR_STATE::BadDynamics_CouldNotFindLinkInRBDynTree; return false; }

        flag = flag && arg_dyn.computeJacobian(data_->J_6_,*rbd,
            arg_sensors.q_,data_->pos_in_parent_);

        //Use the position jacobian only. This is an op-point task.
        data_->J_ = data_->J_6_.block(0,0,3,dof);

        //Operational space mass/KE matrix:
        if(false == data_->flag_compute_op_inertia_)
        {
          data_->M_task_inv_ = Eigen::Matrix3d::Identity();
          data_->M_task_ = Eigen::Matrix3d::Identity();
        }
        else
        {
          //Lambda = (J * Ainv * J')^-1
          data_->M_task_inv_ = data_->J_ * arg_gcm.M_gc_inv_ * data_->J_.transpose();

          if(!use_svd_for_lambda_inv_)
          {
            //The general inverse function works very well for op-point controllers.
            //3x3 matrix inversion behaves quite well. Even near singularities where
            //singular values go down to ~0.001. If the model is coarse, use a n-k rank
            //approximation with the SVD for a k rank loss in a singularity.
            qr_.compute(data_->M_task_inv_);
            if(qr_.isInvertible())
            { data_->M_task_ = qr_.inverse();  }
            else
            { use_svd_for_lambda_inv_ = true; }
          }

          if(use_svd_for_lambda_inv_)
          {
            //Use a Jacobi svd. No preconditioner is required coz lambda inv is square.
            //NOTE : This is slower and generally performs worse than the simple inversion
            //for small (3x3) matrices that are usually used in op-space controllers.
            svd_.compute(data_->M_task_inv_,
                Eigen::ComputeFullU | Eigen::ComputeFullV | Eigen::ColPivHouseholderQRPreconditioner);

            int rank_loss=0;

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
            else { singular_values_(0,0) = 0.0; rank_loss++; }
            if(svd_.singularValues()(1) > 0.005)
            { singular_values_(1,1) = 1.0/svd_.singularValues()(1);  }
            else { singular_values_(1,1) = 0.0; rank_loss++; }
            if(svd_.singularValues()(2) > 0.005)
            { singular_values_(2,2) = 1.0/svd_.singularValues()(2);  }
            else { singular_values_(2,2) = 0.0; rank_loss++; }

            if(0 < rank_loss)
            { std::cout<<"\nCTaskOpPos::computeModel() : Warning. Lambda_inv is ill conditioned. SVD rank loss (@.005) = "<<rank_loss; }

            data_->M_task_ = svd_.matrixV() * singular_values_ * svd_.matrixU().transpose();

            //Turn off the svd after 50 iterations
            //Don't worry, the qr will pop back to svd if it is still singular
            static sInt svd_ctr = 0; svd_ctr++;
            if(50>=svd_ctr)
            { svd_ctr = 0; use_svd_for_lambda_inv_ = false;  }
          }
        }

        //Compute the Jacobian dynamically consistent generalized inverse :
        //J_dyn_inv = Ainv * J' (J * Ainv * J')^-1
        data_->J_dyn_inv_ = arg_gcm.M_gc_inv_ * data_->J_.transpose() * data_->M_task_;

        //J' * J_dyn_inv'
        data_->null_space_ = Eigen::MatrixXd::Identity(dof, dof) -
            data_->J_.transpose() * data_->J_dyn_inv_.transpose();

        // We do not use the centrifugal/coriolis forces. They can cause instabilities.
        // NOTE TODO : Fix this...
        //    if(data_->flag_compute_op_cc_forces_)
        //    { /** I need some code */ }
        //    else
        //    { data_->force_task_cc_.setZero(data_->dof_task_,1);  }
        data_->force_task_cc_.setZero(data_->dof_task_,1);

        // J' * J_dyn_inv' * g(q)
        if(data_->flag_compute_op_gravity_)
        { data_->force_task_grav_ =  data_->J_dyn_inv_.transpose() * arg_gcm.force_gc_grav_;  }

        return flag;
      }
      else
      { return false; }
    }

    /* **************************************************************
     *                   Status Get/Set Functions
     * ************************************************************** */
    /** Sets the current goal position, velocity, and acceleration.
     * Leave vector pointers NULL if you don't want to set them. */
    bool CTaskOpPos::setStateGoal(
        const Eigen::VectorXd * arg_xgoal,
        const Eigen::VectorXd * arg_dxgoal,
        const Eigen::VectorXd * arg_ddxgoal)
    {
      if(NULL != arg_xgoal)
      {
        if((data_->dof_task_ == arg_xgoal->cols() && 1 == arg_xgoal->rows()) ||
            (1 == arg_xgoal->cols() && data_->dof_task_ == arg_xgoal->rows()) )
        { data_->x_goal_ = *arg_xgoal; }
        else {
          std::cerr<<"\nCTaskOpPos::setStateGoal() : Error : XGoal vector's size != data_->dof_task_"<<std::flush;
          return false;
        }
      }

      // Set velocities
      if(NULL != arg_xgoal)
      {
        if((data_->dof_task_ == arg_dxgoal->cols() && 1 == arg_dxgoal->rows()) ||
            (1 == arg_dxgoal->cols() && data_->dof_task_ == arg_dxgoal->rows()) )
        { data_->dx_goal_ = *arg_dxgoal; }
        else {
          std::cerr<<"\nCTaskOpPos::setStateGoal() : Error : dXGoal vector's size != data_->dof_task_"<<std::flush;
          return false;
        }
      }

      // Set accelerations
      if(NULL != arg_ddxgoal)
      {
        if((data_->dof_task_ == arg_ddxgoal->cols() && 1 == arg_ddxgoal->rows()) ||
            (1 == arg_ddxgoal->cols() && data_->dof_task_ == arg_ddxgoal->rows()) )
        { data_->ddx_goal_ = *arg_ddxgoal; }
        else {
          std::cerr<<"\nCTaskOpPos::setStateGoal() : Error : ddXGoal vector's size != data_->dof_task_"<<std::flush;
          return false;
        }
      }

      // All done...
      return true;
    }

    /** Gets the current goal position, velocity and acceleration.
     * If a passed pointer is NULL, nothing is returned.. */
    bool CTaskOpPos::getStateGoal(Eigen::VectorXd * ret_xgoal,
        Eigen::VectorXd * ret_dxgoal,
        Eigen::VectorXd * ret_ddxgoal) const
    {
      if(NULL!=ret_xgoal) *ret_xgoal = data_->x_goal_;
      if(NULL!=ret_dxgoal) *ret_dxgoal = data_->dx_goal_;
      if(NULL!=ret_ddxgoal) *ret_ddxgoal = data_->ddx_goal_;
      return true;
    }

    /** Gets the current position, velocity and acceleration.
     * If a passed pointer is NULL, nothing is returned.. */
    bool CTaskOpPos::getState(Eigen::VectorXd * ret_x,
        Eigen::VectorXd * ret_dx,
        Eigen::VectorXd * ret_ddx) const
    {
      if(NULL!=ret_x) *ret_x = data_->x_;
      if(NULL!=ret_dx) *ret_dx = data_->dx_;
      if(NULL!=ret_ddx) *ret_ddx = data_->ddx_;
      return true;
    }

    //************************
    // Task specific stuff
    //************************

    bool CTaskOpPos::achievedGoalPos()
    {
      sFloat dist;
      dist = fabs((data_->x_goal_ - data_->x_).norm());

      if(dist > data_->spatial_resolution_)
      { return false; }
      else
      { return true;  }
    }

    /* *******************************
     * Initialization specific functions
     ******************************** */
    bool CTaskOpPos::init(
        const sutil::CMappedList<std::string, std::string>& arg_params,
        const SRobotParsed &arg_rds)
    {
      try
      {
        // Reset the object..
        reset();
        data_ = new STaskOpPos();

        bool flag = data_->init(arg_params, &arg_rds);
        if(false == flag)
        {
          data_->error_state_ = ERROR_STATE::BadInitState_CouldNotInitFromMappedList;
          throw(std::runtime_error("Could not initialize STaskOpPos data structure from the passed mapped list key:value pairs"));
        }

        initCommon();
      }
      catch(std::exception& e)
      {
        std::cerr<<"\nCTaskOpPos::init() : ERROR : "<<e.what();
        reset();
      }
      return has_been_init_;
    }

    /** Initializes the task object. Copies values from passed object. */
    bool CTaskOpPos::init(const STaskBase &arg_task_obj)
    {
      try
      {
        // Reset the object..
        reset();

        if(false == arg_task_obj.has_been_init_)
        { throw(std::runtime_error("Passed uninitialized STaskBase object")); }

        const STaskOpPos * tmp = dynamic_cast<const STaskOpPos *>(&arg_task_obj);
        if(NULL == tmp)
        { throw(std::runtime_error(std::string("Passed STaskBase object that has incompatible type: ")+arg_task_obj.getType() )); }

        // Initialize the data..
        if(NULL != data_){  delete data_; }
        data_ = new STaskOpPos(*tmp);

        initCommon();
      }
      catch(std::exception& e)
      {
        std::cerr<<"\nCTaskOpPos::init() :"<<e.what();
        reset();
      }
      return has_been_init_;
    }

    /** Initializes the task object. Required to set output
     * gc force dofs
     *
     *  Input : A JSON string that contains all the data required to
     *          initialize the data structure (STaskOpPos). */
    bool CTaskOpPos::init(const std::string &arg_json_ds_string)
    {
      try
      {
        // Reset the object..
        reset();

        if(NULL != data_){  delete data_; }
        data_ = new STaskOpPos();

        bool flag = scl::deserializeFromJSONString(*data_,arg_json_ds_string);
        if(false == flag)
        {
          data_->error_state_ = ERROR_STATE::BadInitState_CouldNotDeserializeDataFromJSON;
          throw(std::runtime_error("Could not initialize STaskOpPos data structure from json string"));
        }

        initCommon();
      }
      catch(std::exception& e)
      {
        std::cerr<<"\nCTaskOpPos::init() :"<<e.what()<<"\n JSON String passed: \n"<<arg_json_ds_string;
        reset();
      }
      return has_been_init_;
    }


    /** Standard init stuff done by all other init functions */
    bool CTaskOpPos::initCommon()
    {
      data_->has_been_init_ = true;
      data_->error_state_ = ERROR_STATE::None;
      name_ = data_->name_;

      //Try to use the householder qr instead of the svd in general
      //Computing this once here initializes memory and resizes qr_
      //It will be used later.
      qr_.compute(data_->M_task_);

      has_been_init_ = true;
      return has_been_init_;
    }

  } // Namespace tasks
} // Namespace scl

