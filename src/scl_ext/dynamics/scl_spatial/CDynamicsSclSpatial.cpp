/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/*
 * CDynamicsSclSpatial.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: Nayan Singhal <singhalnayan91@gmail.com>
 *              Brains in Silicon Lab,
 *              Stanford University.
 */

#include "CDynamicsSclSpatial.hpp"
#include "CDynamicsSclSpatialMath.hpp"

#include <iostream>
#include <vector>
#include <map>

namespace scl_ext
{
  CDynamicsSclSpatial::CDynamicsSclSpatial() {
    // TODO Auto-generated constructor stub

  }

  CDynamicsSclSpatial::~CDynamicsSclSpatial() {
    // TODO Auto-generated destructor stub
  }

  bool CDynamicsSclSpatial::forwardDynamicsABA( const scl::SRobotIO *arg_io_data ,
      scl::SGcModel *arg_gc_model ,
      Eigen::VectorXd &ret_ddq) const
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
    assert(has_been_init_);
#endif

    if(false == has_been_init_){return false;}

    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::const_iterator it,ite;
    // Include motor inerta in the generalized inertia matrix
    for(it = arg_gc_model->rbdyn_tree_.begin(), ite = arg_gc_model->rbdyn_tree_.end();it!=ite;++it)
    {
      if(it->link_ds_->is_root_){ continue; }
      //NOTE TODO : Remove this AFTER the ABA implementation supports inertias
      if(fabs(it->link_ds_->inertia_gc_) > 0.0001)
      {
        std::cout<<"\n\n\t\t **************** ERROR ****************"
            <<"\n\t The ABA implementation doesn't support gc inertia"
            <<"\n\t Use the CRBA implementation for fwd dynamics instead"
            <<"\n\t Or re-specify robot inertias";
        return false;
      }
    }

    //calculate spatial inertia and transformation matrix
    calculateTransformationAndInertia(arg_gc_model);

    //calculate tree processing order
    if(arg_gc_model->processing_order_.size() == 0)
    {
      std::vector<std::string>processing_order;
      calculateOrderOfProcessing(arg_gc_model , processing_order);
      arg_gc_model->processing_order_ = processing_order;
    }

    scl::sInt body , link_id , total_link = arg_gc_model->processing_order_.size();
    std::vector<Eigen::MatrixXd>   C(total_link)  , Xup(total_link)  , biasforce(total_link)  , H(total_link) ,
        D(total_link) , temp(total_link);
    Eigen::MatrixXd Vi(6,1) , Vj(6,1) , Vcross(6,6) , XJ(6,6), sp_gravity(6,1);
    sp_gravity << 0,0,0, - robot_parsed_data_->gravity_(0), - robot_parsed_data_->gravity_(1), - robot_parsed_data_->gravity_(2);

    std::string link_name;

    std::vector<Eigen::MatrixXd>articulated_inertia;

    //Initializing articulated inertia with spatial inertia
    for(it = arg_gc_model->rbdyn_tree_.begin(),ite = arg_gc_model->rbdyn_tree_.end(); it!=ite; ++it)
    {
      if(it->link_ds_->is_root_){ continue; }
      articulated_inertia.push_back(it->sp_inertia_);
    }

    // first iteration :Calculate joint velocity and bias force
    for(body = 0 ; body < static_cast<int>(total_link)  ; ++body )
    {
      scl::SRigidBodyDyn *link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      link->sp_S_joint_ .setZero(6,1);
      link->spatial_acceleration_.setZero(6,1);
      link->spatial_velocity_.setZero(6,1);
      link->spatial_force_.setZero(6,1);


      //calculate joint transformation and motion subspace.
      calculateTransformationAndSubspace(XJ, link->sp_S_joint_ , link->link_ds_->joint_type_, arg_io_data->sensors_.q_.array()[link_id]);

      Vj = link->sp_S_joint_  * arg_io_data->sensors_.dq_[link_id];

      //calculate transformation from one link frame to another consecutive link frame
      Xup[link_id] = (XJ * link->sp_X_within_link_);
      link->sp_X_joint_.create(link->parent_name_,Xup[link_id]);

      //calculate velocity for root node
      if( link->parent_addr_->link_ds_->is_root_)
      {
        Vi = Vj;
        // C = dS * dq
        C[link_id].setZero(6,1);
      }
      //calculate velocity for all other nodes
      else
      {
        Vi = Xup[link_id]  * link->parent_addr_->spatial_velocity_ + Vj;
        computeCrossForVelocity(Vcross,Vi);
        // C = dS * dq
        C[link_id] = Vcross * Vj;
      }

      link->spatial_velocity_ = Vi;

      computeCrossForVelocity(Vcross,Vi);
      //calculate bias force for each link
      biasforce[link_id] =  (-Vcross.transpose())* link->sp_inertia_*Vi;
    }

    // Second Iteration : update articulated inertia and Bias force
    for( body = static_cast<int>(total_link)  - 1 ; body >= 0 ; body-- )
    {
      scl::SRigidBodyDyn *link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      H[link_id] = articulated_inertia[link_id] * link->sp_S_joint_;
      D[link_id] =  link->sp_S_joint_.transpose() * (articulated_inertia[link_id] * link->sp_S_joint_);

      temp[link_id].setZero(1,1);
      temp[link_id](0,0) = ( arg_io_data->actuators_.force_gc_commanded_(link_id,0) -
          (link->sp_S_joint_.transpose()*biasforce[link_id])(0,0));

      //updated articulated inertia and bias for all links except root node
      if(false == link->parent_addr_->link_ds_->is_root_)
      {
        articulated_inertia[link->parent_addr_->link_ds_->link_id_] += (Xup[link_id].transpose() *
            (articulated_inertia[link_id] - H[link_id] * D[link_id].inverse() * H[link_id].transpose()) * Xup[link_id]);
        //NOTE TODO : Clean up, comment this, and split it into a few lines to make it easy to read.
        biasforce[link->parent_addr_->link_ds_->link_id_] += (Xup[link_id].transpose()*(biasforce[link_id] +
            (articulated_inertia[link_id] - H[link_id]*D[link_id].inverse()*H[link_id].transpose())*C[link_id] +
            H[link_id]*temp[link_id]*D[link_id].inverse()));
      }
    }

    //Third Iteration : Calculate joint acceleration
    Eigen::MatrixXd ddq( total_link,1);
    for( body = 0; body <  static_cast<int>(total_link) ; body++ )
    {
      scl::SRigidBodyDyn *link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[link_id];

      //calculate generalized acceleration for root node
      if( link->parent_addr_->link_ds_->is_root_)
      { link->spatial_acceleration_ = Xup[link_id]*(sp_gravity) + C[link_id];  }
      //calculate generalized acceleration for each link except root node
      else
      { link->spatial_acceleration_  = Xup[link_id] * link->parent_addr_->spatial_acceleration_ + C[link_id]; }

      if(D[link_id].determinant()!=0)
        //calculate joint acceleration
        ddq(link_id,0) = ((temp[link_id] - H[link_id].transpose()*link->spatial_acceleration_ )*D[link_id].inverse())(0,0);
      else
        ddq(link_id,0) = 0;
      link->spatial_acceleration_  += (link->sp_S_joint_*ddq(link_id,0));
    }

    //return joint acceleration
    ret_ddq =  ddq;
    return true;
  }


  bool CDynamicsSclSpatial::forwardDynamicsCRBA(const scl::SRobotIO *arg_io_data,
      scl::SGcModel *arg_gc_model,
      Eigen::VectorXd &ret_ddq) const
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
    assert(has_been_init_);
#endif

    if(false == has_been_init_){return false;}


    //calculate spatial inertia and transformation matrix
    calculateTransformationAndInertia(arg_gc_model);

    //calculate tree processing order
    if(arg_gc_model->processing_order_.size() == 0)
    {
      std::vector<std::string>processing_order;
      calculateOrderOfProcessing(arg_gc_model , processing_order);
      arg_gc_model->processing_order_ = processing_order;
    }

    scl::sInt body , link_id , total_link = arg_gc_model->processing_order_.size();

    std::vector<Eigen::MatrixXd>  Xup(total_link);
    Eigen::MatrixXd  Vi(6,1) , Vj(6,1), temp_force(6,1), Vcross(6,6),  XJ(6,6) , sp_gravity(6,1);

    std::string link_name;

    sp_gravity << 0,0,0, - robot_parsed_data_->gravity_(0), - robot_parsed_data_->gravity_(1), - robot_parsed_data_->gravity_(2);

    //First iteration : Calculate joint velocity and Acceleration
    for(body = 0 ; body < static_cast<int>(total_link) ; ++body )
    {
      scl::SRigidBodyDyn *link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      link->sp_S_joint_ = Eigen::MatrixXd::Zero(6,1);

      //calculate joint transformation and motion subspace.
      calculateTransformationAndSubspace(XJ, link->sp_S_joint_, link->link_ds_->joint_type_ , arg_io_data->sensors_.q_(link_id));

      Vj = link->sp_S_joint_*arg_io_data->sensors_.dq_[link_id];

      //calculate transformation from one link frame to another consecutive link frame
      Xup[link_id]  = XJ*link->sp_X_within_link_;

      //calculate velocity and acceleration for root node
      // NOTE TODO : This is extra ( link->parent_addr_->link_ds_->link_id_ ==-1 ).. Root node ids are not always -1.. This might
      // not work for multiple robots etc.
      if( link->parent_addr_->link_ds_->is_root_ && link->parent_addr_->link_ds_->link_id_ ==-1)
      {
        Vi = Vj;
        link->spatial_acceleration_ = Xup[link_id] * sp_gravity;
      }
      //calculate velocity and acceleration for all other nodes
      else
      {
        Vi = Xup[link_id] * link->parent_addr_->spatial_velocity_ + Vj;
        computeCrossForVelocity(Vcross,Vi);
        link->spatial_acceleration_ = Xup[link_id] * link->parent_addr_->spatial_acceleration_+ Vcross * Vj;
      }

      link->spatial_velocity_ = Vi;
      computeCrossForVelocity(Vcross,link->spatial_velocity_);

      //calculate rigid body force at each link
      link->spatial_force_ =  link->sp_inertia_*link->spatial_acceleration_ +
          (-Vcross.transpose())* link->sp_inertia_*link->spatial_velocity_; //calculate joint force
    }

    //Second iteration : Calculate joint force
    for(body= static_cast<int>(total_link) - 1 ; body >= 0 ; body-- )
    {
      scl::SRigidBodyDyn* link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      //calculate coriolis+centrifugal force
      arg_gc_model->force_gc_cc_[link_id] =  (link->sp_S_joint_.transpose() * link->spatial_force_)(0,0);

      //calculate rigid body force
      if(false == link->parent_addr_->link_ds_->is_root_ &&  link->parent_addr_->link_ds_->link_id_ !=-1)
      {
        link->parent_addr_->spatial_force_ += ( Xup[link_id ].transpose() * link->spatial_force_ );  //update joint force for parent i
      }
    }

    //Third iteration : Calculate Composite Body Inertia
    std::vector<Eigen::MatrixXd> composite_inertia;
    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::const_iterator it,ite;

    //initializing composite  inertia with spatial inertia
    for(it = arg_gc_model->rbdyn_tree_.begin(),ite = arg_gc_model->rbdyn_tree_.end(); it!=ite; ++it)
    {
      if(it->link_ds_->is_root_){ continue; }
      composite_inertia.push_back(it->sp_inertia_);
    }

    arg_gc_model->M_gc_.setZero(total_link,total_link);

    //calculate composite inertia of each body
    for( body = static_cast<int>(total_link)-1 ; body>=0; body-- )
    {
      scl::SRigidBodyDyn* link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      if(-1 == link_id) { continue; } //Do nothing for the root node.
      if( link->parent_addr_->link_ds_->link_id_ != -1)
      {
        composite_inertia[link->parent_addr_->link_ds_->link_id_] += (Xup[link_id].transpose()*composite_inertia[link_id]*Xup[link_id]);
      }
    }

    //Fourth iteration : Calculate joint space inertia matrix
    for( body = 0 ; body < static_cast<int>(total_link) ; body++ )
    {
      scl::SRigidBodyDyn* link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      std::string name = arg_gc_model->processing_order_[body];
      //calculate force at each link
      temp_force = composite_inertia[link_id] * link->sp_S_joint_;
      arg_gc_model->M_gc_(link_id,link_id) = (link->sp_S_joint_.transpose() * temp_force)(0,0);
      scl::sInt j =link_id;

      while(arg_gc_model->rbdyn_tree_.at(name)->parent_addr_->link_ds_->link_id_ > -1)
      {
        temp_force = Xup[j].transpose() * temp_force;
        name = arg_gc_model->rbdyn_tree_.at(name)->parent_addr_->name_;
        j = arg_gc_model->rbdyn_tree_.at(j)->parent_addr_->link_ds_->link_id_;

        //calculate joint space inertia
        arg_gc_model->M_gc_(link_id,j) = (arg_gc_model->rbdyn_tree_.at(name)->sp_S_joint_.transpose() * temp_force)(0,0);
        arg_gc_model->M_gc_(j,link_id) = arg_gc_model->M_gc_(link_id,j);
      }
    }

    // Include motor inerta in the generalized inertia matrix
    for(it = arg_gc_model->rbdyn_tree_.begin(), ite = arg_gc_model->rbdyn_tree_.end();it!=ite;++it)
    {
      if(it->link_ds_->is_root_){ continue; }
      arg_gc_model->M_gc_(it->link_ds_->link_id_, it->link_ds_->link_id_) += it->link_ds_->inertia_gc_;
    }

    //calculate joint acceleration
    if(arg_gc_model->M_gc_.determinant()!=0)
    {
      ret_ddq = (arg_gc_model->M_gc_.inverse()*( arg_io_data->actuators_.force_gc_commanded_ -arg_gc_model->force_gc_cc_ ));
    }
    else
    { ret_ddq = Eigen::VectorXd::Zero(total_link); }

    return true;
  }


  bool CDynamicsSclSpatial::inverseDynamicsNER(const scl::SRobotIO *arg_io_data,
      scl::SGcModel *arg_gc_model , Eigen::VectorXd &ret_fgc) const
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
    assert(has_been_init_);
#endif

    if(false == has_been_init_){return false;}

    //calculate spatial inertia and transformation matrix
    calculateTransformationAndInertia(arg_gc_model);

    //calculate tree processing order
    if(arg_gc_model->processing_order_.size() == 0)
    {
      std::vector<std::string>processing_order;
      calculateOrderOfProcessing(arg_gc_model , processing_order);
      arg_gc_model->processing_order_ = processing_order;
    }

    scl::sInt body , link_id , total_link = arg_gc_model->processing_order_.size();

    //NOTE TODO : This is horrendously slow....
    std::vector<Eigen::MatrixXd> Xup(total_link) ;

    Eigen::MatrixXd Vi(6,1) , Vj(6,1) , Tau(total_link,1) , XJ(6,6) , Vcross(6,6) , sp_gravity(6,1);
    sp_gravity << 0,0,0, - robot_parsed_data_->gravity_(0), - robot_parsed_data_->gravity_(1), - robot_parsed_data_->gravity_(2);
    std::string link_name;

    //First iteration : Calculate Joint Force and Acceleration
    for(body = 0 ; body < static_cast<int>(total_link) ; ++body )
    {
      scl::SRigidBodyDyn* link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      link->sp_S_joint_ .setZero(6,1);

      //calculate joint transformation and motion subspace.
      calculateTransformationAndSubspace(XJ, link->sp_S_joint_ , link->link_ds_->joint_type_ , arg_io_data->sensors_.q_.array()[link_id]);

      //calculate velocity for each link
      Vj = link->sp_S_joint_ * arg_io_data->sensors_.dq_[link_id];

      //calculate transformation from one link frame to another consecutive link frame
      Xup[link_id] = XJ * link->sp_X_within_link_;

      //calculate velocity and acceleration for root node
      if( link->parent_addr_->link_ds_->is_root_ && link->parent_addr_->link_ds_->link_id_ ==-1)
      {
        Vi = Vj;
        link->spatial_acceleration_= Xup[link_id] *( sp_gravity )+ link->sp_S_joint_ * arg_io_data->sensors_.ddq_[link_id];
      }
      //calculate velocity and acceleration for all other nodes
      else
      {
        Vi = Xup[link_id] * link->parent_addr_->spatial_velocity_ + Vj;
        computeCrossForVelocity(Vcross,Vi);
        link->spatial_acceleration_= Xup[link_id] * link->parent_addr_->spatial_acceleration_ +
            link->sp_S_joint_ * arg_io_data->sensors_.ddq_[link_id] + Vcross * Vj;
      }


      link->spatial_velocity_ = Vi;
      computeCrossForVelocity(Vcross,link->spatial_velocity_);

      //calculate rigid body force at each link
      link->spatial_force_ = link->sp_inertia_*link->spatial_acceleration_  +
          (-Vcross.transpose())* link->sp_inertia_*link->spatial_velocity_;  //calculate force at each link
    }

    //Second iteration : Calculate Joint Torque

    for(body = static_cast<int>(total_link) - 1 ; body >= 0 ; body-- )
    {
      scl::SRigidBodyDyn* link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];


      if(-1 == link_id) { continue; } //Do nothing for the root node.

      // Calculate torque at each link
      //NOTE TODO : S was intended to be a matrix to support multi-dof joints. Instead right now
      // it has been hacked to pick the value corresponding to the first mobility direction (or joint axis).
      Tau(link_id,0) = (link->sp_S_joint_.transpose() * link->spatial_force_)(0,0);

      //calculate rigid body force
      if( false == link->parent_addr_->link_ds_->is_root_ && link->parent_addr_->link_ds_->link_id_ !=-1)
      {
        link->parent_addr_->spatial_force_+= ( Xup[link_id].transpose() * link->spatial_force_ );
      }
    }

    ret_fgc = Tau;

    return true;
  }

  bool CDynamicsSclSpatial::integrate(
      /** Individual link Jacobians, and composite inertial,
          centrifugal/coriolis gravity estimates. */
      scl::SGcModel &arg_gc_model,
      /** Current robot state. q, dq, ddq,
          sensed generalized forces and perceived external forces.*/
      scl::SRobotIO &arg_io_data,
      /** step dt time */
      const scl::sFloat arg_time_interval) const
  {
    // Cache the current state.
    arg_gc_model.vec_scratch_[0] = arg_io_data.sensors_.q_;
    arg_gc_model.vec_scratch_[1] = arg_io_data.sensors_.dq_;
    arg_gc_model.vec_scratch_[2] = arg_io_data.sensors_.ddq_;

    // Original integrator: Simple forward euler
    arg_io_data.sensors_.dq_ += arg_io_data.sensors_.ddq_ * arg_time_interval;
    arg_io_data.sensors_.q_ += arg_io_data.sensors_.dq_  * arg_time_interval;

    // We use the forward euler integrator results here to compute the forward dynamics.
    forwardDynamicsCRBA(&arg_io_data, &arg_gc_model ,arg_io_data.sensors_.ddq_);

    // Now use Heun's method to correct for higher order terms.
    arg_io_data.sensors_.dq_ = arg_gc_model.vec_scratch_[1] +
        arg_time_interval*0.5*(arg_gc_model.vec_scratch_[2]+arg_io_data.sensors_.ddq_);
    arg_io_data.sensors_.q_ = arg_gc_model.vec_scratch_[0] +
        arg_time_interval*0.5*(arg_gc_model.vec_scratch_[1] + arg_io_data.sensors_.dq_);

    // Finally recompute the accelerations.
    forwardDynamicsCRBA(&arg_io_data, &arg_gc_model,arg_io_data.sensors_.ddq_);

    return true;
  }

  bool CDynamicsSclSpatial::integrateWithConstraints(
      /** Individual link Jacobians, and composite inertial,
          centrifugal/coriolis gravity estimates. */
      scl::SGcModel &arg_gc_model,
      /** Current robot state. q, dq, ddq,
          sensed generalized forces and perceived external forces.*/
      scl::SRobotIO &arg_io_data,
      /** The constraint Jacobian. This is a (constrained direction x dof) matrix*/
      Eigen::MatrixXd &arg_Jc,
      /** step dt time */
      const scl::sFloat arg_time_interval) const
  {
    /** Compute the null space of the constraint :
     * The constraint Jacobian will always be fat. Its transpose is skinny and has
     * a left inverse. The null space of the transpose is the space of unconstrained
     * displacements. The column vectors of the transpose are the basis vectors for
     * motions along the constraint.
     * (I - J' J'+)
     * (I - (J' J)^-1 J' J) */
    // First set the null space matrix to identity.
    Eigen::MatrixXd &Ndq = arg_gc_model.mat_scratch_n_n[0];
    Ndq.setIdentity(arg_io_data.dof_,arg_io_data.dof_);

    // Compute the svd of the Jacobian transpose (for doing null spaces later).
    arg_gc_model.svd_scratch_n_n.compute(arg_Jc.transpose(),
            Eigen::ComputeThinU | Eigen::ComputeThinV | Eigen::ColPivHouseholderQRPreconditioner);

    // Now invert the singular values
    int n_svd = arg_gc_model.svd_scratch_n_n.matrixV().rows();
    Eigen::MatrixXd & svd_S = arg_gc_model.mat_scratch_n_n[1];
    svd_S.setIdentity(n_svd,n_svd);
    for(int i=0;i<n_svd;++i)
    {
      if(arg_gc_model.svd_scratch_n_n.singularValues()(i) > 0.0005)
      { svd_S(i,i) = 1.0/arg_gc_model.svd_scratch_n_n.singularValues()(i);  }
      else
      { svd_S(i,i) = 0;  }
    }

    // Matrix sizes: J : c,n. J' : n,c.
    // SVD Matrix sizes: U : n,c; S : c,c; V' : c,c.
    // J' * Pinv : J' V S U' : [ (n,c) [(c,c) (c,c)] ] (c,n)
    // Finally compute the kinematic null space of the Jacobian.
    // NOTE TODO : Test if this actually multiplies things in the order indicated by parentheses?
    Ndq -= (arg_Jc.transpose() * /**/(arg_gc_model.svd_scratch_n_n.matrixV() * svd_S)/**/) *
        arg_gc_model.svd_scratch_n_n.matrixU().transpose();

    // Cache the current state.
    arg_gc_model.vec_scratch_[0] = arg_io_data.sensors_.q_;
    // Zero out any velocity along the constrained axis.
    arg_gc_model.vec_scratch_[1] = Ndq * arg_io_data.sensors_.dq_;
    // Zero out any acceleration along the constrained axis.
    // NOTE TODO : For now, we assume that d/dt(Ndq) is zero (changes slowly).
    // This should be fixed.
    arg_gc_model.vec_scratch_[2] = Ndq * arg_io_data.sensors_.ddq_;

    // Original integrator: Simple forward euler
    arg_io_data.sensors_.dq_ += arg_io_data.sensors_.ddq_ * arg_time_interval;
    arg_io_data.sensors_.q_ += arg_io_data.sensors_.dq_  * arg_time_interval;

    // We use the forward euler integrator results here to compute the forward dynamics.
    forwardDynamicsCRBA(&arg_io_data, &arg_gc_model ,arg_io_data.sensors_.ddq_);

    // Now use Heun's method to correct for higher order terms.
    arg_io_data.sensors_.dq_ = arg_gc_model.vec_scratch_[1] +
        arg_time_interval*0.5*(arg_gc_model.vec_scratch_[2]+ Ndq * arg_io_data.sensors_.ddq_);
    arg_io_data.sensors_.q_ = arg_gc_model.vec_scratch_[0] +
        arg_time_interval*0.5*(arg_gc_model.vec_scratch_[1] + Ndq * arg_io_data.sensors_.dq_);

    // Finally recompute the accelerations.
    forwardDynamicsCRBA(&arg_io_data, &arg_gc_model,arg_io_data.sensors_.ddq_);

    arg_io_data.sensors_.ddq_ *= Ndq;

    return true;
  }

  bool CDynamicsSclSpatial::computeEnergyKinetic(
      /** Individual link Jacobians, and composite inertial,
                centrifugal/coriolis gravity estimates. */
      scl::SGcModel &arg_gc_model,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The current generalized velocities. */
      const Eigen::VectorXd& arg_dq,
      scl::sFloat &ret_kinetic_energy) const
  {
#ifdef DEBUG
    assert(has_been_init_);
#endif

    if(false == has_been_init_){return false;}

    // Set energy to zero.
    ret_kinetic_energy = 0.0;

    //calculate spatial inertia and transformation matrix
    calculateTransformationAndInertia(&arg_gc_model);

    //calculate tree processing order
    if(arg_gc_model.processing_order_.size() == 0)
    {
      std::vector<std::string>processing_order;
      calculateOrderOfProcessing(&arg_gc_model , processing_order);
      arg_gc_model.processing_order_ = processing_order;
    }

    scl::sInt body , link_id;
    Eigen::MatrixXd transformation(6,6);
    std::string link_name;

    // Calculate joint velocity and kinetic energy
    for(body = 0 ; body < static_cast<int>(arg_gc_model.processing_order_.size()) ; ++body )
    {
      scl::SRigidBodyDyn* link = arg_gc_model.rbdyn_tree_.at(arg_gc_model.processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model.processing_order_[body];

      link->sp_S_joint_ = Eigen::MatrixXd::Zero(6,1);

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      //calculate joint transformation and motion subspace.
      calculateTransformationAndSubspace(transformation, link->sp_S_joint_ , link->link_ds_->joint_type_ ,
          arg_q.array()[link_id]);

      //calculate link velocity
      link->spatial_velocity_= link->sp_S_joint_*arg_dq[link_id];

      //calculate transformation from one link frame to another consecutive link frame
      transformation = transformation*link->sp_X_within_link_;

      //calculate velocity for all other link except root
      if( false  == link->parent_addr_->link_ds_->is_root_)
      {
        link->spatial_velocity_ = transformation * link->parent_addr_->spatial_velocity_ + link->spatial_velocity_;
      }

      ret_kinetic_energy += 0.5* (link->spatial_velocity_.transpose()*link->sp_inertia_*link->spatial_velocity_)(0,0);
    }

    //Add the kinetic energy due to the motor inertias

    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::const_iterator it,ite;

    //initializing composite  inertia with spatial inertia
    for(it = arg_gc_model.rbdyn_tree_.begin(),ite = arg_gc_model.rbdyn_tree_.end(); it!=ite; ++it)
    {
      if(it->link_ds_->is_root_){ continue; }
      int idx = it->link_ds_->link_id_;
      ret_kinetic_energy += 0.5* arg_dq(idx) * arg_dq(idx) *
          it->link_ds_->inertia_gc_;
    }

    return true;
  }

  bool CDynamicsSclSpatial::computeEnergyPotential(
      /** Individual link Jacobians, and composite inertial,
                centrifugal/coriolis gravity estimates. */
      scl::SGcModel &arg_gc_model,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      scl::sFloat &ret_potential_energy)  const
  {
#ifdef DEBUG
    assert(has_been_init_);
#endif

    if(false == has_been_init_){return false;}

    //calculate spatial inertia and transformation matrix
    calculateTransformationAndInertia(&arg_gc_model);

    //calculate tree processing order
    if(arg_gc_model.processing_order_.size() == 0)
    {
      std::vector<std::string>processing_order;
      calculateOrderOfProcessing(&arg_gc_model, processing_order);
      arg_gc_model.processing_order_ = processing_order;
    }
    scl::sInt body , link_id , total_link = arg_gc_model.processing_order_.size();

    std::vector<Eigen::MatrixXd> Xup(total_link) ;
    Eigen::MatrixXd  XJ(6,6) ;
    std::string link_name;
    std::vector<Eigen::MatrixXd>articulated_inertia(total_link);

    //First iteration : Calculate transformation matrix from parent's frame to body frame
    for(body = 0 ; body < static_cast<int>(total_link) ; ++body )
    {
      scl::SRigidBodyDyn* link = arg_gc_model.rbdyn_tree_.at(arg_gc_model.processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model.processing_order_[body];

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      link->sp_S_joint_ .setZero(6,1);

      //calculate joint transformation and motion subspace.
      calculateTransformationAndSubspace(XJ, link->sp_S_joint_ , link->link_ds_->joint_type_ ,
          arg_q.array()[link_id]);

      //calculate transformation from one link frame to another consecutive link frame
      Xup[link_id] = XJ * link->sp_X_within_link_;

      articulated_inertia[link_id] = link->sp_inertia_;
    }

    //calculate composite inertia
    Eigen::MatrixXd total_inertia = Eigen::MatrixXd::Zero(6,6);
    for(body = static_cast<int>(total_link) - 1 ; body >= 0 ; --body )
    {
      scl::SRigidBodyDyn* link = arg_gc_model.rbdyn_tree_.at(arg_gc_model.processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model.processing_order_[body];

      if( -1  != link->parent_addr_->link_ds_->link_id_)
      {
        articulated_inertia[link->parent_addr_->link_ds_->link_id_] +=
            Xup[link_id].transpose() * articulated_inertia[link_id] * Xup[link_id];
      }
      else
      {
        total_inertia += Xup[link_id].transpose() * articulated_inertia[link_id] * Xup[link_id];
      }
    }

    //calculate final center of mass position
    // I = [ Ic - m cx cx , m cx , -mcx , m1 ]
    // so I(4,4) = I(5,5) = I(6,6) = mass
    Eigen::Vector3d com;
    com(0) = total_inertia(2,4)/total_inertia(5,5);
    com(1) = total_inertia(0,5)/total_inertia(5,5);
    com(2) = total_inertia(1,3)/total_inertia(5,5);
    //calculate potential energy
    scl::sFloat total_mass = total_inertia(5,5);
    // NOTE : We assume zero potential energy at the origin plane normal to the gravity vector.
    // So going below the origin will lead to negative energy.
    ret_potential_energy = -1*total_mass*com.transpose()*robot_parsed_data_->gravity_;

    return true;
  }
} /* namespace scl_app */
