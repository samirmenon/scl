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
      Eigen::VectorXd &ret_ddq)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
#endif

    //calculate spatial inertia and transformation matrix
    if(false == arg_gc_model->computed_spatial_transformation_and_inertia_ )
    {
      calculateTransformationAndInertia(arg_gc_model);
      arg_gc_model->computed_spatial_transformation_and_inertia_ = true;
    }

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
    Eigen::MatrixXd Vi(6,1) , Vj(6,1) , Vcross(6,6) , XJ(6,6), gravity(6,1);
    gravity << 0,0,0,0,0,9.81;

    std::string link_name;

    std::vector<Eigen::MatrixXd>articulated_inertia;
    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;

    //Initializing articulated inertia with spatial inertia
    for(it = arg_gc_model->rbdyn_tree_.begin();it != arg_gc_model->rbdyn_tree_.end(); ++it)
    {
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
      { link->spatial_acceleration_ = Xup[link_id]*(gravity) + C[link_id];  }
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
      Eigen::VectorXd &ret_ddq)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
#endif


    //calculate spatial inertia and transformation matrix
    if(false == arg_gc_model->computed_spatial_transformation_and_inertia_ )
    {
      calculateTransformationAndInertia(arg_gc_model);
      arg_gc_model->computed_spatial_transformation_and_inertia_ = true;
    }

    //calculate tree processing order
    if(arg_gc_model->processing_order_.size() == 0)
    {
      std::vector<std::string>processing_order;
      calculateOrderOfProcessing(arg_gc_model , processing_order);
      arg_gc_model->processing_order_ = processing_order;
    }

    scl::sInt body , link_id , total_link = arg_gc_model->processing_order_.size();

    std::vector<Eigen::MatrixXd>  Xup(total_link);
    Eigen::MatrixXd  Vi(6,1) , Vj(6,1), temp_force(6,1), Vcross(6,6),  XJ(6,6) , gravity(6,1);

    std::string link_name;

    gravity<< 0 , 0 , 0 , 0 , 0 , 9.81;

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
      if( link->parent_addr_->link_ds_->is_root_ && link->parent_addr_->link_ds_->link_id_ ==-1)
      {
        Vi = Vj;
        link->spatial_acceleration_ = Xup[link_id] * gravity;
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
    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;

    //initializing composite  inertia with spatial inertia
    for(it = arg_gc_model->rbdyn_tree_.begin();it != arg_gc_model->rbdyn_tree_.end(); ++it)
    {
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

    //calculate joint acceleration
    if(arg_gc_model->M_gc_.determinant()!=0)
    {
      ret_ddq = (arg_gc_model->M_gc_.inverse()*( arg_io_data->actuators_.force_gc_commanded_ -arg_gc_model->force_gc_cc_ ));
    }
    else
    { ret_ddq = Eigen::VectorXd::Zero(total_link); }

    return true;
  }


  bool CDynamicsSclSpatial::inverseDynamicsNER(const scl::SRobotIO *arg_io_data, scl::SGcModel *arg_gc_model , Eigen::VectorXd &ret_fgc)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
#endif

    //calculate spatial inertia and transformation matrix
    if(false == arg_gc_model->computed_spatial_transformation_and_inertia_ )
    {
      calculateTransformationAndInertia(arg_gc_model);
      arg_gc_model->computed_spatial_transformation_and_inertia_ = true;
    }
    //calculate tree processing order
    if(arg_gc_model->processing_order_.size() == 0)
    {
      std::vector<std::string>processing_order;
      calculateOrderOfProcessing(arg_gc_model , processing_order);
      arg_gc_model->processing_order_ = processing_order;
    }
    scl::sInt body , link_id , total_link = arg_gc_model->processing_order_.size();

    std::vector<Eigen::MatrixXd> Xup(total_link) ;
    Eigen::MatrixXd Vi(6,1) , Vj(6,1) , Tau(total_link,1) , XJ(6,6) , Vcross(6,6) , gravity(6,1);
    gravity << 0 , 0 , 0 , 0 , 0 , 9.81;
    std::string link_name;

    //First iteration : Calculate Joint Force and Acceleration
    for(body = 0 ; body < static_cast<int>(total_link) ; ++body )
    {
      scl::SRigidBodyDyn* link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      link->sp_S_joint_ .setZero(6,1);
      link->spatial_acceleration_.setZero(6,1);
      link->spatial_velocity_.setZero(6,1);
      link->spatial_force_.setZero(6,1);

      //calculate joint transformation and motion subspace.
      calculateTransformationAndSubspace(XJ, link->sp_S_joint_ , link->link_ds_->joint_type_ , arg_io_data->sensors_.q_.array()[link_id]);

      //calculate velocity for each link
      Vj = link->sp_S_joint_ * arg_io_data->sensors_.dq_[link_id];

      //calculate transformation from one link frame to another consecutive link frame
      Xup[link_id] = XJ * link->sp_X_within_link_;

      //calculate velocity and acceleration for root node
      if(true == link->parent_addr_->link_ds_->is_root_)
      {
        Vi = Vj;
        link->spatial_acceleration_= Xup[link_id] *( gravity )+ link->sp_S_joint_ * arg_io_data->sensors_.ddq_[link_id];
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
      Tau(link_id,0) = (link->sp_S_joint_.transpose() * link->spatial_force_)(0,0);

      //calculate rigid body force
      if(false == link->parent_addr_->link_ds_->is_root_)
      {
        link->parent_addr_->spatial_force_+= ( Xup[link_id].transpose() * link->spatial_force_ );
      }
    }

    ret_fgc = Tau;

    return true;
  }

  bool CDynamicsSclSpatial::integrator(/** Current robot state. q, dq, ddq,
            sensed generalized forces and perceived external forces.*/
      scl::SRobotIO &arg_io_data,
      /** Individual link Jacobians, and composite inertial,
            centrifugal/coriolis gravity estimates. */
      scl::SGcModel *arg_gc_model,
      /** step dt time */
      const scl::sFloat arg_time_interval)
  {
    // Cache the current state.
    arg_gc_model->vec_scratch_[0] = arg_io_data.sensors_.q_;
    arg_gc_model->vec_scratch_[1] = arg_io_data.sensors_.dq_;
    arg_gc_model->vec_scratch_[2] = arg_io_data.sensors_.ddq_;

    // Original integrator: Simple forward euler
    arg_io_data.sensors_.dq_ += arg_io_data.sensors_.ddq_ * arg_time_interval;
    arg_io_data.sensors_.q_ += arg_io_data.sensors_.dq_  * arg_time_interval;

    // We use the forward euler integrator results here to compute the forward dynamics.
    forwardDynamicsCRBA(&arg_io_data, arg_gc_model ,arg_io_data.sensors_.ddq_);

    // Now use Heun's method to correct for hot.
    arg_io_data.sensors_.dq_ = arg_gc_model->vec_scratch_[1] +
        arg_time_interval*0.5*(arg_gc_model->vec_scratch_[2]+arg_io_data.sensors_.ddq_);
    arg_io_data.sensors_.q_ = arg_gc_model->vec_scratch_[0] +
        arg_time_interval*0.5*(arg_gc_model->vec_scratch_[1] + arg_io_data.sensors_.dq_);

    // Finally recompute the accelerations.
    forwardDynamicsCRBA(&arg_io_data, arg_gc_model,arg_io_data.sensors_.ddq_);

    return true;
  }

  bool CDynamicsSclSpatial::calculateKineticEnergy(const scl::SRobotIO *arg_io_data, scl::SGcModel *arg_gc_model,
      scl::sFloat &ret_kinetic_energy)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
#endif
    //calculate spatial inertia and transformation matrix
    if(false == arg_gc_model->computed_spatial_transformation_and_inertia_ )
    {
      calculateTransformationAndInertia(arg_gc_model);
      arg_gc_model->computed_spatial_transformation_and_inertia_ = true;
    }

    //calculate tree processing order
    if(arg_gc_model->processing_order_.size() == 0)
    {
      std::vector<std::string>processing_order;
      calculateOrderOfProcessing(arg_gc_model , processing_order);
      arg_gc_model->processing_order_ = processing_order;
    }

    scl::sInt body , link_id;
    Eigen::MatrixXd transformation(6,6);
    std::string link_name;

    // Calculate joint velocity and kinetic energy
    for(body = 0 ; body < static_cast<int>(arg_gc_model->processing_order_.size()) ; ++body )
    {
      scl::SRigidBodyDyn* link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      link->sp_S_joint_ = Eigen::MatrixXd::Zero(6,1);

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      //calculate joint transformation and motion subspace.
      calculateTransformationAndSubspace(transformation, link->sp_S_joint_ , link->link_ds_->joint_type_ ,
          arg_io_data->sensors_.q_.array()[link_id]);

      //calculate link velocity
      link->spatial_velocity_= link->sp_S_joint_*arg_io_data->sensors_.dq_[link_id];

      //calculate transformation from one link frame to another consecutive link frame
      transformation = transformation*link->sp_X_within_link_;

      //calculate velocity for all other link except root
      if( false  == link->parent_addr_->link_ds_->is_root_)
      {
        link->spatial_velocity_ = transformation * link->parent_addr_->spatial_velocity_ + link->spatial_velocity_;
      }

      ret_kinetic_energy += 0.5* (link->spatial_velocity_.transpose()*link->sp_inertia_*link->spatial_velocity_)(0,0);
    }
    return true;
  }

  bool CDynamicsSclSpatial::calculatePotentialEnergy( const scl::SRobotIO *arg_io_data,
      scl::SGcModel *arg_gc_model,
      scl::sFloat &ret_potential_energy)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
#endif
    //calculate spatial inertia and transformation matrix
    if(false == arg_gc_model->computed_spatial_transformation_and_inertia_ )
    {
      calculateTransformationAndInertia(arg_gc_model);
      arg_gc_model->computed_spatial_transformation_and_inertia_ = true;
    }
    //calculate tree processing order
    if(arg_gc_model->processing_order_.size() == 0)
    {
      std::vector<std::string>processing_order;
      calculateOrderOfProcessing(arg_gc_model , processing_order);
      arg_gc_model->processing_order_ = processing_order;
    }
    scl::sInt body , link_id , total_link = arg_gc_model->processing_order_.size();

    std::vector<Eigen::MatrixXd> Xup(total_link) ;
    Eigen::MatrixXd  XJ(6,6) ;
    std::string link_name;
    std::vector<Eigen::MatrixXd>articulated_inertia(total_link);

    //First iteration : Calculate transformation matrix from parent's frame to body frame
    for(body = 0 ; body < static_cast<int>(total_link) ; ++body )
    {
      scl::SRigidBodyDyn* link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

      if(-1 == link_id) { continue; } //Do nothing for the root node.

      link->sp_S_joint_ .setZero(6,1);

      //calculate joint transformation and motion subspace.
      calculateTransformationAndSubspace(XJ, link->sp_S_joint_ , link->link_ds_->joint_type_ ,
          arg_io_data->sensors_.q_.array()[link_id]);

      //calculate transformation from one link frame to another consecutive link frame
      Xup[link_id] = XJ * link->sp_X_within_link_;

      articulated_inertia[link_id] = link->sp_inertia_;
    }

    //calculate composite inertia
    Eigen::MatrixXd total_inertia = Eigen::MatrixXd::Zero(6,6);
    for(body = static_cast<int>(total_link) - 1 ; body >= 0 ; --body )
    {
      scl::SRigidBodyDyn* link = arg_gc_model->rbdyn_tree_.at(arg_gc_model->processing_order_[body]);
      link_id = link->link_ds_->link_id_;
      link_name = arg_gc_model->processing_order_[body];

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
    std::cout<<com<<"\n";
    //calculate potential energy
    scl::sFloat total_mass = total_inertia(5,5);
    ret_potential_energy = total_mass*com.transpose()*this->robot_parsed_data_->gravity_;

    return true;
  }
} /* namespace scl_app */
