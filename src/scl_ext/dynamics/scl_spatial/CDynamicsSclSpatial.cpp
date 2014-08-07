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

namespace scl_ext
{
  CDynamicsSclSpatial::CDynamicsSclSpatial() {
    // TODO Auto-generated constructor stub

  }

  CDynamicsSclSpatial::~CDynamicsSclSpatial() {
    // TODO Auto-generated destructor stub
  }

  bool CDynamicsSclSpatial::forwardDynamicsABA( const scl::SRobotIO *arg_io_data , scl::SGcModel *arg_gc_model , Eigen::VectorXd &ret_ddq)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
#endif
    std::vector<std::string>processing_order;

    //calculate spatial inertia and transformation matrix
    calculateTransformationAndInertia(arg_gc_model);
    //calculate processing order
    calculateOrderOfProcessing(arg_gc_model , processing_order);

    scl::sInt i;
    std::vector<Eigen::MatrixXd>   velocity(processing_order.size())  , C(processing_order.size())  , Xup(processing_order.size())  , biasforce(processing_order.size())  , U(processing_order.size()) , D(processing_order.size()) , u(processing_order.size()) , acceleration(processing_order.size()) ;
    Eigen::MatrixXd subspace_i(6,1) , Vi(6,1) , Vj(6,1) , Ci(6,1) , gravity(6,1) , ui(1,1) ,acceleration_i(6,1);

    gravity << 0 , 0 , 0 , 0 , 0 , 9.81;
    scl::sFloat q;
    std::vector<Eigen::MatrixXd>articulated_inertia_;
    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;

    //initializing articulated inertia with spatial inertia
    for(it = arg_gc_model->rbdyn_tree_.begin();it != arg_gc_model->rbdyn_tree_.end(); ++it)
    {
      articulated_inertia_.push_back(it->sp_inertia_);
    }

    // first iteration :Calculate joint velocity and acceleration

    for(i = 0 ; i < processing_order.size()  ; ++i )
    {
      int value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      Eigen::MatrixXd XJ(6,6);
      q = arg_io_data->sensors_.q_.array()[value];

      calculateTransformationAndSubspace(XJ, subspace_i ,arg_gc_model->rbdyn_tree_.at(value)->link_ds_->joint_type_, q); //calclate Xj ,S

      arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_ = (subspace_i);
      Vj = subspace_i * arg_io_data->sensors_.dq_[value];

      Xup[value] = (XJ * arg_gc_model->rbdyn_tree_.at(value)->sp_X_within_link_);
      arg_gc_model->rbdyn_tree_.at(value)->sp_X_joint_.create(arg_gc_model->rbdyn_tree_.at(value)->parent_name_,Xup[value]);

      Eigen::MatrixXd Vcross(6,6);

      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->name_ ==std::string("ground"))
      {
        Vi = Vj;
        Ci = Eigen::MatrixXd::Zero(6,1);
      }
      else
      {
        Vi = Xup[value]  * velocity[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] + Vj;
        computeCrossForVelocity(Vcross,Vi);
        Ci = Vcross * Vj;
      }

      velocity[value] = (Vi);
      C[value] = (Ci);

      computeCrossForVelocity(Vcross,Vi);
      biasforce[value] =  (-Vcross.transpose())* arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_inertia_*Vi; //calulate pa matrix

    }

    // Second Iteration : update articulated inertia and Bias force

    for( i = processing_order.size()  - 1 ; i >= 0 ; i-- )
    {
      int value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      U[value]=articulated_inertia_[value] * arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_;
      D[value] =  arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_.transpose() * (articulated_inertia_[value] * arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_);

      ui(0,0) = ( arg_io_data->actuators_.force_gc_commanded_(value,0)-(arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_.transpose()*biasforce[value])(0,0));
      u[value] = ui;

      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->name_ !=std::string("ground"))
      {
        articulated_inertia_[arg_gc_model->rbdyn_tree_.at(value)->parent_addr_->link_ds_->link_id_] += (Xup[value].transpose()*(articulated_inertia_[value] - U[value]*D[value].inverse()*U[value].transpose())*Xup[value]);
        biasforce[arg_gc_model->rbdyn_tree_.at(value)->parent_addr_->link_ds_->link_id_] += (Xup[value].transpose()*(biasforce[value] + (articulated_inertia_[value] - U[value]*D[value].inverse()*U[value].transpose())*C[value] + U[value]*u[value]*D[value].inverse()));
      }
    }

    //Third Iteration : Calculate joint acceleration

    Eigen::MatrixXd ddq( processing_order.size(),1);
    for( i = 0; i <  processing_order.size() ; i++ )
    {
      int value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;

      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->name_ ==std::string("ground"))
      {
        acceleration_i = Xup[value]*(gravity) + C[value];
      }
      else
      {
        acceleration_i = Xup[value]*acceleration[arg_gc_model->rbdyn_tree_.at(value)->parent_addr_->link_ds_->link_id_] + C[value];
      }

      if(D[value].determinant()!=0)
        ddq(value,0) = ((u[value] - U[value].transpose()*acceleration_i)*D[value].inverse())(0,0);  //calculate joint acceleration
      else
        ddq(value,0) = 0;
      acceleration_i += (arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_*ddq(value,0));
      acceleration[value] = (acceleration_i);
    }

    ret_ddq =  ddq;   //return joint acceleration
    return true;
  }


  bool CDynamicsSclSpatial::forwardDynamicsCRBA(const scl::SRobotIO *arg_io_data, scl::SGcModel *arg_gc_model, Eigen::VectorXd &ret_ddq)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
#endif
    Eigen::MatrixXd xtree(6,6);

    std::vector<std::string>processing_order;

    //calculate spatial inertia and transformation matrix
    calculateTransformationAndInertia(arg_gc_model);
    //calculate processing order
    calculateOrderOfProcessing(arg_gc_model , processing_order);

    std::vector<Eigen::MatrixXd>  velocity(processing_order.size()) , acceleration(processing_order.size()) , Xup(processing_order.size()) , force(processing_order.size()), force_subspace(processing_order.size());
    Eigen::MatrixXd subspace(6,1) , Vi(6,1) , Vj(6,1) , acceleration_i(6,1) , gravity(6,1) , Xupi , force_i(6,1);
    Eigen::MatrixXd C(processing_order.size(),1);

    scl::sInt i;

    gravity << 0 , 0 , 0 , 0 , 0 , 9.81;
    scl::sFloat q;

    //First iteration : Calculate joint velocity and Acceleration
    for(i = 0 ; i < processing_order.size() ; ++i )
    {
      scl::sInt  value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      Eigen::MatrixXd XJ(6,6);
      q = arg_io_data->sensors_.q_.array()[value];
      calculateTransformationAndSubspace(XJ, subspace, arg_gc_model->rbdyn_tree_.at(value)->link_ds_->joint_type_ , q); //calclate Xj ,S

      arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_ = subspace;

      Vj = subspace*arg_io_data->sensors_.dq_[value];
      Xupi = XJ*arg_gc_model->rbdyn_tree_.at(value)->sp_X_within_link_;
      Xup[value] = (Xupi);

      Eigen::MatrixXd Vcross(6,6);

      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->name_ ==std::string("ground"))
      {
        Vi = Vj;
        acceleration_i= Xupi * gravity ;
      }
      else
      {
        Vi = Xupi * velocity[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] + Vj;
        computeCrossForVelocity(Vcross,Vi);
        acceleration_i = Xupi * acceleration[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] + Vcross * Vj;
      }
      velocity[value] = (Vi);
      acceleration[value] = (acceleration_i);

      computeCrossForVelocity(Vcross,Vi);
      force[value] =  arg_gc_model->rbdyn_tree_.at(value)->sp_inertia_*acceleration_i + (-Vcross.transpose())* arg_gc_model->rbdyn_tree_.at(value)->sp_inertia_*Vi; //calculate joint force
    }

    //Second iteration : Calculate joint force

    for(i = processing_order.size() - 1 ; i >= 0 ; i-- )
    {
      scl::sInt value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      C(value,0) =  (arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_.transpose() * force[value])(0,0); //calculate C matrix
      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->name_ !=std::string("ground"))
      {
        force[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] += ( Xup[value ].transpose() * force[value] );  //update joint force for parent i
      }
    }

    //Third iteration : Calculate Composite Body Inertia

    std::vector<Eigen::MatrixXd> IC;
    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;

    //initializing composite  inertia with spatial inertia
    for(it = arg_gc_model->rbdyn_tree_.begin();it != arg_gc_model->rbdyn_tree_.end(); ++it)
    {
      IC.push_back(it->sp_inertia_);
    }

    Eigen::MatrixXd H(processing_order.size(),processing_order.size());
    H=Eigen::MatrixXd::Zero(processing_order.size(),processing_order.size());

    //calculate composite inertia of each body
    for( i = processing_order.size()-1 ; i>=0; i-- )
    {
      scl::sInt value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->name_ !=std::string("ground"))
      {
        IC[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] += (Xup[value].transpose()*IC[value]*Xup[value]);
      }
    }

    //Fourth iteration : Calculate joint space inertia matrix

    for( i = 0 ; i < processing_order.size() ; i++ )
    {
      scl::sInt value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      std::string name = arg_gc_model->rbdyn_tree_.at(processing_order[i])->name_;
      force_i = IC[value] * arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_;
      H(value,value) = (arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_.transpose() * force_i)(0,0);
      scl::sInt j =value;

      while(arg_gc_model->rbdyn_tree_.at(name)->parent_addr_->link_ds_->link_id_ > -1)
      {
        force_i = Xup[j].transpose() * force_i;
        name = arg_gc_model->rbdyn_tree_.at(name)->parent_addr_->name_;
        j = arg_gc_model->rbdyn_tree_.at(j)->parent_addr_->link_ds_->link_id_;
        H(value,j) = (arg_gc_model->rbdyn_tree_.at(name)->sp_S_joint_.transpose() * force_i)(0,0);   //calculate H matrix
        H(j,value) = H(value,j);
      }
    }

    //calculate joint torque
    if(H.determinant()!=0)
      ret_ddq = (H.inverse()*( arg_io_data->actuators_.force_gc_commanded_ -C)); //joint acceleration
    else
      ret_ddq = Eigen::VectorXd::Zero(processing_order.size());

    return true;
  }


  bool CDynamicsSclSpatial::inverseDynamicsNER(const scl::SRobotIO *arg_io_data, scl::SGcModel *arg_gc_model , Eigen::VectorXd &ret_fgc)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
#endif
    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;
    Eigen::MatrixXd xtree_(6,6);

    std::vector<std::string>processing_order;


    //calculate spatial inertia and transformation matrix
    calculateTransformationAndInertia(arg_gc_model);
    //calculate processing order
    calculateOrderOfProcessing(arg_gc_model , processing_order);

    scl::sInt i;
    std::vector<Eigen::MatrixXd> velocity(processing_order.size()) , acceleration(processing_order.size()) , Xup(processing_order.size()) , force(processing_order.size());
    Eigen::MatrixXd subspace(6,1) , Vi(6,1) , Vj(6,1) , acceleration_i(6,1) , gravity(6,1) , Xupi;
    Eigen::MatrixXd Tau(processing_order.size(),1);
    gravity << 0 , 0 , 0 , 0 , 0 , 9.81;
    scl::sFloat q;

    //First iteration : Calculate Joint Force and Acceleration
    for(i = 0 ; i < processing_order.size() ; ++i )
    {
      Eigen::MatrixXd XJ(6,6);
      scl::sInt value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      q = arg_io_data->sensors_.q_.array()[value];

      calculateTransformationAndSubspace(XJ, subspace ,arg_gc_model->rbdyn_tree_.at(value)->link_ds_->joint_type_ , q);

      arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_= subspace;
      Vj = subspace * arg_io_data->sensors_.dq_[value];

      Xupi = XJ * arg_gc_model->rbdyn_tree_.at(value)->sp_X_within_link_;  //transformation matrix from one link to another
      Xup[value] = (Xupi);

      Eigen::MatrixXd Vcross(6,6);

      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->name_ ==std::string("ground"))
      {
        Vi = Vj;
        acceleration_i = Xupi *( gravity )+ subspace * arg_io_data->sensors_.ddq_[value];
      }
      else
      {
        Vi = Xupi * velocity[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] + Vj;
        computeCrossForVelocity(Vcross,Vi);
        acceleration_i = Xupi * acceleration[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] + subspace * arg_io_data->sensors_.ddq_[value] + Vcross * Vj;
      }

      velocity[value] = (Vi);
      acceleration[value] = (acceleration_i);

      computeCrossForVelocity(Vcross,Vi);

      force[value] = arg_gc_model->rbdyn_tree_.at(value)->sp_inertia_*acceleration_i + (-Vcross.transpose())* arg_gc_model->rbdyn_tree_.at(value)->sp_inertia_*Vi;  //calculate force at each link
    }

    //Second iteration : Calculate Joint Torque

    for(i = processing_order.size() - 1 ; i >= 0 ; i-- )
    {
      scl::sInt value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;

      Tau(value,0) = (arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_.transpose() * force[value])(0,0);    //calculate torque at each link
      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->name_ !=std::string("ground"))
      {
        force[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_ ] += ( Xup[value].transpose() * force[value] );   //update force at parent link
      }
    }

    ret_fgc = Tau;

    return true;
  }

  bool CDynamicsSclSpatial::integrator(const scl::SRobotIO *arg_io_data , scl::SGcModel *arg_gc_model, const scl::sFloat arg_time_interval , Eigen::VectorXd &ret_q, Eigen::VectorXd &ret_dq )
  {
    Eigen::VectorXd q(arg_io_data->sensors_.q_.size()),dq(arg_io_data->sensors_.q_.size());
    for(int i = 0 ; i < arg_io_data->sensors_.q_.size() ; i++ )
    {
      dq(i) = arg_io_data->sensors_.dq_(i) + arg_io_data->sensors_.ddq_(i) * arg_time_interval;
      q(i) = arg_io_data->sensors_.q_(i) + arg_io_data->sensors_.dq_(i)  * arg_time_interval  ;
    }

    ret_q = q;
    ret_dq = dq;

    return true;
  }

  bool CDynamicsSclSpatial::calculateKineticEnergy(const scl::SRobotIO *arg_io_data, scl::SGcModel *arg_gc_model, scl::sFloat &ret_kinetic_energy)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(arg_io_data!=NULL);
#endif
    Eigen::MatrixXd xtree(6,6);

    std::vector<std::string>processing_order;

    //calculate spatial inertia and transformation matrix
    calculateTransformationAndInertia(arg_gc_model);
    //calculate processing order
    calculateOrderOfProcessing(arg_gc_model , processing_order);

    std::vector<Eigen::MatrixXd>  velocity(processing_order.size()) , acceleration(processing_order.size()) , Xup(processing_order.size()) , force(processing_order.size()), force_subspace(processing_order.size());
    Eigen::MatrixXd subspace(6,1) , Vi(6,1) , Vj(6,1) , acceleration_i(6,1) , gravity(6,1) , Xupi , force_i(6,1);
    Eigen::MatrixXd C(processing_order.size(),1);

    scl::sInt i;

    gravity << 0 , 0 , 0 , 0 , 0 , 9.81;
    scl::sFloat q;

    //First iteration : Calculate joint velocity and Acceleration
    for(i = 0 ; i < processing_order.size() ; ++i )
    {
      scl::sInt  value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      Eigen::MatrixXd XJ(6,6);
      q = arg_io_data->sensors_.q_.array()[value];
      calculateTransformationAndSubspace(XJ, subspace, arg_gc_model->rbdyn_tree_.at(value)->link_ds_->joint_type_ , q); //calclate Xj ,S

      arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_ = subspace;

      Vj = subspace*arg_io_data->sensors_.dq_[value];
      Xupi = XJ*arg_gc_model->rbdyn_tree_.at(value)->sp_X_within_link_;
      Xup[value] = (Xupi);

      Eigen::MatrixXd Vcross(6,6);

      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->name_ ==std::string("ground"))
      {
        Vi = Vj;
        acceleration_i= Xupi * gravity ;
      }
      else
      {
        Vi = Xupi * velocity[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] + Vj;
        computeCrossForVelocity(Vcross,Vi);
        acceleration_i = Xupi * acceleration[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] + Vcross * Vj;
      }
      velocity[value] = (Vi);
      acceleration[value] = (acceleration_i);

      computeCrossForVelocity(Vcross,Vi);
      force[value] =  arg_gc_model->rbdyn_tree_.at(value)->sp_inertia_*acceleration_i + (-Vcross.transpose())* arg_gc_model->rbdyn_tree_.at(value)->sp_inertia_*Vi; //calculate joint force
    }

    //Second iteration : Calculate joint force

    for(i = processing_order.size() - 1 ; i >= 0 ; i-- )
    {
      scl::sInt value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      C(value,0) =  (arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_.transpose() * force[value])(0,0); //calculate C matrix
      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->name_ !=std::string("ground"))
      {
        force[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] += ( Xup[value ].transpose() * force[value] );  //update joint force for parent i
      }
    }

    //Third iteration : Calculate Composite Body Inertia

    std::vector<Eigen::MatrixXd> IC;
    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;

    //initializing composite  inertia with spatial inertia
    for(it = arg_gc_model->rbdyn_tree_.begin();it != arg_gc_model->rbdyn_tree_.end(); ++it)
    {
      IC.push_back(it->sp_inertia_);
    }

    Eigen::MatrixXd H(processing_order.size(),processing_order.size());
    H=Eigen::MatrixXd::Zero(processing_order.size(),processing_order.size());

    //calculate composite inertia of each body
    for( i = processing_order.size()-1 ; i>=0; i-- )
    {
      scl::sInt value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      if( arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->name_ !=std::string("ground"))
      {
        IC[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_] += (Xup[value].transpose()*IC[value]*Xup[value]);
      }
    }

    //Fourth iteration : Calculate joint space inertia matrix

    for( i = 0 ; i < processing_order.size() ; i++ )
    {
      scl::sInt value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      std::string name = arg_gc_model->rbdyn_tree_.at(processing_order[i])->name_;
      force_i = IC[value] * arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_;
      H(value,value) = (arg_gc_model->rbdyn_tree_.at(processing_order[i])->sp_S_joint_.transpose() * force_i)(0,0);
      scl::sInt j =value;

      while(arg_gc_model->rbdyn_tree_.at(name)->parent_addr_->link_ds_->link_id_ > -1)
      {
        force_i = Xup[j].transpose() * force_i;
        name = arg_gc_model->rbdyn_tree_.at(name)->parent_addr_->name_;
        j = arg_gc_model->rbdyn_tree_.at(j)->parent_addr_->link_ds_->link_id_;
        H(value,j) = (arg_gc_model->rbdyn_tree_.at(name)->sp_S_joint_.transpose() * force_i)(0,0);   //calculate H matrix
        H(j,value) = H(value,j);
      }
    }

    ret_kinetic_energy  = 0.5*(arg_io_data->sensors_.dq_.transpose()*H*arg_io_data->sensors_.dq_ )(0,0);
    return true;
  }
} /* namespace scl_app */
