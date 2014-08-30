/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/*
 * CDynamicsSclSpatialMath.cpp
 *
 *  Created on: Jun 9, 2014
 *      Author: Nayan Singhal <singhalnayan91@gmail.com>
 *              Brains in Silicon Lab,
 *              Stanford University.
 */

#include "CDynamicsSclSpatialMath.hpp"

namespace scl_ext
{
  bool calculateForceSubspace(Eigen::MatrixXd &arg_subspace, Eigen::MatrixXd &ret_force_subspace)
  {
#ifdef DEBUG
    assert(arg_subspace.rows()==6);
#endif
    Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(6,6);
    temp << arg_subspace;

    //Finding the invertible matrix
    while(temp.determinant() == 0)
    {
    	for(int  i=0;i<6;i++)
    		for(int j=arg_subspace.cols();j<6;j++)
    			temp(i,j) = rand()%10;
    }

    temp = temp.inverse();

    //6-ni bottom rows of matrix contains transpose of force subspace matrix
    Eigen::MatrixXd force_subspace_t = Eigen::MatrixXd::Zero(6-arg_subspace.cols(),6);
    for(int i=1;i<6;i++)
    	for(int j=0;j<6;j++)
    		force_subspace_t(i-1,j) =temp(i,j);

    ret_force_subspace = force_subspace_t.transpose();
	return true;
  }

  bool calculateTransformationAndSubspace( Eigen::MatrixXd & ret_Xlink , Eigen::MatrixXd & ret_subspace , scl::sInt arg_joint_type, scl::sFloat arg_q)
  {
#ifdef DEBUG
    assert(ret_Xlink.rows()==6);
    assert(ret_Xlink.cols()==6);
    assert(ret_subspace.rows()==6);
    assert(ret_subspace.cols()==1);
    assert(arg_joint_type<=6 && arg_joint_type >=0 && arg_joint_type!=3);
#endif
    Eigen::MatrixXd Si (6,1);
    if ( arg_joint_type >= 4 )        //revolute joint
    {
      if(arg_joint_type== 6)          //rotation joint around z axis
      {
        computeRotZAxis( ret_Xlink , arg_q  );
        ret_subspace << 0,0,1,0,0,0;
      }
      else if(arg_joint_type == 5 )   //rotation joint around y axis
      {
        computeRotYAxis( ret_Xlink, arg_q );
        ret_subspace<< 0,1,0,0,0,0;
      }
      else                            //rotation joint around x axis
      {
        computeRotXAxis( ret_Xlink , arg_q );
        ret_subspace<< 1,0,0,0,0,0;
      }
    }
    else                             //prismatic joint
    {
      Eigen::Vector3d r;
      if(arg_joint_type == 2)        //prismatic joint around z axis
      {
        r<< 0 , 0 , arg_q;
        computeTranslation( ret_Xlink , r );
        ret_subspace << 0,0,0,0,0,1;
      }
      else if(arg_joint_type == 1)   //prismatic joint around y axis
      {
        r<<0 , arg_q , 0;
        computeTranslation( ret_Xlink, r);
        ret_subspace << 0,0,0,0,1,0;
      }
      else                          //prismatic joint around x axis
      {
        r<<arg_q , 0 , 0;
        computeTranslation( ret_Xlink , r);
        ret_subspace << 0,0,0,1,0,0;
      }
    }
    return true;
  }

  bool calculateTransformationAndInertia(scl::SGcModel *arg_gc_model)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
#endif
    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;

    for(it = arg_gc_model->rbdyn_tree_.begin();it != arg_gc_model->rbdyn_tree_.end(); ++it) //return spec data
    {
      if(it++!=arg_gc_model->rbdyn_tree_.end())      //To avoid one extra value
      {
        it--;
        calculateSpatialInertia(it->sp_inertia_ , it->link_ds_->inertia_ , it->link_ds_->com_ , it->link_ds_->mass_);
        Eigen::MatrixXd xtree(6,6);
        Eigen::MatrixXd rotfromquaternion(6,6);
        computeTranslation(xtree,it->link_ds_->pos_in_parent_);
        computeRotFromQuaternion(rotfromquaternion,it->link_ds_->ori_parent_quat_);
        it->sp_X_within_link_ = rotfromquaternion*xtree;
      }

    }
    return true;
  }

  bool calculateOrderOfProcessing ( scl::SGcModel *arg_gc_model , std::vector <std::string> &ret_processing_order)
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
    assert(ret_processing_order.size()==0);
#endif
    sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;
    std::map<std::string, scl::sInt> parent_count;

    //count parent's number of children

    for(it = arg_gc_model->rbdyn_tree_.begin();it != arg_gc_model->rbdyn_tree_.end(); ++it) //return spec data
    {
      if(it++!=arg_gc_model->rbdyn_tree_.end())      //To avoid one extra value
      {
        it--;

        if(parent_count[it->name_] == 0 )
          parent_count[it->name_]=0;

        if(false == it->parent_addr_->link_ds_->is_root_) //name_!=std::string("ground"))
          parent_count[it->parent_name_]++;

      }
    }

    //calculate processing order

    std::map<std::string,scl::sInt> ::iterator it2;
    scl::sInt i=0;
    std::map<std::string,scl::sInt> ::iterator it1;
    std::stack<std::string> tree_order;

    while(static_cast<std::size_t>(i) < arg_gc_model->rbdyn_tree_.size()-1)
    {
      std::map<std::string,scl::sInt> ::iterator it1;
      int count=0;
      for(it1 = parent_count.begin() ; it1 != parent_count.end() ; it1++ )
      {
        if( parent_count[it1->first] == 0 )// && it1->second!=std::string("ground"))        //if parent counter becomes 0
        {
          tree_order.push( it1->first );               //push to stack
          if(false == (arg_gc_model->rbdyn_tree_.at(it1->first))->parent_addr_->link_ds_->is_root_)
          {
            parent_count[(arg_gc_model->rbdyn_tree_.at(it1->first))->parent_name_]--; //decrease the counter of its parent by 1
          }
          parent_count[it1->first]--;         //decrease the child counter by 1
          i++;
        }
      }
    }

    while(!tree_order.empty())
    {
      ret_processing_order.push_back(tree_order.top());   //processing tree structure formulation
      tree_order.pop();
    }

    return true;
  }

  bool computeRotXAxis(Eigen::MatrixXd &ret_transform , scl::sFloat arg_q)
  {
#ifdef DEBUG
    assert(ret_transform.rows()==6);
    assert(ret_transform.cols()==6);
#endif
    Eigen::Matrix3d rotx;
    rotx << 1 ,       0,     0 ,
        0 ,  cos(arg_q), sin(arg_q),
        0 , -sin(arg_q), cos(arg_q);
    Eigen::Matrix3d zero;
    zero = Eigen::MatrixXd::Zero(3,3);
    ret_transform << rotx,zero,zero,rotx;
    return true;
  }

  bool computeRotYAxis(Eigen::MatrixXd &ret_transform , scl::sFloat arg_q)
  {
#ifdef DEBUG
    assert(ret_transform.rows()==6);
    assert(ret_transform.cols()==6);
#endif
    Eigen::Matrix3d roty;
    roty << cos(arg_q), 0, -sin(arg_q),
        0     , 1,      0 ,
        sin(arg_q), 0,  cos(arg_q);

    Eigen::Matrix3d zero;
    zero = Eigen::MatrixXd::Zero(3,3);
    ret_transform << roty,zero,zero,roty;
    return true;
  }

  bool  computeRotZAxis(Eigen::MatrixXd &ret_transform , scl::sFloat arg_q)
  {
#ifdef DEBUG
    assert(ret_transform.rows()==6);
    assert(ret_transform.cols()==6);
#endif
    Eigen::Matrix3d rotz;
    rotz << cos(arg_q), sin(arg_q), 0,
        -sin(arg_q), cos(arg_q), 0,
        0,      0, 1;

    Eigen::Matrix3d zero;
    zero = Eigen::MatrixXd::Zero(3,3);
    ret_transform << rotz , zero , zero , rotz;
    return true;
  }

  bool  computeTranslation(Eigen::MatrixXd &ret_transform , Eigen::Vector3d arg_r)
  {
#ifdef DEBUG
    assert(ret_transform.rows()==6);
    assert(ret_transform.cols()==6);
#endif
    Eigen::Matrix3d one,rx,zero;
    one<< 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    zero = Eigen::MatrixXd::Zero(3,3);
    rx <<        0,  arg_r(2),  -arg_r(1),
        -arg_r(2),     0,   arg_r(0),
        arg_r(1), -arg_r(0),     0 ;
    ret_transform << one, zero, rx ,one;
    return true;
  }

  bool computeRotFromQuaternion( Eigen::MatrixXd &ret_transform , Eigen::Quaternion<scl::sFloat> arg_ori)
  {
#ifdef DEBUG
    assert(ret_transform.rows()==6);
    assert(ret_transform.cols()==6);
#endif
    Eigen::MatrixXd Xrtx(6,6);
    double qx = arg_ori.x() , qy = arg_ori.y() , qz = arg_ori.z() , qw = arg_ori.w();

    Eigen::Matrix3d rotation,zero;
    /** calculate rotation matrix from quaternion */
    rotation<<     -1 + 2*qx*qx + 2*qw*qw , 2*(qx*qy - qz*qw)    ,  2*(qx*qz+ qy*qw),
        2*(qx*qy + qz*qw)   , -1 + 2*qy*qy+ 2*qw*qw ,  2*(qy*qz - qx*qw),
        2*(qx*qz - qy*qw)   , 2*(qx*qw + qy*qz)  , -1 + 2*qz*qz + 2*qw*qw;

    zero = Eigen::MatrixXd::Zero(3,3);
    Xrtx<<rotation.transpose(),zero,zero,rotation.transpose();
    ret_transform = Xrtx;
    return true;
    //return Xrtx;
  }

  bool computeCrossForVelocity(Eigen::MatrixXd &ret_vcross , Eigen::MatrixXd arg_spatial_velocity)
  {
#ifdef DEBUG
    assert(ret_vcross.rows()==6);
    assert(ret_vcross.cols()==6);
    assert(arg_spatial_velocity.rows()==6);
    assert(arg_spatial_velocity.cols()==1);
#endif
    Eigen::Matrix3d v1x3,v4x6;
    const Eigen::Matrix3d & zero = Eigen::Matrix3d::Zero();
    v1x3<<                        0 , -arg_spatial_velocity(2,0) ,  arg_spatial_velocity(1,0) ,
        arg_spatial_velocity(2,0) ,                       0  , -arg_spatial_velocity(0,0) ,
        -arg_spatial_velocity(1,0) ,   arg_spatial_velocity(0,0) ,                        0;

    v4x6<<                       0 , -arg_spatial_velocity(5,0) ,  arg_spatial_velocity(4,0) ,
        arg_spatial_velocity(5,0) ,                       0 , -arg_spatial_velocity(3,0) ,
        -arg_spatial_velocity(4,0) ,  arg_spatial_velocity(3,0) ,                       0 ;

    ret_vcross<< v1x3 , zero , v4x6 , v1x3;
    return true;
  }

  bool calculateSpatialInertia(scl::sSpatialXForm &ret_spatial_inertia, Eigen::Matrix3d arg_inertia, Eigen::Vector3d arg_com, scl::sFloat arg_mass)
  {
    Eigen::Matrix3d temp;

    temp<<      0 , -arg_com[2] ,    arg_com[1],
        arg_com[2],         0,   -arg_com[0],
        -arg_com[1],   arg_com[0],         0 ;

    Eigen::MatrixXd spatial_inertia(6,6);
    Eigen::Matrix3d A = arg_inertia + arg_mass * temp * temp.transpose();
    Eigen::Matrix3d B = arg_mass * temp;
    Eigen::Matrix3d C = arg_mass * temp.transpose();
    Eigen::Matrix3d D ;
    D << arg_mass,     0,     0,
        0, arg_mass,     0,
        0,     0, arg_mass;
    spatial_inertia<<A,B,C,D;
    ret_spatial_inertia = spatial_inertia;
    return true;
  }

  bool calculateJacobian ( scl::SGcModel *arg_gc_model , const scl::SRobotSensors *arg_sensor_data, Eigen::MatrixXd &ret_jacobian )
  {
#ifdef DEBUG
    assert(arg_gc_model!=NULL);
#endif

    std::vector<std::string> processing_order;

    calculateTransformationAndInertia(arg_gc_model);
    calculateOrderOfProcessing(arg_gc_model , processing_order);
    std::vector<Eigen::MatrixXd> Xa(processing_order.size());
    std::vector<Eigen::MatrixXd > J(processing_order.size());

    for (unsigned int i = 0; i < processing_order.size() ; i++ )
    {
      scl::sInt  value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->link_ds_->link_id_;
      Eigen::VectorXd e = Eigen::VectorXd::Zero(processing_order.size());
      while(value != -1)
      {
        e(value) = 1;
        value = arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_;
      }

      Eigen::MatrixXd j = Eigen::MatrixXd::Zero(6,processing_order.size());
      for(unsigned int j = 0; j < processing_order.size(); j++)
      {
        value = arg_gc_model->rbdyn_tree_.at(processing_order[j])->link_ds_->link_id_;
        if(e(value) == 1)
        {
          Eigen::MatrixXd XJ(6,6) , subspace(1,6);
          double q = arg_sensor_data->q_.array()[value];
          calculateTransformationAndSubspace(XJ, subspace ,arg_gc_model->rbdyn_tree_.at(value)->link_ds_->joint_type_, q); //calclate Xj ,S

          Xa[value] = (XJ * arg_gc_model->rbdyn_tree_.at(value)->sp_inertia_);

          if(arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_ != -1)
          {
            Xa[value] = Xa[value] * Xa[arg_gc_model->rbdyn_tree_.at(processing_order[i])->parent_addr_->link_ds_->link_id_];
          }
          J[i] = subspace * Xa[i].inverse();
        }
      }
    }
    return true;
  }
}
