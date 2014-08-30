/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/*
 * test_dynamics_scl_spatial_math.cpp
 *
 *  Created on: Jun 12, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author : Nayan Singhal <singhalnayan91@gmail.com>
 *              Brains in Silicon Lab,
 *              Stanford University.
 *
 *  Edited by: Samir Menon <smenon@stanford.edu>
 */
#include "test_dynamics_sclspatial_math.hpp"

#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatialMath.hpp>
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>

#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/robot/CRobotApp.hpp>
#include <scl/util/HelperFunctions.hpp>

#include <sutil/CSystemClock.hpp>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <stdexcept>
#include <iostream>

using namespace scl;
using namespace scl_ext;

namespace scl_test
{
  bool checkMatrix(Eigen::MatrixXd & arg_matrix , Eigen::MatrixXd & arg_check_matrix)
  {
    sFloat precision = 0.00001;
    for(sInt row = 0 ; row < arg_matrix.rows(); row++ )
    {
      for(sInt col = 0 ; col < arg_matrix.cols() ; col++ )
      {
        if(!(((arg_matrix(row,col) - precision ) < arg_check_matrix(row,col) ) && ((arg_matrix(row,col) + precision ) > arg_check_matrix(row,col))))
        {
          return false;
        }

      }
    }

    return true;
  }

  void test_dynamics_sclspatial_math()
  {
    scl::sUInt test_id = 1;
    try
    {
      //Test 1
      Eigen::MatrixXd transform(6,6);
      scl::sFloat q=3.14159265359;
      std::cout<<"Joint position value : "<<q<<"\n";

      //value after calculating rotation around x axis
      Eigen::MatrixXd check_computeRotXAxis (6,6);
      check_computeRotXAxis << 1 , 0 , 0 , 0 , 0 , 0 ,
          0 , -1 , 0 , 0 , 0 , 0 ,
          0 , 0 , -1 , 0 , 0 , 0 ,
          0 , 0 , 0 , 1 , 0 , 0 ,
          0 , 0 , 0 , 0 , -1 , 0 ,
          0 , 0 , 0 , 0 , 0 , -1;


      if ( false == scl_ext::computeRotXAxis(transform,q) )
      { throw(std::runtime_error("Failed to calculate transformation matrix around x axis"));  }
      else
      {
        if ( true == checkMatrix(transform , check_computeRotXAxis ))
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated transformation matrix around x axis\n\n";
        else
        {
          throw(std::runtime_error("Failed to calculate transformation matrix around x axis"));
        }
      }

      //Test 2

      //value after calculating rotation around y axis
      Eigen::MatrixXd check_computeRotYAxis (6,6);
      check_computeRotYAxis <<     -1 , 0 , 0 , 0 , 0 , 0 ,
          0 , 1 , 0 , 0 , 0 , 0 ,
          0 , 0 , -1 , 0 , 0 , 0 ,
          0 , 0 , 0 , -1 , 0 , 0 ,
          0 , 0 , 0 , 0 , 1 , 0 ,
          0 , 0 , 0 , 0 , 0 , -1;

      if (false == scl_ext::computeRotYAxis(transform,q) )
      { throw(std::runtime_error("Failed to calculate transformation matrix around y axis"));  }
      else
      {
        if ( true  == checkMatrix(transform , check_computeRotYAxis))
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated transformation matrix around y axis\n\n";
        else
        {
          throw(std::runtime_error("Failed to calculate transformation matrix around y axis"));
        }
      }

      //Test 3
      Eigen::MatrixXd check_computeRotZAxis (6,6);
      //value after calculating rotation around z axis
      check_computeRotZAxis << -1 , 0 , 0 , 0 , 0 , 0 ,
          0 , -1 , 0 , 0 , 0 , 0 ,
          0 , 0 , 1 , 0 , 0 , 0 ,
          0 , 0 , 0 , -1 , 0 , 0 ,
          0 , 0 , 0 , 0 , -1 , 0 ,
          0 , 0 , 0 , 0 , 0 , 1 ;

      if (false == scl_ext::computeRotZAxis(transform,q) && false == checkMatrix(transform , check_computeRotZAxis))
      { throw(std::runtime_error("Failed to calculate transformation matrix around z axis"));  }
      else
      {
        if ( true  == checkMatrix(transform , check_computeRotZAxis))
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated transformation matrix around z axis\n\n";
        else
        {
          throw(std::runtime_error("Failed to calculate transformation matrix around z axis"));
        }
      }

      //Test 4
      Eigen::Vector3d r;
      r<<1,2,3;

      Eigen::MatrixXd check_computeTranslation (6,6);
      //value after calculating translation matrix from one link to another
      check_computeTranslation << 1 , 0 , 0 , 0 , 0 , 0 ,
          0 , 1 , 0 , 0 , 0 , 0 ,
          0 , 0 , 1 , 0 , 0 , 0 ,
          0 , 3 , -2, 1 , 0 , 0 ,
          -3, 0 , 1 , 0 , 1 , 0 ,
          2 , -1, 0 , 0 , 0 , 1 ;


      if (false == computeTranslation(transform,r))
      { throw(std::runtime_error("Failed to calculate translation matrix from one link to another"));  }
      else
      {
        if ( true  == checkMatrix(transform , check_computeTranslation))
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated transformation matrix from one link to another\n\n";
        else
        {
          throw(std::runtime_error("Failed to calculate transformation matrix afrom one link to another"));
        }
      }

      //Test 5
      Eigen::Quaternion<scl::sFloat> ori (1,2,3,4);

      Eigen::MatrixXd check_computeRotFromQuaternion (6,6);
      //value after calculating transformation matrix from quaternion
      check_computeRotFromQuaternion << 9 , 20 , 10 , 0 , 0 , 0 ,
          4 , 19 , 28 , 0 , 0 , 0 ,
          22, 20 , 33 , 0 , 0 , 0 ,
          0 ,  0 , 0  , 9 , 20, 10,
          0 ,  0 , 0  , 4 , 19, 28,
          0 ,  0 , 0  , 22, 20, 33;

      if (false == computeRotFromQuaternion(transform,ori))
      { throw(std::runtime_error("Failed to calculate transformation matrix from quaternion"));  }
      else
      {
        if ( true  == checkMatrix(transform , check_computeRotFromQuaternion))
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated transformation matrix from quaternion\n\n";
        else
        {
          throw(std::runtime_error("Failed to calculate transformation matrix from quaternion"));
        }
      }

      //Test 6
      Eigen::MatrixXd velocity(6,1);
      velocity << 1,2,1,2,1,2;

      Eigen::MatrixXd check_computeCrossForVelocity (6,6);
      //value after calculating cross product of spatial velocity matrix
      check_computeCrossForVelocity<<0 ,-1, 2 , 0 , 0 , 0 ,
          1 , 0 , -1 , 0 , 0 , 0 ,
          -2 , 1 ,  0 , 0 , 0 , 0 ,
          0 ,-2 ,  1 , 0 , -1, 2 ,
          2 , 0 , -2 , 1 ,  0, -1,
          -1 , 2 ,  0 , -2,  1,  0;

      if (false == computeCrossForVelocity(transform,velocity))
      { throw(std::runtime_error("Failed to calculate cross product of spatial velocity matrix"));  }
      else
      {
        if ( true  == checkMatrix(transform , check_computeCrossForVelocity))
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated cross product of spatial velocity matrix\n\n";
        else
        {
          throw(std::runtime_error("Failed to calculate cross product of spatial velocity matrix"));
        }
      }

      //Test 7
      Eigen::Matrix3d inertia;
      inertia = Eigen::Matrix3d::Identity();
      Eigen::Vector3d com;
      com<<1,2,1;
      scl::sFloat mass = 1.0;

      Eigen::MatrixXd check_calculateSpatialInertia(6,6);
      //value after calculating individual spatial inertia
      check_calculateSpatialInertia << 6 , -2 , -1 , 0 , -1 , 2 ,
          -2 , 3 ,-2 , 1 , 0 ,-1,
          -1 ,-2 , 6 ,-2 , 1 , 0,
          0 , 1 ,-2 , 1 , 0 , 0,
          -1 , 0 , 1 , 0 , 1 , 0,
          2 ,-1 , 0 , 0 , 0 , 1;

      scl::sSpatialXForm sp_inertia;
      if (false == calculateSpatialInertia(sp_inertia,inertia,com,mass))
      { throw(std::runtime_error("Failed to calculate individual spatial inertia"));  }
      else
      {
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated individual spatial inertia\n\n";
      }

      //Test 8
      Eigen::MatrixXd Xlink(6,6),subspace(6,1);
      scl::sInt joint_type =6;

      Eigen::MatrixXd check_Xlink(6,6);
      //value after calculating transformation matrix
      check_Xlink<< -1 , 0 , 0 , 0 , 0 , 0 ,
          0 , -1 , 0 , 0 , 0 , 0 ,
          0 , 0 , 1 , 0 , 0 , 0 ,
          0 , 0 , 0 , -1 , 0 , 0 ,
          0 , 0 , 0 , 0 , -1 , 0 ,
          0 , 0 , 0 , 0 , 0 , 1 ;

      Eigen::MatrixXd check_subspace(6,1);
      //value after calculating motion subspace
      check_subspace<< 0, 0, 1, 0, 0, 0;
      if (false == calculateTransformationAndSubspace(Xlink,subspace,joint_type,q))
      { throw(std::runtime_error("Failed to calculate transformation matrix and motion subspace matrix "));  }
      else
      {
        if ( true  == checkMatrix(Xlink , check_Xlink) && true == checkMatrix(subspace , check_subspace) )
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated transformation matrix and motion subspace matrix\n\n";
        else
        {
          throw(std::runtime_error("Failed to calculate transformation matrix and motion subspace matrix"));
        }
      }


      //Test 9
      /**
       * Tests the mapped tree with the tree:
       *
       *              --
       *             ground
       *            /
       *           l0
       *          /  \
       *         l1  l3
       *        /
       *       l2
       */

      SGcModel model;
      SRigidBodyDyn l0,l1,l2,l3,l4,l5;
      SRigidBody link_ds_0,link_ds_1,link_ds_2,link_ds_3,link_ds_4,link_ds_5;

      //Create node l0
      l0.name_ = "l0";  l0.parent_name_ = "ground";
      l0.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l0.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
      link_ds_0.name_="l0"; link_ds_0.link_id_=0; l0.link_ds_ = &link_ds_0;
      model.rbdyn_tree_.create(l0.name_,l0, false);

      //Create node l1
      l1.name_ = "l1";  l1.parent_name_ = "l0";
      l1.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l1.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
      link_ds_1.name_="l1"; link_ds_1.link_id_=1; l1.link_ds_ = &link_ds_1;
      model.rbdyn_tree_.create(l1.name_,l1, false);

      //Create node l2
      l2.name_ = "l2";  l2.parent_name_ = "l1";
      l2.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l2.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
      link_ds_2.name_="l2"; link_ds_2.link_id_=2; l2.link_ds_ = &link_ds_2;
      model.rbdyn_tree_.create(l2.name_,l2, false);

      //Create node l3
      l3.name_ = "l3";  l3.parent_name_ = "l0";
      l3.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l3.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
      link_ds_3.name_="l3"; link_ds_3.link_id_=3; l3.link_ds_ = &link_ds_3;
      model.rbdyn_tree_.create(l3.name_,l3, false);

      //Create node ground (root)
      l4.name_ = "ground";  l4.parent_name_ = "not assigned";
      l4.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l4.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
      link_ds_4.name_="ground"; link_ds_4.link_id_=-1; l4.link_ds_ = &link_ds_4;
      link_ds_4.is_root_ = true;
      model.rbdyn_tree_.create(l4.name_,l4, true);

      model.rbdyn_tree_.linkNodes();

      std::cout<<"\n Linked nodes in rbdyn tree";

      sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;
      std::vector <std::string> ret_processing_order;

      if (false == calculateOrderOfProcessing( &model , ret_processing_order))
      { throw(std::runtime_error("Failed to calculate tree processing order"));  }
      {
        if( ret_processing_order[0] == std::string("l0") && ret_processing_order[1] == std::string("l1") && ret_processing_order[2] == std::string("l3") && ret_processing_order[3] == std::string("l2"))
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated tree processing order\n\n";
        else
        {
          throw(std::runtime_error("Failed to calculate tree processing order"));
        }
      }

      //Test 10
      if (false == calculateTransformationAndInertia( &model ))
      { throw(std::runtime_error("Failed to Calculate Spatial Inertia & Individual Transformation Matrix within joint"));  }
      {
        sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::iterator it;
        bool flag  = true;
        for(it = model.rbdyn_tree_.begin();it != model.rbdyn_tree_.end() &&  true == flag ; ++it) //return spec data
        {
          if(it++!=model.rbdyn_tree_.end())      //To avoid one extra value
          {
            it--;
            if(it->sp_inertia_  != Eigen::MatrixXd::Identity(6,6))
              flag = false;

            if(it->sp_X_within_link_  != Eigen::MatrixXd::Identity(6,6))
              flag = false;
          }
        }

        if(true == flag)
          std::cout<<"\nTest Result ("<<test_id++<<") Calculated Spatial Inertia & Individual Transformation Matrix within joint \n\n";
        else
        {
          throw(std::runtime_error("Failed to calculate spatial Inertia & Individual Transformation Matrix within joint "));
        }
      }


    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Error ("<<test_id++<<") : "<<ee.what();
      std::cout<<"\nTest #"<<" (Dynamics Scl Math Test) Failed.";
    }
  }
}
