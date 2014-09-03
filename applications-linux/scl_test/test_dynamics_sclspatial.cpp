/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/*
 * test_dynamics_sclspatial.cpp
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

#include "test_dynamics_sclspatial.hpp"

#include <scl/DataTypes.hpp>
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>

#include <stdexcept>
#include <iostream>
#include <stdlib.h>

using namespace scl;
using namespace scl_ext;

namespace scl_test
{
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

void test_dynamics_sclspatial(int id)
{
	scl::sUInt test_id = 1;
	try
	{
		SGcModel model;
		SRobotParsed rds;
		SRigidBodyDyn l0,l1,l2,l3,l4,l5;
		SRigidBody link_ds_0,link_ds_1,link_ds_2,link_ds_3,link_ds_4,link_ds_5;

		//create l0 node
		l0.name_ = "l0";  l0.parent_name_ = "ground";
		l0.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l0.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_0.name_="l0"; link_ds_0.link_id_=0; l0.link_ds_ = &link_ds_0;
		link_ds_0.joint_type_ =  JOINT_TYPE_PRISMATIC_Z;
		model.rbdyn_tree_.create(l0.name_,l0, false);
		rds.rb_tree_.create(l0.name_,link_ds_0, false);

		//create l1 node
		l1.name_ = "l1";  l1.parent_name_ = "l0";
		l1.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l1.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_1.name_="l1"; link_ds_1.link_id_=1; l1.link_ds_ = &link_ds_1;
		link_ds_1.joint_type_ =  JOINT_TYPE_PRISMATIC_X;
		model.rbdyn_tree_.create(l1.name_,l1, false);
		rds.rb_tree_.create(l1.name_,link_ds_1, false);

		//create l2 node
		l2.name_ = "l2";  l2.parent_name_ = "l1";
		l2.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l2.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_2.name_="l2"; link_ds_2.link_id_=2; l2.link_ds_ = &link_ds_2;
		link_ds_2.joint_type_ =  JOINT_TYPE_REVOLUTE_Z;
		model.rbdyn_tree_.create(l2.name_,l2, false);
		rds.rb_tree_.create(l2.name_,link_ds_2, false);

		//create l3 node
		l3.name_ = "l3";  l3.parent_name_ = "l0";
		l3.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l3.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_3.name_="l3"; link_ds_3.link_id_=3; l3.link_ds_ = &link_ds_3;
		link_ds_3.joint_type_ =  JOINT_TYPE_REVOLUTE_Z;
		model.rbdyn_tree_.create(l3.name_,l3, false);
		rds.rb_tree_.create(l3.name_,link_ds_3, false);

		//create ground(root) node
		l4.name_ = "ground";  l4.parent_name_ = "not assigned";
		l4.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l4.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_4.name_="ground"; link_ds_4.link_id_=-1; l4.link_ds_ = &link_ds_4;
		link_ds_4.joint_type_ =  JOINT_TYPE_NOTASSIGNED;
		link_ds_4.is_root_ = true;
		model.rbdyn_tree_.create(l4.name_,l4, true);
		rds.rb_tree_.create(l4.name_,link_ds_4, true);

		model.rbdyn_tree_.linkNodes();
		model.computed_spatial_transformation_and_inertia_ = false;

		rds.rb_tree_.linkNodes();
		//rds.gravity_ <<0,0,-9.81;
		rds.gravity_ <<0,0,0;
		rds.has_been_init_ = true;

		//initialize sensors data
		scl::SRobotSensors sensors;
		scl::SRobotIO io_data;
		Eigen::VectorXd value;
		io_data.sensors_.ddq_.setZero(4);

		//Initialize the dynamics object
		CDynamicsSclSpatial test;
		bool flag = test.init(rds);
		if(false == flag)
		{ throw(std::runtime_error("Failed to initialize the scl spatial object"));  }

		std::cout<<"\n\n***** Testing CRBA, ABA and RNEA for random inputs... *****";
		for(int i=0; i<2; ++i)
		{
		  rds.gravity_<<0,0,0.0;
		  value = Eigen::VectorXd::Random(4);
		  io_data.sensors_.q_ =  value;

      value = Eigen::VectorXd::Random(4);
		  io_data.sensors_.dq_ =  value;

      value = Eigen::VectorXd::Random(4);
		  io_data.sensors_.force_gc_measured_ =  value;

      value = Eigen::VectorXd::Random(4);
		  io_data.actuators_.force_gc_commanded_ = value;

		  model.force_gc_cc_ = Eigen::VectorXd::Zero(4);
		  model.M_gc_ = Eigen::MatrixXd::Zero(4,4);

		  Eigen::VectorXd ret_fgc;

		  //Test 1
		  Eigen::VectorXd ret_ddq;
		  if (false == test.forwardDynamicsCRBA(&io_data, &model , ret_ddq))
		  { throw(std::runtime_error("Failed to calculate Joint Acceleration using Composite Rigid Body Algorithm "));  }
		  std::cout<<"\n\tSub-Test ("<<i<<") Calculated Joint Acceleration using Composite Rigid Body Algorithm...";
		  std::cout<<"\n ddq :\n"<<ret_ddq.transpose()<<"\n";

		  //Test 2
		  if (false == test.forwardDynamicsABA(&io_data, &model , ret_ddq))
		  { throw(std::runtime_error("Failed to calculate Joint Acceleration using Articulated Body Algorithm "));  }
		  std::cout<<"\n\tSub-Test ("<<i<<") Calculated Joint Acceleration using Articulated Rigid Body Algorithm...";
		  std::cout<<"\n ddq :\n"<<ret_ddq.transpose()<<"\n";

		  io_data.sensors_.ddq_ = ret_ddq;

		  //Test 3 (without grav)
		  if (false == test.inverseDynamicsNER(&io_data, &model , ret_fgc))
		  { throw(std::runtime_error("Failed to calculate joint Torque using Newton Euler Recursive Algorithm "));  }
		  std::cout<<"\n\tSub-Test ("<<i<<") Calculated Joint Torque using Newton Euler Recursive Algorithm...";
		  std::cout<<"\n Fgc commanded (using ABA to get ddq):\n"<<io_data.actuators_.force_gc_commanded_.transpose()<<"\n";
		  std::cout<<"\n Fgc estimated (using NER(ddq) to get Fgc):\n"<<ret_fgc.transpose()<<"\n";

		  //Test 3 (with grav)
		  rds.gravity_<<0,0,9.81;
		  if (false == test.inverseDynamicsNER(&io_data, &model , ret_fgc))
		  { throw(std::runtime_error("Random crap error"));  }
		  std::cout<<"\n Fgc estimated (with grav):\n"<<ret_fgc.transpose()<<"\n";

		  //Test 3 (with grav)
		  rds.gravity_<<10,10,9.81;
		  if (false == test.inverseDynamicsNER(&io_data, &model , ret_fgc))
		  { throw(std::runtime_error("Random crap error"));  }
		  std::cout<<"\n Fgc estimated (with more grav):\n"<<ret_fgc.transpose()<<"\n";
		}

		std::cout<<"\nTest Result ("<<test_id++<<") Tested CRBA, ABA and RNEA for random inputs...";
    std::cout<<"\nTest #"<<id<<" : Succeeded.";
	}
	catch (std::exception& ee)
	{
		std::cout<<"\nTest Error ("<<test_id++<<") : "<<ee.what();
		std::cout<<"\nTest #"<<" (Dynamics Scl Math Test) Failed.";
	}

}

}
