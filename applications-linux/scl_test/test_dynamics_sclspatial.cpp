/*
 * test_DynamcisNarm.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: nayan
 */

#include "test_dynamics_sclspatial.hpp"

#include <scl/DataTypes.hpp>
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>

#include <stdexcept>
#include <iostream>

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

void test_dynamics_sclspatial()
{
	scl::sUInt test_id = 1;
	try
	{

		SGcModel model;
		SRigidBodyDyn l0,l1,l2,l3,l4,l5;
		SRigidBody link_ds_0,link_ds_1,link_ds_2,link_ds_3,link_ds_4,link_ds_5;



		//create l0 node
		l0.name_ = "l0";  l0.parent_name_ = "ground";
		l0.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l0.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_0.name_="l0"; link_ds_0.link_id_=0; l0.link_ds_ = &link_ds_0;
		link_ds_0.joint_type_ =  JOINT_TYPE_REVOLUTE_Z;
		model.rbdyn_tree_.create(l0.name_,l0, false);

		//create l1 node
		l1.name_ = "l1";  l1.parent_name_ = "l0";
		l1.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l1.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_1.name_="l1"; link_ds_1.link_id_=1; l1.link_ds_ = &link_ds_1;
		link_ds_1.joint_type_ =  JOINT_TYPE_REVOLUTE_Z;
		model.rbdyn_tree_.create(l1.name_,l1, false);

		//create l2 node
		l2.name_ = "l2";  l2.parent_name_ = "l1";
		l2.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l2.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_2.name_="l2"; link_ds_2.link_id_=2; l2.link_ds_ = &link_ds_2;
		link_ds_2.joint_type_ =  JOINT_TYPE_REVOLUTE_Z;
		model.rbdyn_tree_.create(l2.name_,l2, false);

		//create l3 node
		l3.name_ = "l3";  l3.parent_name_ = "l0";
		l3.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l3.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_3.name_="l3"; link_ds_3.link_id_=3; l3.link_ds_ = &link_ds_3;
		link_ds_3.joint_type_ =  JOINT_TYPE_REVOLUTE_Z;
		model.rbdyn_tree_.create(l3.name_,l3, false);

		//create ground(root) node
		l4.name_ = "ground";  l4.parent_name_ = "not assigned";
		l4.sp_inertia_=Eigen::MatrixXd::Zero(6,6); l4.sp_X_within_link_=Eigen::MatrixXd::Zero(6,6);
		link_ds_4.name_="ground"; link_ds_4.link_id_=-1; l4.link_ds_ = &link_ds_4;
		link_ds_4.joint_type_ =  JOINT_TYPE_NOTASSIGNED;
		link_ds_4.is_root_ = true;
		model.rbdyn_tree_.create(l4.name_,l4, true);


		model.rbdyn_tree_.linkNodes();

		//initialize sensors data
		scl::SRobotSensors sensors;
		scl::SRobotIO io_data;
		Eigen::VectorXd value = Eigen::VectorXd::Zero(4);
		io_data.sensors_.q_ =  value;
		io_data.sensors_.dq_ =  value;
		io_data.sensors_.ddq_ =  value;
		io_data.sensors_.force_gc_measured_ =  value;
		io_data.actuators_.force_gc_commanded_ = value;
		model.force_gc_cc_ = value;
		model.M_gc_ = Eigen::MatrixXd::Zero(4,4);

		Eigen::VectorXd ret_fgc;
		CDynamicsSclSpatial test;

		//Test 1
		Eigen::VectorXd ret_ddq;
		if (false == test.forwardDynamicsCRBA(&io_data, &model , ret_ddq))
		{ throw(std::runtime_error("Failed to calculate Joint Acceleration using Composite Rigid Body Algorithm "));  }
		{ std::cout<<"\nTest Result ("<<test_id++<<") Calculated Joint Acceleration using Composite Rigid Body Algorithm \n\n";
		std::cout<<"actual :\n"<<io_data.sensors_.ddq_<<"\ncalculated :\n"<<ret_ddq<<"\n";
		}

		//Test 2
		if (false == test.forwardDynamicsABA(&io_data, &model , ret_ddq))
		{ throw(std::runtime_error("Failed to calculate Joint Acceleration using Articulated Body Algorithm "));  }
		{ std::cout<<"\nTest Result ("<<test_id++<<") Calculated jJoint Acceleration using Articulated Rigid Body Algorithm \n\n";
		std::cout<<"actual value :\n"<<io_data.sensors_.ddq_<<"\ncalculated value :\n"<<ret_ddq<<"\n";
		}

		//Test 3
		if (false == test.inverseDynamicsNER(&io_data, &model , ret_fgc))
		{ throw(std::runtime_error("Failed to calculate joint Torque using Newton Euler Recursive Algorithm "));  }
		{ std::cout<<"\nTest Result ("<<test_id++<<") Calculated joint Torque using Newton Euler Recursive Algorithm \n\n";
		std::cout<<"actual :\n"<<io_data.sensors_.force_gc_measured_<<"\ncalculated :\n"<<ret_fgc<<"\n"; }
	}
	catch (std::exception& ee)
	{
		std::cout<<"\nTest Error ("<<test_id++<<") : "<<ee.what();
		std::cout<<"\nTest #"<<" (Dynamics Scl Math Test) Failed.";
	}

}

}
