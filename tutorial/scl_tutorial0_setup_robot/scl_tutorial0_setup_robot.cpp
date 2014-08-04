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
/* \file scl_tutorial0_setup_robot.cpp
 *
 *  Created on: Jul 22, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/util/DatabaseUtils.hpp>

//Eigen 3rd party lib for math computations
#include <Eigen/Dense>

//Standard includes
#include <iostream>

/** A sample application to demonstrate specifying a robot in scl.
 *
 * This application is conceptually simple and explicitly specifies
 * a robot, computes its dynamics and some control quantities.
 *
 * However, it lacks a GUI, rigorous error checks, and has too many
 * hard-coded quantities. Future tutorials will cover those issues.
 *
 * NOTE : This application requires MUCH more code than is necessary.
 *        Proceed to tutorial 1 to see how compact this can be in scl.
 *        (The code will shrink from ~120 lines to about ~40 lines).
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (kinematics etc. computations)
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Tutorial #0";
  std::cout<<"\n***************************************\n\n";
  // Create a robot. This requires static (general info) and dynamic (dynamics info) trees
  // The trees are additionally indexed by some number (or string)
  const std::string rname("rrrbot"); //Revolute-revolute-revolute robot...
  bool flag;

  // First we need to specify all the robot's properties. We will do this in a robot data structure.
  scl::SRobotParsed rds; //Robot data structure....

  /******************************Static Tree************************************/
  // First we need some place to store generalized coordinates and velocities (sensor data)
  Eigen::VectorXd q, dq; q.setZero(3); dq.setZero(3);
  scl::SRigidBody rb;    //Tmp node. Inits rds. Many entries; but we won't set them all for now.

  //Root link
  rb.init();          //This resets the rigid body
  rb.link_id_ = -1;  rb.name_ = "root"; //(root starts at -1)
  rb.robot_name_ = rname;  rb.parent_name_ = "none";
  rb.joint_name_ = "none"; rb.joint_type_ = scl::JOINT_TYPE_NOTASSIGNED;
  rb.pos_in_parent_<<0,0,0;  rb.is_root_ = true;
  rds.rb_tree_.create(rb.name_,rb,true);

  // 1st link
  rb.init(); rb.link_id_ = 0; rb.name_ = "link_0";
  rb.robot_name_ = rname;  rb.parent_name_ = "root";
  rb.joint_name_ = "joint_0"; rb.joint_type_ = scl::JOINT_TYPE_REVOLUTE_Y;
  rb.com_<<0,0,-0.1; rb.pos_in_parent_<<0,0,0;
  rb.mass_ = 1.0;rb.inertia_<<1,0,0,0,1,0,0,0,1;
  rds.rb_tree_.create(rb.name_,rb,false);
  q(0) = 1;

  // 2st link
  rb.init(); rb.link_id_ = 1; rb.name_ = "link_1";
  rb.robot_name_ = rname;  rb.parent_name_ = "link_0";
  rb.joint_name_ = "joint_1"; rb.joint_type_ = scl::JOINT_TYPE_REVOLUTE_Y;
  rb.com_<<0,0,-0.05; rb.pos_in_parent_<<0,0,-0.2;
  rb.mass_ = 1.0;rb.inertia_<<1,0,0,0,1,0,0,0,1;
  rds.rb_tree_.create(rb.name_,rb,false);
  q(1) = 0.0;

  // 3rd link
  rb.init(); rb.link_id_ = 2; rb.name_ = "link_2";
  rb.robot_name_ = rname;  rb.parent_name_ = "link_1";
  rb.joint_name_ = "joint_2"; rb.joint_type_ = scl::JOINT_TYPE_REVOLUTE_X;
  rb.com_<<0,0,-0.1; rb.pos_in_parent_<<0,0,-0.1;
  rb.mass_ = 1.0;rb.inertia_<<1,0,0,0,1,0,0,0,1;
  rds.rb_tree_.create(rb.name_,rb,false);
  q(2) = -1.57;

  if(false == rds.rb_tree_.linkNodes())
  { std::cout<<"\nError. Could not initialize robot tree.\n"; return 1; }

  //Now set up the initial state and sort node ordering so indexing works properly
  std::cout<<"\nRobot initial node order: 0,"<<rds.rb_tree_.at(0)->name_
      <<"; 1,"<<rds.rb_tree_.at(1)->name_<<"; 2,"<<rds.rb_tree_.at(2)->name_;
  std::vector<std::string> ss;
  ss.push_back("link_0");ss.push_back("link_1");ss.push_back("link_2");ss.push_back("root");
  rds.rb_tree_.sort(ss);
  std::cout<<"\nRobot sorted node order: 0,"<<rds.rb_tree_.at(0)->name_<<"; 1,"
      <<rds.rb_tree_.at(1)->name_<<"; 2,"<<rds.rb_tree_.at(2)->name_<<"\n\n";

  //We now consider the robot data structure to be initialized.
  rds.has_been_init_ = true;

  std::cout<<"\nPrinting parsed robot "<<rname;
  scl_util::printRobotLinkTree(*(rds.rb_tree_.getRootNode()),0);

  std::cout<<"\n\n **** Progress : Initialized robot data structure.";

  /******************************Dynamic Tree************************************/
  sutil::CMappedTree<std::string, scl::SRigidBodyDyn> rbd_tree;
  scl::SRigidBodyDyn rbd;

  // Following our earlier (static tree example). Also need to connect the static tree.
  rbd.name_ = "root"; rbd.link_ds_ = rds.rb_tree_.at(rbd.name_);
  rbd.parent_name_ = rbd.link_ds_->parent_name_; rbd_tree.create(rbd.name_,rbd,true);

  rbd.name_ = "link_0"; rbd.link_ds_ = rds.rb_tree_.at(rbd.name_);
  rbd.parent_name_ = rbd.link_ds_->parent_name_; rbd_tree.create(rbd.name_,rbd,false);

  rbd.name_ = "link_1"; rbd.link_ds_ = rds.rb_tree_.at(rbd.name_);
  rbd.parent_name_ = rbd.link_ds_->parent_name_; rbd_tree.create(rbd.name_,rbd,false);

  rbd.name_ = "link_2"; rbd.link_ds_ = rds.rb_tree_.at(rbd.name_);
  rbd.parent_name_ = rbd.link_ds_->parent_name_; rbd_tree.create(rbd.name_,rbd,false);

  if(false == rbd_tree.linkNodes())
  { std::cout<<"\nError. Could not initialize dynamic robot tree.\n"; return 1; }

  rbd_tree.sort(ss);
  std::cout<<"\nRobot dyn sorted node order: 0,"<<rbd_tree.at(0)->name_<<"; 1,"
        <<rbd_tree.at(1)->name_<<"; 2,"<<rbd_tree.at(2)->name_;

  std::cout<<"\n\n **** Progress : Initialized robot dynamics tree for future dynamics computations.";

  /** Question : Why do we use two trees?
   *  Answer   : For a robot, there is static and dynamic data. For instance, the link lengths don't change
   *  very often but the joint angles do. Separating both data types reduces the likelihood of errors
   *  in some dynamics code affecting the static structure. Dynamics data, however, always has access to
   *  a read-only copy of the static data.
   */

  /******************************SclDynamics************************************/
  scl::CDynamicsScl dyn_scl;
  flag = dyn_scl.init(rds);
  if(false == flag) { std::cout<<"\nERROR : Could not initialize control & dynamics engine\n\n"; return 1;  }
  else  { std::cout<<"\n\n **** Progress : Initialized control & dynamics engine."; }

  // Let's compute some kinematic quantities with the dynamics object (depend on q,dq only, not on inertia etc.)
  dyn_scl.computeTransformsForAllLinks(rbd_tree, q);
  dyn_scl.computeJacobianComForAllLinks(rbd_tree, q);

  // Let's print this stuff
  sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::const_iterator it,ite;
  for(it = rbd_tree.begin(), ite = rbd_tree.end(); it!=ite; ++it)
  { std::cout<<"\n\nLink:"<<it->name_<<"\npar_T_link\n"<<(it->T_lnk_.matrix())<<"\nJcom\n"<<(it->J_com_);  }

  std::cout<<"\n\n **** Progress : Computed robot kinematics.";

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
