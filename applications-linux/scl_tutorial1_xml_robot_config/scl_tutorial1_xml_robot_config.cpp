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
/* \file scl_tutorial1_xml_robot_config.cpp
 *
 *  Created on: Jul 30, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/util/DatabaseUtils.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes
#include <iostream>

/** A sample application to demonstrate specifying a robot in scl.
 *
 * Moving forward from the first tutorial, we will now specify a
 * robot in an xml config file! This makes the code more compact.
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (kinematics, inertia etc. computations)
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Tutorial #1";
  std::cout<<"\n***************************************\n\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl::CParserScl p;         //This time, we'll parse the tree from a file...

  bool flag = p.readRobotFromFile("./RRRCfg.xml","rrrbot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_scl.init(rds);         //Set up dynamics object
  flag = flag && rio.init(rds.name_,rds.dof_);
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }

  if(false == flag){ return 1; }            //Error check.
  std::cout<<"\nRobot generalized coordinates (q) = "<<rio.sensors_.q_.transpose();
  scl_util::printRobotLinkTree(*(rds.rb_tree_.getRootNode()),0);
  std::cout<<"\n\n **** Progress : Initialized static and dynamic robot trees, and dynamics object.";

  // Compute dynamics.
  dyn_scl.computeGCModel(&rio.sensors_,&rgcm);

  // Structured way to compute kinematic and dynamic quantities...
  std::cout<<"\n\nRobot's KE = "<<dyn_scl.computeEnergyKinetic(rgcm.rbdyn_tree_,rio.sensors_.q_,rio.sensors_.dq_);
  std::cout<<"\nRobot's PE = "<<dyn_scl.computeEnergyPotential(rgcm.rbdyn_tree_,rio.sensors_.q_);

  sutil::CMappedTree<std::string, scl::SRigidBodyDyn>::const_iterator it,ite;
  for(it = rgcm.rbdyn_tree_.begin(), ite = rgcm.rbdyn_tree_.end(); it!=ite; ++it)
  { std::cout<<"\n\nLink:"<<it->name_<<"\npar_T_link\n"<<(it->T_lnk_.matrix())<<"\nJcom\n"<<(it->J_com_);  }

  std::cout<<"\n\nInertia Matrix:\n"<<rgcm.M_gc_;
  std::cout<<"\n\nInv Inertia Matrix:\n"<<rgcm.M_gc_inv_;
  std::cout<<"\n\nGeneralized Gravity Force: "<<rgcm.force_gc_grav_.transpose();

  // Since we're at it, let's also compute a Jacobian
  Eigen::MatrixXd Jhand; Jhand.setZero(6,3); Eigen::Vector3d hpos(0,0,-0.2);
  dyn_scl.computeJacobian(Jhand,*rgcm.rbdyn_tree_.at("link_2"),rio.sensors_.q_,hpos);
  std::cout<<"\n\nHand Jacobian:\n"<<Jhand;

  std::cout<<"\n\n **** Progress : Computed robot's energy, kinematics, and dynamics.";

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
