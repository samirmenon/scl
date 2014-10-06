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
/* \file scl_tutorial2_physics_integration.cpp
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
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/util/DatabaseUtils.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes
#include <iostream>

/** A sample application to demonstrate specifying a robot in scl.
 *
 * Moving forward from the second tutorial, we will now use our
 * demo robot (rrrbot) with the physics engine. We will integrate
 * the robot's dynamics as it swings around under the effect of
 * gravity. The robot's energy should be conserved.
 *
 * NOTE : Run this multiple times to convince yourself that the
 *        robot's physics integration is deterministic.
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (kinematics, inertia etc. computations)
 * 3. dynamics (physics)
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Tutorial #2";
  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl::CDynamicsTao dyn_tao; //Robot physics integrator...
  scl::CParserScl p;         //This time, we'll parse the tree from a file...

  //We will reuse the xml spec file from the first tutorial
  bool flag = p.readRobotFromFile("./RRRCfg.xml","./","rrrbot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_scl.init(rds);         //Set up dynamics object
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && rio.init(rds.name_,rds.dof_);
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Error check.

  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the rrrbot's physics";
  for(long n_iters=10000; n_iters < 500000; n_iters*=2)
  {
    double e_init, e_fin, dt=0.0001;
    e_init = dyn_scl.computeEnergyKinetic(rgcm.rbdyn_tree_,rio.sensors_.q_,rio.sensors_.dq_)+
        dyn_scl.computeEnergyPotential(rgcm.rbdyn_tree_,rio.sensors_.q_);
    for(long ii=0; ii<n_iters; ++ii)  { dyn_tao.integrate(rio,dt);  }
    e_fin = dyn_scl.computeEnergyKinetic(rgcm.rbdyn_tree_,rio.sensors_.q_,rio.sensors_.dq_)+
        dyn_scl.computeEnergyPotential(rgcm.rbdyn_tree_,rio.sensors_.q_);

    std::cout<<"\n Sim time (s) = "<<static_cast<double>(n_iters)*dt;
    std::cout<<". Energy change (% of start) = "<<(e_fin-e_init)/e_init;
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
