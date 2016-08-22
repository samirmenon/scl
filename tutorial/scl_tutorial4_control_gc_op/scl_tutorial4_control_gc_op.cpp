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
/* \file scl_tutorial4_control_gc_op.cpp
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
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

/** A sample application to demonstrate a physics simulation in scl.
 *
 * Moving forward from tutorial 3, we will now control the 6 DOF
 * demo robot (r6bot) with the physics engine running.
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (physics)
 * 4. dynamics (control matrices)
 * 4. graphics (chai)
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Tutorial #4";
  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities (for the control)...
  scl::SGcModel rgcm_dyn;    //Robot data structure with dynamic quantities (for the physics)...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl_ext::CDynamicsSclSpatial dyn_scl_sp; //Robot physics integrator
  scl::CParserScl p;         //This time, we'll parse the tree from a file...
  sutil::CSystemClock::start();

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  bool flag = p.readRobotFromFile("./R6Cfg.xml","./","r6bot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && rgcm_dyn.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_scl_sp.init(rds);      //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds);             //Set up the I/O data structure
  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile("./R6Cfg.xml","r6graphics",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the r6bot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.";
  long iter = 0, n_iters=100000; double dt=0.0001;

  omp_set_num_threads(2);
  int thread_id; double tstart, tcurr; flag = false;
  const Eigen::Vector3d hpos(0,0,-0.4); //control position of op-point wrt. hand
  Eigen::MatrixXd Jx;
  Eigen::Vector3d x, x_des, x_init, dx;
  scl::SRigidBodyDyn *rhand = rgcm.rbdyn_tree_.at("hand");

#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
    {
      // Controller 1 : gc controller
      std::cout<<"\n\n***************************************************************"
          <<"\n Starting joint space (generalized coordinate) controller..."
          <<"\n This will move the joints in a sine wave"
          <<"\n***************************************************************";
      tstart = sutil::CSystemClock::getSysTime();
      while(iter < n_iters && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        tcurr = sutil::CSystemClock::getSysTime();
        // Controller : fgc = kp * (sin(t) - q) + kv(0 - dq)
        // Set the desired positions so that the joints move in a sine wave
        rio.actuators_.force_gc_commanded_ = 100 * (sin(tcurr-tstart) - rio.sensors_.q_.array()) - 20 * rio.sensors_.dq_.array();
        dyn_scl_sp.integrate(rgcm_dyn,rio,dt);
        iter++; const timespec ts = {0, 5000};/*.05ms*/ nanosleep(&ts,NULL);

        if(iter % n_iters/10 == 0){std::cout<<"\nTracking error: "<<(sin(tcurr-tstart) - rio.sensors_.q_.array()).transpose(); }
      }
      sleep(1);

      // Controller 2 : Operational space controller
      std::cout<<"\n\n***************************************************************"
          <<"\n Starting op space (task coordinate) controller..."
          <<"\n This will move the hand in a circle. x =sin(t), y=cos(t)."
          <<"\n***************************************************************";
      tstart = sutil::CSystemClock::getSysTime(); iter = 0;
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        tcurr = sutil::CSystemClock::getSysTime();

        // Compute kinematic quantities
        dyn_scl.computeTransformsForAllLinks(rgcm.rbdyn_tree_,rio.sensors_.q_);
        dyn_scl.computeJacobianWithTransforms(Jx,*rhand,rio.sensors_.q_,hpos);
        if(false == flag) { x_init = rhand->T_o_lnk_ * hpos; flag = true; }
        x = rhand->T_o_lnk_ * hpos; //We'll make the hand draw a sine trajectory
        dx = Jx.block(0,0,3,rio.dof_) * rio.sensors_.dq_;

        // Controller : fgc = Jx' (kp * (sin(t)*.1 - (x-xinit)) + kv(0 - dx)) - kqv * dq
        // Set the desired positions so that the hand draws a circle
        double sin_ampl = 0.15;
        x_des(0) = x_init(0)+sin(tcurr-tstart)*sin_ampl; x_des(1) = x_init(1)+cos(tcurr-tstart)*sin_ampl; x_des(2) = 0;
        rio.actuators_.force_gc_commanded_ = Jx.block(0,0,3,rio.dof_).transpose() * (100*(x_des-x) - 20 * dx) - 20*rio.sensors_.dq_;

        // Integrate the dynamics
        dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, 5000};/*.05ms*/ nanosleep(&ts,NULL);

        if(iter % static_cast<long>(n_iters/10) == 0)
        { std::cout<<"\nTracking error: "<<(x_des-x).transpose()<<". Norm: "<<(x_des-x).norm(); }
      }
      //Then terminate
      scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
    }
    else  //Read the rio data structure and updated rendererd robot..
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
