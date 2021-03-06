/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

This file is released under the MIT license.
See COPYING.MIT in the scl base directory.
*/
/* \file scl_tutorial3_graphics_physics_cmake.cpp
 *
 *  Created on: Jul 30, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/scl.hpp>
#include <scl_ext/scl_ext.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

/** A sample application to demonstrate a physics simulation in scl.
 *
 * Moving forward from tutorial 2, we will now use a 4 DOF
 * demo robot (rrrrbot) with the physics engine and display it
 * in an OpenGL graphics window. We will integrate
 * the robot's dynamics as it swings around under the effect of
 * gravity.
 *
 * This simulation will also show you how to use a simplistic multi-threaded
 * setup for doing the graphics and physics. From this point on, the
 * actual code might start becoming computationally expensive. So we
 * will typically use multi-threading.
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (physics)
 * 3. graphics (chai)
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Tutorial #3";
  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl_ext::CDynamicsSclSpatial dyn_scl_sp; //Robot physics integrator...
  scl::CParserScl p;         //This time, we'll parse the tree from a file...

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  bool flag = p.readRobotFromFile("./RRRRCfg.xml","./","rrrrbot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_scl_sp.init(rds);         //Set up integrator object
  flag = flag && rio.init(rds);
  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile("./RRRRCfg.xml","rrrrgraphics",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the rrrbot's physics. Press (x) to exit.";
  long iter = 0, n_iters=100000; double dt=0.0001;

  // Set the io data initial config to something interesting
  rio.sensors_.q_(0) = 0.5;
  omp_set_num_threads(2);
  int thread_id;
#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
      while(iter < n_iters && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { dyn_scl_sp.integrate(rgcm,rio,dt); iter++; const timespec ts = {0, 5000};/*.05ms*/ nanosleep(&ts,NULL); }
    else  //Read the rio data structure and updated rendererd robot..
      while(iter < n_iters && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
