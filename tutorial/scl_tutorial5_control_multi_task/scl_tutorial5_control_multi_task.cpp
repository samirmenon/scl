/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

This file is released under the MIT license.
See COPYING.MIT in the scl base directory.
*/
/* \file scl_tutorial5_multi_task.cpp
 *
 *  Created on: Aug 10, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/scl.hpp>
#include <scl_ext/scl_ext.hpp>

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
 * Moving forward from tutorial 4, we will now control the 6 DOF
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
  std::cout<<"Standard Control Library Tutorial #5";
  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SGcModel rgcm_dyn;    //Robot data structure with dynamic quantities (for the integrator)...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl_ext::CDynamicsSclSpatial dyn_scl_sp; //Robot physics integrator
  scl::CParserScl p;         //This time, we'll parse the tree from a file...

  scl::SControllerMultiTask rctr_ds; //A multi-task controller data structure
  scl::CControllerMultiTask rctr;    //A multi-task controller
  std::vector<scl::STaskBase*> rtasks;              //A set of executable tasks
  std::vector<scl::SNonControlTaskBase*> rtasks_nc; //A set of non-control tasks
  std::vector<scl::sString2> ctrl_params;        //Used to parse extra xml tags
  scl::STaskOpPos* rtask_hand;       //Will need to set hand desired positions etc.

  sutil::CSystemClock::start(); //Start the clock

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  bool flag = p.readRobotFromFile("./R6Cfg.xml","./","r6bot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && rgcm_dyn.init(rds);        //Simple way to set up dynamic tree (for the integ)...
  flag = flag && dyn_scl_sp.init(rds);      //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds);             //Set up the I/O data structure
  if(false == flag){ return 1; }            //Error check.

  /******************************Set up Controller Specification************************************/
  // Read xml file info into task specifications.
  flag = p.readTaskControllerFromFile("./R6Cfg.xml","opc",rtasks,rtasks_nc,ctrl_params);
  flag = flag && rctr_ds.init("opc",&rds,&rio,&rgcm); //Set up the control data structure..
  // Tasks are initialized after find their type with dynamic typing.
  flag = flag && scl::init::registerNativeDynamicTypes();
  flag = flag && scl::init::initMultiTaskCtrlDsFromParsedTasks(rtasks,rtasks_nc,rctr_ds);
  flag = flag && rctr.init(&rctr_ds,&dyn_scl);        //Set up the controller (needs parsed data and a dyn object)
  if(false == flag){ return 1; }            //Error check.

  rtask_hand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand")) );
  if(NULL == rtask_hand)  {return 1;}       //Error check

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
  long long iter = 0; double dt=0.0001;

  // Initialize the robot to a reasonable configuration
  rio.sensors_.q_<< 0.3, -0.3, 0.3, -0.3, -0.3, 0.3;

  // Set up the threading
  omp_set_num_threads(2);
  int thread_id; double tstart, tcurr; flag = false;

#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
    {
      // Controller : Operational space controller
      std::cout<<"\n\n***************************************************************"
          <<"\n Starting op space (task coordinate) controller..."
          <<"\n This will move the hand in a circle. x =sin(t), y=cos(t)."
          <<"\n***************************************************************";
      tstart = sutil::CSystemClock::getSysTime(); iter = 0;
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        tcurr = sutil::CSystemClock::getSysTime();

        // Move the hand in a sine wave
        rtask_hand->x_goal_(0) = 0.15*sin(tcurr-tstart)+0.6; rtask_hand->x_goal_(1) = 0.15*cos(tcurr-tstart);

        // Compute control forces (note that these directly have access to the io data ds).
        rctr.computeDynamics();
        rctr.computeControlForces();

        // Integrate the dynamics
        dyn_scl_sp.integrate(rgcm_dyn,rio,dt); iter++;

        if(iter % 5000 == 0){std::cout<<"\nTracking error: "<<(rtask_hand->x_goal_-rtask_hand->x_).transpose()
        <<". Norm: "<<(rtask_hand->x_goal_-rtask_hand->x_).norm(); }
      }
      //Then terminate
      scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
    }
    else  //Read the rio data structure and updated rendererd robot..
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\n\tSystem time = "<<sutil::CSystemClock::getSysTime()-tstart;
  std::cout<<"\n\tSimulated time = "<<static_cast<double>(iter)*dt;
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
