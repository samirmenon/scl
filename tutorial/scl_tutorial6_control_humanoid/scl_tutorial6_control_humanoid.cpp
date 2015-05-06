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
/* \file scl_tutorial6_control_humanoid.cpp
 *
 *  Created on: Aug 11, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>
//scl functions to simplify dynamic typing and data sharing
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <scl/util/HelperFunctions.hpp>

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
 * Moving forward from tutorial 5, we will now control a
 * robot to touch a plane and apply a force on it.
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
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl::CDynamicsTao dyn_tao; //Robot physics integrator
  scl::CParserScl p;         //This time, we'll parse the tree from a file...

  scl::SControllerMultiTask rctr_ds; //A multi-task controller data structure
  scl::CControllerMultiTask rctr;    //A multi-task controller
  std::vector<scl::STaskBase*> rtasks;              //A set of executable tasks
  std::vector<scl::SNonControlTaskBase*> rtasks_nc; //A set of non-control tasks
  std::vector<scl::sString2> ctrl_params;        //Used to parse extra xml tags
  scl::STaskOpPos* rtask_hand, *rtask_lhand; //Will need to set hand desired positions etc.

  sutil::CSystemClock::start(); //Start the clock

  /******************************Set up Shared Memory (Database)************************************/
  // This will help scl find the graphics files. Usually they are specified wrt. some
  // specs dir. An alternate way is to set this dir as an environment variable (but we won't
  // do that for now). Instead we'll set the dir in a global shared memory "database".
  if(NULL == scl::CDatabase::getData()) { std::cout<<"\n ERROR : Could not initialize global data storage"; return 1; }
  scl_util::getCurrentDir(scl::CDatabase::getData()->cwd_);
  scl::CDatabase::getData()->dir_specs_ = scl::CDatabase::getData()->cwd_ + std::string("../../specs/");

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  const std::string fname("../../specs/Stanbot/StanbotCfg.xml");
  bool flag = p.readRobotFromFile(fname,"../../specs/","Stanbot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds);             //Set up the I/O data structure
  if(false == flag){ return 1; }            //Error check.

  /******************************Set up Controller Specification************************************/
  // Read xml file info into task specifications.
  flag = p.readTaskControllerFromFile(fname,"opc",rtasks,rtasks_nc,ctrl_params);
  flag = flag && rctr_ds.init("opc",&rds,&rio,&rgcm); //Set up the control data structure..
  // Tasks are initialized after find their type with dynamic typing.
  flag = flag && scl_registry::registerNativeDynamicTypes();
  flag = flag && scl_util::initMultiTaskCtrlDsFromParsedTasks(rtasks,rtasks_nc,rctr_ds);
  flag = flag && rctr.init(&rctr_ds,&dyn_scl);        //Set up the controller (needs parsed data and a dyn object)
  if(false == flag){ return 1; }            //Error check.

  rtask_hand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand")) );
  if(NULL == rtask_hand)  {return 1;}       //Error check

  rtask_lhand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand2")) );
  if(NULL == rtask_lhand)  {return 1;}       //Error check

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile(fname,"StanbotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the robot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.";
  long long iter = 0; double dt=0.001;

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
          <<"\n This will move the humanoid's hands in circles."
          <<"\n NOTE: This controller works with the system clock. So it will"
          <<"\n       behave differently on different computers."
          <<"\n\n Press '1' to flip control of the left hand \nto either the {sw,da,eq keys} or {a default sine wave}"
          <<"\n***************************************************************";
      tstart = sutil::CSystemClock::getSysTime(); iter = 0;
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        tcurr = sutil::CSystemClock::getSysTime();

        // Move the right hand in a sine wave
        rtask_hand->x_goal_(0) = 0.15*sin(tcurr-tstart)-0.15;
        rtask_hand->x_goal_(1) = 0.25*cos(tcurr-tstart);
        rtask_hand->x_goal_(2) = -0.25;

        // Move the left hand in a different sine wave
        if(!scl::CDatabase::getData()->s_gui_.ui_flag_[1]){
          rtask_lhand->x_goal_(0) = 0.15*sin((tcurr-tstart)*4.0)-0.05;
          rtask_lhand->x_goal_(1) = 0.15*cos((tcurr-tstart)*4.0)+0.45;
          rtask_lhand->x_goal_(2) = -0.1;
        }
        else
        { rtask_lhand->x_goal_ = scl::CDatabase::getData()->s_gui_.ui_point_[0]; }

        // Compute control forces (note that these directly have access to the io data ds).
        rctr.computeDynamics();
        rctr.computeControlForces();

        // Integrate the dynamics
        dyn_tao.integrate(rio,dt); iter++;

        /** Slow down sim to real time */
        sutil::CSystemClock::tick(dt);
        double tcurr = sutil::CSystemClock::getSysTime();
        double tdiff = sutil::CSystemClock::getSimTime() - tcurr;
        timespec ts = {0, 0};
        if(tdiff > 0)
        {
          ts.tv_sec = static_cast<int>(tdiff);
          tdiff -= static_cast<int>(tdiff);
          ts.tv_nsec = tdiff*1e9;
          nanosleep(&ts,NULL);
        }
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
