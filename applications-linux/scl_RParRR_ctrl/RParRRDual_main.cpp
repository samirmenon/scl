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
/* \file gc_main.cpp
 *
 *  Created on: Apr 22, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <sutil/CSystemClock.hpp>

#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>

#include <omp.h>
#include <GL/freeglut.h>
#include <Eigen/Dense>

#include <iostream>
#include <stdexcept>
#include <cassert>
#include <sstream>

/**
 * A sample application to compute the dynamics of an RParRRDual robot
 *
 * Use it as a template to write your own (more detailed/beautiful/functional)
 * application.
 */
int main(int argc, char** argv)
{
  bool flag;
  std::cout<<"\nThe scl-RParRRDual application demonstrates how to compute dynamics for a closed chain ."
      <<"\nNote that this application will use the RParRRDual as an example.\n";
  try
  {
    /******************************Initialization************************************/
    //1. Initialize the database and clock.
    if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }
    scl::SDatabase* db = scl::CDatabase::getData(); //Sanity Check
    if(S_NULL==db) { throw(std::runtime_error("Database not initialized"));  }
    db->dir_specs_ = db->cwd_ + std::string("../../specs/"); //Set the specs dir so scl knows where the graphics are.
    std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

    /******************************Data Structures etc.************************************/
    scl::SRobotParsed rds;     //Robot data structure....
    scl::SGraphicsParsed rgr;  //Robot graphics data structure...
    scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
    scl::SRobotIO rio;         //I/O data structure
    scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
    scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
    scl::SGcModel rgcm_integ;  //Robot data structure with dynamic quantities...
    scl_ext::CDynamicsSclSpatial dyn_scl_sp; //Robot physics integrator
    scl::CParserScl p;         //This time, we'll parse the tree from a file...
    const std::string fname("../../specs/RParRRDual/RParRRDualCfg.xml"), robot_name("RParRRDualBot");

    scl::SControllerMultiTask rctr_ds; //A multi-task controller data structure
    scl::CControllerMultiTask rctr;    //A multi-task controller
    std::vector<scl::STaskBase*> rtasks;              //A set of executable tasks
    std::vector<scl::SNonControlTaskBase*> rtasks_nc; //A set of non-control tasks
    std::vector<scl::sString2> ctrl_params;        //Used to parse extra xml tags
    scl::STaskOpPos* rtask_lhand, *rtask_rhand; //Will need to set hand desired positions etc.

    /******************************File Parsing************************************/
    bool flag = p.readRobotFromFile(fname,"../../specs/",robot_name,rds);
    flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree for control...
    flag = flag && rgcm_integ.init(rds);      //Simple way to set up dynamic tree for physics...
    flag = flag && dyn_scl_sp.init(rds);      //Set up integrator object
    flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
    flag = flag && rio.init(rds);             //Set up the I/O data structure
    if(false == flag){ return 1; }            //Error check.
    std::cout<<"\n Parsed file and initialized static data structures."<<std::endl;

    /******************************Set up Controller Specification************************************/
    // Read xml file info into task specifications.
    flag = p.readTaskControllerFromFile(fname,"opc",rtasks,rtasks_nc,ctrl_params);
    flag = flag && rctr_ds.init("opc",&rds,&rio,&rgcm); //Set up the control data structure..
    // Tasks are initialized after find their type with dynamic typing.
    flag = flag && scl_registry::registerNativeDynamicTypes();
    flag = flag && scl_util::initMultiTaskCtrlDsFromParsedTasks(rtasks,rtasks_nc,rctr_ds);
    flag = flag && rctr.init(&rctr_ds,&dyn_scl);  //Set up the controller (needs parsed data and a dyn object)
    if(false == flag){ return 1; }                //Error check.

    //Compute dynamics and control forces to make sure everything is ready.
    rctr.computeDynamics();
    rctr.computeControlForces();

    rtask_lhand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand")) );
    if(NULL == rtask_lhand)  {return 1;}       //Error check

    rtask_rhand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand2")) );
    if(NULL == rtask_rhand)  {return 1;}       //Error check

    std::cout<<"\n Set up controller and initialized task data structures."<<std::endl;

    /******************************ChaiGlut Graphics************************************/
    glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).
    flag = p.readGraphicsFromFile(fname,"RParRRDualBotStdView",rgr);
    flag = flag && rchai.initGraphics(&rgr);
    flag = flag && rchai.addRobotToRender(&rds,&rio);
    flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);

    // Set balls at op point(s)
    chai3d::cGenericObject *rtask_lhand_gr, *rtask_lhand_des_gr;
    flag = flag && rchai.addSphereToRender("RParRRDualBot", "link4", Eigen::Vector3d(1.5,0,0), 0.0275, &rtask_lhand_gr);
    flag = flag && rchai.addSphereToRender(Eigen::Vector3d(0,0,0), rtask_lhand_des_gr, 0.03);
    if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }
    // Color the balls...
    chai3d::cMaterial mat;
    mat.setRedLightSalmon(); rtask_lhand_des_gr->setMaterial(mat,true);
    mat.setWhiteIvory(); rtask_lhand_gr->setMaterial(mat,true);

    // Set balls at op point(s)
    chai3d::cGenericObject *rtask_rhand_gr, *rtask_rhand_des_gr;
    flag = flag && rchai.addSphereToRender("RParRRDualBot", "link9", Eigen::Vector3d(1.5,0,0), 0.0275, &rtask_rhand_gr);
    flag = flag && rchai.addSphereToRender(Eigen::Vector3d(0,0,0), rtask_rhand_des_gr, 0.03);
    if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }
    // Color the balls...
    mat.setRedLightSalmon(); rtask_rhand_des_gr->setMaterial(mat,true);
    mat.setWhiteIvory(); rtask_rhand_gr->setMaterial(mat,true);

    std::cout<<"\n Set up graphics and initialized task marker balls."<<std::endl;

    /******************************Set Control Point Initial Position************************************/
    rtask_lhand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand")) );
    if(NULL == rtask_lhand)  {return 1;}       //Error check
    rtask_rhand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand2")) );
    if(NULL == rtask_rhand)  {return 1;}       //Error check
    // Once the controller has been initialized, set the goal position to the task position. (To keep things stable).
    db->s_gui_.ui_point_[0] = rtask_lhand->x_;
    db->s_gui_.ui_point_[1] = rtask_rhand->x_;

    std::cout<<"\n Set up initial ui control point positions."<<std::endl;

    /******************************Main Loop************************************/
    std::cout<<"\nStarting simulation. Timestep : "<<db->sim_dt_<<std::flush;

    scl::sLongLong ctrl_ctr=0;//Controller computation counter
    scl::sLongLong gr_ctr=0;//Controller computation counter
    scl::sFloat tstart, tcurr;

    omp_set_num_threads(2);
    int thread_id;
    tstart = sutil::CSystemClock::getSysTime();
    db->pause_ctrl_dyn_ = false;
#pragma omp parallel private(thread_id)
    {
      thread_id = omp_get_thread_num();
      if(thread_id==1)
      {
        std::cout<<"\n\n***************************************************************"
            <<"\n Starting op space (task coordinate) controller..."
            <<"\n This will move the BFR's hands in task (cartesian) space."
            <<"\n\n Press '1' to toggle control of the hand into either the {sw,da,eq keys} or {a default sine wave}"
            <<"\n***************************************************************";
        while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
        {
          /** ******************************************************************************************
           *                           Update the controller
           * ****************************************************************************************** */
          //1. Update the controller
          rtask_lhand->x_goal_ = db->s_gui_.ui_point_[0];
          rtask_rhand->x_goal_ = db->s_gui_.ui_point_[1];

          // Compute control forces (note that these directly have access to the io data ds).
          rctr.computeDynamics();
          rctr.computeControlForces();

          /** ******************************************************************************************
           *                           Integrate physics.
           * ****************************************************************************************** */
          //2. Simulation Dynamics
          if(db->pause_ctrl_dyn_ == false)
          {
            flag = dyn_scl_sp.integrate(rgcm_integ, rio, db->sim_dt_);
#ifdef BFR_CAPSTAN_LINKS_FREE
            // Here we consider the into-the-mri link to be constrained..
            rio.sensors_.q_(4) = -(rio.sensors_.q_(1) - rio.sensors_.q_(2));
            rio.sensors_.dq_(4) = -(rio.sensors_.dq_(1) - rio.sensors_.dq_(2));

            rio.sensors_.q_(9) = -(rio.sensors_.q_(6) - rio.sensors_.q_(7));
            rio.sensors_.dq_(9) = -(rio.sensors_.dq_(6) - rio.sensors_.dq_(7));
#else
            // Here we consider the front facing capstan link to be constrained..
            rio.sensors_.q_(2) = (rio.sensors_.q_(1) + rio.sensors_.q_(4));
            rio.sensors_.dq_(2) = (rio.sensors_.dq_(1) + rio.sensors_.dq_(4));

            rio.sensors_.q_(7) = (rio.sensors_.q_(6) + rio.sensors_.q_(9));
            rio.sensors_.dq_(7) = (rio.sensors_.dq_(6) + rio.sensors_.dq_(9));
#endif
            // The front bar link is always constrained
            rio.sensors_.q_(3) = (rio.sensors_.q_(1) - rio.sensors_.q_(2));
            rio.sensors_.dq_(3) = (rio.sensors_.dq_(1) - rio.sensors_.dq_(2));

            rio.sensors_.q_(8) = (rio.sensors_.q_(6) - rio.sensors_.q_(7));
            rio.sensors_.dq_(8) = (rio.sensors_.dq_(6) - rio.sensors_.dq_(7));

            ctrl_ctr++;//Increment the counter for dynamics computed.
          }

          /** ******************************************************************************************
           *                           Non-essential stuff (prints, real-time)
           * ****************************************************************************************** */
          // 2x a second we'll print stuff...
          const int prints_per_sec = 2;
          bool print_flag = /** Convert time from sec to sim ticks */static_cast<int>(sutil::CSystemClock::getSimTime()/db->sim_dt_)%
              /**Normalize by sim ticks and make % work */ static_cast<int>(1/(prints_per_sec*db->sim_dt_)) == 0;
          if(print_flag && !(db->pause_ctrl_dyn_)){
            std::cout<<"\n ***************************"
                <<"\n    t : "<<sutil::CSystemClock::getSimTime()
                <<"\n    Q : "<<rio.sensors_.q_.transpose()
                <<"\n   dQ : "<<rio.sensors_.dq_.transpose()
                <<"\n  Fgc : "<<rio.actuators_.force_gc_commanded_.transpose()
                <<"\n xcur : "<<rtask_lhand->x_.transpose()
                <<"\n xdes : "<<rtask_lhand->x_goal_.transpose()
                <<"\n xdes : "<<rtask_lhand->x_goal_.transpose()<<std::flush;
          }

          /** Slow down sim to real time */
          sutil::CSystemClock::tick(db->sim_dt_);
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
        }// End of the dynamics thread while loop
      }// End of the dynamics thread

      else{//Thread #2
        while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
        {
          glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL);

          /** Update the graphics */
          dynamic_cast<chai3d::cShapeSphere*>(rtask_lhand_des_gr)->setLocalPos(db->s_gui_.ui_point_[0](0),
              db->s_gui_.ui_point_[0](1),db->s_gui_.ui_point_[0](2));
          dynamic_cast<chai3d::cShapeSphere*>(rtask_rhand_des_gr)->setLocalPos(db->s_gui_.ui_point_[1](0),
                        db->s_gui_.ui_point_[1](1),db->s_gui_.ui_point_[1](2));
        }
      }// End of graphics thread
    }// End of threaded region
    tcurr = sutil::CSystemClock::getSysTime();

    /****************************Print Collected Statistics*****************************/
    std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
    std::cout<<"\nSimulation Took Time : "<<tcurr-tstart <<" sec";
    std::cout<<"\nReal World End Time  : "<<sutil::CSystemClock::getSysTime() <<" sec \n";
    std::cout<<"\nTotal Control Model and Servo Updates : "<<ctrl_ctr;
    std::cout<<"\nTotal Graphics Updates                : "<<gr_ctr;

    /****************************Deallocate Memory And Exit*****************************/
    flag = rchai.destroyGraphics();
    if(false == flag) { throw(std::runtime_error("Error deallocating graphics pointers")); } //Sanity check.

    std::cout<<"\nSCL : Executed Successfully";
    std::cout<<"\n*************************\n"<<std::flush;
    return 0;
  }
  catch(std::exception & e)
  {
    std::cout<<"\nEnd Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

    std::cout<<"\nSCL Failed: "<< e.what();
    std::cout<<"\n*************************\n";
    return 1;
  }
}
