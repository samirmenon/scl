/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/* \file gc_main.cpp
 *
 *  Created on: Nov 22, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/scl.hpp>
#include <scl_ext/scl_ext.hpp>

#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cassert>

#include <omp.h>
#include <GL/freeglut.h>

/**
 * A sample application to demonstrate scl running one or more robots.
 *
 * Use it as a template to write your own (more detailed/beautiful/functional)
 * application.
 *
 * A simulation requires running 3 things:
 * 1. A dynamics/physics engine                  :  SCL Spatial
 * 2. A controller                               :  Scl
 * 3. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
 */
int main(int argc, char** argv)
{
  bool flag;
  if((argc < 2)&&(argc > 4))
  {
    std::cout<<"\nscl-robot demo application demonstrates how scl controls joint angles of single robots."
        <<"\nThe command line input is: ./<executable> <file_name.xml> <optional: robot name> <optional: controller name>\n"
        <<"\n ********** \n Options : "
        <<"\n Press 1 : Toggle (enable/disable) integral gain. Default off. \n";
    return 0;
  }
  else
  {
    try
    {
      std::cout<<"\nscl-robot demo application demonstrates how scl controls joint angles of single robots."
          <<"\n ********** \n Options : "
          <<"\n Press 1 : Toggle (enable/disable) integral gain. Default off. \n";
      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

      scl::SDatabase* db = scl::CDatabase::getData(); //Sanity Check
      if(S_NULL==db) { throw(std::runtime_error("Database not initialized"));  }

      db->dir_specs_ = db->cwd_ + std::string("../../specs/"); //Set the specs dir so scl knows where the graphics are.

      //Get going..
      std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

      std::string tmp_infile(argv[1]);
      std::cout<<"Running scl benchmarks for input file: "<<tmp_infile;

      /******************************File Parsing************************************/
      scl::CParserScl tmp_lparser;//Use the scl tinyxml parser

      std::string robot_name;
      if(argc<3)
      {//Find the robot specs in the file if one isn't specified by the user.
        std::vector<std::string> robot_names;
        flag = tmp_lparser.listRobotsInFile(tmp_infile,robot_names);
        if(false == flag) { throw(std::runtime_error("Could not read robot names from the file"));  }
        robot_name = robot_names[0];//Use the first available robot.
      }
      else { robot_name = argv[2];}//If robot name was passed, use it.

      if(S_NULL == scl_registry::parseRobot(tmp_infile, robot_name, &tmp_lparser))
      { throw(std::runtime_error("Could not register robot with the database"));  }

      std::vector<std::string> graphics_names;
      flag = tmp_lparser.listGraphicsInFile(tmp_infile,graphics_names);
      if(false == flag) { throw(std::runtime_error("Could not list graphics names from the file"));  }

      if(S_NULL == scl_registry::parseGraphics(tmp_infile, graphics_names[0], &tmp_lparser))
      { throw(std::runtime_error("Could not register graphics with the database"));  }

      scl::SRobotParsed *rob_ds = scl::CDatabase::getData()->s_parser_.robots_.at(robot_name);
      if(NULL == rob_ds)
      { throw(std::runtime_error("Could not find registered robot data struct in the database"));  }

      std::string ctrl_name; //Parse all the gc controllers for this robot!
      if(argc<4)
      {//Find the robot controller in the file if one isn't specified by the user.
        ctrl_name = "";

        std::vector<std::pair<std::string,std::string> > ctrl_names;//<name,type>
        flag = tmp_lparser.listControllersInFile(tmp_infile,ctrl_names);
        if(false == flag) { throw(std::runtime_error("Could not list controllers in the file"));  }

        //Find only the gc controller
        std::vector<std::pair<std::string,std::string> >::iterator itc, itce;
        for(itc = ctrl_names.begin(), itce = ctrl_names.end();itc!=itce;++itc)
        { if((*itc).second=="gc"){ ctrl_name = (*itc).first; break; } }

        if("" == ctrl_name)
        { throw(std::runtime_error("Could not find any gc controllers in the file"));  }
      }
      else { ctrl_name = argv[3];}//If robot name was passed, use it.

      if(S_NULL == scl_registry::parseGcController(tmp_infile, robot_name, ctrl_name, &tmp_lparser))
      { throw(std::runtime_error("Could not register controller with the database"));  }

      /******************************Scl Spatial Dynamics************************************/
      scl_ext::CDynamicsSclSpatial dyn_scl_sp_int;
      flag = dyn_scl_sp_int.init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
      if(false == flag) { throw(std::runtime_error("Could not initialize physics simulator"));  }

      scl::SGcModel dyn_scl_gcm;
      flag = dyn_scl_gcm.init(*rob_ds);
      if(false == flag) { throw(std::runtime_error("Could not initialize gc model for the physics simulator"));  }

      /******************************Shared I/O Data Structure************************************/
      scl::SRobotIO* rob_io_ds;
      rob_io_ds = db->s_io_.io_data_.at(robot_name);
      if(S_NULL == rob_io_ds)
      { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

      /******************************ChaiGlut Graphics************************************/
      glutInit(&argc, argv);

      scl::CGraphicsChai chai_gr;
      scl::SGraphicsParsed *gr_parsed = db->s_parser_.graphics_worlds_.at(graphics_names[0]);
      scl::SGraphicsChai *chai_ds = db->s_gui_.chai_data_.at(graphics_names[0]);
      flag = chai_gr.initGraphics(gr_parsed,chai_ds);
      if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

      flag = chai_gr.addRobotToRender(rob_ds,rob_io_ds);
      if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

      if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_names[0], &chai_gr))
      { throw(std::runtime_error("Glut initialization error")); }

      /**********************Initialize Robot Dynamics and Controller*******************/
      scl::CDynamicsScl dyn_scl; //Use for model updates.
      flag = dyn_scl.init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name)); //Reads stuff from the database.
      if(false == flag) { throw(std::runtime_error("Could not initialize dynamics object"));  }

      scl::SControllerGc * gc_ctrl_ds;
      gc_ctrl_ds = dynamic_cast<scl::SControllerGc*>(*(db->s_controller_.controllers_.at(ctrl_name)));
      if(S_NULL == gc_ctrl_ds) { throw(std::runtime_error("Could not find the controller in the database"));  }

      scl::CControllerGc robot_gc_ctrl;
      flag = robot_gc_ctrl.init(gc_ctrl_ds,&dyn_scl);
      if(false == flag) { throw(std::runtime_error("Could not initialize the controller object"));  }

      /******************************Main Loop************************************/
      std::cout<<std::flush;

      scl::sLongLong ctrl_ctr=0;//Controller computation counter
      scl::sLongLong gr_ctr=0;//Controller computation counter

      scl::sFloat t_start, t_end;

      //Simulation loop.
      std::cout<<"\nStarting simulation. Timestep : "<<db->sim_dt_<<std::flush;

      bool flag_status_pida = false;
      double sine_amplitude = 0.2;

#ifndef NOPARALLEL
      // This is the threaded version (for release mode)...
      omp_set_num_threads(2);
      int thread_id;
      t_start = sutil::CSystemClock::getSysTime();
#pragma omp parallel private(thread_id)
      {
        thread_id = omp_get_thread_num();
        if(thread_id==1)
        {
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            //1. Simulation Dynamics
            if(scl::CDatabase::getData()->pause_ctrl_dyn_ == false)
            {
              flag = dyn_scl_sp_int.integrate(dyn_scl_gcm,(*rob_io_ds), scl::CDatabase::getData()->sim_dt_);
              if(rob_ds->flag_apply_gc_damping_)
              { rob_io_ds->sensors_.dq_ -= rob_io_ds->sensors_.dq_ * (db->sim_dt_/100); }//1% Velocity damping.

              /** Slow down sim to real time */
              sutil::CSystemClock::tick(scl::CDatabase::getData()->sim_dt_);
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

            //2. Update the controller
            for(unsigned int i=0; i< gc_ctrl_ds->robot_->dof_;i++)
            { gc_ctrl_ds->des_q_(i) = sine_amplitude*sin(sutil::CSystemClock::getSimTime());  }

            //Slower dynamics update.
            if(ctrl_ctr%3 == 0)
            {
              robot_gc_ctrl.computeKinematics();
              robot_gc_ctrl.computeDynamics();
            }

            // The actual control loop and some code to print transitions between PDA and PIDA control
            if(db->s_gui_.ui_flag_[1])
            {
              if(false == flag_status_pida)
              { std::cout<<"\n scl_gc_ctrl: Moving to PIDA control."; std::cout.flush(); }
              flag_status_pida = true;
              robot_gc_ctrl.computeControlForcesPIDA(sutil::CSystemClock::getSysTime());
            }
            else
            {
              if(true == flag_status_pida)
              { std::cout<<"\n scl_gc_ctrl: Moving to PDA control."; std::cout.flush(); }
              flag_status_pida = false;
              robot_gc_ctrl.computeControlForces();
            }

            //Set the command torques for the simulator to the controller's computed torques
            rob_io_ds->actuators_.force_gc_commanded_ = gc_ctrl_ds->des_force_gc_;
            ctrl_ctr++;//Increment the counter for dynamics computed.

            // Once in a while we'll print stuff...
            if(ctrl_ctr%1000 == 0 && !(scl::CDatabase::getData()->pause_ctrl_dyn_)){
              std::cout<<"\n ***************************"
                       <<"\n Q   : "<<rob_io_ds->sensors_.q_.transpose();
              std::cout<<"\n Qd  : "<<gc_ctrl_ds->des_q_.transpose();
              std::cout<<"\n dQ  : "<<rob_io_ds->sensors_.dq_.transpose();
              std::cout<<"\n dQd : "<<gc_ctrl_ds->des_dq_.transpose();
              std::cout<<"\n Fgc : "<<rob_io_ds->actuators_.force_gc_commanded_.transpose();
            }
          }
        }
        else
        {
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            glutMainLoopEvent();
            gr_ctr++;
            const timespec ts = {0, 15000000};//Sleep for 15ms
            nanosleep(&ts,NULL);
          }
        }
      }
#else
      // This is the non-threaded version (for debug mode)...
      t_start = sutil::CSystemClock::getSysTime();
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        assert(rob_io_ds == gc_ctrl_ds->io_data_);

#ifdef SCL_PRINT_INFO_MESSAGES
        rob_io_ds->printInfo();
        std::cout<<"\n A :"<<gc_ctrl_ds->gc_model_->A_;
        std::cout<<"\n b :"<<gc_ctrl_ds->gc_model_->b_.transpose();
        std::cout<<"\n g :"<<gc_ctrl_ds->gc_model_->g_.transpose();
        std::cout<<"\n com :"<<gc_ctrl_ds->gc_model_->pos_com_.transpose();
#endif

        //1. Simulation Dynamics
        if(scl::CDatabase::getData()->pause_ctrl_dyn_ == false)
        {
          flag = dyn_scl_sp_int.integrate(dyn_scl_gcm,(*rob_io_ds), scl::CDatabase::getData()->sim_dt_);
          if(rob_ds->flag_apply_gc_damping_)
          { rob_io_ds->sensors_.dq_ -= rob_io_ds->sensors_.dq_ * (db->sim_dt_/100); }//1% Velocity damping.

          /** Slow down sim to real time */
          sutil::CSystemClock::tick(scl::CDatabase::getData()->sim_dt_);
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

        //2. Update the controller
        for(unsigned int i=0; i< gc_ctrl_ds->robot_->dof_;i++)
        { gc_ctrl_ds->des_q_(i) = sine_amplitude*sin(sutil::CSystemClock::getSysTime());  }

        //(Much) Slower dynamics update.
        if(ctrl_ctr%50 == 0)
        {
          robot_gc_ctrl.computeKinematics();
          robot_gc_ctrl.computeDynamics();
        }

        // The actual control loop and some code to print transitions between PDA and PIDA control
        if(db->s_gui_.ui_flag_[1])
        {
          if(false == flag_status_pida)
          { std::cout<<"\n scl_gc_ctrl: Moving to PIDA control."; std::cout.flush(); }
          flag_status_pida = true;
          robot_gc_ctrl.computeControlForcesPIDA(sutil::CSystemClock::getSysTime());
        }
        else
        {
          if(true == flag_status_pida)
          { std::cout<<"\n scl_gc_ctrl: Moving to PDA control."; std::cout.flush(); }
          flag_status_pida = false;
          robot_gc_ctrl.computeControlForces();
        }

        //Set the command torques for the simulator to the controller's computed torques
        rob_io_ds->actuators_.force_gc_commanded_ = gc_ctrl_ds->des_force_gc_;
        ctrl_ctr++;//Increment the counter for dynamics computed.

        if(ctrl_ctr%50 == 0)
        {
          glutMainLoopEvent();
          gr_ctr++;
        }
      }
#endif

      t_end = sutil::CSystemClock::getSysTime();

      /****************************Print Collected Statistics*****************************/
      std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
      std::cout<<"\nSimulation Took Time : "<<t_end-t_start <<" sec";
      std::cout<<"\nReal World End Time  : "<<sutil::CSystemClock::getSysTime() <<" sec \n";
      std::cout<<"\nTotal Control Model and Servo Updates : "<<ctrl_ctr;
      std::cout<<"\nTotal Graphics Updates                : "<<gr_ctr;

      /****************************Deallocate Memory And Exit*****************************/
      flag = chai_gr.destroyGraphics();
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
}
