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
/* \file CScl2RobAppTask.hpp
 *
 *  Created on: May 15, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
/*
 * Setup steps for a standard 1-robot simulation.
 */

#ifndef CSCL_2ROB_APP_TASK_HPP_
#define CSCL_2ROB_APP_TASK_HPP_

//Standard includes
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CSclParser.hpp>
#include <scl/dynamics/tao/CTaoDynamics.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/graphics/chai/CChaiGraphics.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/robot/CRobot.hpp>
#include <sutil/CSystemClock.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <scl/util/HelperFunctions.hpp>

#include <omp.h>
#include <GL/freeglut.h>

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <string>

//User modified includes to suit your application
#include <scl/control/task/tasks/CTaskOpPos.hpp>


namespace scl_app
{
  /* Generic code required to run a scl simulation
   * A simulation requires running 3 things. This example uses:
   * 1. A dynamics/physics engine                  :  Tao
   * 2. A controller                               :  Scl
   * 3. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
   */
  class CScl2RobAppTask
  {
  public:
    /************************************************************************/
    /***********NOTE : You can modify this to suit your application *********/
    /************************************************************************/
    scl::CControllerMultiTask* ctrl[2];           //Use a task controller

    std::string op_link_name[2],op_link2_name[2];
    scl::CTaskOpPos* tsk[2], *tsk2[2];
    scl::STaskOpPos* tsk_ds[2], *tsk2_ds[2];
    scl::sBool op_link_set[2], op_link2_set[2];

    /** Implement this function. Else you will get a linker error. */
    inline void stepMySimulation();

    /** Implement this function. Else you will get a linker error. */
    scl::sBool initMyController(int argc, char** argv, unsigned int arg_rob_idx);

    /** Default constructor. Sets stuff to zero. */
    CScl2RobAppTask()
    {
      for(int i=0; i<2; ++i)
      {
        ctrl[i] = S_NULL;           //Use a task controller
        op_link_name[i] = "not_init";
        op_link2_name[i] = "not_init";
        tsk[i] = S_NULL; tsk2[i] = S_NULL;;
        tsk_ds[i] = S_NULL; tsk2_ds[i] = S_NULL;
        op_link_set[i] = false; op_link2_set[i] = false;
      }
    }

    /** Destructor: Cleans up */
    ~CScl2RobAppTask()
    {
      for(int i=0; i<2; ++i)
      {
        log_file_[i].close();
        log_file_J_[i].close();
      }
    }

    /************************************************************************/
    /****************NOTE : You should NOT need to modify this **************/
    /****************       But feel free to use the objects   **************/
    /************************************************************************/
  public:
    /** Initialize the basic global variables, database etc.*/
    scl::sBool init(int argc, char** argv);

    /** Terminates the simulation and prints some statistics */
    void terminate();

    /** Runs a simulation using two threads:
     * Thread 1: Computes the robot dynamics
     * Thread 2: Renders the graphics and handles gui interaction */
    void runMainLoopThreaded();

    /** Runs a simulation using one thread.
     * 1: Computes the robot dynamics
     * 2: Renders the graphics and handles gui interaction */
    void runMainLoop();

    //Data types. Feel free to use them.
    scl::SDatabase* db;                 //Generic database (for sharing data)

    std::vector<std::string> robots_parsed;   //Parsed robots
    std::vector<std::string> graphics_parsed; //Parsed graphics views

    std::string robot_name[2];             //Currently selected robot
    std::string ctrl_name[2];              //Currently selected controller

    scl::CRobot robot[2];                  //Generic robot
    scl::SRobotIO* rob_io_ds[2];       //Access the robot's sensors and actuators

    scl::CTaoDynamics* tao_dyn[2];          //Generic tao dynamics
    scl::CChaiGraphics chai_gr;         //Generic chai graphics

    scl::sLongLong ctrl_ctr;            //Controller computation counter
    scl::sLongLong gr_ctr;              //Controller computation counter
    scl::sFloat t_start, t_end;         //Start and end times

    std::fstream log_file_[2];             //Logs vectors of [q, dq, x]
    std::fstream log_file_J_[2];           //Logs J
  };


  /************************************************************************/
  /****************NOTE : You should NOT need to modify this **************/
  /****************       All the function implementations   **************/
  /************************************************************************/

  scl::sBool CScl2RobAppTask::init(int argc, char** argv)
  {
    bool flag;
    if((argc != 11))
    {
      std::cout<<"\nDemo application demonstrates operational space task control for two humanoid robots,"
          <<"each with at-least two operational points."
          <<"\nThe command line input is: ./<executable> "
          <<"<file_name.xml> <robot_name> <controller_name> <operational task point> <operational task point2> "
          <<"<file_name2.xml> <robot_name2> <controller_name> <operational task point> <operational task point2>";
      return false;
    }
    else
    {
      try
      {
        /******************************Initialization************************************/
        //1. Initialize the database and clock.
        if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

        db = scl::CDatabase::getData(); //Sanity Check
        if(S_NULL==db) { throw(std::runtime_error("Database not initialized"));  }

        db->dir_specs_ = db->cwd_ + std::string("../../specs/"); //Set the specs dir so scl knows where the graphics are.

        //For parsing controllers
        flag = scl_registry::registerNativeDynamicTypes();
        if(false ==flag)  { throw(std::runtime_error("Could not register native dynamic types"));  }

        //Get going..
        std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

        std::string tmp_infile;

        for(int i=0; i<2; ++i)
        {
          tmp_infile = argv[i*5+1];
          std::cout<<"Parsing scl task controller for input file: "<<tmp_infile;

          /******************************File Parsing************************************/
          scl_parser::CSclParser tmp_lparser;//Use the scl tinyxml parser
          flag = scl_registry::parseEverythingInFile(tmp_infile,
              &tmp_lparser,&robots_parsed,&graphics_parsed);
          if((false == flag) || (robots_parsed.size()<=0)
              || (graphics_parsed.size()<=0) )
          { throw(std::runtime_error("Could not parse the file"));  }

          robot_name[i] = argv[i*5+2];
          if(!scl_util::isStringInVector(robot_name[i],robots_parsed))
          { throw(std::runtime_error("Could not find passed robot name in file"));  }

          /******************************TaoDynamics************************************/
          tao_dyn[i] = new scl::CTaoDynamics();
          flag = tao_dyn[i]->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name[i]));
          if(false == flag) { throw(std::runtime_error("Could not initialize physics simulator"));  }

          /******************************ChaiGlut Graphics************************************/
          if(!db->s_gui_.glut_initialized_)
          {
            glutInit(&argc, argv);
            db->s_gui_.glut_initialized_ = true;
          }

          //Bit of a hack, ignore the second robot's graphics spec
          //Fix later...
          if(i==0)
          {
            flag = chai_gr.initGraphics(graphics_parsed[0]);
            if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }
          }

          flag = chai_gr.addRobotToRender(robot_name[i]);
          if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

          //Bit of a hack, ignore the second robot's graphics spec
          //Fix later...
          if(i==0)
          {
            if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_parsed[0], &chai_gr))
            { throw(std::runtime_error("Glut initialization error")); }
          }

          /******************************Shared I/O Data Structure************************************/
          rob_io_ds[i] = db->s_io_.io_data_.at(robot_name[i]);
          if(S_NULL == rob_io_ds[i])
          { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

          /**********************Initialize Robot Dynamics and Controller*******************/
          flag = robot[i].initFromDb(robot_name[i],tao_dyn[i],tao_dyn[i]);//Note: The robot deletes these pointers.
          if(false == flag) { throw(std::runtime_error("Could not initialize robot"));  }

          ctrl_name[i] = argv[i*5+3];
          flag = robot[i].setControllerCurrent(ctrl_name[i]);
          if(false == flag) { throw(std::runtime_error("Could not initialize robot's controller"));  }

          /**********************Initialize Single Control Task *******************/
          flag = initMyController(argc,argv,i);
          if(false == flag)
          { throw(std::runtime_error("Could not initialize user's custom controller"));  }

          /******************************Initialize Log File **********************/
          std::string tmp_name = robot_name[i] + std::string(".log");
          log_file_[i].open(tmp_name.c_str(),std::fstream::out);
          if(!log_file_[i].is_open())
          { throw(std::runtime_error(std::string("Could not open log file: ") + tmp_name));  }

          tmp_name = robot_name[i] + std::string("_J.log");
          log_file_J_[i].open(tmp_name.c_str(),std::fstream::out);
          if(!log_file_J_[i].is_open())
          { throw(std::runtime_error(std::string("Could not open log file for Jacobian: ") + tmp_name));  }
        }


        ctrl_ctr=0;//Controller computation counter
        gr_ctr=0;//Controller computation counter

        //Simulation loop.
        std::cout<<"\nStarting simulation. Integration timestep: "<<db->sim_dt_<<std::flush;

        return true;
      }
      catch(std::exception & e)
      {
        std::cout<<"\nCScl2RobAppTask::setup() Failed : "<< e.what();
        terminate();
        return false;
      }
    }
  }

  void CScl2RobAppTask::terminate()
  {
    /****************************Print Collected Statistics*****************************/
    std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
    std::cout<<"\nTotal Control Model and Servo Updates : "<<ctrl_ctr;
    std::cout<<"\nTotal Graphics Updates                : "<<gr_ctr;

    /******************************Termination************************************/
    bool flag = chai_gr.destroyGraphics();
    if(false == flag) { std::cout<<"\nError deallocating graphics pointers"; } //Sanity check.

    std::cout<<"\nEnd Time:"<<sutil::CSystemClock::getSysTime();
    std::cout<<"\n********************************************\n"<<std::flush;
  }

  void CScl2RobAppTask::runMainLoopThreaded()
  {
    omp_set_num_threads(2);
    int thread_id;

#pragma omp parallel private(thread_id)
    {//Start threaded region
      thread_id = omp_get_thread_num();
      if(thread_id==1)
      {
        //Thread 1 : Run the simulation
        while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
        {
          if(!scl::CDatabase::getData()->pause_ctrl_dyn_)
          { stepMySimulation(); }
          //If paused, but step required, step it and set step flag to false.
          else if(scl::CDatabase::getData()->step_ctrl_dyn_)
          {
            scl::CDatabase::getData()->step_ctrl_dyn_ = false;
            stepMySimulation();
          }
          else
          {//Paused and no step required. Sleep for a bit.
            const timespec ts = {0, 15000000};//Sleep for 15ms
            nanosleep(&ts,NULL);
          }

          if(scl::CDatabase::getData()->param_logging_on_)
          {//Logs vectors of [q, dq, x, J]
            for(int i=0; i<2; ++i)
            {
              log_file_[i]<<rob_io_ds[i]->sensors_.q_.transpose()<<" "
                  <<rob_io_ds[i]->sensors_.dq_.transpose()<<" "
                  <<scl::CDatabase::getData()->s_gui_.ui_point_[2*i].transpose()
                  <<scl::CDatabase::getData()->s_gui_.ui_point_[2*i+1].transpose()
                  <<std::endl;
              log_file_J_[i]<<tsk_ds[i]->J_<<std::endl;
              log_file_J_[i]<<tsk2_ds[i]->J_<<std::endl;
            }
          }
        }
      }
      else
      {
        //Thread 2 : Run the graphics and gui
        while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
        {
          const timespec ts = {0, 15000000};//Sleep for 15ms
          nanosleep(&ts,NULL);
          glutMainLoopEvent(); //Update the graphics
          gr_ctr++;
        }
      }
    }//End of threaded region
  }

  void CScl2RobAppTask::runMainLoop()
  {
    while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
    {
      //If not paused, step the simulation.
      if(!scl::CDatabase::getData()->pause_ctrl_dyn_)
      { stepMySimulation(); }
      //If paused, but step required, step it and set step flag to false.
      else if(scl::CDatabase::getData()->step_ctrl_dyn_)
      {
        scl::CDatabase::getData()->step_ctrl_dyn_ = false;
        stepMySimulation();
      }

      static int gr_skip_ctr=0;
      if(gr_skip_ctr<=500)
      { gr_skip_ctr++; continue; }
      gr_skip_ctr = 0;
      glutMainLoopEvent(); //Update the graphics
      gr_ctr++;
    }
  }

}
#endif /* CSCL_2ROB_APP_TASK_HPP_ */
