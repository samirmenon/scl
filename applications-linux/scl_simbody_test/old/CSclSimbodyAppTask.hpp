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
/* \file CSclSimbodyAppTask.hpp
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

#ifndef CSCL_SIMBODY_APP_TASK_HPP_
#define CSCL_SIMBODY_APP_TASK_HPP_

//Standard includes
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/dynamics/tao/CTaoDynamics.hpp>
#include <scl/dynamics/simbody/CSimbodyDynamics.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
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

namespace scl_app
{
  /* Generic code required to run a scl simulation
   * A simulation requires running 3 things. This example uses:
   * 1. A dynamics/physics engine                  :  Tao
   * 2. A controller                               :  Scl
   * 3. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
   */
  class CSclSimbodyAppTask
  {
  public:
    /************************************************************************/
    /***********NOTE : You can modify this to suit your application *********/
    /************************************************************************/
    scl::CControllerMultiTask* ctrl;           //Use a task controller

    std::string op_link_name,op_link2_name;
    scl::CTaskOpPos* tsk, *tsk2;
    scl::STaskOpPos* tsk_ds, *tsk2_ds;
    scl::sBool op_link_set, op_link2_set;

    /** Implement this function. Else you will get a linker error. */
    inline void stepMySimulation();

    /** Implement this function. Else you will get a linker error. */
    scl::sBool initMyController(int argc, char** argv);

    /** Default constructor. Sets stuff to zero. */
    CSclSimbodyAppTask()
    {
      ctrl = S_NULL;           //Use a task controller
      op_link_name = "not_init";
      op_link2_name = "not_init";
      tsk = S_NULL; tsk2 = S_NULL;;
      tsk_ds = S_NULL; tsk2_ds = S_NULL;
      op_link_set = false; op_link2_set = false;
    }

    /** Destructor: Cleans up */
    ~CSclSimbodyAppTask()
    { log_file_.close(); log_file_J_.close(); }

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

    std::string robot_name;             //Currently selected robot
    std::string ctrl_name;              //Currently selected controller

    scl::CRobot robot;                  //Generic robot
    scl::SRobotIO* rob_io_ds;       //Access the robot's sensors and actuators

    scl::CTaoDynamics* tao_dyn;          //Generic tao dynamics
    scl::CSimbodyDynamics simbody_dyn;          //Generic tao dynamics
    scl::CGraphicsChai chai_gr;         //Generic chai graphics

    scl::sLongLong ctrl_ctr;            //Controller computation counter
    scl::sLongLong gr_ctr;              //Controller computation counter
    scl::sFloat t_start, t_end;         //Start and end times

    std::fstream log_file_;             //Logs vectors of [q, dq, x]
    std::fstream log_file_J_;           //Logs J
  };


  /************************************************************************/
  /****************NOTE : You should NOT need to modify this **************/
  /****************       All the function implementations   **************/
  /************************************************************************/

  scl::sBool CSclSimbodyAppTask::init(int argc, char** argv)
  {
    bool flag;
    if((argc != 5)&&(argc != 6))
    {
      std::cout<<"\nDemo application demonstrates operational space task control."
          <<"\nThe command line input is: ./<executable> <file_name.xml> <robot_name.xml> <controller_name> <operational task point> <optional : operational task point2>";
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

        std::string tmp_infile(argv[1]);
        std::cout<<"Running scl task controller for input file: "<<tmp_infile;

        /******************************File Parsing************************************/
        scl_parser::CLotusParser tmp_lparser;//Use the lotus tinyxml parser
        flag = scl_registry::parseEverythingInFile(tmp_infile,
            &tmp_lparser,&robots_parsed,&graphics_parsed);
        if((false == flag) || (robots_parsed.size()<=0)
            || (graphics_parsed.size()<=0) )
        { throw(std::runtime_error("Could not parse the file"));  }

        robot_name = argv[2];
        if(!scl_util::isStringInVector(robot_name,robots_parsed))
        { throw(std::runtime_error("Could not find passed robot name in file"));  }

        /******************************TaoDynamics************************************/
        tao_dyn = new scl::CTaoDynamics();
        flag = tao_dyn->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
        if(false == flag) { throw(std::runtime_error("Could not initialize object to compute dynamics matrices"));  }

        flag = simbody_dyn.init(*scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
        if(false == flag) { throw(std::runtime_error("Could not initialize physics integrator"));  }

        /******************************ChaiGlut Graphics************************************/
        if(!db->s_gui_.glut_initialized_)
        {
          glutInit(&argc, argv);
          db->s_gui_.glut_initialized_ = true;
        }

        flag = chai_gr.initGraphics(graphics_parsed[0]);
        if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

        flag = chai_gr.addRobotToRender(robot_name);
        if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

        if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_parsed[0], &chai_gr))
        { throw(std::runtime_error("Glut initialization error")); }

        /******************************Shared I/O Data Structure************************************/
        rob_io_ds = db->s_io_.io_data_.at(robot_name);
        if(S_NULL == rob_io_ds)
        { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

        /**********************Initialize Robot Dynamics and Controller*******************/
        flag = robot.initFromDb(robot_name,tao_dyn,&simbody_dyn);//Note: The robot deletes these pointers.
        if(false == flag) { throw(std::runtime_error("Could not initialize robot"));  }

        ctrl_name = argv[3];
        flag = robot.setControllerCurrent(ctrl_name);
        if(false == flag) { throw(std::runtime_error("Could not initialize robot's controller"));  }

        /**********************Initialize Single Control Task *******************/
        flag = initMyController(argc,argv);
        if(false == flag)
        { throw(std::runtime_error("Could not initialize user's custom controller"));  }

        /******************************Initialize Log File **********************/
        std::string tmp_name = robot_name + std::string(".log");
        log_file_.open(tmp_name.c_str(),std::fstream::out);
        if(!log_file_.is_open())
        { throw(std::runtime_error(std::string("Could not open log file: ") + tmp_name));  }

        tmp_name = robot_name + std::string("_J.log");
        log_file_J_.open(tmp_name.c_str(),std::fstream::out);
        if(!log_file_J_.is_open())
        { throw(std::runtime_error(std::string("Could not open log file for Jacobian: ") + tmp_name));  }


        ctrl_ctr=0;//Controller computation counter
        gr_ctr=0;//Controller computation counter

        //Simulation loop.
        std::cout<<"\nStarting simulation. Integration timestep: "<<db->sim_dt_<<std::flush;

        return true;
      }
      catch(std::exception & e)
      {
        std::cout<<"\nCSclSimbodyAppTask::setup() Failed : "<< e.what();
        terminate();
        return false;
      }
    }
  }

  void CSclSimbodyAppTask::terminate()
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

  void CSclSimbodyAppTask::runMainLoopThreaded()
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
            log_file_<<rob_io_ds->sensors_.q_.transpose()<<" "
                  <<rob_io_ds->sensors_.dq_.transpose()<<" "
                  <<scl::CDatabase::getData()->s_gui_.ui_point_[0].transpose()
                  <<std::endl;
            log_file_J_<<tsk_ds->J_<<std::endl;
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

  void CSclSimbodyAppTask::runMainLoop()
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
#endif /* CSCL_SIMBODY_APP_TASK_HPP_ */
