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
/* \file CSclRedisTask.hpp
 *
 *  Created on: Jan 20, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
/*
 * Setup steps for a standard 1-robot simulation.
 */

#ifndef CSCL_REDIS_TASK_HPP_
#define CSCL_REDIS_TASK_HPP_

//Standard includes
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
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

//User modified includes to suit your application
#include <scl/control/task/tasks/CTaskOpPos.hpp>

#define SCL_REDIS_MAX_MARKERS 50

namespace scl_app
{
  /* Generic code required to run a scl simulation
   * A simulation requires running 3 things. This example uses:
   * 1. A dynamics/physics engine                  :  Tao
   * 2. A controller                               :  Scl
   * 3. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
   */
  class CSclRedisTask
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
    CSclRedisTask()
    {
      db_ = S_NULL;
      rob_io_ds_ = S_NULL;
      dyn_tao_ = S_NULL;
      dyn_scl_ = S_NULL;

      ctrl = S_NULL;           //Use a task controller
      op_link_name = "not_init";
      op_link2_name = "not_init";
      tsk = S_NULL; tsk2 = S_NULL;;
      tsk_ds = S_NULL; tsk2_ds = S_NULL;
      op_link_set = false; op_link2_set = false;

      ctrl_ctr_ = 0;
      gr_ctr_ = 0;
      t_start_ = 0.0;
      t_end_ = 0.0;
      curr_traj_marker_id_ = 0;
    }

    /** Destructor: Cleans up */
    ~CSclRedisTask()
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
    scl::SDatabase* db_;                 //Generic database (for sharing data)

    std::vector<std::string> robots_parsed_;   //Parsed robots
    std::vector<std::string> graphics_parsed_; //Parsed graphics views

    std::string robot_name_;             //Currently selected robot
    std::string ctrl_name_;              //Currently selected controller

    scl::CRobot robot_;                  //Generic robot
    scl::SRobotParsed *rob_ds_;          //Generic robot ds
    scl::SRobotIO* rob_io_ds_;       //Access the robot's sensors and actuators

    scl::CDynamicsTao* dyn_tao_;          //Generic tao dynamics
    scl::CDynamicsScl* dyn_scl_;          //Generic tao dynamics
    scl::CGraphicsChai chai_gr_;         //Generic chai graphics

    scl::sLongLong ctrl_ctr_;            //Controller computation counter
    scl::sLongLong gr_ctr_;              //Controller computation counter
    scl::sFloat t_start_, t_end_;         //Start and end times

    std::fstream log_file_;             //Logs vectors of [q, dq, x]
    std::fstream log_file_J_;           //Logs J

    chai3d::cGenericObject* traj_markers_[SCL_REDIS_MAX_MARKERS];
    int curr_traj_marker_id_;
    bool flag_all_markers_added_=false;

    bool flag_enable_logging_ = false;
  };


  /************************************************************************/
  /****************NOTE : You should NOT need to modify this **************/
  /****************       All the function implementations   **************/
  /************************************************************************/

  scl::sBool CSclRedisTask::init(int argc, char** argv)
  {
    bool flag;
    if((argc != 5)&&(argc != 6))
    {
      std::cout<<"\nDemo application demonstrates operational space task control."
          <<"\nThe command line input is: ./<executable> <file_name.xml> <robot_name> <controller_name> <operational task point> <optional : operational task point2>";
      return false;
    }
    else
    {
      try
      {
        /******************************Initialization************************************/
        //1. Initialize the database and clock.
        if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

        db_ = scl::CDatabase::getData(); //Sanity Check
        if(S_NULL==db_) { throw(std::runtime_error("Database not initialized"));  }

        db_->dir_specs_ = db_->cwd_ + std::string("../../specs/"); //Set the specs dir so scl knows where the graphics are.

        //For parsing controllers
        flag = scl_registry::registerNativeDynamicTypes();
        if(false ==flag)  { throw(std::runtime_error("Could not register native dynamic types"));  }

        //Get going..
        std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

        std::string tmp_infile(argv[1]);
        std::cout<<"Running scl task controller for input file: "<<tmp_infile;

        /******************************File Parsing************************************/
        scl::CParserScl tmp_lparser;//Use the scl tinyxml parser
        flag = scl_registry::parseEverythingInFile(tmp_infile,
            &tmp_lparser,&robots_parsed_,&graphics_parsed_);
        if((false == flag) || (robots_parsed_.size()<=0)
            || (graphics_parsed_.size()<=0) )
        { throw(std::runtime_error("Could not parse the file"));  }

        robot_name_ = argv[2];
        if(!scl_util::isStringInVector(robot_name_,robots_parsed_))
        { throw(std::runtime_error("Could not find passed robot name in file"));  }

        /******************************TaoDynamics************************************/
        dyn_tao_ = new scl::CDynamicsTao();
        flag = dyn_tao_->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name_));
        if(false == flag) { throw(std::runtime_error("Could not initialize physics simulator"));  }

        /******************************SclDynamics************************************/
        dyn_scl_ = new scl::CDynamicsScl();
        flag = dyn_scl_->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name_));
        if(false == flag) { throw(std::runtime_error("Could not initialize dynamics algorithms"));  }

        /******************************Shared I/O Data Structure************************************/
        rob_ds_ = db_->s_parser_.robots_.at(robot_name_);
        if(S_NULL == rob_ds_)
        { throw(std::runtime_error("Robot data structure does not exist in the database"));  }

        rob_io_ds_ = db_->s_io_.io_data_.at(robot_name_);
        if(S_NULL == rob_io_ds_)
        { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

        /**********************Initialize Robot Dynamics and Controller*******************/
        flag = robot_.initFromDb(robot_name_,dyn_scl_,dyn_tao_);//Note: The robot deletes these pointers.
        if(false == flag) { throw(std::runtime_error("Could not initialize robot"));  }

        ctrl_name_ = argv[3];
        flag = robot_.setControllerCurrent(ctrl_name_);
        if(false == flag) { throw(std::runtime_error("Could not initialize robot's controller"));  }

        /******************************ChaiGlut Graphics************************************/
        if(!db_->s_gui_.glut_initialized_)
        {
          glutInit(&argc, argv);
          db_->s_gui_.glut_initialized_ = true;
        }

        scl::SGraphicsParsed *gr_parsed = db_->s_parser_.graphics_worlds_.at(graphics_parsed_[0]);
        scl::SGraphicsChai *chai_ds = db_->s_gui_.chai_data_.at(graphics_parsed_[0]);
        flag = chai_gr_.initGraphics(gr_parsed,chai_ds);
        if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

        flag = chai_gr_.addRobotToRender(rob_ds_,rob_io_ds_);
        if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

        if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_parsed_[0], &chai_gr_))
        { throw(std::runtime_error("Glut initialization error")); }

        /**********************Initialize Single Control Task *******************/
        flag = initMyController(argc,argv);
        if(false == flag)
        { throw(std::runtime_error("Could not initialize user's custom controller"));  }

        /******************************Initialize Log File **********************/
        std::string tmp_name = robot_name_ + std::string(".log");
        log_file_.open(tmp_name.c_str(),std::fstream::out);
        if(!log_file_.is_open())
        { throw(std::runtime_error(std::string("Could not open log file: ") + tmp_name));  }

        tmp_name = robot_name_ + std::string("_J.log");
        log_file_J_.open(tmp_name.c_str(),std::fstream::out);
        if(!log_file_J_.is_open())
        { throw(std::runtime_error(std::string("Could not open log file for Jacobian: ") + tmp_name));  }


        ctrl_ctr_=0;//Controller computation counter
        gr_ctr_=0;//Controller computation counter
        curr_traj_marker_id_=0;

        //Simulation loop.
        std::cout<<"\nStarting simulation. Integration timestep: "<<db_->sim_dt_<<std::flush;

        return true;
      }
      catch(std::exception & e)
      {
        std::cout<<"\nCSclRedisTask::setup() Failed : "<< e.what();
        terminate();
        return false;
      }
    }
  }

  void CSclRedisTask::terminate()
  {
    /****************************Print Collected Statistics*****************************/
    std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
    std::cout<<"\nTotal Control Model and Servo Updates : "<<ctrl_ctr_;
    std::cout<<"\nTotal Graphics Updates                : "<<gr_ctr_;

    /******************************Termination************************************/
    bool flag = chai_gr_.destroyGraphics();
    if(false == flag) { std::cout<<"\nError deallocating graphics pointers"; } //Sanity check.

    std::cout<<"\nEnd Time:"<<sutil::CSystemClock::getSysTime();
    std::cout<<"\n********************************************\n"<<std::flush;
  }

  void CSclRedisTask::runMainLoopThreaded()
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

          if(flag_enable_logging_)
          {//Logs vectors of [q, dq, x, J]
            log_file_<<rob_io_ds_->sensors_.q_.transpose()<<" "
                  <<rob_io_ds_->sensors_.dq_.transpose()<<" "
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
          gr_ctr_++;
        }
      }
    }//End of threaded region
  }

  void CSclRedisTask::runMainLoop()
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
      gr_ctr_++;
    }
  }

}
#endif /* CSCL_REDIS_TASK_HPP_ */
