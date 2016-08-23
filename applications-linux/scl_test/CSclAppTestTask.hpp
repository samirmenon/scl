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
/* \file CSclAppTestTask.hpp
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

#ifndef CSCL_APP_TEST_TASK_HPP_
#define CSCL_APP_TEST_TASK_HPP_

//Standard includes
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/Init.hpp>

#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/robot/CRobot.hpp>
#include <sutil/CSystemClock.hpp>
#include <scl/util/HelperFunctions.hpp>

#include <omp.h>
#include <GL/freeglut.h>

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <string>

//User modified includes to suit your application
#include <scl/control/task/tasks/CTaskOpPos.hpp>


namespace scl_test
{
  /* Generic code required to run a scl simulation
   * A simulation requires running 3 things. This example uses:
   * 1. A dynamics/physics engine                  :  Scl Spatial
   * 2. A controller                               :  Scl
   * 3. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
   */
  class CSclAppTestTask
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

    /** Steps the simulation and adds (system) controller time to the var passed */
    inline void  stepMySimulationClockCtrl(
          /** Increments the controller time */
          double& arg_ctrl_time);

    /** Implement this function. Else you will get a linker error. */
    scl::sBool initMyController(int argc, char** argv,
        const std::string & arg_op_link_name1,
        const std::string & arg_op_link_name2);

    /** Default constructor. Sets stuff to zero. */
    CSclAppTestTask()
    {
      ctrl = S_NULL;           //Use a task controller
      op_link_name = "not_init";
      op_link2_name = "not_init";
      tsk = S_NULL; tsk2 = S_NULL;;
      tsk_ds = S_NULL; tsk2_ds = S_NULL;
      op_link_set = false; op_link2_set = false;
      dyn_scl_sp = NULL; scl_dyn = NULL;
    }

    ~CSclAppTestTask()
    { }

    /************************************************************************/
    /****************NOTE : You should NOT need to modify this **************/
    /****************       But feel free to use the objects   **************/
    /************************************************************************/
  public:
    /** Initialize the basic global variables, database etc.*/
    scl::sBool init(int argc, char** argv,
        const std::string & arg_file,
        const std::string & arg_robot_name,
        const std::string & arg_controller_name,
        const std::string & arg_op_link_name1,
        const std::string & arg_op_link_name2);

    /** Terminates the simulation and prints some statistics */
    void terminate();

    /** Runs a simulation using two threads:
     * Thread 1: Computes the robot dynamics
     * Thread 2: Renders the graphics and handles gui interaction */
    void runMainLoopThreaded(double arg_exec_sim_time);

    /** Runs a simulation using one thread.
     * 1: Computes the robot dynamics
     * 2: Renders the graphics and handles gui interaction */
    void runMainLoop(double arg_exec_sim_time);

    /** Runs a simulation sans graphics */
    void runSimulation(double arg_exec_sim_time);

    /** Runs a simulation sans graphics. Returns the time taken
     * by the controller */
    double runSimulationClockCtrl(double arg_exec_sim_time);

    //Data types. Feel free to use them.
    scl::SDatabase* db=NULL;            //Generic database (for sharing data)

    std::vector<std::string> robots_parsed;   //Parsed robots
    std::vector<std::string> graphics_parsed; //Parsed graphics views

    std::string robot_name;             //Currently selected robot
    std::string ctrl_name;              //Currently selected controller

    scl::CRobot robot;                  //Generic robot
    scl::SRobotParsed *rob_ds=NULL;     //Generic robot data structure
    scl::SRobotIO* rob_io_ds=NULL;      //Access the robot's sensors and actuators

    scl_ext::CDynamicsSclSpatial* dyn_scl_sp=NULL;//Generic scl_spatial dynamics
    scl::CDynamicsScl* scl_dyn=NULL;    //Generic scl dynamics
    scl::CGraphicsChai chai_gr;         //Generic chai graphics

    scl::sLongLong ctrl_ctr=0;          //Controller computation counter
    scl::sLongLong gr_ctr=0;            //Controller computation counter
    scl::sFloat t_start=0.0, t_end=0.0; //Start and end times
  };


  /************************************************************************/
  /****************NOTE : You should NOT need to modify this **************/
  /****************       All the function implementations   **************/
  /************************************************************************/

  scl::sBool CSclAppTestTask::init(int argc, char** argv,
      const std::string & arg_file,
      const std::string & arg_robot_name,
      const std::string & arg_controller_name,
      const std::string & arg_op_link_name1,
      const std::string & arg_op_link_name2)
  {
    bool flag;
    try
    {
      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

      db = scl::CDatabase::getData(); //Sanity Check
      if(S_NULL==db) { throw(std::runtime_error("Database not initialized"));  }

      db->dir_specs_ = db->cwd_ + std::string("../../specs/"); //Set the specs dir so scl knows where the graphics are.

      //For parsing controllers
      flag = scl::init::registerNativeDynamicTypes();
      if(false ==flag)  { throw(std::runtime_error("Could not register native dynamic types"));  }

      //Get going..
      std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";
      std::cout<<"Running scl task controller for input file: "<<arg_file;

      /******************************File Parsing************************************/
      scl::CParserScl tmp_lparser;//Use the scl tinyxml parser
      flag = scl_registry::parseEverythingInFile(arg_file,
          &tmp_lparser,&robots_parsed,&graphics_parsed);
      if((false == flag) || (robots_parsed.size()<=0)
          || (graphics_parsed.size()<=0) )
      { throw(std::runtime_error("Could not parse the file"));  }

      robot_name = arg_robot_name;
      if(!scl_util::isStringInVector(robot_name,robots_parsed))
      { throw(std::runtime_error("Could not find passed robot name in file"));  }

      rob_ds = scl::CDatabase::getData()->s_parser_.robots_.at(robot_name);
      if(NULL == rob_ds)
      { throw(std::runtime_error("Could not find registered robot data struct in the database"));  }

      /******************************Scl Spatial Dynamics************************************/
      // NOTE : We do NOT delete this.. When passed to the controller, the controller takes care of it.
      dyn_scl_sp = new scl_ext::CDynamicsSclSpatial();
      flag = dyn_scl_sp->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
      if(false == flag) { throw(std::runtime_error("Could not initialize physics simulator"));  }

      /******************************SclDynamics************************************/
      // NOTE : We do NOT delete this.. When passed to the controller, the controller takes care of it.
      scl_dyn = new scl::CDynamicsScl();
      flag = scl_dyn->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
      if(false == flag) { throw(std::runtime_error("Could not initialize dynamics object"));  }

      /******************************Shared I/O Data Structure************************************/
      rob_io_ds = db->s_io_.io_data_.at(robot_name);
      if(S_NULL == rob_io_ds)
      { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

      /******************************ChaiGlut Graphics************************************/
      //Check if glut has already been initialized.
      if(1!=glutGet(GLUT_INIT_STATE))
      {
        glutInit(&argc, argv);
        db->s_gui_.glut_initialized_ = true;
      }

      scl::SGraphicsParsed *gr_parsed = db->s_parser_.graphics_worlds_.at(graphics_parsed[0]);
      scl::SGraphicsChai *chai_ds = db->s_gui_.chai_data_.at(graphics_parsed[0]);
      flag = chai_gr.initGraphics(gr_parsed,chai_ds);
      if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

      flag = chai_gr.addRobotToRender(rob_ds,rob_io_ds);
      if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

      if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_parsed[0], &chai_gr))
      { throw(std::runtime_error("Glut initialization error")); }

      /**********************Initialize Robot Dynamics and Controller*******************/
      flag = robot.initFromDb(robot_name,scl_dyn,dyn_scl_sp);//Note: The robot deletes these pointers.
      if(false == flag) { throw(std::runtime_error("Could not initialize robot"));  }

      ctrl_name = arg_controller_name;
      flag = robot.setControllerCurrent(ctrl_name);
      if(false == flag) { throw(std::runtime_error("Could not initialize robot's controller"));  }

      /**********************Initialize Single Control Task *******************/
      flag = initMyController(argc,argv,arg_op_link_name1,arg_op_link_name2);
      if(false == flag)
      { throw(std::runtime_error("Could not initialize user's custom controller"));  }

      ctrl_ctr=0;//Controller computation counter
      gr_ctr=0;//Controller computation counter

      //Simulation loop.
      std::cout<<"\nStarting simulation. Integration timestep: "<<db->sim_dt_<<std::flush;

      return true;
    }
    catch(std::exception & e)
    {
      std::cout<<"\nCSclAppTestTask::setup() Failed : "<< e.what();
      terminate();
      return false;
    }
  }

  void CSclAppTestTask::terminate()
  {
    /****************************Print Collected Statistics*****************************/
    std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
    std::cout<<"\nTotal Control Model and Servo Updates : "<<ctrl_ctr;
    std::cout<<"\nTotal Graphics Updates                : "<<gr_ctr;

    /******************************Termination************************************/
    bool flag = chai_gr.destroyGraphics();
    if(false == flag) { std::cout<<"\nError deallocating graphics pointers"; } //Sanity check.

    std::cout<<"\nEnd Time: "<<sutil::CSystemClock::getSysTime();
    std::cout<<"\n********************************************\n"<<std::flush;
  }

  void CSclAppTestTask::runMainLoopThreaded(double arg_exec_sim_time)
  {
    double start_sim_time = sutil::CSystemClock::getSimTime();

    omp_set_num_threads(2);
    int thread_id;

#pragma omp parallel private(thread_id)
    {//Start threaded region
      thread_id = omp_get_thread_num();
      if(thread_id==1)
      {
        //Thread 1 : Run the simulation
        while(true == scl::CDatabase::getData()->running_)
        {
          stepMySimulation();

          if(arg_exec_sim_time<sutil::CSystemClock::getSimTime() - start_sim_time)
          { scl::CDatabase::getData()->running_ = false; }
        }
      }
      else
      {
        //Thread 2 : Run the graphics and gui
        while(true == scl::CDatabase::getData()->running_)
        {
          const timespec ts = {0, 15000000};//Sleep for 15ms
          nanosleep(&ts,NULL);
          glutMainLoopEvent(); //Update the graphics
          gr_ctr++;
        }
      }
    }//End of threaded region
    scl::CDatabase::getData()->running_ = true;
  }

  void CSclAppTestTask::runMainLoop(double arg_exec_sim_time)
  {
    double start_sim_time = sutil::CSystemClock::getSimTime();

    while(true == scl::CDatabase::getData()->running_)
    {
      stepMySimulation();

      if(arg_exec_sim_time<sutil::CSystemClock::getSimTime() - start_sim_time)
      { scl::CDatabase::getData()->running_ = false; }

      static int gr_skip_ctr=0;
      if(gr_skip_ctr<=500)
      { gr_skip_ctr++; continue; }
      gr_skip_ctr = 0;
      glutMainLoopEvent(); //Update the graphics
      gr_ctr++;
    }
    scl::CDatabase::getData()->running_ = true;
  }

  void CSclAppTestTask::runSimulation(double arg_exec_sim_time)
  {
    double start_sim_time = sutil::CSystemClock::getSimTime();

    while(1)
    {
      stepMySimulation();
      if(arg_exec_sim_time<sutil::CSystemClock::getSimTime() - start_sim_time)
      { break; }
    }
  }

  double CSclAppTestTask::runSimulationClockCtrl(double arg_exec_sim_time)
  {
    double start_sim_time = sutil::CSystemClock::getSimTime();
    double ctrl_time=0.0;

    while(1)
    {
      stepMySimulationClockCtrl(ctrl_time);
      if(arg_exec_sim_time<sutil::CSystemClock::getSimTime() - start_sim_time)
      { break; }
    }

    return ctrl_time;
  }

  scl::sBool CSclAppTestTask::initMyController(int argc, char** argv,
      const std::string & arg_op_link_name1,
      const std::string & arg_op_link_name2)
  {
    bool flag;
    try
    {
      /** Initialize Single Control Task */
      ctrl = (scl::CControllerMultiTask*) robot.getControllerCurrent();
      if(S_NULL == ctrl)
      { throw(std::runtime_error("Could not get current controller"));  }

      //Initialize the first op point controller
      op_link_name = arg_op_link_name1;
      op_link_set = true;
      db->s_gui_.ui_point_[0]<<0,0.1,0; //Ctrl tracks this control point.
      tsk = (scl::CTaskOpPos*)(ctrl->getTask(op_link_name));
      if(S_NULL == tsk)
      { throw(std::runtime_error("Could not get specified task"));  }
      tsk_ds = dynamic_cast<scl::STaskOpPos*>(tsk->getTaskData());

      /** Render a sphere at the op-point task's position */
      flag = chai_gr.addSphereToRender(robot_name,tsk_ds->link_ds_->name_,tsk_ds->pos_in_parent_);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at op task"));  }

      //If second op point name was passed, use it.
      if(std::string("null")!=arg_op_link_name2)
      {
        op_link2_name = arg_op_link_name2;
        op_link2_set = true;
        db->s_gui_.ui_point_[1]<<0,-0.1,0; //Ctrl2 tracks this control point.
        tsk2 = (scl::CTaskOpPos*)(ctrl->getTask(op_link2_name));
        if(S_NULL == tsk2)
        { throw(std::runtime_error("Could not get specified task"));  }
        tsk2_ds = dynamic_cast<scl::STaskOpPos*>(tsk2->getTaskData());

        /** Render a sphere at the op-point task's position */
        flag = chai_gr.addSphereToRender(robot_name,tsk2_ds->link_ds_->name_,tsk2_ds->pos_in_parent_);
        if(false == flag) { throw(std::runtime_error("Could not add sphere at op task"));  }
      }

      return true;
    }
    catch(std::exception &e)
    { std::cout<<"\nCSclAppTestTask::initMyController() : "<<e.what(); }
    return false;
  }

  void  CSclAppTestTask::stepMySimulation()
  {
    sutil::CSystemClock::tick(db->sim_dt_);//Tick the clock.
    double t = sutil::CSystemClock::getSimTime();
    t = (t - static_cast<int>(t))/2.0;//Between 0 and 0.1

    db->s_gui_.ui_point_[0](0) = t; //Keep the robot moving
    tsk->setGoalPos(db->s_gui_.ui_point_[0]); //Interact with the gui

    if(op_link2_set)//Use only if the second task was also initialized.
    {
      db->s_gui_.ui_point_[1](0) = -t; //Keep the robot moving
      tsk2->setGoalPos(db->s_gui_.ui_point_[1]);
    }

    if(ctrl_ctr%100 == 0)           //Update dynamics at a slower rate
    { robot.computeDynamics();  }
    robot.computeServo();           //Run the servo loop
    robot.integrateDynamics();      //Integrate system

    ctrl_ctr++;//Increment the counter for dynamics computed.
  }

  void  CSclAppTestTask::stepMySimulationClockCtrl(
      /** Increments the controller time */
      double& arg_ctrl_time)
  {
    sutil::CSystemClock::tick(db->sim_dt_);//Tick the clock.
    double t = sutil::CSystemClock::getSimTime();
    t = (t - static_cast<int>(t))/2.0;//Between 0 and 0.1

    db->s_gui_.ui_point_[0](0) = t; //Keep the robot moving
    tsk->setGoalPos(db->s_gui_.ui_point_[0]); //Interact with the gui

    if(op_link2_set)//Use only if the second task was also initialized.
    {
      db->s_gui_.ui_point_[1](0) = -t; //Keep the robot moving
      tsk2->setGoalPos(db->s_gui_.ui_point_[1]);
    }

    double tinit = sutil::CSystemClock::getSysTime();
    if(ctrl_ctr%100 == 0)           //Update dynamics at a slower rate
    { robot.computeDynamics();  }
    robot.computeServo();           //Run the servo loop
    t = sutil::CSystemClock::getSysTime();

    arg_ctrl_time += (t-tinit);

    robot.integrateDynamics();      //Integrate system

    ctrl_ctr++;//Increment the counter for dynamics computed.
  }

}
#endif /* CSCL_APP_TASK_HPP_ */
