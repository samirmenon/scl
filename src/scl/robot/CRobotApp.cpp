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
/* \file CRobotApp.cpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CRobotApp.hpp"


#include <scl/Singletons.hpp>
#include <scl/Init.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>

#ifdef GRAPHICS_ON
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <GL/freeglut.h>
#include <chai3d.h>
#endif
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <scl/util/HelperFunctions.hpp>

#include <sutil/CSystemClock.hpp>
#include <sutil/CRegisteredCallbacks.hpp>

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <string>

//String tokenizer uses std::algorithm
#include <iostream>
#include <algorithm>
#include <iterator>

#include <Eigen/Core>

#include <stdio.h>
#include <ncurses.h>

namespace scl
{
  /************************************************************************/
  /****************       All the function implementations   **************/
  /************************************************************************/

  CRobotApp::CRobotApp() :
      db_(NULL),
      rob_ds_(NULL),
      rob_io_ds_(NULL),
      dyn_scl_sp_(NULL),
      dyn_scl_(NULL),
      ctrl_ctr_(0),
      t_start_(0.0),
      t_end_(0.0)
#ifdef GRAPHICS_ON
      ,
      gr_ctr_(0),
      gr_frm_skip_(0),
      gr_frm_ctr_(0)
#endif
  {}

  scl::sBool CRobotApp::init(const std::vector<std::string>& argv)
  {
    bool flag;
    if(4 > argv.size())
    {
      std::cout<<"\nApplication requires the following command line input:"
          <<"\n./<executable> <robot_config_file_name.xml> <robot_name> <controller_name> <app-specific-arguments>";
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

        //For parsing controllers
        flag = scl::init::registerNativeDynamicTypes();
        if(false ==flag)  { throw(std::runtime_error("Could not register native dynamic types"));  }

        //For parsing your (custom) tasks
        flag = registerCustomDynamicTypes();
        if(false ==flag)  { throw(std::runtime_error("Could not register custom dynamic types"));  }

        //Get going..
        std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

        //The file to be opened, wrt the current working directory path
        std::string tmp_infile = argv[1];
        std::cout<<"Running scl task controller for input file: "<<tmp_infile;

        /******************************File Parsing************************************/
        scl::CParserScl tmp_lparser;//Use the scl tinyxml parser
        flag = scl_registry::parseEverythingInFile(tmp_infile,
            &tmp_lparser,&robots_parsed_,
#ifdef GRAPHICS_ON
            &graphics_parsed_);
#else
            NULL);
#endif
        if((false == flag) || (robots_parsed_.size()<=0)
#ifdef GRAPHICS_ON
            || (graphics_parsed_.size()<=0)
#endif
            )
        { throw(std::runtime_error("Could not parse the file"));  }

        robot_name_ = argv[2];
        if(!scl_util::isStringInVector(robot_name_,robots_parsed_))
        { throw(std::runtime_error("Could not find passed robot name in file"));  }

        rob_ds_ = scl::CDatabase::getData()->s_parser_.robots_.at(robot_name_);
        if(NULL == rob_ds_)
        { throw(std::runtime_error("Could not find registered robot data struct in the database"));  }

        /******************************TaoDynamics************************************/
        dyn_scl_sp_ = new scl_ext::CDynamicsSclSpatial();
        flag = dyn_scl_sp_->init(*rob_ds_);
        if(false == flag) { throw(std::runtime_error("Could not initialize physics simulator"));  }

        /******************************Scl Dynamics************************************/
        dyn_scl_ = new scl::CDynamicsScl();
        flag = dyn_scl_->init(*rob_ds_);
        if(false == flag) { throw(std::runtime_error("Could not initialize dynamics algorithms"));  }

        /******************************Shared I/O Data Structure************************************/
        rob_io_ds_ = db_->s_io_.io_data_.at(robot_name_);
        if(S_NULL == rob_io_ds_)
        { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

        /******************************ChaiGlut Graphics************************************/
#ifdef GRAPHICS_ON
        scl::SGraphicsParsed *gr_parsed = db_->s_parser_.graphics_worlds_.at(graphics_parsed_[0]);
        scl::SGraphicsChai *chai_ds = db_->s_gui_.chai_data_.at(graphics_parsed_[0]);
        flag = chai_gr_.initGraphics(gr_parsed,chai_ds);
        if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

        flag = chai_gr_.addRobotToRender(rob_ds_,rob_io_ds_);
        if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

        if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_parsed_[0], &chai_gr_))
        { throw(std::runtime_error("Glut initialization error")); }

        //Set up the graphics refresh interval defaults (for multi-threaded mode)
        timespec t = {0, 15000000};
        ts_.tv_nsec = t.tv_nsec;
        ts_.tv_sec = t.tv_sec;

        //Set up the graphics frames to skip per controller update (for single threaded mode).
        gr_frm_skip_ = 500;
        gr_frm_ctr_ = 0;
#endif

        /**********************Initialize Robot Dynamics and Controller*******************/
        flag = robot_.initFromDb(robot_name_,dyn_scl_,dyn_scl_sp_);//Note: The robot deletes these pointers.
        if(false == flag) { throw(std::runtime_error("Could not initialize robot"));  }

        ctrl_name_ = argv[3];
        flag = robot_.setControllerCurrent(ctrl_name_);
        if(false == flag) { throw(std::runtime_error("Could not initialize robot's controller"));  }

        /**********************Initialize Misc. Options *******************/
        //Ctr in array of args_parsed = (args_parsed - 1)
        //So ctr for un-parsed arg = (args_parsed - 1) + 1
        scl::sUInt args_ctr = 4, ui_points_used=0;

        /** Test whether Controller is a task controller. If so, parse task options. */
        CControllerMultiTask* tmp_task_ctrl = dynamic_cast<scl::CControllerMultiTask*> (robot_.getControllerCurrent());
        bool flag_is_task_controller = false;
        if(S_NULL != tmp_task_ctrl)
        { flag_is_task_controller = true;  }

        // Check that we haven't finished parsing everything
        while(args_ctr < argv.size())
        {
          if ("-p" == argv[args_ctr])
          {//Start simulation paused
            if(S_NULL == scl::CDatabase::getData())
            { throw(std::runtime_error("Database not intialized. Can't pause simulation."));  }
            scl::CDatabase::getData()->pause_ctrl_dyn_ = true;
            args_ctr++;
          }
          else if ("-l" == argv[args_ctr])
          {// We know the next argument *should* be the log file's name
            if(args_ctr+1 < argv.size())
            {
              flag = robot_.setLogFile(argv[args_ctr+1]);
              if(false == flag) { throw(std::runtime_error("Could not set up log file"));  }
              args_ctr+=2;
            }
            else
            { throw(std::runtime_error("Specified -l flag but did not specify log file"));  }
          }
          else if (("-com" == argv[args_ctr]) || ("-op" == argv[args_ctr]) || ("-ui" == argv[args_ctr]))
          {// We know the next argument *should* be the op pos task's name
            if(false == flag_is_task_controller)
            {
              std::cout<<"\nUI points are only supported for task controllers";
              args_ctr+=2; continue;
            }
            if(args_ctr+1 < argv.size())
            {
              if(ui_points_used >= SCL_NUM_UI_POINTS)
              {
                std::cout<<"\nThe keyboard can only support "<<SCL_NUM_UI_POINTS<<" ui points. Can't add: "
                    << argv[args_ctr]<<" "<< argv[args_ctr+1];
                args_ctr+=2; continue;
              }

              //Initialize the ui ctrl point task
              // NOTE : There may be many ui point tasks, so we create a tmp, initialize it
              //        and store it with the others (if any) in the taskvec_ui_ctrl_point_ a few lines below
              SUiCtrlPointData tmp_ui_ctrl_pt;
              tmp_ui_ctrl_pt.name_ = argv[args_ctr+1];
              tmp_ui_ctrl_pt.task_ = tmp_task_ctrl->getTask(tmp_ui_ctrl_pt.name_);
              if(S_NULL == tmp_ui_ctrl_pt.task_)
              { throw(std::runtime_error(std::string("Could not find task in the current controller: ")+tmp_ui_ctrl_pt.name_));  }
              flag = tmp_ui_ctrl_pt.task_->setGoalPos(Eigen::Vector3d::Zero());
              if(false == flag)
              { throw(std::runtime_error(std::string("Task doesn't support setting 3dim position for -op or -com: ")+tmp_ui_ctrl_pt.name_));  }
              tmp_ui_ctrl_pt.ui_pt_ = ui_points_used; ui_points_used++;
              tmp_ui_ctrl_pt.has_been_init_ = true;

#ifdef GRAPHICS_ON
              /** Render a sphere at the op-point task's position */
              flag = chai_gr_.addSphereToRender(Eigen::Vector3d::Zero(), tmp_ui_ctrl_pt.chai_pos_, 0.015);
              if(false == flag) { throw(std::runtime_error("Could not add sphere at ui ctrl task"));  }

              /** Render a sphere at the op-point task's position */
              flag = chai_gr_.addSphereToRender(Eigen::Vector3d::Zero(), tmp_ui_ctrl_pt.chai_pos_des_, 0.02);
              if(false == flag) { throw(std::runtime_error("Could not add sphere at ui ctrl desired pos"));  }

              if(NULL == tmp_ui_ctrl_pt.chai_pos_des_){ throw(std::runtime_error("Could not add sphere at ui ctrl desired pos"));  }

              //Make desired position red/orange
              chai3d::cMaterial tmp_mat;
              tmp_mat.setRedFireBrick();
              tmp_ui_ctrl_pt.chai_pos_des_->setMaterial(tmp_mat, false);
              tmp_mat.setGreenYellow();
              tmp_ui_ctrl_pt.chai_pos_->setMaterial(tmp_mat, false);
#endif

              //Add the initialized task to the vector
              // NOTE : We add it after initializing the graphics so that we don't end up with a junk
              //        task in the vector if the graphics fail.
              taskvec_ui_ctrl_point_.push_back(tmp_ui_ctrl_pt);

              args_ctr+=2; continue;
            }
            else
            { throw(std::runtime_error("Specified -op or -com flag but did not specify the task's name"));  }
          }
          else
          { args_ctr++; }
        }

        /**********************Initialize Single Control Task *******************/
        flag = initMyController(argv,4);
        if(false == flag)
        { throw(std::runtime_error("Could not initialize user's custom controller"));  }

        ctrl_ctr_=0;//Controller computation counter
#ifdef GRAPHICS_ON
        gr_ctr_=0;//Controller computation counter
#endif

        //Simulation loop.
        std::cout<<"\nStarting simulation. Integration timestep: "<<db_->sim_dt_<<std::flush;
        if(scl::CDatabase::getData()->pause_ctrl_dyn_)
        { std::cout<<"\nSimulation started in PAUSE mode. Press 'P' to resume.";  }

        return true;
      }
      catch(std::exception & e)
      {
        std::cout<<"\nCRobotApp::init() Failed : "<< e.what();
        terminate();
        return false;
      }
    }
  }

#ifdef GRAPHICS_ON
  scl::sBool CRobotApp::initGraphics(const timespec& arg_ts, const int arg_gr_frm_ctr)
  {
    if(0 > arg_gr_frm_ctr)
    { return false; }

    //Set up the graphics refresh interval (for multi-threaded mode)
    ts_.tv_nsec = arg_ts.tv_nsec;
    ts_.tv_sec = arg_ts.tv_sec;

    //Set up the graphics frames to skip per controller update (for single threaded mode).
    gr_frm_skip_ = arg_gr_frm_ctr;

    return true;
  }
#endif

  void CRobotApp::terminate()
  {
    /****************************Print Collected Statistics*****************************/
    std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
    std::cout<<"\nTotal Control Model and Servo Updates : "<<ctrl_ctr_;
#ifdef GRAPHICS_ON
    std::cout<<"\nTotal Graphics Updates                : "<<gr_ctr_;

    /******************************Termination************************************/
    bool flag = chai_gr_.destroyGraphics();
    if(false == flag) { std::cout<<"\nError deallocating graphics pointers"; } //Sanity check.
#endif

    std::cout<<"\nEnd Time:"<<sutil::CSystemClock::getSysTime();
    std::cout<<"\n********************************************\n"<<std::flush;
  }

  void CRobotApp::runMainLoopThreaded(const int& thread_id)
  {
    if(thread_id==1)
    {
      //Thread 1 : Run the simulation
      while(true == scl::CDatabase::getData()->running_)
      {
        if(scl::CDatabase::getData()->pause_ctrl_dyn_)
        { sleep(1); continue; }
        else
        { stepMySimulation(); }
      }
    }
    else
    {
#ifdef GRAPHICS_ON
      //Thread 0 : Run the graphics and gui
      while(true == scl::CDatabase::getData()->running_)
      {
        if(scl::CDatabase::getData()->pause_graphics_)
        { sleep(1); continue; }
        else
        {
          nanosleep(&ts_,NULL);
          glutMainLoopEvent(); //Update the graphics
          gr_ctr_++;
        }
      }
#endif
    }
  }

  void CRobotApp::runMainLoop()
  {
    while(true == scl::CDatabase::getData()->running_)
    {
      if(scl::CDatabase::getData()->pause_ctrl_dyn_)
      { sleep(1); continue; }
      else
      { stepMySimulation(); }

#ifdef GRAPHICS_ON
      if(scl::CDatabase::getData()->pause_graphics_)
      { continue; }
      else
      {
        if(gr_frm_ctr_<=gr_frm_skip_)
        { gr_frm_ctr_++; continue; }
        gr_frm_ctr_ = 0;
        glutMainLoopEvent(); //Update the graphics
        gr_ctr_++;
      }
#endif
    }
  }

  void CRobotApp::runConsoleShell()
  {
    bool flag, flag2;
    bool mode_char = false;

    using namespace std;

    std::vector<std::string> last_command;
    last_command.clear();

    char cmd_buf[1024];

    while(scl::CDatabase::getData()->running_)
    {
      while(scl::CDatabase::getData()->running_ &&
          mode_char == false)
      {
        std::cout<<"\nscl>> ";

        char* ret = fgets(cmd_buf,1024,stdin);
        if(ret!=cmd_buf)
        { std::cout<<"\nError in shell: Could not read stdin";  }

        //Split the string into string tokens
        std::istringstream iss(cmd_buf);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
            std::istream_iterator<std::string>(),
            std::back_inserter<std::vector<std::string> >(tokens));

        //Some error checks
        if(tokens.begin() == tokens.end())
        {//Pressing enter executes the last command
          if(0==last_command.size())
          { std::cout<<"Command not found"; continue; }
          else
          { tokens = last_command; }
        }

        last_command = tokens;//Set the last command

        if(std::string("exit") == *(tokens.begin()) )
        {
          scl::CDatabase::getData()->running_ = false;
          break;
        }
        else if(std::string("x") == *(tokens.begin()) )
        { mode_char = true; std::cout<<"  Switched to char mode and back"; continue; }
        else if(std::string("p") == *(tokens.begin()) )
        {//Toggle pause
          if(scl::CDatabase::getData()->pause_ctrl_dyn_)
          { std::cout<<"  Un-paused controller and dynamics engine"; }
          else { std::cout<<"  Paused controller and dynamics engine"; }

          scl::CDatabase::getData()->pause_ctrl_dyn_= !scl::CDatabase::getData()->pause_ctrl_dyn_;
          continue;
        }

        //Find and call the callback
        flag = sutil::callbacks::call<std::string, std::vector<std::string> >(
            *(tokens.begin()),tokens);
        if(false == flag) { std::cout<<"Command not found"; }
      }

      initscr(); //Start ncurses (the library for console io)
      raw();     //Disable line buffering (disable the enter key press)
      printw("  Char mode (press x to exit)\n\n>>");
      while(scl::CDatabase::getData()->running_ &&
          mode_char == true)
      {
        char input='1';
        input = getch();

        if('x' == tolower(input)){ mode_char = false; continue; }
        if('p' == tolower(input))
        {scl::CDatabase::getData()->pause_ctrl_dyn_= !scl::CDatabase::getData()->pause_ctrl_dyn_;
        continue;}

        flag2 = tolower(input) != input;
        flag = sutil::callbacks::call<char,bool,double>(tolower(input), flag2);
        if(false == flag)
        {
          flag = sutil::callbacks::call<char,bool,Eigen::VectorXd>(tolower(input), flag2);
          if(false == flag) { std::cout<<" NotFound "; }
        }
      }
      endwin(); //Stop ncurses
    }
  }

}
