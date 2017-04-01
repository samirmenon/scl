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
/* \file scl_redis_ctrl_main.cpp
 *
 *  Created on: Jul 10, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/scl.hpp>

#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>
//Redis
#include <hiredis/hiredis.h>

//Standard includes
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

// Handle ctrl+c
bool flag_running=true;
void handleExit(int s) { flag_running=false; }

/** A sample application to control a robot in scl.
 *
 * Reads from redis :
 *   - q
 *   - dq
 *   - x_task
 *   - dx_task
 * Writes to redis :
 *   - fgc
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n*******************************************************\n";
  std::cout<<  "               SCL Redis Task Controller";
  std::cout<<"\n*******************************************************\n";
  std::cout<<"\n 'scl_redis_ctrl'"
           <<"\n\n Application computes fgc commands given an task goal input"
           <<"\n and sets the appropriate redis key.\n";
  std::cout<<"\n NOTE : This application assumes a default redis server is "
             <<"\n        running on the standard port (6379) and that "
             <<"\n        appropriate keys are set"
             <<"\n\n Required keys include setting ui keys (these are usually "
             <<"\n set by the gui; if you don't have a gui, set them yourself)";
  std::cout<<"\n*******************************************************\n\n";

  // Set up terminate handler.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = handleExit;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  bool flag;
  if(argc < 4)
  {
    std::cout<<"\n The 'scl_redis_ctrl' application computes fgc commands and sends them (to a robot) using a redis io interface."
        <<"\n ERROR : Provided incorrect arguments. The correct input format is:"
        <<"\n   ./scl_redis_ctrl <file_name.xml> <robot_name> <controller_name>  -op <task0> -op <task1> ... \n";
    return 0;
  }
  else
  {
    try
    {
      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }
      if(false == scl::init::registerNativeDynamicTypes()){ throw(std::runtime_error("Could not set up dynamic types"));  }

      // Parsing data structures
      scl::CParserScl p;         //This time, we'll parse the tree from a file.
      scl::SCmdLineOptions_OneRobot rcmd; // For parsing command line options
      char tmp_message[1024];    // Use with the redis comm..

      // Robot data structures
      scl::SRobotParsed rds;     //Robot data structure.
      scl::SRobotIO rio;         //I/O data structure.
      scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
      scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...

      // Controller data structures..
      scl::SControllerMultiTask rctr_ds; //A multi-task controller data structure
      scl::CControllerMultiTask rctr;    //A multi-task controller
      std::vector<scl::STaskBase*> rtasks;              //A set of executable tasks
      std::vector<scl::SNonControlTaskBase*> rtasks_nc; //A set of non-control tasks
      std::vector<scl::sString2> ctrl_params;        //Used to parse extra xml tags

      // Data structures that connect the user interface (or trajectory gen) to the controller..
      scl::STaskOpPos *rtask_ds_[SCL_NUM_UI_POINTS];    // Control task (support as many as there are ui points)
      int n_tasks=0;             //The number of tasks used in this controller (with the ui/redis update)

      /******************************Cmd Arg Parsing************************************/
      flag = scl::cmdLineArgReaderOneRobot(argc,argv,rcmd);
      if(false == flag) { throw(std::runtime_error("Could not parse command line args")); }
      // Find the number of ui tasks...
      n_tasks = rcmd.name_tasks_.size();

      std::cout<<"\nRunning scl_redis_muscle_ctrl for:"
          <<"\n Input file: "<<rcmd.name_file_config_
          <<"\n      Robot: "<<rcmd.name_robot_
          <<"\n Controller: "<<rcmd.name_ctrl_;
      for(int i=0; i<n_tasks; ++i)
      { std::cout<<"\n      Task("<<i<<"): "<<rcmd.name_tasks_[i]; }

      /******************************Load Robot Specification************************************/
      flag = p.readRobotFromFile(rcmd.name_file_config_,"../../specs/",rcmd.name_robot_,rds);
      flag = flag && rio.init(rds);             //Set up the IO data structure
      flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
      flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
      if(false == flag){ throw(std::runtime_error("Could not initialize robot spec")); }            //Error check.

      /******************************Set up Controller Specification************************************/
      // Read xml file info into task specifications.
      flag = p.readTaskControllerFromFile(rcmd.name_file_config_,rcmd.name_ctrl_,rtasks,rtasks_nc,ctrl_params);
      flag = flag && rctr_ds.init(rcmd.name_ctrl_,&rds,&rio,&rgcm); //Set up the control data structure..

      // Tasks are initialized after we find their type with dynamic typing.
      flag = flag && scl::init::initMultiTaskCtrlDsFromParsedTasks(rtasks,rtasks_nc,rctr_ds);
      flag = flag && rctr.init(&rctr_ds,&dyn_scl);  //Set up the controller (needs parsed data and a dyn object)
      if(false == flag){ throw(std::runtime_error("Could not initialize controller")); }            //Error check.

      // Set up the tasks that will receive goal positions etc.
      if(0 >= n_tasks)
      { std::cout<<"\n WARNING : Did not provide any tasks in the command line arguments: -t/-op/-ui <task name>"; }
      if(SCL_NUM_UI_POINTS < n_tasks)
      {
        sprintf(tmp_message,"Too many tasks provided. Can support at most : %d", SCL_NUM_UI_POINTS);
        throw(std::runtime_error( tmp_message ));
      }

      for(int i=0; i< n_tasks; ++i)
      {
        // Find the task data structure
        rtask_ds_[i] = dynamic_cast<scl::STaskOpPos*>(*rctr_ds.tasks_.at(rcmd.name_tasks_[i]));
        if(NULL == rtask_ds_[i])
        { throw(std::runtime_error(std::string("Did not find the task : ") + rcmd.name_tasks_.at(i))); }
      }

      // Compute the dynamics to begin with (flushes state across all matrices).
      rctr.computeDynamics();
      rctr.computeControlForces(); //Directly update io data structure for now...

      /******************************Redis Initialization************************************/
      scl::CIORedis ioredis;
      scl::SIORedis ioredis_ds;
      flag = ioredis.connect(ioredis_ds,false);
      if(false == flag)
      { throw(std::runtime_error( std::string("Could not connect to redis server : ") + std::string(ioredis_ds.context_->errstr) ));  }

      // Set up the keys here so we don't have to run sprintfs in the while loop...
      char rstr_robot_base[SCL_MAX_REDIS_KEY_LEN_CHARS], 
        rstr_q[SCL_MAX_REDIS_KEY_LEN_CHARS], rstr_dq[SCL_MAX_REDIS_KEY_LEN_CHARS],
        rstr_fgc[SCL_MAX_REDIS_KEY_LEN_CHARS],
      	rstr_fgcenab[SCL_MAX_REDIS_KEY_LEN_CHARS];
      int enable_fgc_command = 0;

      sprintf(rstr_robot_base, "scl::robot::%s",rcmd.name_robot_.c_str());
      sprintf(rstr_fgc, "%s::actuators::fgc", rstr_robot_base);
      sprintf(rstr_fgcenab, "%s::fgc_command_enabled", rstr_robot_base);
      sprintf(rstr_q, "%s::sensors::q", rstr_robot_base);
      sprintf(rstr_dq, "%s::sensors::dq", rstr_robot_base);

      char rstr_ui_master[SCL_MAX_REDIS_KEY_LEN_CHARS];
      sprintf(rstr_ui_master, "scl::robot::%s::ui::master",rcmd.name_robot_.c_str());

      // Add strings for the special (data) ui vars
      char rstr_ui_pt[SCL_NUM_UI_POINTS][SCL_MAX_REDIS_KEY_LEN_CHARS];
      for(int i=0; i<SCL_NUM_UI_POINTS;++i)
      { sprintf(rstr_ui_pt[i], "scl::robot::%s::ui::point::%d", rcmd.name_robot_.c_str(), i); }

      // Check to see if all keys are available:
      flag = false;
      while(false == flag && flag_running){
        flag = true;//Hopefully.

        // REDIS IO : Get q and dq keys. If unavailable, throw an error..
        flag = flag && ioredis.get(ioredis_ds, rstr_q, rio.sensors_.q_);
        flag = flag && ioredis.get(ioredis_ds, rstr_dq, rio.sensors_.dq_);
        if(flag){  break;  } // Found both keys and so flag is still true...

        std::cout<<"\n WARNING : Could not find {q, dq} redis keys for robot: "<<rstr_robot_base<<". Will wait for it...";
        std::cout<<"\n    q : "<<rstr_q;
        std::cout<<"\n   dq : "<<rstr_dq;
        const timespec ts = {0, 500000000};/*.5s sleep */ nanosleep(&ts,NULL);
      }

      // Now that we have the actual q and dq, let's update control matrices..
      rctr.computeDynamics(); // Update all the positions given that we have q values now.
      rctr.computeControlForces(); //Directly update io data structure for now...

      // REDIS IO : Create fgc key and set it to zero (initial state)
      rio.actuators_.force_gc_commanded_.setZero(rio.dof_);
      flag = ioredis.set(ioredis_ds, rstr_fgc, rio.actuators_.force_gc_commanded_);

      // Reset all task positions
      for(int j=0; j< n_tasks; ++j)
      { rtask_ds_[j]->x_goal_ = rtask_ds_[j]->x_; }

      if(false == flag){ throw(std::runtime_error("Could not set up fgc and/or xgoal key(s) in the redis server")); }  //Error check.

      // Get the
      flag = ioredis.get(ioredis_ds, rstr_fgcenab, enable_fgc_command);
      if(false == flag)
      {  //Error check.
        std::cout<<"\n WARNING : Could not find fgc enabled key in the redis server; setting to zero.";
        enable_fgc_command = 0;
        ioredis.set(ioredis_ds, rstr_fgcenab, enable_fgc_command);
      }

      std::cout<<"\n ** Started: To monitor Redis messages, open a redis-cli and type 'monitor' **\n"<<std::flush;

      int enable_fgc_command_pre = 0;
      if(0 == enable_fgc_command)
      {
        std::cout<<"\n ** NOTE: The '"<<rstr_fgcenab<<"' key needs to be '1' before the controller torques are used **\n"<<std::flush;
        enable_fgc_command_pre = 1;// Needs to the the complement of enable_fgc_command
      }

      /****************************** Control Loop************************************/
      while(flag_running)
      {
        flag = true;
        /* ************************************ READ FROM REDIS ************************** */
        // REDIS IO : Get q and dq keys. If unavailable, throw an error..
        flag = flag && ioredis.get(ioredis_ds, rstr_q, rio.sensors_.q_);
        flag = flag && ioredis.get(ioredis_ds, rstr_dq, rio.sensors_.dq_);
        flag = flag && ioredis.get(ioredis_ds, rstr_fgcenab, enable_fgc_command);

        if(false == flag){
          std::cout<<"\n WARNING : Could not find {q, dq, fgcenab} redis keys for robot: "<<rstr_robot_base<<". Will wait for it...";
          const timespec ts = {0, 50000000};/*50ms sleep */ nanosleep(&ts,NULL);
          continue;
        }

        /* ************************************ COMPUTE CONTROL FORCES ************************** */
        // Compute control forces (note that these directly have access to the io data ds).
        rctr.computeDynamics();

        // If the torque command is enabled, use the latest goal position.
        if(1 == enable_fgc_command)
        {
          if(enable_fgc_command_pre!=enable_fgc_command)
          { // Just flipped flag. Relinquish the UI master position (give the gui or something write control over the keys)
            ioredis.del(ioredis_ds,rstr_ui_master);
            std::cout<<"\n Relinquishing the UI master position. Will now get goal positions from redis";
          }

          // FGC Enabled : Read goal positions
          for(int i=0; i< n_tasks; ++i)
          {
            flag = ioredis.get(ioredis_ds, rstr_ui_pt[i], rtask_ds_[i]->x_goal_);
            if(false == flag)
            {
              std::cout<<"\n ERROR : Could not get xgoal for a task. Resetting all x_goal values to x."
                  <<"\n Check key : "<<rstr_ui_pt[i];
              // Reset all task positions
              for(int j=0; j< n_tasks; ++j)
              { rtask_ds_[j]->x_goal_ = rtask_ds_[j]->x_; }
              break;
            }
          }
        }
        // If the torque command is not enabled, move the goal point to the current position..
        else
        {
          if(enable_fgc_command_pre!=enable_fgc_command)
          { // Just flipped flag. Become the UI master (obtain write control over the keys)
            ioredis.set(ioredis_ds,rstr_ui_master, std::string("scl_redis_ctrl_main::")+rcmd.id_time_created_str_);
            std::cout<<"\n Assuming the UI master position. Will now set goal positions to present positions in redis";
          }

          // FGC Disabled : Write goal positions (helps remain smooth; no jerky return to goal when fgc is enabled again..)
          for(int i=0; i< n_tasks; ++i)
          {// Move the goal position to the actual position and reset it in redis...
            rtask_ds_[i]->x_goal_ = rtask_ds_[i]->x_;
            flag = ioredis.set(ioredis_ds, rstr_ui_pt[i], rtask_ds_[i]->x_goal_);
          }

          rio.actuators_.force_gc_commanded_.setZero(rio.dof_);
          flag = flag && ioredis.set(ioredis_ds, rstr_fgc, rio.actuators_.force_gc_commanded_);

          if(false == flag) { std::cout<<"\n WARNING : Could not reset xgoal or fgc. Check robot before re-enabling fgc commands"; }
        }

        rctr.computeControlForces(); //Directly update io data structure for now...

        /* ************************************ WRITE TO REDIS ************************** */
        // REDIS IO : Set fgc_commanded
        flag = ioredis.set(ioredis_ds, rstr_fgc, rio.actuators_.force_gc_commanded_);
        if(false == flag){  std::cout<<"\n ERROR : Could not set force gc and/or xgoal. Probably serious. Consider aborting."; }

        // Optional, sleep a bit.
//        const timespec ts = {0, 20000000};/*20ms sleep : ~50Hz update*/
//        nanosleep(&ts,NULL);

        // Need to refresh the enable command cycle..
        enable_fgc_command_pre = enable_fgc_command;
      }// ************** END OF CONTROL LOOP!!!

      /******************************Exit Gracefully************************************/
      // Send Zero torques to redis
      rio.actuators_.force_gc_commanded_.setZero(rio.dof_);
      ioredis.set(ioredis_ds, rstr_fgc, rio.actuators_.force_gc_commanded_);

      std::cout<<"\n\n Executed Successfully";
      std::cout<<"\n**********************************\n"<<std::flush;

      return 0;
    }
    catch(std::exception & e)
    {
      std::cout<<"\n\n ERROR: \n SCL Failed: "<< e.what();
      std::cout<<"\n*************************\n";
      return 1;
    }
  }
}
