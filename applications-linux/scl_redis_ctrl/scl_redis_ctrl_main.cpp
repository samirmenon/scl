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
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>

// Used for dynamic typing
#include <scl/robot/DbRegisterFunctions.hpp>
// Used to simplify the controller setup (has a few helper functions).
#include <scl/util/DatabaseUtils.hpp>


#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>

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


/** Basic data for reading from and writing to a redis database...
 * Makes it easy to keep track of things..*/
class SHiredisStruct_RobotCtrl{
public:
  redisContext *context_ = NULL;
  redisReply *reply_ = NULL;
  const char *hostname_ = "127.0.0.1";
  const int port_ = 6379;
  const timeval timeout_ = { 1, 500000 }; // 1.5 seconds
};


/** A sample application to render a physics simulation being run in scl.
 *
 * It will display the robot in an OpenGL graphics window. */
int main(int argc, char** argv)
{
  std::cout<<"\n*******************************************************\n";
  std::cout<<  "               SCL Redis Task Controller";
  std::cout<<"\n*******************************************************\n";
  std::cout<<"\n NOTE : This application assumes a default redis server is "
             <<"\n        running on the standard port (6379) and that "
             <<"\n        appropriate keys are set";

  // Set up terminate handler.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = handleExit;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  bool flag;
  if(argc < 5)
  {
    std::cout<<"\n The 'scl_redis_visualizer' application computes fgc commands and sends them (to a robot) using a redis io interface."
        <<"\n ERROR : Provided incorrect arguments. The correct input format is:"
        <<"\n   ./scl_redis_visualizer <file_name.xml> <robot_name> <controller_name> <control point> <optional: control point 2> ... \n";
    return 0;
  }
  else
  {
    try
    {
      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

      scl::SRobotParsed rds;     //Robot data structure.
      scl::SRobotIO rio;         //I/O data structure.
      scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
      scl::SGraphicsParsed rgr;  //Robot graphics data structure.
      scl::CParserScl p;         //This time, we'll parse the tree from a file.
      SHiredisStruct_RobotCtrl redis_ds; //The data structure we'll use for redis comm.

      scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
      scl::SControllerMultiTask rctr_ds; //A multi-task controller data structure
      scl::CControllerMultiTask rctr;    //A multi-task controller
      std::vector<scl::STaskBase*> rtasks;              //A set of executable tasks
      std::vector<scl::SNonControlTaskBase*> rtasks_nc; //A set of non-control tasks
      std::vector<scl::sString2> ctrl_params;        //Used to parse extra xml tags
      scl::STaskOpPos* rtask_hand;       //Will need to set hand desired positions etc.

      /******************************File Parsing************************************/
      std::string name_infile(argv[1]), name_robot(argv[2]), name_ctrl(argv[3]), name_task(argv[4]);
      std::cout<<"\nRunning scl_redis_ctrl for:"
          <<"\n Input file: "<<name_infile
          <<"\n      Robot: "<<name_robot
          <<"\n Controller: "<<name_ctrl
          <<"\n       Task: "<<name_task;

      /******************************Load Robot Specification************************************/
      //We will use a slightly more complex xml spec than the first few tutorials
      bool flag = p.readRobotFromFile(name_infile,"../../specs/",name_robot,rds);
      flag = flag && rio.init(rds);             //Set up the IO data structure
      flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
      flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
      if(false == flag){ return 1; }            //Error check.

      /******************************Set up Controller Specification************************************/
      // Read xml file info into task specifications.
      flag = p.readTaskControllerFromFile(name_infile,name_ctrl,rtasks,rtasks_nc,ctrl_params);
      flag = flag && rctr_ds.init(name_ctrl,&rds,&rio,&rgcm); //Set up the control data structure..

      // Tasks are initialized after we find their type with dynamic typing.
      flag = flag && scl_registry::registerNativeDynamicTypes();
      flag = flag && scl_util::initMultiTaskCtrlDsFromParsedTasks(rtasks,rtasks_nc,rctr_ds);
      flag = flag && rctr.init(&rctr_ds,&dyn_scl);  //Set up the controller (needs parsed data and a dyn object)
      if(false == flag){ return 1; }                //Error check.

      // Set up a special task
      rtask_hand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at(name_task)) );
      if(NULL == rtask_hand)  {return 1;}           //Error check

      // Compute the dynamics to begin with (flushes state across all matrices).
      rctr.computeDynamics();
      rctr.computeControlForces(); //Directly update io data structure for now...

      /******************************Redis Initialization************************************/
      std::cout<<"\n The default REDIS keys used are: ";
      std::cout<<"\n  scl::robot::"<<name_robot<<"::sensors::q";
      std::cout<<"\n  scl::robot::"<<name_robot<<"::sensors::dq";
      std::cout<<"\n  scl::robot::"<<name_robot<<"::actuators::fgc";


      char rstr[1024], rstr_robot_base[1024]; //For redis key formatting
      sprintf(rstr_robot_base, "scl::robot::%s",name_robot.c_str());

      redis_ds.context_= redisConnectWithTimeout(redis_ds.hostname_, redis_ds.port_, redis_ds.timeout_);
      if (redis_ds.context_ == NULL) { throw(std::runtime_error("Could not allocate redis context."));  }

      if(redis_ds.context_->err)
      {
        std::string err = std::string("Could not connect to redis server : ") + std::string(redis_ds.context_->errstr);
        redisFree(redis_ds.context_);
        throw(std::runtime_error(err.c_str()));
      }

      // PING server to make sure things are working..
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_,"PING");
      std::cout<<"\n\n Redis : Redis server is live. Reply to PING is, "<<redis_ds.reply_->str<<"\n";
      freeReplyObject((void*)redis_ds.reply_);

      // Check to see if all keys are available:
      bool flag_keys_available = false;
      while(false == flag_keys_available){
        flag_keys_available = true;//Hopefully.

        // REDIS IO : Get q key. If unavailable, throw an error..
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "GET %s::sensors::q", rstr_robot_base);
        if(redis_ds.reply_->len > 0)
        { std::stringstream ss; ss<<redis_ds.reply_->str; for(scl::sUInt i=0;i<rio.dof_;++i) ss>>rio.sensors_.q_(i);  }
        else {  flag_keys_available=false;  }
        freeReplyObject((void*)redis_ds.reply_);

        // REDIS IO : Get dq key. If unavailable, throw an error..
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "GET %s::sensors::dq", rstr_robot_base);
        if(redis_ds.reply_->len > 0)
        { std::stringstream ss; ss<<redis_ds.reply_->str; for(scl::sUInt i=0;i<rio.dof_;++i) ss>>rio.sensors_.dq_(i);  }
        else {  flag_keys_available=false;  }
        freeReplyObject((void*)redis_ds.reply_);

        if(flag_keys_available){  break;  } // Found both keys...

        std::cout<<"\n WARNING : Could not find {q, dq} redis keys for robot: "<<rstr_robot_base<<". Will wait for it...";
        const timespec ts = {0, 50000000};/*50ms sleep */ nanosleep(&ts,NULL);
      }

      // REDIS IO : Create fgc key and set it to zero (initial state)
      { std::stringstream ss; for(scl::sUInt i=0;i<rio.dof_;++i) ss<<rio.actuators_.force_gc_commanded_(i)<<" "; sprintf(rstr, "%s", ss.str().c_str());  }
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::actuators::fgc %s", rstr_robot_base,rstr, rstr);
      freeReplyObject((void*)redis_ds.reply_);

      // REDIS IO : Create xgoal key and set it to zero (initial state)
      { std::stringstream ss; for(scl::sUInt i=0;i<3;++i) ss<<rtask_hand->x_(i)<<" "; sprintf(rstr, "%s", ss.str().c_str());  }
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::traj::xgoal %s", rstr_robot_base,rstr, rstr);
      freeReplyObject((void*)redis_ds.reply_);

      std::cout<<"\n ** To monitor Redis messages, open a redis-cli and type 'monitor' **";

      /****************************** Control Loop************************************/
      Eigen::Vector3d xgoal(0,0,0);
      int enable_fgc_command = 0;
      while(flag_running)
      {
        flag_keys_available = true;
        /* ************************************ READ FROM REDIS ************************** */
        // REDIS IO : Get q
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "GET %s::sensors::q", rstr_robot_base);
        if(redis_ds.reply_->len > 0)
        { std::stringstream ss; ss<<redis_ds.reply_->str; for(scl::sUInt i=0;i<rio.dof_;++i) ss>>rio.sensors_.q_(i);  }
        else {  flag_keys_available = false;  }
        freeReplyObject((void*)redis_ds.reply_);

        // REDIS IO : Get dq
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "GET %s::sensors::dq", rstr_robot_base);
        if(redis_ds.reply_->len > 0)
        { std::stringstream ss; ss<<redis_ds.reply_->str; for(scl::sUInt i=0;i<rio.dof_;++i) ss>>rio.sensors_.dq_(i);  }
        else { flag_keys_available=false;  }
        freeReplyObject((void*)redis_ds.reply_);

        // REDIS IO : Get xgoal
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "GET %s::traj::xgoal", rstr_robot_base);
        if(redis_ds.reply_->len > 0)
        { std::stringstream ss; ss<<redis_ds.reply_->str; for(scl::sUInt i=0;i<3;++i) ss>>xgoal(i);  }
        else { flag_keys_available=false;  }
        freeReplyObject((void*)redis_ds.reply_);

        // REDIS IO : Get fgc_enabled key : fgc_command_enabled
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "GET %s::fgc_command_enabled", rstr_robot_base);
        if(redis_ds.reply_->len >0)
        { std::stringstream ss; ss<<redis_ds.reply_->str; ss>>enable_fgc_command;  }
        else { flag_keys_available=false;  }

        if(false == flag_keys_available){
          std::cout<<"\n WARNING : Could not find {q, dq} redis keys for robot: "<<rstr_robot_base<<". Will wait for it...";
          const timespec ts = {0, 50000000};/*50ms sleep */ nanosleep(&ts,NULL);
          continue;
        }

        /* ************************************ COMPUTE CONTROL FORCES ************************** */
        // Compute control forces (note that these directly have access to the io data ds).
        rctr.computeDynamics();

        // If the torque command is enabled, use the latest goal position.
        if(1 == enable_fgc_command) { rtask_hand->x_goal_ = xgoal;  }
        // If the torque command is not enabled, move the goal point to the current position..
        else {
          rtask_hand->x_goal_ = rtask_hand->x_;

          { std::stringstream ss; for(scl::sUInt i=0;i<3;++i) ss<<rtask_hand->x_(i)<<" "; sprintf(rstr, "%s", ss.str().c_str());  }
          redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::traj::xgoal %s", rstr_robot_base,rstr, rstr);
          freeReplyObject((void*)redis_ds.reply_);
        }

        rctr.computeControlForces(); //Directly update io data structure for now...

        /* ************************************ WRITE TO REDIS ************************** */
        // REDIS IO : Set fgc_commanded
        { std::stringstream ss; for(scl::sUInt i=0;i<rio.dof_;++i) ss<<rio.actuators_.force_gc_commanded_(i)<<" "; sprintf(rstr, "%s", ss.str().c_str());  }
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::actuators::fgc %s",rstr_robot_base,rstr);
        freeReplyObject((void*)redis_ds.reply_);

        // Optional, sleep a bit.
//        const timespec ts = {0, 20000000};/*20ms sleep : ~50Hz update*/
//        nanosleep(&ts,NULL);
      }

      /******************************Exit Gracefully************************************/
      // Send Zero torques to redis
      { std::stringstream ss; for(scl::sUInt i=0;i<rio.dof_;++i) ss<<0.0<<" "; sprintf(rstr, "%s", ss.str().c_str());  }
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::actuators::fgc %s",rstr_robot_base,rstr);
      freeReplyObject((void*)redis_ds.reply_);

      // Give the controller some time to read the zero torques.
      const timespec ts = {0, 20000000};/*20ms sleep : ~50Hz update*/
      nanosleep(&ts,NULL);

      // REDIS IO : Disable the fgc enabled key and then exit : fgc_command_enabled
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::fgc_command_enabled 0", rstr_robot_base);
      if(redis_ds.reply_->len <= 0)
      { std::cout<<"\n\n Failed to exit in a clean manner. The robot might misbehave. Be careful!";  }
      else
      { std::cout<<"\n\nExecuted Successfully"; }
      std::cout<<"\n**********************************\n"<<std::flush;

      return 0;
    }
    catch(std::exception & e)
    {
      std::cout<<"\nSCL Failed: "<< e.what();
      std::cout<<"\n*************************\n";
      return 1;
    }
  }
}
