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
/* \file scl_redis_sim.cpp
 *
 *  Created on: Jul 10, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl headers used
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>

//sutil clock.
#include <sutil/CSystemClock.hpp>

// 3rd party libs
#include <Eigen/Dense>
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
bool flag_sim_enabled=true;
void handleExit(int s) { flag_sim_enabled=false; }

/** Basic data for reading from and writing to a redis database...
 * Makes it easy to keep track of things..*/
class SHiredisStruct_RobotSim{
public:
  redisContext *context_ = NULL;
  redisReply *reply_ = NULL;
  const char *hostname_ = "127.0.0.1";
  const int port_ = 6379;
  const timeval timeout_ = { 1, 500000 }; // 1.5 seconds
};

/**
 * A sample server application to simulate the physics of a robot and provide a
 * redis interface similar to the standard scl robot driver(s) using redis.
 */
int main(int argc, char** argv)
{
  std::cout<<"\n*******************************************************\n";
  std::cout<<  "              SCL Redis Robot Simulator";
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
  if((argc != 2)&&(argc != 3))
  {
    std::cout<<"\n The 'scl_redis_sim' application uses scl to simulate the physics of a robot with redis io."
        <<"\n ERROR : Provided incorrect arguments. The correct input format is:"
        <<"\n   ./scl_redis_sim <file_name.xml> <optional: robot_name.xml>"
        <<"\n If a robot name isn't provided, the first one from the xml file will be used.\n";
    return 0;
  }
  else
  {
    try
    {
      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

      scl::SRobotParsed rds;     //Robot data structure....
      scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
      scl::SRobotIO rio;         //I/O data structure
      scl_ext::CDynamicsSclSpatial dyn_scl_sp; //Robot physics integrator...
      scl::CParserScl p;         //This time, we'll parse the tree from a file...
      SHiredisStruct_RobotSim redis_ds; //The data structure we'll use for redis comm.

      double sim_dt = 0.001;     //Simulation timestep..

      /******************************File Parsing************************************/
      std::string tmp_infile(argv[1]);
      std::cout<<"\nRunning scl_redis_sim for input file: "<<tmp_infile;

      std::string robot_name;
      if(argc==2)
      {//Use the first robot spec in the file if one isn't specified by the user.
        std::vector<std::string> robot_names;
        flag = p.listRobotsInFile(tmp_infile,robot_names);
        if(false == flag) { throw(std::runtime_error("Could not read robot names from the file"));  }
        robot_name = robot_names[0];//Use the first available robot.
      }
      else { robot_name = argv[2];}

      std::cout<<"\nParsing robot: "<<robot_name;
      if(false == flag) { throw(std::runtime_error("Could not read robot description from file"));  }

      /******************************Initialization************************************/
      //We will use a slightly more complex xml spec than the first few tutorials
      flag = p.readRobotFromFile(tmp_infile,"../../specs/",robot_name,rds);
      flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
      flag = flag && rio.init(rds);             //Set up the IO data structure
      flag = flag && dyn_scl_sp.init(rds);      //Set up integrator object
      if(false == flag){ throw(std::runtime_error("Could not initialize data structures"));  } //Error check.

      std::cout<<"\nInitialized data structures, starting physics integrator and redis communication.";

      /******************************Redis Initialization************************************/
      std::cout<<"\n The default REDIS keys used are: ";
      std::cout<<"\n  scl::robot::"<<robot_name<<"::sensors::q";
      std::cout<<"\n  scl::robot::"<<robot_name<<"::sensors::dq";
      std::cout<<"\n  scl::robot::"<<robot_name<<"::actuators::fgc";
      std::cout<<"\n  scl::robot::"<<robot_name<<"::fgc_command_enabled";
      std::cout<<"\n  scl::robot::"<<robot_name<<"::dof";
      std::cout<<"\n  (note some drivers also require) scl::robot::"<<robot_name<<"::operate_mode";

      char rstr[1024], rstr_robot_base[1024]; //For redis key formatting
      int enable_fgc_command=0;

      sprintf(rstr_robot_base, "scl::robot::%s",robot_name.c_str());

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

      // REDIS IO : Add robot to set of active robots..
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SADD %s %s","scl::robots",robot_name.c_str());
      freeReplyObject((void*)redis_ds.reply_);

      // REDIS IO : Add DOF key..
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::dof %d", rstr_robot_base, static_cast<int>(rio.dof_));
      freeReplyObject((void*)redis_ds.reply_);

      // REDIS IO : Create fgc key and set it to zero (initial state)
      { std::stringstream ss; ss<< Eigen::VectorXd::Zero(rio.dof_).transpose(); sprintf(rstr, "%s", ss.str().c_str());  }
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::actuators::fgc %s", rstr_robot_base,rstr, rstr);
      freeReplyObject((void*)redis_ds.reply_);

      // REDIS IO : Get fgc_enabled key : fgc_command_enabled
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "GET %s::fgc_command_enabled", rstr_robot_base);
      if(redis_ds.reply_->len >0)
      { std::stringstream ss; ss<<redis_ds.reply_->str; ss>>enable_fgc_command;  }

      if(false == enable_fgc_command)
      { std::cout<<"\n\n ** WARNING : To enable torque commands set the following key to '1' : "<<rstr_robot_base<<"::fgc_command_enabled **\n\n"; }

      std::cout<<"\n ** To monitor Redis messages, open a redis-cli and type 'monitor' **";

      /******************************Main Loop************************************/
      std::cout<<"\nRunning. Press 'ctrl+c' to exit...\n"<<std::flush;
      scl::sFloat t_start, t_end;
      t_start = sutil::CSystemClock::getSysTime();

      while(flag_sim_enabled)
      {
        // The physics integrator
        sutil::CSystemClock::tick(sim_dt);
        flag = dyn_scl_sp.integrate(rgcm, rio, sim_dt); // Run the integrator with a 1ms timestep..

        // Update sensors. (Assume perfect torque control for now).
        rio.sensors_.force_gc_measured_ = rio.actuators_.force_gc_commanded_;

        //rob_io_ds->sensors_.dq_ -= rob_io_ds->sensors_.dq_/100000;

        // REDIS IO : Set q
        { std::stringstream ss; for(scl::sUInt i=0;i<rio.dof_;++i) ss<<rio.sensors_.q_(i)<<" "; sprintf(rstr, "%s", ss.str().c_str());  }
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::sensors::q %s",rstr_robot_base,rstr);
        freeReplyObject((void*)redis_ds.reply_);

        // REDIS IO : Set dq
        { std::stringstream ss; for(scl::sUInt i=0;i<rio.dof_;++i) ss<<rio.sensors_.dq_(i)<<" "; sprintf(rstr, "%s", ss.str().c_str());  }
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::sensors::dq %s",rstr_robot_base,rstr);
        freeReplyObject((void*)redis_ds.reply_);

        // REDIS IO : Set fgc_sensed
        { std::stringstream ss; for(scl::sUInt i=0;i<rio.dof_;++i) ss<<rio.sensors_.force_gc_measured_(i)<<" "; sprintf(rstr, "%s", ss.str().c_str());  }
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SET %s::sensors::fgc %s",rstr_robot_base,rstr);
        freeReplyObject((void*)redis_ds.reply_);

        // REDIS IO : Get fgc_enabled key : fgc_command_enabled
        redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "GET %s::fgc_command_enabled", rstr_robot_base);
        if(redis_ds.reply_->len > 0)
        { std::stringstream ss; ss<<redis_ds.reply_->str; ss>>enable_fgc_command;  }
        else  { enable_fgc_command = 0;  }
        freeReplyObject((void*)redis_ds.reply_);

        if(enable_fgc_command){ //Read command torques if the enable flag is true
          // REDIS IO : Get fgc_commanded
          redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "GET %s::actuators::fgc", rstr_robot_base);
          { std::stringstream ss; ss<<redis_ds.reply_->str; for(scl::sUInt i=0;i<rio.dof_;++i) ss>>rio.actuators_.force_gc_commanded_(i);  }
          freeReplyObject((void*)redis_ds.reply_);
        }
        else // If the enable flag is false, set torques to zero.
        { rio.actuators_.force_gc_commanded_.setZero(rio.dof_); }
      }

      t_end = sutil::CSystemClock::getSysTime();

      /******************************Redis Shutdown (remove keys)************************************/
      // REDIS IO : Remove robot from set of active robots..
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "SREM %s %s","scl::robots",robot_name.c_str());
      freeReplyObject((void*)redis_ds.reply_);

      // REDIS IO : Remove all keys..
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "DEL %s::dof", rstr_robot_base); freeReplyObject((void*)redis_ds.reply_);
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "DEL %s::sensors::q", rstr_robot_base); freeReplyObject((void*)redis_ds.reply_);
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "DEL %s::sensors::dq", rstr_robot_base); freeReplyObject((void*)redis_ds.reply_);
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "DEL %s::sensors::fgc", rstr_robot_base); freeReplyObject((void*)redis_ds.reply_);
      redis_ds.reply_ = (redisReply *)redisCommand(redis_ds.context_, "DEL %s::actuators::fgc", rstr_robot_base); freeReplyObject((void*)redis_ds.reply_);

      /****************************Print Collected Statistics*****************************/
      //Now you can get the energies
      std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
      std::cout<<"\nSimulation Took Time : "<<t_end-t_start <<" sec";
      std::cout<<"\nReal World End Time  : "<<sutil::CSystemClock::getSysTime() <<" sec \n";

      /****************************Deallocate Memory And Exit*****************************/
      std::cout<<"\nSCL Executed Successfully";
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
