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
#include <scl/scl.hpp>
#include <scl_ext/scl_ext.hpp>

//sutil clock.
#include <sutil/CSystemClock.hpp>

// 3rd party libs
#include <Eigen/Dense>

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
        <<"\n   ./scl_redis_sim <file_name.xml> <optional: robot_name>"
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

      scl::SRobotParsed rds;     // Robot data structure....
      scl::SGcModel rgcm;        // Robot data structure with dynamic quantities...
      scl::SRobotIO rio;         // I/O data structure
      scl_ext::CDynamicsSclSpatial dyn_scl_sp; //Robot physics integrator...
      scl::CParserScl p;         // This time, we'll parse the tree from a file...

      double sim_dt = 0.001;     // Simulation timestep..
      int enable_fgc_command=0;  // Whether to use fgc commands

      flag = scl::init::registerNativeDynamicTypes();
      if(false == flag) { throw(std::runtime_error("Could not initialize native dynamic types (parser might not work)"));  }

      /******************************File Parsing************************************/
      std::string name_infile(argv[1]);
      std::cout<<"\nRunning scl_redis_sim for input file: "<<name_infile;

      std::string name_robot;
      if(argc==2)
      {//Use the first robot spec in the file if one isn't specified by the user.
        std::vector<std::string> robot_names;
        flag = p.listRobotsInFile(name_infile,robot_names);
        if(false == flag) { throw(std::runtime_error("Could not read robot names from the file"));  }
        name_robot = robot_names[0];//Use the first available robot.
      }
      else { name_robot = argv[2];}

      std::cout<<"\nParsing robot: "<<name_robot;
      if(false == flag) { throw(std::runtime_error("Could not read robot description from file"));  }

      /******************************Initialization************************************/
      //We will use a slightly more complex xml spec than the first few tutorials
      flag = p.readRobotFromFile(name_infile,"../../specs/",name_robot,rds);
      flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
      flag = flag && rio.init(rds);             //Set up the IO data structure
      flag = flag && dyn_scl_sp.init(rds);      //Set up integrator object
      if(false == flag){ throw(std::runtime_error("Could not initialize data structures"));  } //Error check.

      std::cout<<"\nInitialized data structures, starting physics integrator and redis communication.";

      /******************************Redis Initialization************************************/
      scl::CIORedis ioredis;
      scl::SIORedis ioredis_ds;
      flag = ioredis.connect(ioredis_ds,false);
      if(false == flag)
      { throw(std::runtime_error( std::string("Could not connect to redis server : ") + std::string(ioredis_ds.context_->errstr) ));  }

      // Set up the keys here so we don't have to run sprintfs in the while loop...
      char rstr[1024], rstr_robot_base[1024], rstr_actfgc[1024], rstr_fgcenab[1024],
           rstr_q[1024], rstr_dq[1024], rstr_sensfgc[1024]; //For redis key formatting
      sprintf(rstr_robot_base, "scl::robot::%s",name_robot.c_str());
      sprintf(rstr_actfgc, "%s::actuators::fgc", rstr_robot_base);
      sprintf(rstr_fgcenab, "%s::fgc_command_enabled", rstr_robot_base);
      sprintf(rstr_q, "%s::sensors::q", rstr_robot_base);
      sprintf(rstr_dq, "%s::sensors::dq", rstr_robot_base);
      sprintf(rstr_sensfgc, "%s::sensors::fgc", rstr_robot_base);

      std::cout<<"\n The default REDIS keys used are: ";
      std::cout<<"\n  "<<rstr_q<<"\n  "<<rstr_dq<<"\n  "<<rstr_sensfgc<<"\n  "<<rstr_actfgc<<"\n  "<<rstr_fgcenab;
      std::cout<<"\n  scl::robot::"<<name_robot<<"::dof";

      // REDIS IO : Add robot to set of active robots..
      ioredis_ds.reply_ = (redisReply *)redisCommand(ioredis_ds.context_, "SADD scl::robots %s", name_robot.c_str());
      freeReplyObject((void*)ioredis_ds.reply_);

      // REDIS IO : Add DOF key..
      ioredis_ds.reply_ = (redisReply *)redisCommand(ioredis_ds.context_, "SET %s::dof %d", rstr_robot_base, static_cast<int>(rio.dof_));
      freeReplyObject((void*)ioredis_ds.reply_);

      // REDIS IO : Create fgc key and set it to zero (initial state)
      rio.actuators_.force_gc_commanded_.setZero(rio.dof_);
      flag = flag && ioredis.set(ioredis_ds, rstr_actfgc, rio.actuators_.force_gc_commanded_);

      if(false == flag) { throw(std::runtime_error("Could not complete initial redis key set/gets." ));  }

      // REDIS IO : Get fgc_enabled key : fgc_command_enabled
      flag = ioredis.get(ioredis_ds, rstr_fgcenab, enable_fgc_command);
      if(false == flag)
      {
        std::cout<<"\n NOTE : Did not find an '"<< rstr_fgcenab<<"' key so creating it and setting it to zero";
        enable_fgc_command = 0;
        ioredis.set(ioredis_ds, rstr_fgcenab, enable_fgc_command);
      }

      if(0 == enable_fgc_command)
      { std::cout<<"\n ** WARNING : To enable torque commands set the following key to '1' : "<<rstr_robot_base<<"::fgc_command_enabled **\n"; }

      std::cout<<"\n ** To monitor Redis messages, open a redis-cli and type 'monitor' **";

      /******************************Main Loop************************************/
      std::cout<<"\nRunning. Press 'ctrl+c' to exit...\n"<<std::flush;
      scl::sFloat t_start, t_end;
      t_start = sutil::CSystemClock::getSysTime();

      while(flag_sim_enabled)
      {
        // ***************** The physics integrator *****************
        sutil::CSystemClock::tick(sim_dt);
        flag = dyn_scl_sp.integrate(rgcm, rio, sim_dt); // Run the integrator with a 1ms timestep..

        /** Slow down sim to real time */
        double tcurr = sutil::CSystemClock::getSysTime() - t_start;
        double tdiff = sutil::CSystemClock::getSimTime() - tcurr;
        timespec ts = {0, 0};
        if(tdiff > 0)
        {
          ts.tv_sec = static_cast<int>(tdiff);
          tdiff -= static_cast<int>(tdiff);
          ts.tv_nsec = tdiff*1e9;
          nanosleep(&ts,NULL);
        }

        // Update sensors. (Assume perfect torque control for now).
        rio.sensors_.force_gc_measured_ = rio.actuators_.force_gc_commanded_;

        //rio.sensors_.dq_ -= rio.sensors_.dq_/1000;

        // ***************** The Redis IO *****************
        flag = flag && ioredis.set(ioredis_ds, rstr_q, rio.sensors_.q_);   // REDIS IO : Set q
        flag = flag && ioredis.set(ioredis_ds, rstr_dq, rio.sensors_.dq_); // REDIS IO : Set dq
        flag = flag && ioredis.set(ioredis_ds, rstr_sensfgc, rio.sensors_.force_gc_measured_); // REDIS IO : Set fgc_sensed
        flag = flag && ioredis.get(ioredis_ds, rstr_fgcenab, enable_fgc_command); // REDIS IO : Get fgc_enabled key : fgc_command_enabled

        if(false == flag){  enable_fgc_command = 0; } // Just to be safe..

        if(enable_fgc_command) //Read command torques if the enable flag is true
        { flag = flag && ioredis.get(ioredis_ds, rstr_actfgc, rio.actuators_.force_gc_commanded_);  } // REDIS IO : Get fgc_commanded
        else // If the enable flag is false, set torques to zero.
        { rio.actuators_.force_gc_commanded_.setZero(rio.dof_); }

        if(false == flag){ std::cout<<"\n ERROR : Can't get fgc command. Something is probably seriously wrong. Should quit"; break;  }
      } // End of while loop

      t_end = sutil::CSystemClock::getSysTime();

      /******************************Redis Shutdown (remove keys)************************************/
      // Disable fgc commands (we won't delete it because it might be used by others)
      flag = ioredis.set(ioredis_ds, rstr_fgcenab, 0); // REDIS IO : Set fgc_enabled key
      if(false == flag){ std::cout<<"\n ERROR : Could not cleanly remove fgc enabled key in redis ds. Consider setting it manually.";  }

      // REDIS IO : Remove robot from set of active robots..
      ioredis_ds.reply_ = (redisReply *)redisCommand(ioredis_ds.context_, "SREM scl::robots %s", name_robot.c_str());
      freeReplyObject((void*)ioredis_ds.reply_);


      // REDIS IO : Remove all keys. This is just easier with the raw redis command so we won't use the SCL wrapper.
      ioredis_ds.reply_ = (redisReply *)redisCommand(ioredis_ds.context_, "DEL %s::dof", rstr_robot_base); freeReplyObject((void*)ioredis_ds.reply_);
      ioredis_ds.reply_ = (redisReply *)redisCommand(ioredis_ds.context_, "DEL %s::sensors::q", rstr_robot_base); freeReplyObject((void*)ioredis_ds.reply_);
      ioredis_ds.reply_ = (redisReply *)redisCommand(ioredis_ds.context_, "DEL %s::sensors::dq", rstr_robot_base); freeReplyObject((void*)ioredis_ds.reply_);
      ioredis_ds.reply_ = (redisReply *)redisCommand(ioredis_ds.context_, "DEL %s::sensors::fgc", rstr_robot_base); freeReplyObject((void*)ioredis_ds.reply_);
      ioredis_ds.reply_ = (redisReply *)redisCommand(ioredis_ds.context_, "DEL %s::actuators::fgc", rstr_robot_base); freeReplyObject((void*)ioredis_ds.reply_);

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
