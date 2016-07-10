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

//Eigen 3rd party lib
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
/**
 * A sample server application to simulate the physics of a robot and provide a
 * redis interface similar to the standard scl robot driver(s) using redis.
 */
int main(int argc, char** argv)
{
  std::cout<<"\n*******************************************************\n";
  std::cout<<  "              SCL Redis Robot Simulator";
  std::cout<<"\n*******************************************************\n";

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

      std::cout<<"\nInitialized data structures, starting physics integrator and redis communication."
               <<"\nRunning. Press 'ctrl+c' to exit. Open redis-cli and type 'monitor' to see what it's doing)...\n"<<std::flush;

      /******************************Main Loop************************************/
      scl::sFloat t_start, t_end;

      t_start = sutil::CSystemClock::getSysTime();

      while(flag_sim_enabled)
      {
        sutil::CSystemClock::tick(sim_dt);
        flag = dyn_scl_sp.integrate(rgcm, rio, sim_dt); // Run the integrator with a 1ms timestep..
        //rob_io_ds->sensors_.dq_ -= rob_io_ds->sensors_.dq_/100000;

        //std::cout<<"\n q: "<<rio.sensors_.q_.transpose();
      }

      t_end = sutil::CSystemClock::getSysTime();

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
