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
/* \file test_dynamics.cpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "test_dynamics.hpp"

#include <sutil/CSystemClock.hpp>

#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/lotusparser/CLotusParser.hpp>
#include <scl/util/DatabaseUtils.hpp>

//Tao Dynamics
#include <scl/dynamics/tao/CTaoDynamics.hpp>
#include <scl/dynamics/tao/CTaoRepCreator.hpp>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <cmath>
#include <stdio.h>
#include <math.h>

#include <Eigen/Dense>

using namespace scl;
using namespace std;

namespace scl_test
{

  /**
   * Tests the performance of the tao dynamics engine:
   *
   * Reads in a toy robot specification and lets it fall under gravity
   * with full dynamics.
   */
  void test_dynamics(int id, const std::string &file_name)
  {
    scl::CTaoDynamics * dynamics = S_NULL;
    scl::sUInt r_id=0;
    bool flag;

    try
    {
      //0. Create vars
      long long i; //Counters
      long long imax; //Counter limits
      sClock t1,t2; //Clocks: pre and post

      //Test database
      scl::SDatabase * db = scl::CDatabase::getData();
      if(S_NULL==db)
      { throw(std::runtime_error("Database not initialized."));  }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized database"<<std::flush;  }

      db->dir_specs_ = scl::CDatabase::getData()->cwd_ + std::string("../../specs/");

      //0. Parse the file for robots
      std::string tmp_infile;
      tmp_infile = scl::CDatabase::getData()->cwd_+ file_name;
      std::cout<<"\nTest Result ("<<r_id++<<")  Opening file : "<<tmp_infile;

      scl_parser::CLotusParser tmp_lparser;

      //1.a Read in a world
      flag = scl_registry::parseWorld(tmp_infile, &tmp_lparser);
      if(false == flag)
      { throw(std::runtime_error("Could not register world with the database"));  }

      //1.b Create robot from a file specification (And register it with the db)
      std::vector<std::string> rob_names;
      flag = tmp_lparser.listRobotsInFile(tmp_infile,rob_names);
      if(false == flag)
      { throw(std::runtime_error("Could not read a list of robots from the file"));  }

      std::string robot_name; robot_name = rob_names.at(0);
      flag = scl_registry::parseRobot(tmp_infile, robot_name, &tmp_lparser);
      if(false == flag)
      { throw(std::runtime_error("Could not register robot with the database"));  }
      else
      {
        std::cout<<"\nTest Result ("<<r_id++<<")  Created a robot "
            <<robot_name<<" on the pile"<<std::flush;
      }

#ifdef W_TESTING
      std::cout<<"\nPrinting parsed robot "
          <<db->s_parser_.robots_.at(robot_name)->name_;
      scl_util::printRobotLinkTree(*( db->s_parser_.robots_.at(robot_name)->robot_br_rep_.getRootNode()),0);
#endif

      //Initialize the dynamics computational object
      dynamics = new scl::CTaoDynamics();
      if (S_NULL==dynamics)
      { throw(std::runtime_error("Failed to allocate memory for tao dynamics."));  }

      flag = dynamics->init(* db->s_parser_.robots_.at(robot_name));
      if (false==flag) { throw(std::runtime_error("Failed to initialize tao dynamics."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized tao dynamics for the robot.";  }

      SRobotIOData * io_ds;
      io_ds = scl::CDatabase::getData()->s_io_.io_data_.at(robot_name);
      if(S_NULL == io_ds)
      { throw(std::runtime_error("Could not find the robot's I/O data structure in the database"));  }

      scl::sFloat ke[2], pe[2];
      flag = dynamics->integrate((*io_ds), scl::CDatabase::getData()->sim_dt_); //Need to integrate once to flush the state
      if(false == flag)
      { throw(std::runtime_error("Could not integrate with the dynamics engine"));  }

      ke[0] = dynamics->getKineticEnergy(); //Now you can get the energies
      pe[0] = dynamics->getPotentialEnergy();
      if(0.001 > fabs(pe[0]))
      { throw(std::runtime_error("Zero potential energy at start."));  }

      imax = 200;
      sFloat tstep = 0.0001;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        flag = dynamics->integrate((*io_ds), scl::CDatabase::getData()->sim_dt_);
      }
      t2 = sutil::CSystemClock::getSysTime();

      ke[1] = dynamics->getKineticEnergy(); //Now you can get the energies
      if(0.001 > fabs(ke[1]))
      { throw(std::runtime_error("Zero kinetic energy after dynamics simulation."));  }
      pe[1] = dynamics->getPotentialEnergy();
      if(0.001 > fabs(pe[1]))
      { throw(std::runtime_error("Zero potential energy after dynamics simulation."));  }

      scl::sFloat energy_err = ((ke[1]+pe[1]) - (ke[0]+pe[0]))/(ke[0]+pe[0]);
      std::cout<<"\nTest Result ("<<r_id++<<") Initial Energy : "<<(ke[0]+pe[0])
                             <<". Final Energy : "<<(ke[1]+pe[1])<<". Error : "<<energy_err;
      std::cout<<"\nTest Result ("<<r_id++<<") Total Simulated Time : "<<((double)imax)*tstep <<" sec";
      std::cout<<"\nTest Result ("<<r_id++<<") Simulation Took Time : "<<t2-t1 <<" sec";

      //Now stress test the setup -- Only in release mode.
#ifndef W_TESTING
      ke[0] = dynamics->getKineticEnergy(); //Now you can get the energies
      pe[0] = dynamics->getPotentialEnergy();

      imax = 20000;
      tstep = 0.0001;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        flag = dynamics->integrate((*io_ds), scl::CDatabase::getData()->sim_dt_);
      }
      t2 = sutil::CSystemClock::getSysTime();

      ke[1] = dynamics->getKineticEnergy(); //Now you can get the energies
      pe[1] = dynamics->getPotentialEnergy();
      energy_err = ((ke[1]+pe[1]) - (ke[0]+pe[0]))/(ke[0]+pe[0]);
      std::cout<<"\nTest Result ("<<r_id++<<") Initial Energy : "<<(ke[0]+pe[0])
                                       <<". Final Energy : "<<(ke[1]+pe[1])<<". Error : "<<energy_err;
      std::cout<<"\nTest Result ("<<r_id++<<") Total Simulated Time : "<<((double)imax)*tstep <<" sec";
      std::cout<<"\nTest Result ("<<r_id++<<") Simulation Took Time : "<<t2-t1 <<" sec";
#endif

      //Delete stuff
      if(S_NULL!= dynamics)  {  delete dynamics; dynamics = S_NULL; }

      std::cout<<"\nTest #"<<id<<" : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" : Failed.";

      if(S_NULL!= dynamics)  {  delete dynamics; dynamics = S_NULL; }
    }
  }
}

