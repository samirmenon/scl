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
/* \file scl_benchmarks_main.cpp
 *
 *  Created on: Nov 22, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <sstream>

//Standard includes
#include <iostream>
#include <stdexcept>

//Eigen 3rd party lib
#include <Eigen/Dense>

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/dynamics/tao/CTaoDynamics.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/graphics/chai/CChaiGraphics.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/parser/lotusparser/CLotusParser.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <sutil/CSystemClock.hpp>

//Openmp
#include <omp.h>

/***
 * A sample application to demonstrate scl running one or more robots.
 *
 * Use it as a template to write your own (more detailed/beautiful/functional)
 * application.
 *
 * A simulation requires running 3 things:
 * 1. A dynamics/physics engine                  :  Tao
 * 2. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
 */
int main(int argc, char** argv)
{
  bool flag;
  if((argc != 2)&&(argc != 3))
  {
    std::cout<<"\nscl-benchmarks demo application demonstrates how scl simulates the physics of single robots."
        <<"\nThe command line input is: ./<executable> <file_name.xml> <optional: robot_name.xml>\n";
    return 0;
  }
  else
  {
    try
    {
      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start())
      { throw(std::runtime_error("Could not start clock"));  }

      scl::SDatabase* db = scl::CDatabase::getData(); //Sanity Check
      if(S_NULL==db) { throw(std::runtime_error("Database not initialized"));  }

      db->dir_specs_ = db->cwd_ + "../../specs/"; //Set the specs dir so scl knows where the graphics are.

      //Get going..
      std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

      std::string tmp_infile(argv[1]);
      std::cout<<"Running scl benchmarks for input file: "<<tmp_infile;

      /******************************File Parsing************************************/
      scl_parser::CLotusParser tmp_lparser;//Use the lotus tinyxml parser

      std::string robot_name;
      if(argc==2)
      {//Find the robot specs in the file if one isn't specified by the user.
        std::vector<std::string> robot_names;
        flag = tmp_lparser.listRobotsInFile(tmp_infile,robot_names);
        if(false == flag) { throw(std::runtime_error("Could not read robot names from the file"));  }
        robot_name = robot_names[0];//Use the first available robot.
      }
      else { robot_name = argv[2];}

      if(S_NULL == scl_registry::parseRobot(tmp_infile, robot_name, &tmp_lparser))
      { throw(std::runtime_error("Could not register robot with the database"));  }

      std::vector<std::string> graphics_names;
      flag = tmp_lparser.listGraphicsInFile(tmp_infile,graphics_names);
      if(false == flag) { throw(std::runtime_error("Could not list graphics names from the file"));  }

      if(S_NULL == scl_registry::parseGraphics(tmp_infile, graphics_names[0], &tmp_lparser))
      { throw(std::runtime_error("Could not register graphics with the database"));  }

#ifdef W_TESTING
      std::cout<<"\nPrinting parsed robot "<<robot_name;
      scl_util::printRobotLinkTree(*( scl::CDatabase::getData()->
          s_parser_.robots_.at(robot_name)->robot_br_rep_.getRootNode()),0);
#endif

      /******************************TaoDynamics************************************/
      scl::CTaoDynamics tao_dyn;
      flag = tao_dyn.init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
      if(false == flag) { throw(std::runtime_error("Could not initialize dynamic simulator"));  }

      /******************************ChaiGlut Graphics************************************/
      glutInit(&argc, argv);

      scl::CChaiGraphics chai_gr;
      flag = chai_gr.initGraphics(graphics_names[0]);
      if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

      flag = chai_gr.addRobotToRender(robot_name);
      if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

      if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_names[0], &chai_gr))
      { throw(std::runtime_error("Glut initialization error")); }

      /******************************Shared I/O Data Structure************************************/
      scl::SRobotIOData* rob_io_ds;
      rob_io_ds = db->s_io_.io_data_.at(robot_name);
      if(S_NULL == rob_io_ds)
      { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

      /******************************Main Loop************************************/
      std::cout<<std::flush;
      //Initialize some useful statistics
      scl::sFloat ke[2], pe[2];
      flag = tao_dyn.integrate((*rob_io_ds),scl::CDatabase::getData()->sim_dt_); //Need to integrate once to flush the state
      if(false == flag)
      { throw(std::runtime_error("Could not integrate with the dynamics engine"));  }
      ke[0] = tao_dyn.getKineticEnergy(); //Now you can get the energies
      pe[0] = tao_dyn.getPotentialEnergy();

      scl::sLongLong gr_ctr=0;//Graphics computation counter

      scl::sFloat t_start, t_end;

      //NOTE TODO : DELETE THIS:
      std::string tmp;
      tmp = robot_name + ".xml";
      tmp_lparser.saveRobotToFile(*(scl::CDatabase::getData()->
          s_parser_.robots_.at(robot_name)),tmp);

      omp_set_num_threads(2);
      int thread_id;
      t_start = sutil::CSystemClock::getSysTime();
#pragma omp parallel private(thread_id)
      {
        thread_id = omp_get_thread_num();
        if(thread_id==1)
        {
          std::cout<<"\nI am the simulation thread. Id = "<<thread_id<<std::flush;
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            sutil::CSystemClock::tick(db->sim_dt_);
            flag = tao_dyn.integrate((*rob_io_ds), scl::CDatabase::getData()->sim_dt_);
            //rob_io_ds->sensors_.dq_ -= rob_io_ds->sensors_.dq_/100000;
          }
        }
        else
        {
          std::cout<<"\nI am the graphics thread. Id = "<<thread_id<<std::flush;
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            glutMainLoopEvent();
            gr_ctr++;
            const timespec ts = {0, 15000000};//Sleep for 15ms
            nanosleep(&ts,NULL);
          }
        }
      }

      t_end = sutil::CSystemClock::getSysTime();

      /****************************Print Collected Statistics*****************************/
      ke[1] = tao_dyn.getKineticEnergy(); //Now you can get the energies
      pe[1] = tao_dyn.getPotentialEnergy();
      scl::sFloat energy_err = ((ke[1]+pe[1]) - (ke[0]+pe[0]))/(ke[0]+pe[0]);
      std::cout<< "\n\nSimulation Statistics:\nInitial Energy: "<<(ke[0]+pe[0])
               <<". Final Energy : "<<(ke[1]+pe[1])<<". Error: "<<energy_err;
      std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
      std::cout<<"\nSimulation Took Time : "<<t_end-t_start <<" sec";
      std::cout<<"\nReal World End Time  : "<<sutil::CSystemClock::getSysTime() <<" sec \n";
      std::cout<<"\nTotal Graphics Updates                : "<<gr_ctr;

      /****************************Deallocate Memory And Exit*****************************/
      flag = chai_gr.destroyGraphics();
      if(false == flag) { throw(std::runtime_error("Error deallocating graphics pointers")); } //Sanity check.

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
