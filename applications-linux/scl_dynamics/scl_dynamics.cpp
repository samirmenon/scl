/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/* \file scl_benchmarks_main.cpp
 *
 *  Created on: Nov 22, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
//scl lib
#include <scl/scl.hpp>
#include <scl_ext/scl_ext.hpp>

#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes
#include <sstream>
#include <iostream>
#include <stdexcept>

//Openmp
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

/**
 * A sample application to demonstrate scl running one or more robots.
 *
 * Use it as a template to write your own (more detailed/beautiful/functional)
 * application.
 *
 * A simulation requires running 3 things:
 * 1. A dynamics/physics engine                  :  Scl
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
      scl::CParserScl tmp_lparser;//Use the scl tinyxml parser

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

      scl::SRobotParsed *rob_ds = scl::CDatabase::getData()->s_parser_.robots_.at(robot_name);
      if(NULL == rob_ds)
      { throw(std::runtime_error("Could not find registered robot data struct in the database"));  }

      /******************************SclDynamics************************************/
      scl_ext::CDynamicsSclSpatial dyn_scl_sp;
      flag = dyn_scl_sp.init(*rob_ds);
      if(false == flag) { throw(std::runtime_error("Could not initialize dynamic simulator"));  }

      /******************************Shared I/O Data Structure************************************/
      scl::SRobotIO* rob_io_ds;
      rob_io_ds = db->s_io_.io_data_.at(robot_name);
      if(S_NULL == rob_io_ds)
      { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

      /******************************ChaiGlut Graphics************************************/
      glutInit(&argc, argv);

      scl::CGraphicsChai chai_gr;
      scl::SGraphicsParsed *gr_parsed = db->s_parser_.graphics_worlds_.at(graphics_names[0]);
      scl::SGraphicsChai *chai_ds = db->s_gui_.chai_data_.at(graphics_names[0]);
      flag = chai_gr.initGraphics(gr_parsed,chai_ds);
      if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

      flag = chai_gr.addRobotToRender(rob_ds,rob_io_ds);
      if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

      if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_names[0], &chai_gr))
      { throw(std::runtime_error("Glut initialization error")); }

      /******************************Dynamic data struct************************************/
      scl::SGcModel rob_gc, rob_gc_integ; // One for the scl dynamics (to compute KE, PE) and one for the spatial integrator
      flag = rob_gc.init(*rob_ds);
      flag = flag && rob_gc_integ.init(*rob_ds);
      if(false == flag)
      { throw(std::runtime_error("Could not initialize gc model"));  }

      /******************************Main Loop************************************/
      std::cout<<std::flush;
      //Initialize some useful statistics
      scl::sFloat ke[2], pe[2];
      flag = dyn_scl_sp.integrate(rob_gc_integ,(*rob_io_ds),scl::CDatabase::getData()->sim_dt_); //Need to integrate once to flush the state
      flag = flag && dyn_scl_sp.computeEnergyKinetic(rob_gc,rob_io_ds->sensors_.q_,rob_io_ds->sensors_.dq_, ke[0]);
      flag = flag && dyn_scl_sp.computeEnergyPotential(rob_gc,rob_io_ds->sensors_.q_, pe[0]);
      if(false == flag)
      { throw(std::runtime_error("Could not integrate and/or compute energy with the dynamics engine"));  }

      scl::sLongLong gr_ctr=0;//Graphics computation counter

      scl::sFloat t_start, t_end;

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
            flag = dyn_scl_sp.integrate(rob_gc_integ,(*rob_io_ds), scl::CDatabase::getData()->sim_dt_);
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
      //Now you can get the energies
      dyn_scl_sp.computeEnergyKinetic(rob_gc,rob_io_ds->sensors_.q_,rob_io_ds->sensors_.dq_,ke[1]);
      dyn_scl_sp.computeEnergyPotential(rob_gc,rob_io_ds->sensors_.q_,pe[1]);
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
