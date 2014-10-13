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
/* \file scl_spatial_integrator.cpp
 *
 *  Created on: Aug 6, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/util/DatabaseUtils.hpp>

#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>

#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <iomanip>
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

/** A sample application to demonstrate a physics simulation in scl
 * using the spatial vector physics implementation.
 *
 * We will use a demo robot with the physics engine and display it
 * in an OpenGL graphics window. We will integrate
 * the robot's dynamics as it swings around under the effect of
 * gravity.
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n*******************************************************\n";
  std::cout<<  " Standard Control Library Spatial Math Integrator Test";
  std::cout<<"\n*******************************************************\n";

  sutil::CSystemClock::start();

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot physics integrator...
  scl_ext::CDynamicsSclSpatial dyn_sp_scl; //Robot physics integrator...
  scl::CParserScl p;         //This time, we'll parse the tree from a file...

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  bool flag = p.readRobotFromFile("./RRRRCfg.xml","../../specs","rrrrbot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_sp_scl.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up dynamics computation object
  flag = flag && rio.init(rds.name_,rds.dof_);
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile("./RRRRCfg.xml","rrrrgraphics",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the rrrbot's physics. Press (x) to exit.";
  long iter = 0, n_iters=50000; double dt=0.001;

  double ke,pe;

  omp_set_num_threads(2);
  int thread_id;
#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
      while(iter < n_iters && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        dyn_sp_scl.integrator(rio,&rgcm,dt); iter++;

        /** Slow down sim to real time */
        sutil::CSystemClock::tick(dt);
        double tcurr = sutil::CSystemClock::getSysTime();
        double tdiff = sutil::CSystemClock::getSimTime() - tcurr;
        timespec ts = {0, 0};
        if(tdiff > 0)
        {
          ts.tv_sec = static_cast<int>(tdiff);
          tdiff -= static_cast<int>(tdiff);
          ts.tv_nsec = tdiff*1e9;
          nanosleep(&ts,NULL);
        }

        // Compute energy. Energy should be conserved.
        if(iter % 1000 == 0)
        {
          dyn_sp_scl.calculateKineticEnergy(&rio, &rgcm,ke);
          dyn_sp_scl.calculatePotentialEnergy(&rio,&rgcm,pe);
          std::cout<<"\n Time ("<<iter*dt<<" of "<<n_iters*dt<<"s total). SpEnergy : "<<std::setw(10)<<pe<<" + "
          <<std::setw(10)<<ke<<" = "<<std::setw(10)<<pe+ke;
        }
      }
    else  //Read the rio data structure and updated rendererd robot..
      while(iter < n_iters && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }
  }

  std::cout<<"\n\nEnd of simulation. ";
  std::cout<<"\nTotal real-world time : "<<sutil::CSystemClock::getSysTime();
  std::cout<<"\nTotal simulated time  : "<<sutil::CSystemClock::getSimTime();

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
