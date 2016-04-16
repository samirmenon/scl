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
 *  Created on: Jan 7, 2015
 *
 *  Copyright (C) 2015
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
#include <scl/serialization/SerializationJSON.hpp>

#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>

#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

#include <hiredis/hiredis.h>

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
 *
 * SCL will also stream data in real time to a redis server on the
 * localhost address...
 * */
int main(int argc, char** argv)
{
  std::cout<<"\n*******************************************************\n";
  std::cout<<  " Standard Control Library Spatial Redis Puma Integ Test";
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

  // For serialization:
  redisContext *redis_serv = redisConnect("localhost", 6379); //Standard...
  if(NULL == redis_serv)
  { std::cout<<"\n Could not connect to local redis server.. \n Addr : localhost\n Port : 6379\n\nExiting.\n";return 1;  }
  else
  { std::cout<<"\n Connected to local redis server.. Setting \"SCL_STATE\" to true."; }

  redisCommand(redis_serv, "SET SCL_STATE true");

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  bool flag = p.readRobotFromFile("../../specs/Puma/PumaCfg.xml","../../specs/","PumaBot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_sp_scl.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up dynamics computation object
  flag = flag && rio.init(rds.name_,rds.dof_);
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile("../../specs/Puma/PumaCfg.xml","PumaBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the rrrbot's physics. Press (x) to exit.";
  
  long iter = 0, n_iters=50000; double dt=0.004;
  double ke,pe;
  
  dyn_sp_scl.computeEnergyKinetic(rgcm, rio.sensors_.q_, rio.sensors_.dq_, ke);
  dyn_sp_scl.computeEnergyPotential(rgcm, rio.sensors_.q_, pe);
  std::cout<<"\n Time ("<<iter*dt<<" of "<<n_iters*dt<<"s total). SpEnergy : "<<std::setw(10)<<pe<<" + "
  <<std::setw(10)<<ke<<" = "<<std::setw(10)<<pe+ke;

  /******************************Redis State Export************************************/
  // Useful for JSON serialization...
  Json::FastWriter writer;
  Json::Value json_val;

  // Set the parsed data for the Puma
  flag = serializeToJSON(rds,json_val);
  if (!flag) {  printf("\n JSON serialization error.. %s",json_val.toStyledString().c_str());  }
  std::string ss = "SET SCL:PumaBot:Parsed "+ writer.write(json_val);
  void *c = redisCommand(redis_serv,ss.c_str());
  if (c != NULL && redis_serv->err) {  printf("\n Redis Error: %s", redis_serv->errstr);  }
  json_val.clear();

  // Set the graphics data for the Puma
  flag = serializeToJSON(rgr,json_val);
  if (!flag) {  printf("\n JSON serialization error.. %s",json_val.toStyledString().c_str());  }
  ss = "SET SCL:PumaBot:Graphics "+ writer.write(json_val);
  c = redisCommand(redis_serv,ss.c_str());
  if (c != NULL && redis_serv->err) {  printf("\n Redis Error: %s", redis_serv->errstr);  }
  json_val.clear();

  omp_set_num_threads(2);
  int thread_id;
#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
      while(iter < n_iters && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        dyn_sp_scl.integrator(rio,&rgcm,dt); iter++;

        sutil::CSystemClock::tick(dt);
        /** Slow down sim to real time */
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

        // Flush state to redis serv. every 10 iters (25Hz)
        if(iter % 10 == 0)
        {
          const bool simple_comm = false;
          if(simple_comm)
          {        //Communicate with the redis server here:
            char ch[1024];
            sprintf(ch,"SET SCL:PumaBot:IO {%lf,%lf,%lf,%lf,%lf,%lf}",
                rio.sensors_.q_(0),rio.sensors_.q_(1),rio.sensors_.q_(2),
                rio.sensors_.q_(3),rio.sensors_.q_(4),rio.sensors_.q_(5));
            void *c = redisCommand(redis_serv,ch);
            if (c != NULL && redis_serv->err) {  printf("\n Redis Error: %s", redis_serv->errstr);  }
          }
          else{
            flag = serializeToJSON(rio,json_val);
            if (!flag) {  printf("\n JSON serialization error.. %s",json_val.toStyledString().c_str());  }
            ss = "SET SCL:PumaBot:IO "+ writer.write(json_val);
            c = redisCommand(redis_serv,ss.c_str());
            if (c != NULL && redis_serv->err) {  printf("\n Redis Error: %s", redis_serv->errstr);  }
          }
        }

        // Compute energy. Energy should be conserved.
        if(iter % 300 == 0)
        {
          dyn_sp_scl.computeEnergyKinetic(rgcm, rio.sensors_.q_, rio.sensors_.dq_, ke);
          dyn_sp_scl.computeEnergyPotential(rgcm, rio.sensors_.q_, pe);
          std::cout<<"\n Time ("<<iter*dt<<" of "<<n_iters*dt<<"s total). SpEnergy : "<<std::setw(10)<<pe<<" + "
          <<std::setw(10)<<ke<<" = "<<std::setw(10)<<pe+ke;

          char ch[1024];
          sprintf(ch,"SET SCL:PumaBot:Energy %lf",pe+ke);
          void *c = redisCommand(redis_serv, ch);
          if (c != NULL && redis_serv->err) {  printf("\n Redis Error: %s", redis_serv->errstr);  }
        }
      }
    else  //Read the rio data structure and updated rendererd robot..
      while(iter < n_iters && true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }
  }

  std::cout<<"\n\nEnd of simulation. ";
  dyn_sp_scl.computeEnergyKinetic(rgcm, rio.sensors_.q_, rio.sensors_.dq_, ke);
  dyn_sp_scl.computeEnergyPotential(rgcm, rio.sensors_.q_, pe);
  std::cout<<"\n Time ("<<iter*dt<<" of "<<n_iters*dt<<"s total). SpEnergy : "<<std::setw(10)<<pe<<" + "
  <<std::setw(10)<<ke<<" = "<<std::setw(10)<<pe+ke;
  std::cout<<"\nTotal iterations      : "<<iter;
  std::cout<<"\nTotal real-world time : "<<sutil::CSystemClock::getSysTime();
  std::cout<<"\nTotal simulated time  : "<<sutil::CSystemClock::getSimTime();

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";

  std::cout<<"\n Setting \"SCL_STATE\" in redis server to false before termination.";
  redisCommand(redis_serv, "SET SCL_STATE false");
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}
