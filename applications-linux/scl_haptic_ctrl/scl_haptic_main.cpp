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
/* \file generic_main.cpp
 *
 *  Created on: Sep 3, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CHapticApp.hpp"
#include "HapticCallbacks.hpp"

#ifdef GRAPHICS_ON
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#endif
#include <util/HelperFunctions.hpp>

#include <sutil/CSystemClock.hpp>

#include <omp.h>
#include <GL/freeglut.h>

using namespace scl;
using namespace scl_app;


/** A sample application to demonstrate marker tracking with
 * an operational space controller on a robot. */
int main(int argc, char** argv)
{
  //Set the cwd and specs dir so scl knows where the graphics are.
  scl_util::getCurrentDir(CDatabase::getData()->cwd_);
  CDatabase::getData()->dir_specs_ = CDatabase::getData()->cwd_ + std::string("../../specs/");

  //Initialize glut before the app
#ifdef GRAPHICS_ON
  if(!CDatabase::getData()->s_gui_.glut_initialized_)
  {
    glutInit(&argc, argv);
    CDatabase::getData()->s_gui_.glut_initialized_ = true;
  }
#endif

  //Convert the argc and arv into a vector of strings that the app can read
  //If you have custom paths etc, please set them here.
  std::vector<std::string> argvec;
  for(int i=0;i<argc;++i)
  { argvec.push_back(std::string(argv[i])); }

  //Initialize the app
  scl_app::CHapticApp app;
  if(false == app.init(argvec)) {   return 1;  }

  //Register all the callback functions.
  if(false == registerCallbacks())
  { std::cout<<"\nFailed to register callbacks"; return 1; }

  std::cout<<"\nSimulation Starting";

  //Set up the threads
#ifndef DEBUG
  omp_set_num_threads(3);
#else
  omp_set_num_threads(1);
#endif

  int thread_id;
  app.setInitialStateForUIAndDynamics();
  app.t_start_ = sutil::CSystemClock::getSysTime();
#pragma omp parallel private(thread_id)
  {//Start threaded region
    thread_id = omp_get_thread_num();

    /***********************Main Loop*****************************/
    if(2 == thread_id) { app.runConsoleShell(); }
    else {//No shell in debug mode for now.
#ifndef DEBUG
      app.runMainLoopThreaded(thread_id);  //Run multi-threaded in release mode
#else
      app.runMainLoop();          //Run single-threaded in debug mode
#endif
    }
  }
  app.t_end_ = sutil::CSystemClock::getSysTime();
  std::cout<<"\nSimulation Took Time : "<<app.t_end_-app.t_start_ <<" sec";

  /****************************Deallocate Memory And Exit*****************************/
  app.terminate();
  return 0;
}

