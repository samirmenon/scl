/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* \file generic_main.cpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CExampleApp.hpp"
#include "ExampleCallbacks.hpp"

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
  scl_app::CExampleApp app;
  if(false == app.init(argvec)) {   return 1;  }

  //Register all the callback functions.
  if(false == registerCallbacks())
  { std::cout<<"\nFailed to register callbacks"; return 1; }

  //Set up the threads
#ifndef DEBUG
  omp_set_num_threads(3);
#else
  omp_set_num_threads(1);
#endif

  int thread_id;
  if(false == app.setInitialStateForUIAndDynamics()) {   return 1;  }

  app.t_start_ = sutil::CSystemClock::getSysTime();
#pragma omp parallel private(thread_id)
  {//Start threaded region
    thread_id = omp_get_thread_num();

    /***********************Main Loop*****************************/
    if(2 == thread_id) { scl::shell::runConsoleShell(*CDatabase::getData()); }
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

