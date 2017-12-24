/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/* \file task_main.cpp
 *
 *  Created on: Nov 22, 2010
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CScl2RobAppTask.hpp"

#include <cassert>

//Set up the simulation function.
namespace scl_app
{
  scl::sBool  CScl2RobAppTask::initMyController(int argc, char** argv, unsigned int arg_rob_idx)
  {
    bool flag;
    try
    {
      /** Initialize Single Control Task */
      ctrl[arg_rob_idx] = (scl::CControllerMultiTask*) robot[arg_rob_idx].getControllerCurrent();
      if(S_NULL == ctrl[arg_rob_idx])
      { throw(std::runtime_error("Could not get current controller"));  }

      //Initialize the first op point controller
      op_link_name[arg_rob_idx] = argv[5*arg_rob_idx+4];
      if(arg_rob_idx==0){
        db->s_gui_.ui_point_[2*arg_rob_idx+0]<<0.2,0.15,0; //Ctrl tracks this control point.
      }
      else{
        db->s_gui_.ui_point_[2*arg_rob_idx+0]<<0.5,0.1,0; //Ctrl tracks this control point.
      }
      tsk[arg_rob_idx] = (scl::CTaskOpPos*)(ctrl[arg_rob_idx]->getTask(op_link_name[arg_rob_idx]));
      if(S_NULL == tsk[arg_rob_idx])
      { throw(std::runtime_error("Could not get specified task"));  }
      tsk_ds[arg_rob_idx] = dynamic_cast<scl::STaskOpPos*>(tsk[arg_rob_idx]->getTaskData());
      op_link_set[arg_rob_idx] = true;

      /** Render a sphere at the op-point task's position */
      flag = chai_gr.addSphereToRender(robot_name[arg_rob_idx],
          tsk_ds[arg_rob_idx]->link_ds_->name_, tsk_ds[arg_rob_idx]->pos_in_parent_);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at op task"));  }

      //If second op point name was passed, use it.
      op_link2_name[arg_rob_idx] = argv[5*arg_rob_idx+5];
      if(arg_rob_idx==0){
        db->s_gui_.ui_point_[2*arg_rob_idx+1]<<0.2,-0.15,0; //Ctrl tracks this control point.
      }
      else{
        db->s_gui_.ui_point_[2*arg_rob_idx+1]<<0.5,-0.1,0; //Ctrl tracks this control point.
      }
      tsk2[arg_rob_idx] = (scl::CTaskOpPos*)(ctrl[arg_rob_idx]->getTask(op_link2_name[arg_rob_idx]));
      if(S_NULL == tsk2[arg_rob_idx])
      { throw(std::runtime_error("Could not get specified task"));  }
      tsk2_ds[arg_rob_idx] = dynamic_cast<scl::STaskOpPos*>(tsk2[arg_rob_idx]->getTaskData());
      op_link2_set[arg_rob_idx] = true;

      /** Render a sphere at the op-point task's position */
      flag = chai_gr.addSphereToRender(robot_name[arg_rob_idx],
          tsk2_ds[arg_rob_idx]->link_ds_->name_, tsk2_ds[arg_rob_idx]->pos_in_parent_);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at op task"));  }

      return true;
    }
    catch(std::exception &e)
    { std::cout<<"\nCScl2RobAppTask::initMyController() : "<<e.what(); }
    return false;
  }

  void  CScl2RobAppTask::stepMySimulation()
  {
    for(int i=0; i<2; ++i)
    {
      sutil::CSystemClock::tick(db->sim_dt_);//Tick the clock.

      assert(op_link_set[i]);
      assert(op_link2_set[i]);

      tsk[i]->setGoalPos(db->s_gui_.ui_point_[2*i+0]); //Interact with the gui
      tsk2[i]->setGoalPos(db->s_gui_.ui_point_[2*i+1]);

      if(ctrl_ctr%5 == 0)           //Update dynamics at a slower rate
      { robot[i].computeDynamics();  }
      robot[i].computeServo();           //Run the servo loop
      robot[i].integrateDynamics();      //Integrate system
    }

    ctrl_ctr++;//Increment the counter for dynamics computed.
  }
}

bool addCallbacks()
{
  // Add a bunch of callback functions to the command line shell
  bool flag = sutil::callbacks::add<scl::CCallbackHelp, std::string, std::vector<std::string> >(std::string("help") );
  flag = flag && sutil::callbacks::add<scl::CCallbackEcho, std::string, std::vector<std::string> >(std::string("echo") );
  flag = flag && sutil::callbacks::add<scl::CCallbackPrint, std::string, std::vector<std::string> >(std::string("print") );
  flag = flag && scl::printableAddObject<scl::SDatabase>(*scl::CDatabase::getData());
  return flag;
}

/** A sample application to demonstrate marker tracking with
 * an operational space controller on a robot. */
int main(int argc, char** argv)
{
  // Initialize the application
  scl_app::CScl2RobAppTask app;
  if(false == app.init(argc,argv)) {   return 1;  }

  // Initialize the console shell callbacks.
  addCallbacks();

  /***********************Main Loop*****************************/
  app.t_start = sutil::CSystemClock::getSysTime();

#ifndef DEBUG

  omp_set_num_threads(3);
  int thread_id;

#pragma omp parallel private(thread_id)
  {//Start threaded region
    thread_id = omp_get_thread_num();

    if(2==thread_id)
    { scl::shell::runConsoleShell(*scl::CDatabase::getData());  }
    else
    { app.runMainLoopThreaded(thread_id);  }//Run multi-threaded in release mode
  }
#else
  app.runMainLoop();          //Run single-threaded in debug mode
#endif

  app.t_end = sutil::CSystemClock::getSysTime();
  std::cout<<"\nSimulation Took Time : "<<app.t_end-app.t_start <<" sec";

  /****************************Deallocate Memory And Exit*****************************/
  app.terminate();
  return 0;
}

