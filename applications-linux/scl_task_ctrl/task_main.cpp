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

#include "CSclAppTask.hpp"

//Set up the simulation function.
namespace scl_app
{
  scl::sBool  CSclAppTask::initMyController(int argc, char** argv)
  {
    bool flag;
    try
    {
      /** Initialize Single Control Task */
      ctrl = (scl::CControllerMultiTask*) robot_.getControllerCurrent();
      if(S_NULL == ctrl)
      { throw(std::runtime_error("Could not get current controller"));  }

      ctrl->computeDynamics();//Set up all matrices and positions etc.

      //Initialize the first op point controller
      op_link_name = argv[4];
      op_link_set = true;

      tsk = (scl::CTaskOpPos*)(ctrl->getTask(op_link_name));
      if(S_NULL == tsk)
      { throw(std::runtime_error("Could not get specified task"));  }
      tsk_ds = dynamic_cast<scl::STaskOpPos*>(tsk->getTaskData());
      db_->s_gui_.ui_point_[0] = tsk_ds->rbd_->T_o_lnk_ * tsk_ds->pos_in_parent_; //Ctrl tracks this control point.

      /** Render a sphere at the op-point task's position */
      flag = chai_gr_.addSphereToRender(robot_name_,tsk_ds->link_ds_->name_,tsk_ds->pos_in_parent_);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at op task"));  }

      //If second op point name was passed, use it.
      if(argc==6)
      {
        op_link2_name = argv[5];
        op_link2_set = true;

        tsk2 = (scl::CTaskOpPos*)(ctrl->getTask(op_link2_name));
        if(S_NULL == tsk2)
        { throw(std::runtime_error("Could not get specified task"));  }
        tsk2_ds = dynamic_cast<scl::STaskOpPos*>(tsk2->getTaskData());
        db_->s_gui_.ui_point_[1] = tsk_ds->rbd_->T_o_lnk_ * tsk_ds->pos_in_parent_; //Ctrl2 tracks this control point.

        /** Render a sphere at the op-point task's position */
        flag = chai_gr_.addSphereToRender(robot_name_,tsk2_ds->link_ds_->name_,tsk2_ds->pos_in_parent_);
        if(false == flag) { throw(std::runtime_error("Could not add sphere at op task"));  }
      }

      return true;
    }
    catch(std::exception &e)
    { std::cout<<"\nCSclAppTask::initMyController() : "<<e.what(); }
    return false;
  }
  void  CSclAppTask::stepMySimulation()
  {
    sutil::CSystemClock::tick(db_->sim_dt_);//Tick the clock.
    tsk->setGoalPos(db_->s_gui_.ui_point_[0]); //Interact with the gui

    //Add some markers to display trajectory every 1000 ticks.
    if(ctrl_ctr_%10000 == 0)
    {
      if(traj_markers_added_so_far_ < SCL_TASK_APP_MAX_MARKERS_TO_ADD)
      {
        /** Render a sphere at the op-point task's position */
        Eigen::Vector3d tmp = db_->s_gui_.ui_point_[0];
        bool flag = chai_gr_.addSphereToRender(tmp,traj_markers_[traj_markers_added_so_far_]);
        if(false == flag)
        { std::cout<<"\nERROR : Could not add marker. Time = "<<sutil::CSystemClock::getSysTime();  }
        traj_markers_added_so_far_++;
      }
    }

    if(op_link2_set)//Use only if the second task was also initialized.
    { tsk2->setGoalPos(db_->s_gui_.ui_point_[1]); }

    if(ctrl_ctr_%20 == 0)           //Update dynamics at a slower rate
    { robot_.computeDynamics();  }
    robot_.computeServo();           //Run the servo loop
    robot_.integrateDynamics();      //Integrate system

    ctrl_ctr_++;//Increment the counter for dynamics computed.
  }
}

/** A sample application to demonstrate marker tracking with
 * an operational space controller on a robot. */
int main(int argc, char** argv)
{
  scl_app::CSclAppTask app;

  if(false == app.init(argc,argv)) {   return 1;  }

  /***********************Main Loop*****************************/
  app.t_start_ = sutil::CSystemClock::getSysTime();

#ifndef DEBUG
  app.runMainLoopThreaded();  //Run multi-threaded in release mode
#else
  app.runMainLoop();          //Run single-threaded in debug mode
#endif

  app.t_end_ = sutil::CSystemClock::getSysTime();
  std::cout<<"\nSimulation Took Time : "<<app.t_end_-app.t_start_ <<" sec";

  /****************************Deallocate Memory And Exit*****************************/
  app.terminate();
  return 0;
}

