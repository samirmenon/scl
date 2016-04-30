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
/* \file redis_main.cpp
 *
 *  Created on: Jan 20, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CSclRedisTask.hpp"

//Set up the simulation function.
namespace scl_app
{
  scl::sBool  CSclRedisTask::initMyController(int argc, char** argv)
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
    { std::cout<<"\nCSclRedisTask::initMyController() : "<<e.what(); }
    return false;
  }
  void  CSclRedisTask::stepMySimulation()
  {
    // ******************************************
    // Do the IO operations on the redis server
    static double tlast = 0;
    double tnow = sutil::CSystemClock::getSysTime();
    if(tnow - tlast > 0.01)
    {//Push to redis at 100Hz...
      tlast = tnow;

      //Add some markers to display trajectory every 10ms ticks.
      Eigen::VectorXd tmp; tmp.setZero(3);
      tsk->getPos(tmp);// Plot ee point (exec traj)

      if(curr_traj_marker_id_ < SCL_REDIS_MAX_MARKERS)
      {
        /** Render a sphere at the op-point task's position */
        bool flag = chai_gr_.addSphereToRender(tmp,traj_markers_[curr_traj_marker_id_],0.004);
        if(false == flag)
        { std::cout<<"\nERROR : Could not add marker. Time = "<<sutil::CSystemClock::getSysTime();  }
        curr_traj_marker_id_++;
      }
      else{
        //Just wrap around and update the last marker to a new position..
        int id = curr_traj_marker_id_ % SCL_REDIS_MAX_MARKERS;

        // Move the marker to the latest pos...
        traj_markers_[id]->setLocalPos(tmp(0), tmp(1), tmp(2));

        curr_traj_marker_id_++;
        if(curr_traj_marker_id_ >= 2* SCL_REDIS_MAX_MARKERS)
        { curr_traj_marker_id_ = SCL_REDIS_MAX_MARKERS; }
      }

      // Some static vars for redis ops
      static char rstr[1024];
      //Set ee pos in Redis database
      sprintf(rstr,"%lf %lf %lf", tmp(0),tmp(1),tmp(2));
      redis_ds_.reply_ = (redisReply *)redisCommand(redis_ds_.context_, "SET %s %s","scl_pos_ee",rstr);
      freeReplyObject((void*)redis_ds_.reply_);

      //Get desired pos from Redis database..
      redis_ds_.reply_ = (redisReply *)redisCommand(redis_ds_.context_, "GET scl_pos_ee_des");
      if(redis_ds_.reply_->len <=0)
      {// The key probably doesn't exist. Set it instead.
        freeReplyObject((void*)redis_ds_.reply_);
        redis_ds_.reply_ = (redisReply *)redisCommand(redis_ds_.context_, "SET %s %s","scl_pos_ee_des",rstr);
      }
      else{
        sscanf(redis_ds_.reply_->str,"%lf %lf %lf", & (db_->s_gui_.ui_point_[0](0)),
            & (db_->s_gui_.ui_point_[0](1)), & (db_->s_gui_.ui_point_[0](2)));
      }
      freeReplyObject((void*)redis_ds_.reply_);
    }

    //Set the goal position for the task...
    tsk->setGoalPos(db_->s_gui_.ui_point_[0]); //Interact with the gui

    if(op_link2_set)//Use only if the second task was also initialized.
    { tsk2->setGoalPos(db_->s_gui_.ui_point_[1]); }

    if(ctrl_ctr_%20 == 0)           //Update dynamics at a slower rate
    { robot_.computeDynamics();  }
    robot_.computeServo();           //Run the servo loop

    // ********************************************
    /** Slow down sim to real time and integrate */
    robot_.integrateDynamics();      //Integrate system
    sutil::CSystemClock::tick(db_->sim_dt_);
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
    // ********************************************

    ctrl_ctr_++;//Increment the counter for dynamics computed.
  }
}

/** A sample application to demonstrate marker tracking with
 * an operational space controller on a robot. */
int main(int argc, char** argv)
{
  scl_app::CSclRedisTask app;

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

