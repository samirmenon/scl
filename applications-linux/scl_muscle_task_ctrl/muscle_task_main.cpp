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
/* \file muscle_task_main.cpp
 *
 *  Created on: Oct 30, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CSclAppMuscleTask.hpp"

//Set up the simulation function.
namespace scl_app
{
  scl::sBool  CSclAppMuscleTask::initMyController(int argc, char** argv)
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
    { std::cout<<"\nCSclAppMuscleTask::initMyController() : "<<e.what(); }
    return false;
  }
  void  CSclAppMuscleTask::stepMySimulation()
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

    if(ctrl_ctr_%1 == 0)           //Update dynamics at a slower rate
    {
      robot_.computeDynamics();

      rob_mset_.computeJacobian(rob_io_ds_->sensors_.q_, rob_muscle_J_);

      // Compute svd to set up matrix sizes etc.
      rob_svd_.compute(rob_muscle_J_.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV | Eigen::ColPivHouseholderQRPreconditioner);
      for(unsigned int i=0;i<rob_ds->dof_;++i)
      {
        if(rob_svd_.singularValues()(i)>SVD_THESHOLD)
        { rob_sing_val_(i,i) = 1/rob_svd_.singularValues()(i);  }
        else
        { rob_sing_val_(i,i) = 0;  }
      }
      rob_muscle_Jpinv_ = rob_svd_.matrixV() * rob_sing_val_.transpose() * rob_svd_.matrixU().transpose();

      act_->force_actuator_ = rob_muscle_Jpinv_*rob_io_ds_->actuators_.force_gc_commanded_;

      if(ctrl_ctr_%5000 == 0)
      {
        std::cout<<"\nJ':\n"<<rob_muscle_J_.transpose();
        std::cout<<"\nFgc':"<<rob_io_ds_->actuators_.force_gc_commanded_.transpose();
        std::cout<<"\nFm {";
        for (int j=0; j<rob_mset_.getNumberOfMuscles(); j++)
        { std::cout<<rob_ds->muscle_system_.muscle_id_to_name_[j]<<", "; }
        std::cout<<"} : "<<act_->force_actuator_.transpose();
      }
    }
    robot_.computeServo();           //Run the servo loop
    robot_.integrateDynamics();      //Integrate system

    ctrl_ctr_++;//Increment the counter for dynamics computed.
  }
}

/** A sample application to demonstrate marker tracking with
 * an operational space controller on a robot. */
int main(int argc, char** argv)
{
  scl_app::CSclAppMuscleTask app;

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

