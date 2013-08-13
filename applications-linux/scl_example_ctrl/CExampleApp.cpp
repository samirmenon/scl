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
/* \file CExampleApp.cpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CExampleApp.hpp"
#include "CExampleTask.hpp"

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SDatabase.hpp>

#include <sutil/CSystemClock.hpp>

#include <chai3d.h>

#include <iostream>
#include <stdexcept>

using namespace chai3d;

namespace scl_app
{

  /** Default constructor. Sets stuff to zero.
   * Uses a task controller*/
  CExampleApp::CExampleApp() : CRobotApp(),
      ctrl_(S_NULL),
      name_com_task_(""),
      task_com_(S_NULL),
      task_ds_com_(S_NULL),
      ui_pt_com_(-1),
      has_been_init_com_task_(false),
      chai_com_pos_(S_NULL),
      chai_com_pos_des_(S_NULL)
  { }

  scl::sBool CExampleApp::initMyController(const std::vector<std::string>& argv,
      scl::sUInt args_parsed)
  {
    bool flag;
    try
    {
      //Ctr in array of args_parsed = (args_parsed - 1)
      //So ctr for un-parsed arg = (args_parsed - 1) + 1
      scl::sUInt args_ctr = args_parsed, ui_points_used=0;

      /** Initialize Single Control Task */
      ctrl_ = (scl::CTaskController*) robot_.getControllerCurrent();
      if(S_NULL == ctrl_)
      { throw(std::runtime_error("Could not get current controller"));  }

      // Check that we haven't finished parsing everything
      while(args_ctr < argv.size())
      {
        if ("-com" == argv[args_ctr])
        {// We know the next argument *should* be the com pos task's name
          if(args_ctr+1 < argv.size())
          {
            if(ui_points_used >= SCL_NUM_UI_POINTS)
            {
              std::cout<<"\nThe keyboard can only support "<<SCL_NUM_UI_POINTS<<" ui points. Can't add: "
                  << argv[args_ctr]<<" "<< argv[args_ctr+1];
              args_ctr+=2; continue;
            }
            //Initialize the com task
            name_com_task_ = argv[args_ctr+1];
            task_com_ = (scl::CTaskComPos*)(ctrl_->getTask(name_com_task_));
            if(S_NULL == task_com_)
            { throw(std::runtime_error(std::string("Could not find specified com task: ")+name_com_task_));  }
            task_ds_com_ = dynamic_cast<scl::STaskComPos*>(task_com_->getTaskData());
            if(S_NULL == task_ds_com_)
            { throw(std::runtime_error("Error. The com task's data structure is NULL."));  }
            ui_pt_com_ = ui_points_used; ui_points_used++;
            has_been_init_com_task_ = true;

#ifdef GRAPHICS_ON
            /** Render a sphere at the com-point task's position */
            flag = chai_gr_.addSphereToRender(Eigen::Vector3d::Zero(), chai_com_pos_, 0.02);
            if(false == flag) { throw(std::runtime_error("Could not add sphere at com pos"));  }

            if(NULL == chai_com_pos_){ throw(std::runtime_error("Could not add sphere at com pos"));  }

            /** Render a sphere at the com-point task's position */
            flag = chai_gr_.addSphereToRender(Eigen::Vector3d::Zero(), chai_com_pos_des_, 0.02);
            if(false == flag) { throw(std::runtime_error("Could not add sphere at com desired pos"));  }

            if(NULL == chai_com_pos_des_){ throw(std::runtime_error("Could not add sphere at com desired pos"));  }

            //Make desired position red/orange
            cMaterial mat_red; mat_red.setRedFireBrick();
            chai_com_pos_des_->setMaterial(mat_red, false);
#endif
            args_ctr+=2; continue;
          }
          else
          { throw(std::runtime_error("Specified -com flag but did not specify com task's name"));  }
        }
        /* NOTE : ADD MORE COMMAND LINE PARSING OPTIONS IF REQUIRED */
        // else if (argv[args_ctr] == "-p")
        // { }
        else
        {
          std::cout<<"\n Possible example task options: -p (start paused) -l (log file), -com (com control task), -op (op point task)";
          args_ctr++;
        }
    }

      return true;
    }
    catch(std::exception &e)
    { std::cout<<"\nCExampleApp::initMyController() : "<<e.what(); }
    return false;
  }

  scl::sBool CExampleApp::registerCustomDynamicTypes()
  { return registerExampleTaskType();  }

  void CExampleApp::setInitialStateForUIAndDynamics()
  {
    //Compute dynamics and servo once to initialize matrices.
    robot_.computeDynamics();
    robot_.computeNonControlOperations();
    robot_.computeServo();
    robot_.setGeneralizedCoordinatesToZero();
    robot_.setGeneralizedVelocitiesToZero();
    robot_.setGeneralizedAccelerationsToZero();

    //Update the operational point tasks (if any)
    std::vector<SUiCtrlPointData>::iterator it,ite;
    for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
    {
      assert(it->has_been_init_);
      assert(NULL!=it->chai_pos_des_);
      assert(NULL!=it->task_);
      it->task_->getPos(it->pos_);
#ifdef DEBUG
      if( (3 != it->pos_.rows() && 1 != it->pos_.cols()) ||
          (1 != it->pos_.rows() && 3 != it->pos_.cols()) )
      { assert(false);  }
#endif
      db_->s_gui_.ui_point_[it->ui_pt_] = it->pos_;

      //Using a tmp ref to simplify code.
      Eigen::Vector3d& tmp_ref = db_->s_gui_.ui_point_[it->ui_pt_];
      it->chai_pos_des_->setLocalPos(tmp_ref(0),tmp_ref(1),tmp_ref(2));
    }

    if(has_been_init_com_task_) //Update the com task (if any)
    {
      assert(NULL!=chai_com_pos_);
      assert(NULL!=chai_com_pos_des_);
      db_->s_gui_.ui_point_[ui_pt_com_] = task_ds_com_->x_;
      chai_com_pos_->setLocalPos(task_ds_com_->x_(0), task_ds_com_->x_(1), task_ds_com_->x_(2));
      Eigen::Vector3d& tmp_ref2 = db_->s_gui_.ui_point_[ui_pt_com_];
      chai_com_pos_des_->setLocalPos(tmp_ref2(0),tmp_ref2(1),tmp_ref2(2));
    }
  }

  void CExampleApp::stepMySimulation()
  {
    sutil::CSystemClock::tick(db_->sim_dt_);//Tick the clock.

    //Update the operational point tasks (if any)
    std::vector<SUiCtrlPointData>::iterator it,ite;
    for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
    { it->task_->setGoalPos(db_->s_gui_.ui_point_[it->ui_pt_]); } //Set the goal position.

    if(has_been_init_com_task_) //Update the com task (if any)
    { task_com_->setGoalPos(db_->s_gui_.ui_point_[ui_pt_com_]); }

    if(ctrl_ctr_%5 == 0)           //Update dynamics at a slower rate
    {
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
    }

    if(ctrl_ctr_%20 == 0)           //Update graphics and/or log at a slower rate
    { // Every 2ms
      robot_.logState(true,true,true);

      //Set the positions of the ui points
      for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
      {
        Eigen::Vector3d& tmp_ref = db_->s_gui_.ui_point_[it->ui_pt_];
        it->chai_pos_des_->setLocalPos(tmp_ref(0),tmp_ref(1),tmp_ref(2));

        it->task_->getPos(it->pos_);
        Eigen::VectorXd& tmp_ref2 = it->pos_;
        it->chai_pos_->setLocalPos(tmp_ref2(0),tmp_ref2(1),tmp_ref2(2));
      }

      if(has_been_init_com_task_)
      {
        chai_com_pos_->setLocalPos(task_ds_com_->x_(0),task_ds_com_->x_(1),task_ds_com_->x_(2));
        Eigen::Vector3d& tmp_ref2 = db_->s_gui_.ui_point_[ui_pt_com_];
        chai_com_pos_des_->setLocalPos(tmp_ref2(0),tmp_ref2(1),tmp_ref2(2));
      }
    }
    robot_.computeServo();           //Run the servo loop
    robot_.integrateDynamics();      //Integrate system

    ctrl_ctr_++;//Increment the counter for dynamics computed.
  }
}
