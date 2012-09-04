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
/* \file CHapticApp.cpp
 *
 *  Created on: Sep 3, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CHapticApp.hpp"


#include <scl/DataTypes.hpp>
#include <scl/data_structs/SDatabase.hpp>

#include <sutil/CSystemClock.hpp>

#include <chai3d.h>

#include <iostream>
#include <stdexcept>

namespace scl_app
{

  /** Default constructor. Sets stuff to zero.
   * Uses a task controller*/
  CHapticApp::CHapticApp() :
      ctrl_(S_NULL),
      name_com_task_(""),
      task_com_(S_NULL),
      task_ds_com_(S_NULL),
      ui_pt_com_(-1),
      has_been_init_com_task_(false),
      chai_com_pos_(S_NULL),
      chai_com_pos_des_(S_NULL),
      num_haptic_devices_to_use_(0),
      has_been_init_haptics_(false)
  { }

  scl::sBool CHapticApp::initMyController(const std::vector<std::string>& argv,
      scl::sUInt args_parsed)
  {
    bool flag;
    try
    {
      //First set up the haptics
      flag = haptics_.connectToDevices();
      if(false == flag)
      { throw(std::runtime_error("Could not get connect to haptic devices"));  }

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
        if ("-l" == argv[args_ctr])
        {// We know the next argument *should* be the log file's name
          if(args_ctr+1 <= argv.size())
          {
            flag = robot_.setLogFile(argv[args_ctr+1]);
            if(false == flag) { throw(std::runtime_error("Could not set up log file"));  }
            args_ctr = args_ctr+2;
          }
          else
          { throw(std::runtime_error("Specified -l flag but did not specify log file"));  }
        }
        else if ("-com" == argv[args_ctr])
        {// We know the next argument *should* be the com pos task's name
          if(args_ctr+1 <= argv.size())
          {
            if(ui_points_used >= SCL_NUM_UI_POINTS)
            {
              std::cout<<"\nThe keyboard can only support "<<SCL_NUM_UI_POINTS<<" ui points. Can't add: "
                  << argv[args_ctr]<<" "<< argv[args_ctr+1];
              args_ctr+=2; continue;
            }
            //Initialize the com task
            name_com_task_ = argv[args_ctr+1];
            task_com_ = (scl::CComPosTask*)(ctrl_->getTask(name_com_task_));
            if(S_NULL == task_com_)
            { throw(std::runtime_error(std::string("Could not find specified com task: ")+name_com_task_));  }
            task_ds_com_ = dynamic_cast<scl::SComPosTask*>(task_com_->getTaskData());
            if(S_NULL == task_ds_com_)
            { throw(std::runtime_error("Error. The com task's data structure is NULL."));  }
            ui_pt_com_ = ui_points_used; ui_points_used++;
            has_been_init_com_task_ = true;

#ifdef GRAPHICS_ON
            /** Render a sphere at the op-point task's position */
            flag = chai_gr_.addSphereToRender(Eigen::Vector3d::Zero(), chai_com_pos_, 0.02);
            if(false == flag) { throw(std::runtime_error("Could not add sphere at com pos"));  }

            if(NULL == chai_com_pos_){ throw(std::runtime_error("Could not add sphere at com pos"));  }

            /** Render a sphere at the op-point task's position */
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
        else if ("-op" == argv[args_ctr])
        {// We know the next argument *should* be the com pos task's name
          if(args_ctr+1 <= argv.size())
          {
            if(ui_points_used >= SCL_NUM_UI_POINTS)
            {
              std::cout<<"\nThe keyboard can only support "<<SCL_NUM_UI_POINTS<<" ui points. Can't add: "
                  << argv[args_ctr]<<" "<< argv[args_ctr+1];
              args_ctr+=2; continue;
            }

            //Initialize the com task
            SOpPointUiLinkData tmp_op;
            tmp_op.name_ = argv[args_ctr+1];
            tmp_op.task_ = (scl::COpPointTask*)(ctrl_->getTask(tmp_op.name_));
            if(S_NULL == tmp_op.task_)
            { throw(std::runtime_error(std::string("Could not find specified op point task: ")+tmp_op.name_));  }
            tmp_op.task_ds_ = dynamic_cast<scl::SOpPointTask*>(tmp_op.task_->getTaskData());
            if(S_NULL == tmp_op.task_ds_)
            { throw(std::runtime_error(std::string("Error. The op point task's data structure is NULL:"+tmp_op.name_)));  }
            tmp_op.ui_pt_ = ui_points_used; ui_points_used++;
            tmp_op.has_been_init_ = true;

#ifdef GRAPHICS_ON
            /** Render a sphere at the op-point task's position */
            flag = chai_gr_.addSphereToRender(robot_name_,tmp_op.task_ds_->link_ds_->name_,
                tmp_op.task_ds_->pos_in_parent_,0.02, &tmp_op.chai_pos_);
            if(false == flag) { throw(std::runtime_error("Could not add sphere at op task"));  }

            /** Render a sphere at the op-point task's position */
            flag = chai_gr_.addSphereToRender(Eigen::Vector3d::Zero(), tmp_op.chai_pos_des_, 0.02);
            if(false == flag) { throw(std::runtime_error("Could not add sphere at op desired pos"));  }

            if(NULL == tmp_op.chai_pos_des_){ throw(std::runtime_error("Could not add sphere at op desired pos"));  }

            //Make desired position red/orange
            cMaterial mat_red; mat_red.setRedFireBrick();
            tmp_op.chai_pos_des_->setMaterial(mat_red, false);
#endif
            //Add the initialized task to the vector
            taskvec_op_point_.push_back(tmp_op);

            args_ctr+=2; continue;
          }
        }
        else if ("-haptics" == argv[args_ctr])
        {// We know the next argument *should* be the com pos task's name
          if(args_ctr+1 <= argv.size())
          {
            //Set up haptic devices
            std::stringstream ss(argv[args_ctr+1]);
            ss>>num_haptic_devices_to_use_;
            if(num_haptic_devices_to_use_ != haptics_.getNumDevicesConnected())
            {
              std::stringstream ss;
              ss<<"Error. Num haptic devices connected = "<<haptics_.getNumDevicesConnected()
                  <<". Num requested = "<<num_haptic_devices_to_use_;
              throw(std::runtime_error(ss.str()));
            }
            std::cout<<"\nInitializing ["<<num_haptic_devices_to_use_<<"] haptic devices to first ["
                <<num_haptic_devices_to_use_<<"] UI points.";
            has_been_init_haptics_ = true;
            for(int i=0;i<num_haptic_devices_to_use_;++i)
            {
              haptic_pos_.push_back(Eigen::Vector3d());
              haptic_base_pos_.push_back(Eigen::Vector3d());
            }
            args_ctr+=2; continue;
          }
          else
          { throw(std::runtime_error("Specified -haptics flag but did not specify <num_haptic_devices_to_use>"));  }
        }
        /* NOTE : ADD MORE COMMAND LINE PARSING OPTIONS IF REQUIRED */
        // else if (argv[args_ctr] == "-p")
        // { }
        else
        {
          std::cout<<"\n Possible example task options: -l (log file), -com (com control task), -op (op point task)";
          args_ctr++;
        }
    }

      return true;
    }
    catch(std::exception &e)
    { std::cout<<"\nCHapticApp::initMyController() : "<<e.what(); }
    return false;
  }

  scl::sBool CHapticApp::registerCustomDynamicTypes()
  { return true;  }//No custom tasks to register

  void CHapticApp::setInitialStateForUIAndDynamics()
  {
    //Compute dynamics and servo once to initialize matrices.
    robot_.computeDynamics();
    robot_.computeNonControlOperations();
    robot_.computeServo();

    //Update the operational point tasks (if any)
    std::vector<SOpPointUiLinkData>::iterator it,ite;
    for(it = taskvec_op_point_.begin(), ite = taskvec_op_point_.end(); it!=ite; ++it )
    {
      assert(it->has_been_init_);
      assert(NULL!=it->chai_pos_des_);
      assert(NULL!=it->task_);
      assert(NULL!=it->task_ds_);
      db_->s_gui_.ui_point_[it->ui_pt_] = it->task_ds_->x_;
      it->chai_pos_des_->setLocalPos(db_->s_gui_.ui_point_[it->ui_pt_]);
    }

    if(has_been_init_com_task_) //Update the com task (if any)
    {
      assert(NULL!=chai_com_pos_);
      assert(NULL!=chai_com_pos_des_);
      db_->s_gui_.ui_point_[ui_pt_com_] = task_ds_com_->x_;
      chai_com_pos_->setLocalPos(task_ds_com_->x_);
      chai_com_pos_des_->setLocalPos(db_->s_gui_.ui_point_[ui_pt_com_]);
    }

    std::vector<Eigen::Vector3d>::iterator itv,itve;
    scl::sInt i;
    for(i=0, itv = haptic_base_pos_.begin(), itve = haptic_base_pos_.end();
        itv!=itve;++itv, ++i)
    {
      *itv = db_->s_gui_.ui_point_[i];
      std::cout<<"\nBase position of haptic device ["<<i<<"] = "<<itv->transpose();
    }
  }

  void CHapticApp::stepMySimulation()
  {
    sutil::CSystemClock::tick(db_->sim_dt_);//Tick the clock.

    //Update the operational point tasks (if any)
    std::vector<SOpPointUiLinkData>::iterator it,ite;
    for(it = taskvec_op_point_.begin(), ite = taskvec_op_point_.end(); it!=ite; ++it )
    { it->task_->setGoal(db_->s_gui_.ui_point_[it->ui_pt_]); } //Set the goal position.

    if(has_been_init_com_task_) //Update the com task (if any)
    { task_com_->setGoal(db_->s_gui_.ui_point_[ui_pt_com_]); }

    if(ctrl_ctr_%5 == 0)           //Update dynamics at a slower rate
    {
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
    }

    if(ctrl_ctr_%20 == 0)           //Update graphics and/or log at a slower rate
    { // Every 2ms
      robot_.logState(true,true,true);

      if(has_been_init_haptics_)
      {//Update the position of the first few points using the haptics
        bool flag = haptics_.getHapticDevicePositions(haptic_pos_);
        assert(flag);
        std::vector<Eigen::Vector3d>::iterator it,ite, itv,itve;
        int i;
        for(i=0, it = haptic_pos_.begin(), ite = haptic_pos_.end(),
            itv = haptic_base_pos_.begin(), itve = haptic_base_pos_.end();
            it!=ite && itv!=itve;++i,++it,++itve)
        { db_->s_gui_.ui_point_[i] = (*it) + (*itv); }
        assert(itv == itve && it==ite);
      }

      //Set the positions of the ui points
      for(it = taskvec_op_point_.begin(), ite = taskvec_op_point_.end(); it!=ite; ++it )
      { it->chai_pos_des_->setLocalPos(db_->s_gui_.ui_point_[it->ui_pt_]); }

      if(has_been_init_com_task_)
      {// Update the com position.
        chai_com_pos_->setLocalPos(task_ds_com_->x_);
        chai_com_pos_des_->setLocalPos(db_->s_gui_.ui_point_[ui_pt_com_]);
      }
    }
    robot_.computeServo();           //Run the servo loop
    robot_.integrateDynamics();      //Integrate system

    ctrl_ctr_++;//Increment the counter for dynamics computed.
  }
}
