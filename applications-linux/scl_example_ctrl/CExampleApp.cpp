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

#include <sutil/CSystemClock.hpp>

#include <iostream>
#include <stdexcept>

namespace scl_app
{

  /** Default constructor. Sets stuff to zero. */
  CExampleApp::CExampleApp()
  {
    ctrl = S_NULL;           //Use a task controller
    op_link_name = "not_init";
    op_link2_name = "not_init";
    tsk = S_NULL; tsk2 = S_NULL;;
    tsk_ds = S_NULL; tsk2_ds = S_NULL;
    op_link_set = false; op_link2_set = false;
  }

  scl::sBool CExampleApp::initMyController(const std::vector<std::string>& argv)
  {
    bool flag;
    try
    {
      /** Initialize Single Control Task */
      ctrl = (scl::CTaskController*) robot.getCurrentController();
      if(S_NULL == ctrl)
      { throw(std::runtime_error("Could not get current controller"));  }

      //Initialize the first op point controller
      op_link_name = argv[4];
      op_link_set = true;
      db->s_gui_.ui_point_1_<<0,0.1,0; //Ctrl tracks this control point.
      tsk = (scl::COpPointTask*)(ctrl->getTask(op_link_name));
      if(S_NULL == tsk)
      { throw(std::runtime_error("Could not get specified task"));  }
      tsk_ds = dynamic_cast<scl::SOpPointTask*>(tsk->getTaskData());

#ifdef GRAPHICS_ON
      /** Render a sphere at the op-point task's position */
      flag = chai_gr.addSphereToRender(robot_name,tsk_ds->link_ds_->name_,tsk_ds->pos_in_parent_);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at op task"));  }
#endif

      //If second op point name was passed, use it.
      if(6 == argv.size())
      {
        op_link2_name = argv[5];
        op_link2_set = true;
        db->s_gui_.ui_point_2_<<0,-0.1,0; //Ctrl2 tracks this control point.
        tsk2 = (scl::COpPointTask*)(ctrl->getTask(op_link2_name));
        if(S_NULL == tsk2)
        { throw(std::runtime_error("Could not get specified task"));  }
        tsk2_ds = dynamic_cast<scl::SOpPointTask*>(tsk2->getTaskData());

#ifdef GRAPHICS_ON
        /** Render a sphere at the op-point task's position */
        flag = chai_gr.addSphereToRender(robot_name,tsk2_ds->link_ds_->name_,tsk2_ds->pos_in_parent_);
        if(false == flag) { throw(std::runtime_error("Could not add sphere at op task"));  }
#endif
      }

      return true;
    }
    catch(std::exception &e)
    { std::cout<<"\nCExampleApp::initMyController() : "<<e.what(); }
    return false;
  }

  scl::sBool CExampleApp::registerCustomDynamicTypes()
  { return registerExampleTaskType();  }

  void CExampleApp::stepMySimulation()
  {
    sutil::CSystemClock::tick(db->sim_dt_);//Tick the clock.

    tsk->setGoal(db->s_gui_.ui_point_1_); //Interact with the gui

    if(op_link2_set)//Use only if the second task was also initialized.
    { tsk2->setGoal(db->s_gui_.ui_point_2_); }

    if(ctrl_ctr%100 == 0)           //Update dynamics at a slower rate
    { robot.computeDynamics(); }
    robot.computeServo();           //Run the servo loop
    robot.integrateDynamics();      //Integrate system

    ctrl_ctr++;//Increment the counter for dynamics computed.
  }
}
