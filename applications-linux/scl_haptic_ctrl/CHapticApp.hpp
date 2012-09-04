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
/* \file CHapticApp.hpp
 *
 *  Created on: Sep 3, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CHAPTICAPP_HPP_
#define CHAPTICAPP_HPP_

#include <scl/robot/CRobotApp.hpp>

#include <scl/control/task/CTaskController.hpp>
#include <scl/control/task/tasks/COpPointTask.hpp>
#include <scl/control/task/tasks/CComPosTask.hpp>

#include <scl/graphics/chai/data_structs/SChaiGraphics.hpp>

namespace scl_app
{
  class CHapticApp : public scl::CRobotApp
  {
  public:
    // ****************************************************
    //                 The main functions
    // ****************************************************
    /** Runs the task controller. */
    virtual void stepMySimulation();

    // ****************************************************
    //           The initialization functions
    // ****************************************************
    /** Default constructor. Sets stuff to zero. */
    CHapticApp();

    /** Default destructor. Does nothing. */
    virtual ~CHapticApp(){}

    /** Sets up the task controller. */
    virtual scl::sBool initMyController(const std::vector<std::string>& argv,
        scl::sUInt args_parsed);

    /** Register any custom dynamic types that you have. */
    virtual scl::sBool registerCustomDynamicTypes();

    /** Sets all the ui points to their current position and
     * run the dynamics once to flush the state. */
    void setInitialStateForUIAndDynamics();

  private:
    // ****************************************************
    //                      The data
    // ****************************************************
    scl::CTaskController* ctrl_;           //Use a task controller

    /** This is an internal class for organizing the control-task
     * to ui-point connection through the keyboard or an external
     * haptic device */
    class SOpPointUiLinkData
    {
    public:
      std::string name_;
      scl::COpPointTask* task_;
      scl::SOpPointTask* task_ds_;
      scl::sInt ui_pt_;
      scl::sBool has_been_init_;
      cGenericObject *chai_pos_,*chai_pos_des_;
      SOpPointUiLinkData() :
        name_(""),task_(NULL), task_ds_(NULL),ui_pt_(-1),
        has_been_init_(false), chai_pos_(NULL), chai_pos_des_(NULL){}
    };
    /** The operational points that will be linked to keyboard handlers */
    std::vector<SOpPointUiLinkData> taskvec_op_point_;

    //For controlling the com task with a ui point
    std::string name_com_task_;
    scl::CComPosTask* task_com_;
    scl::SComPosTask* task_ds_com_;
    scl::sInt ui_pt_com_;
    scl::sBool has_been_init_com_task_;
    cGenericObject *chai_com_pos_,*chai_com_pos_des_;
  };

}

#endif /* CHAPTICAPP_HPP_ */