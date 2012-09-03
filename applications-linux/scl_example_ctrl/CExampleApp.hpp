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
/* \file CExampleApp.hpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CEXAMPLEAPP_HPP_
#define CEXAMPLEAPP_HPP_

#include <scl/robot/CRobotApp.hpp>

#include <scl/control/task/CTaskController.hpp>
#include <scl/control/task/tasks/COpPointTask.hpp>
#include <scl/control/task/tasks/CComPosTask.hpp>

namespace scl_app
{
  class CExampleApp : public scl::CRobotApp
  {
  private:
    scl::CTaskController* ctrl;           //Use a task controller

    class SOpPointUiLinkData
    {
    public:
      std::string name_;
      scl::COpPointTask* task_;
      scl::SOpPointTask* task_ds_;
      scl::sInt ui_pt_;
      scl::sBool has_been_init_;
      SOpPointUiLinkData() :
        name_(""),task_(NULL), task_ds_(NULL),ui_pt_(-1),
        has_been_init_(false){}
    };
    /** The operational points that will be linked to keyboard handlers */
    std::vector<SOpPointUiLinkData> taskvec_op_point_;

    //For controlling the com task with a ui point
    std::string name_com_task_;
    scl::CComPosTask* task_com_;
    scl::SComPosTask* task_ds_com_;
    scl::sInt ui_pt_com_;
    scl::sBool has_been_init_com_task_;

  public:
    /** Default constructor. Sets stuff to zero. */
    CExampleApp();

    /** Default destructor. Does nothing. */
    virtual ~CExampleApp(){}

    /** Sets up the task controller. */
    virtual scl::sBool initMyController(const std::vector<std::string>& argv,
        scl::sUInt args_parsed);

    /** Register any custom dynamic types that you have. */
    virtual scl::sBool registerCustomDynamicTypes();

    /** Runs the task controller. */
    virtual void stepMySimulation();
  };

}

#endif /* CEXAMPLEAPP_HPP_ */
