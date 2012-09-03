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

    std::string op_link_name,op_link2_name;
    scl::COpPointTask* tsk, *tsk2;
    scl::SOpPointTask* tsk_ds, *tsk2_ds;
    scl::sBool op_link_set, op_link2_set;

    //For controlling the com task with a ui point
    std::string com_task_name_;
    scl::CComPosTask* com_tsk_;
    scl::SComPosTask* com_tsk_ds_;
    scl::sBool com_tsk_set_;

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
