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
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/control/task/tasks/CTaskComPos.hpp>

#include <scl/graphics/chai/data_structs/SChaiGraphics.hpp>

namespace scl_app
{
  class CExampleApp : public scl::CRobotApp
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
    CExampleApp();

    /** Default destructor. Does nothing. */
    virtual ~CExampleApp(){}

    /** Sets up the task controller. */
    virtual scl::sBool initMyController(const std::vector<std::string>& argv,
        scl::sUInt args_parsed);

    /** Register any custom dynamic types that you have. */
    virtual scl::sBool registerCustomDynamicTypes();

    /** Sets all the ui points to their current position and
     * run the dynamics once to flush the state. */
    void setInitialStateForUIAndDynamics();
  };

}

#endif /* CEXAMPLEAPP_HPP_ */
