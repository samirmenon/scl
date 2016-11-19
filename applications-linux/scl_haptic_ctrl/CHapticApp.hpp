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

#include <scl/scl.hpp>

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
    virtual scl::sBool setInitialStateForUIAndDynamics();

  private:
    // ****************************************************
    //                      The data
    // ****************************************************
    //For controlling op points with haptics
    //The app will support dual-mode control, with the haptics controlling op points.
    scl::CHapticsChai haptics_;
    scl::sUInt num_haptic_devices_to_use_; //These will directly control ui-points.
    scl::sBool has_been_init_haptics_;
    std::vector<Eigen::VectorXd> haptic_pos_;
    std::vector<Eigen::VectorXd> haptic_base_pos_;
  };

}

#endif /* CHAPTICAPP_HPP_ */
