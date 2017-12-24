/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
