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
/* \file CRobotApp.hpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CROBOTAPP_HPP_
#define CROBOTAPP_HPP_

//Standard includes
#include <scl/DataTypes.hpp>
#include <scl/robot/CRobot.hpp>
#include <scl/dynamics/tao/CTaoDynamics.hpp>

#ifdef GRAPHICS_ON
#include <scl/graphics/chai/CChaiGraphics.hpp>
#endif

namespace scl
{
  /** Generic code required to run a scl simulation
   * A simulation requires running 3 things. This example uses:
   * 1. A dynamics/physics engine                  :  Tao
   * 2. A controller                               :  Wbc
   * 3. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
   *
   * It allows a single threaded mode (good for debugging) and a multi-threaded
   * mode (good for optimized binaries).
   */
  class CRobotApp
  {
  public:
    /***********************************************************************
     * NOTE : You must subclass and implement these to suit your application
     ************************************************************************/
    /** Implement this function. */
    virtual void stepMySimulation()=0;

    /** Implement this function. */
    virtual scl::sBool initMyController(const std::vector<std::string>&)=0;

    /** Register any custom dynamic types that you have. */
    virtual scl::sBool registerCustomDynamicTypes()
    { return true;  }

  public:
    /************************************************************************/
    /***************NOTE : You should NOT need to modify these **************/
    /****************       But feel free to use the objects   **************/
    /************************************************************************/

    /** Default constructor. Does nothing */
    CRobotApp() {}

    /** Default destructor. Does nothing */
    virtual ~CRobotApp(){}

    /** Initialize the basic global variables, database etc.*/
    scl::sBool init(const std::vector<std::string>& argv);

#ifdef GRAPHICS_ON
    scl::sBool initGraphics(
        /** The time interval between graphics updates. In nsec.
         * A good value is {0, 15000000} */
        const timespec& arg_ts,
        /** The number of graphics frames to skip in single threaded mode */
        const int arg_gr_frm_ctr);
#endif

    /** Terminates the simulation and prints some statistics */
    void terminate();

    /** Runs a simulation using two threads:
     * Thread 1: Computes the robot dynamics
     * Thread 2: Renders the graphics and handles gui interaction */
    void runMainLoopThreaded(const int& thread_id);

    /** Runs a simulation using one thread.
     * 1: Computes the robot dynamics
     * 2: Renders the graphics and handles gui interaction */
    void runMainLoop();

    /** Runs a console shell that enables you to modify
     * the app through your commands (provided you've
     * registered them with the callback registry.
     *
     * NOTE : The console runs in two modes:
     *
     * Mode 1 : String lookup:
     * function name  = string
     * arguments      = vector<string>
     *
     * Mode 2 : Char lookup:
     * function name  = char
     * is_caps?       = bool
     *
     * Default starts in string lookup. Press 'x' to switch between
     * modes.
     *
     * (See the callback registry sutil/CCallbackRegistry.hpp for
     * more details) */
    void runConsoleShell();

    //Data types. Feel free to use them.
    scl::SDatabase* db;                 //Generic database (for sharing data)

    std::vector<std::string> robots_parsed;   //Parsed robots

    std::string robot_name;             //Currently selected robot
    std::string ctrl_name;              //Currently selected controller

    scl::CRobot robot;                  //Generic robot
    scl::SRobotIOData* rob_io_ds;       //Access the robot's sensors and actuators

    scl::CTaoDynamics* tao_dyn;         //Generic tao dynamics

    scl::sLongLong ctrl_ctr;            //Controller computation counter
    scl::sFloat t_start, t_end;         //Start and end times

    //Graphics stuff
#ifdef GRAPHICS_ON
    std::vector<std::string> graphics_parsed; //Parsed graphics views
    scl::CChaiGraphics chai_gr;         //Generic chai graphics
    scl::sLongLong gr_ctr;              //Graphic update counter
    timespec ts_;                       //Sleep for graphics (nanoseconds)
    int gr_frm_skip_;                   //Graphics frames to skip (single-threaded mode)
    int gr_frm_ctr_;                    //Graphics frame counter (single-threaded mode)
#endif
  };
}

#endif /* CROBOTAPP_HPP_ */
