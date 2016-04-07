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
#include <scl/control/task/CTaskBase.hpp>
#include <scl/robot/CRobot.hpp>

#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>

#ifdef GRAPHICS_ON
#include <scl/graphics/chai/CGraphicsChai.hpp>
#endif

namespace scl
{
  /** Generic code to run a scl simulation.
   *
   * A simulation requires running 3 things. This example uses:
   * 1. A dynamics/physics engine                  :  Tao
   * 2. A controller                               :  Wbc
   * 3. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
   *
   * It allows a single threaded mode (good for debugging) and a multi-threaded
   * mode (good for optimized binaries).
   *
   * NOTE : You DO NOT have to use this class. As with everything else in
   * scl/robot, this class is merely a set of helper functions that wrap
   * around the core controller, dynamics and rendering. It is meant to
   * simplify your programs so feel free to build your own API if this doesn't
   * suit you...
   */
  class CRobotApp
  {
  public:
    /***********************************************************************
     * NOTE : You must subclass and implement these to suit your application
     *        Keep in mind that the CRobotApp does most things for you already.
     ************************************************************************/
    /** Implement this function. */
    virtual void stepMySimulation()=0;

    /** Implement this function.
     *
     * NOTE : The init() function initializes most things for you and then
     * automatically calls this function. */
    virtual scl::sBool initMyController(const std::vector<std::string>& argv,
        /** The CRobotApp's init function already parses some command line
         * arguments. This counter tells your controller implementation what
         * is already parsed. The implementation assumes that this function
         * will parse the remaining args. */
        sUInt args_already_parsed)=0;

    /** Register any custom dynamic types that you have. */
    virtual scl::sBool registerCustomDynamicTypes()
    { return true;  }

    /** Sets any vars to their initial position and
     * run the dynamics once to flush the state. */
    virtual scl::sBool setInitialStateForUIAndDynamics()=0;

  public:
    /************************************************************************/
    /***************NOTE : You should NOT need to modify these **************/
    /****************       But feel free to use the objects   **************/
    /************************************************************************/

    /** Default constructor. Sets stuff to zero. */
    CRobotApp();

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
    virtual void terminate();

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
    scl::SDatabase* db_;                 //Generic database (for sharing data)

    std::vector<std::string> robots_parsed_;   //Parsed robots

    std::string robot_name_;             //Currently selected robot
    std::string ctrl_name_;              //Currently selected controller

    scl::CRobot robot_;                  //Generic robot
    scl::SRobotParsed *rob_ds_;          //Generic robot data structure
    scl::SRobotIO* rob_io_ds_;           //Access the robot's sensors and actuators

    scl_ext::CDynamicsSclSpatial* dyn_scl_sp_;   //Generic scl spatial dynamics
    scl::CDynamicsScl* dyn_scl_;          //Generic scl dynamics

    scl::sLongLong ctrl_ctr_;             //Controller computation counter
    scl::sFloat t_start_, t_end_;         //Start and end times

    /** This is an internal class for organizing the control-task
     * to ui-point connection through the keyboard or an external
     * haptic device */
    class SUiCtrlPointData
    {
    public:
      std::string name_;
      scl::CTaskBase* task_;
      scl::sInt ui_pt_;
      scl::sBool has_been_init_;
#ifdef GRAPHICS_ON
      Eigen::VectorXd pos_, pos_des_;
      chai3d::cGenericObject *chai_pos_,*chai_pos_des_;
#endif
      SUiCtrlPointData() :
        name_(""),task_(NULL),ui_pt_(-1),
        has_been_init_(false)
#ifdef GRAPHICS_ON
        , chai_pos_(NULL), chai_pos_des_(NULL)
#endif
      {}
    };
    /** The operational points that will be linked to keyboard handlers.
     * NOTE : This is presently designed for a task controller's xyz ctrl tasks */
    std::vector<SUiCtrlPointData> taskvec_ui_ctrl_point_;

    //Graphics stuff
#ifdef GRAPHICS_ON
    std::vector<std::string> graphics_parsed_; //Parsed graphics views
    scl::CGraphicsChai chai_gr_;         //Generic chai graphics
    scl::sLongLong gr_ctr_;              //Graphic update counter
    timespec ts_;                       //Sleep for graphics (nanoseconds)
    int gr_frm_skip_;                   //Graphics frames to skip (single-threaded mode)
    int gr_frm_ctr_;                    //Graphics frame counter (single-threaded mode)
#endif
  };
}

#endif /* CROBOTAPP_HPP_ */
