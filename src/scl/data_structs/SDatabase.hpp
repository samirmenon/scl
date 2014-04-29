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
/* \file SDatabase.hpp
 *
 *  Created on: Jun 21, 2010
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

/** \file SDatabase.hpp */

#ifndef SDATABASE_HPP_
#define SDATABASE_HPP_

//Controller data --> Dynamically updated information.
#include <scl/control/task/data_structs/SControllerMultiTask.hpp>
#include <scl/control/gc/data_structs/SControllerGc.hpp>

//Parser data --> Static information about the robot
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SGraphicsParsed.hpp>
#include <scl/data_structs/SRobotIO.hpp>
#include <scl/data_structs/SMuscleSetParsed.hpp>
#include <scl/data_structs/SUIParsed.hpp>

#include <scl/graphics/chai/data_structs/SGraphicsChai.hpp>

//3rdparty Utils
#include <sutil/CMappedList.hpp>
#include <sutil/CMappedMultiLevelList.hpp>

#include <map>
#include <string>
#include <iostream>

namespace scl {

/** All the static data required to initialize a robotic
 * simulation.
 *
 * Supports name based robot data lookup. */
struct SParserData
{
  /** Contains all the parsed graphics views. */
  sutil::CMappedList<std::string,SGraphicsParsed> graphics_worlds_;

  /** Contains all the parsed robots. */
  sutil::CMappedList<std::string,SRobotParsed> robots_;

  /** Contains all the parsed muscle systems
   *
   * NOTE TODO : Muscle systems should be attached to parsed robot
   * specifications. They can't do anything independent of the
   * actual robot spec anyway. The original idea was to allow multiple
   * muscle specs for a robot, but that's overkill. More modular to
   * just redefine the robot as something else and give it a new
   * muscle spec. */
  sutil::CMappedList<std::string,SMuscleSetParsed> muscle_systems_;

  /** The parsed gui specification */
  sutil::CMappedList<std::string,SUIParsed> user_interface_;

  /** The config file in which the robot and muscle system is specified.
   * This is typically set by a database registration function.
   * NOTE : It is not set in the parser. */
  std::string file_name_;
};

/** All the data updated by the controller
 *
 * Controller robots - Atleast one controller for each robot.
 *                     Multiple tasks for each controller (in a hierarchy)
 *
 * No controller can exist unless its robot is defined by
 * the parser.
 *
 * NOTE : SRobotParsed is the statically loaded robot information
 * while SCrRobot contains the information about the robot
 * that the controller dynamically generates (eg. matrices)
 *
 * Each controller has : SRobotParsed + SCrRobot
 *
 * Supports twofold robot lookup:
 * 1. Based on the static robot data (Useful if iterating
 * through parsed robot data stored in SParserData)
 *
 * 2. Based on the robot's name */
class SControllerData
{
public:
  /** Enables string access for controller data.
   * NOTE : Each controller is stored along with it's name
   * in a pilemap.
   *
   * A mapped pointer list so that it can manage memory for
   * subclasses as well (by storing and later deleting their
   * pointers, instead of allocating and storing objects
   * themselves).
   * */
  sutil::CMappedPointerList<std::string,SControllerBase, true> controllers_;

  /** Enables string access for controller task data. Each task
   * is stored along with its type as a std::pair.
   * A mapped pointer list so that it can manage memory for
   * subclasses as well (by storing and later deleting their
   * pointers, instead of allocating and storing objects
   * themselves).
   * */
  sutil::CMappedPointerList<std::string,STaskBase, true> tasks_;

  /** Enables string access for non-controller task data. Each task
   * is stored along with its type as a std::pair.
   * A mapped pointer list so that it can manage memory for
   * subclasses as well (by storing and later deleting their
   * pointers, instead of allocating and storing objects
   * themselves).
   * */
  sutil::CMappedPointerList<std::string,SNonControlTaskBase, true> tasks_non_ctrl_;

  /** Returns all the controllers initialized for a specific robot */
  sBool getControllersForRobot(const std::string & arg_robot,
      std::vector<SControllerBase*>& ret_controllers)
  {
    ret_controllers.clear();
    sutil::CMappedPointerList<std::string, scl::SControllerBase, true>::iterator it, ite;
    for(it = controllers_.begin(), ite = controllers_.end(); it!=ite; ++it)
    {
      SControllerBase* tmp = *it;

      if(S_NULL == tmp->robot_)
      {
        std::cout<<"SControllerData::getControllersForRobot()"
            <<"WARNING : Found NULL robot controller ds in the pile.";
        return false;
      }

      if(tmp->robot_->name_ == arg_robot)
      { ret_controllers.push_back(tmp); }
    }
    if(0 < ret_controllers.size())
    { return true;  }
    else
    { return false; }
  }
};

/** The number of UI points (3-d vectors) whose key-handlers are supported. Use these in your app. */
#define SCL_NUM_UI_POINTS 12
/** The number of UI flags (bool) whose key-handlers are supported. Use these in your app. */
#define SCL_NUM_UI_FLAGS 9

/** All the data updated by the gui
 *
 * NOTE : The chai graphics are NOT compiled into scl's dynamic lib.
 * To enable chai graphics while building an "application/executable",
 * be sure to link with the chai library. */
struct SGuiData
{
  // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
  // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** A chai data structure :
   *
   * NOTE : Directly accessing the robot's configuration to render
   * it is complicated by the fact that each chai object's transformation
   * matrix (from the previous link) must be continuously updated as the joint
   * angles change.
   */
  sutil::CMappedList<std::string,scl::SGraphicsChai> chai_data_;

  /** Users may use these ui points in their controller for keyboard based control */
  Eigen::Vector3d ui_point_[SCL_NUM_UI_POINTS];

  sBool ui_flag_[SCL_NUM_UI_FLAGS];

  /** 0 = All points by themselves.
   *  1 = All points together*/
  sUInt ui_point_selector_;

  sBool glut_initialized_;

  SGuiData() : glut_initialized_(false) {ui_point_selector_=0;}
};

/** Contains all the dynamically updated input output data.
 *
 * Wbc's modules (controller, graphics, simulator etc) typically
 * either read or write (not both) to the IO module.
 *
 * Egs.
 * Input Data : Sensor data from a robot. Simulation logs.
 * Output Data : Actuator command torques. Simulation forces.
 */
struct SIOData
{
  /** A pile of io data objects containing robot sensor + actuator data.
   * String ID = Robot name
   * Each robot thus has a unique IO data structure. */
  sutil::CMappedList<std::string, SRobotIO> io_data_;
};


/** A data structure to enable the many different components
 * to access data.
 *
 * NOTE : This structure may be directly accessed with
 * pointers by high-performance code. However, this is
 * only meant for advanced users because direct access
 * is NOT thread-safe.
 *
 * Normally, the CDatabase wrapper should be used
 * to access the data. */
class SDatabase{
public:
  /*******************************************
   ********************************************
   * The Static Data:
   * Doesn't change during runtime
   * [[Zero writers, multiple readers]]
   ********************************************/
  /** Stores all the data obtained by parsing a file etc. */
  SParserData s_parser_;

  /*******************************************
   ********************************************
   * The Dynamic Data:
   * Changes during runtime
   * [[Single writer, multiple readers]]
   *
   * NOTE: The programmer is responsible for
   * ensuring threaded safety.
   ********************************************/
  /** Stores all the data generated by the controller as it runs.
   * [[Interface to the controller]] */
  SControllerData s_controller_;

  /** Stores all the sensor data obtained from the simulation/real world
   * [[Interface to the physics engine or real robot]] */
  SIOData s_io_;

  /** Stores all the data generated by the gui as it runs.
   * [[Interface to the graphics/rendering subsystem]] */
  SGuiData s_gui_;

  /***********
   * Time data:
   ************/
  /** Simulation Time : Ie. The number of iterations of the simulation */
  sLongLong sim_ticks_;

  /** The simulation dt (assuming a constant integration interval or a
   * constant integrator sampling interval). */
  sFloat sim_dt_;

  /** Real time : Time in seconds since the simulation started */
  sFloat time_;

  /** The current working directory */
  std::string cwd_;

  /** The specs directory */
  std::string dir_specs_;

  /***********
   * Exec data:
   ************/
  /** Whether the program is on or not */
  sBool running_;

  /** Whether the controller+dynamics are paused or not */
  sBool pause_ctrl_dyn_;

  /** Whether the dynamics should pause after the next iteration */
  sBool step_ctrl_dyn_;

  /** Whether the graphics updates are paused or not */
  sBool pause_graphics_;

  /** Whether the logging is on or not */
  sBool param_logging_on_;

  SDatabase()
  {
    sim_ticks_ = 0;
#ifdef SCL_HIGH_QUALITY_SIMULATION
    sim_dt_ = 0.00005;//0.05ms
#else
    sim_dt_ = 0.0005;//0.5ms
#endif
    time_ = 0.0;
    cwd_ = "";
    dir_specs_ = "";
    running_ = true;
    pause_ctrl_dyn_ = false;
    step_ctrl_dyn_ = false;
    pause_graphics_ = false;
    param_logging_on_ = false;
  }
};

}

#endif /* SDATABASE_HPP_ */
