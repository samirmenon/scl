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
#include <scl/control/task/data_structs/STaskController.hpp>
#include <scl/control/gc/data_structs/SGcController.hpp>

//Parser data --> Static information about the robot
#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/data_structs/SGraphicsParsedData.hpp>
#include <scl/data_structs/SWorldParsedData.hpp>
#include <scl/data_structs/SRobotIOData.hpp>
#include <scl/data_structs/SMuscleSystem.hpp>

//Yes. I know this is a hack. Remove the day we move CHAI
//into mainstream. Right now all the code is GPL because
//CHAI is GPL.
#ifndef SCL_USE_CHAI_GRAPHICS
#define SCL_USE_CHAI_GRAPHICS 1
#endif

#ifdef SCL_USE_CHAI_GRAPHICS
#include <scl/graphics/chai/data_structs/SChaiGraphics.hpp>
#endif

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
  /** Contains the parsed world : Global properties like Gravity etc.*/
  SWorldParsedData world_;

  /** Contains all the parsed graphics views. */
  sutil::CMappedList<std::string,SGraphicsParsedData> graphics_worlds_;

  /** Contains all the parsed robots. */
  sutil::CMappedList<std::string,SRobotParsedData> robots_;

  /** Contains all the parsed muscle systems */
  sutil::CMappedList<std::string,SMuscleSystem> muscle_systems_;
};

/** All the data updated by the controller
 *
 * Controller robots - One controller for each robot.
 *
 * No controller can exist unless its robot is defined by
 * the parser.
 *
 * NOTE : SRobotParsedData is the statically loaded robot information
 * while SCrRobot contains the information about the robot
 * that the controller dynamically generates (eg. matrices)
 *
 * Each controller has : SRobotParsedData + SCrRobot
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
   * */
  sutil::CMappedList<std::string,SControllerBase*> controllers_;

  /** Enables string access for controller task data. Each task
   * is stored along with its type as a std::pair. */
  sutil::CMappedList<std::string,STaskBase*> tasks_;

  ~SControllerData()
  {
    sutil::CMappedList<std::basic_string<char>, scl::SControllerBase*>::iterator it, ite;
    for(it = controllers_.begin(), ite = controllers_.end(); it!=ite; ++it)
    {
      delete *it;
      *it = NULL;
    }

    sutil::CMappedList<std::basic_string<char>, scl::STaskBase*>::iterator it2, it2e;
    for(it2 = tasks_.begin(), it2e = tasks_.end(); it2!=it2e; ++it2)
    {
      delete *it2;
      *it2 = NULL;
    }
  }

  /** Returns all the controllers initialized for a specific robot */
  sBool getControllersForRobot(const std::string & arg_robot,
      std::vector<SControllerBase*>& ret_controllers)
  {
    sutil::CMappedList<std::basic_string<char>, scl::SControllerBase*>::iterator it, ite;
    for(it = controllers_.begin(), ite = controllers_.end(); it!=ite; ++it)
    {
      SControllerBase* tmp = *it;

      if(S_NULL == tmp->robot_)
      {
        std::cout<<"SControllerData::getControllersForRobot() WARNING : Found NULL robot controller ds in the pile.";
      }

      if(tmp->robot_->name_ == arg_robot)
      { ret_controllers.push_back(tmp); }
    }
    return true;
  }
};

/** All the data updated by the gui
 *
 * NOTE :
 * The chai graphics are NOT compiled into
 * scl's dynamic lib. To enable chai graphics
 * 1. Set a compile flag:
 *
 * #define SCL_USE_CHAI_GRAPHICS 1
 *
 * 2. And while building the "application/executable", be sure to
 * link with the chai library. */
struct SGuiData
{
  // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
  // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //NOTE TODO : Remove the ifdef and the PileMap and use a std::map instead
  //All API implementing classes will NOT store their data in the database.
#ifdef SCL_USE_CHAI_GRAPHICS
  /** A chai data structure :
   *
   * NOTE : Directly accessing the robot's configuration to render
   * it is complicated by the fact that each chai object's transformation
   * matrix (from the previous link) must be continuously updated as the joint
   * angles change.
   */
  sutil::CMappedList<std::string,scl::SChaiGraphics> chai_data_;
#endif

  Eigen::Vector3d ui_point_1_,ui_point_2_, ui_point_[2];

  /** 0 = All points by themselves.
   *  1 = All points together*/
  sUInt ui_point_selector_1_;

  sBool glut_initialized_;

  SGuiData() : glut_initialized_(false) {ui_point_selector_1_=0;}
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
  sutil::CMappedList<std::string, SRobotIOData> io_data_;
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

  SDatabase()
  {
    sim_ticks_ = 0;
    sim_dt_ = 0.0001;//0.1ms
    time_ = 0.0;
    cwd_ = "";
    dir_specs_ = "";
    running_ = true;
    pause_ctrl_dyn_ = false;
    pause_graphics_ = false;
  }
};

}

#endif /* SDATABASE_HPP_ */
