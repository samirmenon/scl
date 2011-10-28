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
/* \file CRobot.hpp
 *
 *  Created on: Dec 27, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CROBOT_HPP_
#define CROBOT_HPP_

#include <string>
#include <vector>

#include <scl/DataTypes.hpp>

#include <scl/Singletons.hpp>
#include <scl/robot/data_structs/SRobot.hpp>
#include <scl/dynamics/CDynamicsBase.hpp>
#include <scl/control/CControllerBase.hpp>

#include <sutil/CMappedList.hpp>

namespace scl
{
  /**
   * The one-stop-do-it-all class that encapsulates dynamics,
   * and a controller for a single robot.
   *
   * NOTE : This class manages its memory. Add pointers and forget
   *        about em!
   *
   * NOTE : You will, however, have to initialize the individual
   *        components yourself. */
  class CRobot
  {
  public:
    /** Constructor sets pointers to NULL */
    CRobot();

    /** Destructor does nothing */
    virtual ~CRobot();

    /** Convenience function. Pulls all the data from the database.
     * Initializes the robot:
     * 1. Verifies that the robot's data in the database is consistent
     * 2. DELETES all its existing data! (NOTE this carefully!)
     * 3. Reads the list of available controllers from the database,
     *    finds the controllers that match this robot, and initializes them.
     *    (a) Creates a controller object for each new controller
     *    (b) Adds the controller data-structure to this robot's database data structure
     *    (c) Sets the last controller as the current controller (default, can be changed) */
    sBool initFromDb(std::string arg_robot_name,
        CDynamicsBase* arg_dynamics,
        CDynamicsBase* arg_integrator);

    /** The actual data required to initialize a robot.
     * Initializes the robot:
     * 1. Verifies that the robot's data in the database is consistent
     * 2. DELETES all its existing data! (NOTE this carefully!)
     * 3. Reads the list of available controllers from the database,
     *    finds the controllers that match this robot, and initializes them.
     *    (a) Creates a controller object for each new controller
     *    (b) Adds the controller data-structure to this robot's database data structure
     *    (c) Sets the last controller as the current controller (default, can be changed) */
    sBool init(std::string arg_robot_name,
        CDynamicsBase* arg_dynamics,
        CDynamicsBase* arg_integrator,
        SWorldParsedData *arg_world,
        SRobotParsedData *arg_robot,
        SRobotIOData *arg_io_data,
        std::vector<SControllerBase*>& arg_ctrls);

    /** Adds a controller based on the initialization information passed
     * in a data structure.
     * Does type-checking (the data structure has a type) and initializes
     * an appropriate controller (controllers are derived from the CControllerBase
     * API).
     * Prints a warning if the passed data structure is invalid.
     *
     * NOTE : init() adds all the specified robot's controllers from
     * the database by default. Ie. It automatically loads controllers defined
     * in the xml file!!
     *
     * Only use this function for controllers defined "in code"
     * */
    sBool addController(SControllerBase* arg_ctrl_ds);

    /** Computes the robot's command torques.
     * Asserts false in debug mode if something bad happens */
    void computeServo();

    /** Computes the robot's dynamic model.
     * Asserts false in debug mode if something bad happens */
    void computeDynamics();

    /** Integrates the robot's dynamics (physics model)
     * Asserts false in debug mode if something bad happens */
    void integrateDynamics();

    /** Initialization state */
    sBool hasBeenInit();

    /** Turn velocity damping on or off. Turning it on will
     * make the robot lose some (1% default) velocity each
     * second */
    void setFlagApplyDamping(sBool arg_flag);

    /** Sets the velocity damping for each joint */
    sBool setDamping(Eigen::VectorXd arg_d);

    /** Turn the actuator limits on or off. Simulates physical
     * force limits of the actuators */
    void setFlagApplyActuatorForceLimits(sBool arg_flag);

    /** Sets the actuator limits for each joint */
    sBool setActuatorForceLimits(Eigen::VectorXd arg_max,Eigen::VectorXd arg_min);

    /** Turn the controller on or off. Controller sends zero
     * command gc forces if off. */
    void setFlagControllerOn(sBool arg_flag);

    /** Selects the passed controller if it exists and has been initialized
     * Returns false if the controller doesn't exist*/
    sBool setController(std::string arg_ctrl_name);

    /** Gets access to the current controller data structure */
    CControllerBase* getCurrentController();

    /** Gets access to the current controller data structure */
    SControllerBase* getController(std::string arg_ctrl_name);

    /** Returns a pointer to the robot's data structure */
    SRobot* getData();

  private:
    /** The data is available in the database for other parts of the
     * program to see.
     *
     * Programs can use it to interact with controllers that
     * don't support polling data structures in the database.
     *
     * However, the recommended way is to communicate through the
     * I/O data section of the database. */
    SRobot data_;

    SDatabase *db_;

    CDynamicsBase* dynamics_;
    CDynamicsBase* integrator_;
    sutil::CMappedList<std::string,CControllerBase*> ctrl_;
    CControllerBase* ctrl_current_;
  };

}

#endif /* CROBOT_HPP_ */
