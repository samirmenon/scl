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
/* \file SRobotIO.hpp
 *
 *  Created on: Jul 22, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 
#ifndef SROBOTIO_HPP_
#define SROBOTIO_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SForce.hpp>
#include <scl/actuation/data_structs/SActuatorSetBase.hpp>

#include <sutil/CMappedList.hpp>
#include <Eigen/Dense>

#include <string>
#include <vector>

namespace scl
{
  /** Contains generic robot sensor information. These
   * are read from the sensors and/or simulation. */
  class SRobotSensors
  {
  public:
    /** The sensed generalized coordinates (Eg. joint angles) */
    Eigen::VectorXd q_;

    /** The sensed generalized velocities (Eg. joint velocities) */
    Eigen::VectorXd  dq_;

    /** The sensed generalized accelerations (Eg. joint accelerations) */
    Eigen::VectorXd  ddq_;

    /** The sensed generalized forces (Eg. joint torques) */
    Eigen::VectorXd force_gc_measured_;

    /** The external forces applied on the robot. The code
     * expects these to be numerous O(n^k), k>1. These may
     * also appear/disappear frequently:
     * Eg.
     * a) A contact force coming from within the dynamics
     * engine.
     * b) Perturbation forces from wherever.
     *
     * NOTE : These may change frequently, so it might
     * be a good idea to check for the existence of a
     * particular contact regularly (if you care about
     * it). The standard case will iterate over all
     * in an arbitrary order. */
    sutil::CMappedList<std::string, SForce> forces_external_;
  };


  /** Contains the controller generated data. These are sent
   * to the actuators. */
  class SRobotActuators
  {
  public:
    /** The control generalized forces (usually torques) to
     * be applied to a robot.
     * Eg.
     * a) By the controller
     * b) By a user interacting through a gui
     */
    Eigen::VectorXd force_gc_commanded_;

    /** Muscle actuator sets. Use the type information to do
     * stuff with the data depending on actuator type.
     *
     * NOTE : This is the dyn data. To utilize this properly, you'll
     * also require parsed data : Something that subclasses SActuatorSetParsed.
     *
     * Typically, the parsed data is stored in the the parsed robot data structure
     * (the one into which you load the config file data).
     *
     * NOTE : The CMappedPointerList true arg indicates that sutil
     * will deallocate memory for these objects. */
    sutil::CMappedPointerList<std::string, SActuatorSetBase, true> actuator_sets_;

    /** The presently activated actuator set (to avoid constant lookups) */
    SActuatorSetBase *aset_curr_=NULL;
  };

  /** Wraps input (sensor) and output (actuator) data for a robot.
   *
   * NOTE : Objects of this struct have the same name as their
   *        parent robot. */
  class SRobotIO : public SObject
  {
  public:
    /** The name of the robot that this IO data structure belongs to */
    std::string name_robot_="";

    /** The degrees of freedom of the robot that these sensors monitor */
    sUInt dof_=0;

    /** Stores information from the real world (or simulation) */
    SRobotSensors sensors_;

    /** Stores the controller's and GUI's output to the real
     * world (or simulation). */
    SRobotActuators actuators_;

    /** Constructor **/
    SRobotIO() : SObject("SRobotIO") { }

    /** Initializes the io data structure. Extracts all the necessary
     * dof data and actuator sets etc. and sets up the associated fields
     * in the io data structure.
     *
     * NOTE : By default it will activate the first actuator set if there
     *        is more than one and one of them isn't specified. */
    sBool init(const SRobotParsed& arg_rds,
        const std::string &actuator_set_to_activate="");

    /** Joint positions and velocities are necessary
     * and sufficient to determine the system's state. */
    void setGcPosition(const Eigen::VectorXd &arg_pos)
    { sensors_.q_ = arg_pos;  }

    /** Joint positions and velocities are necessary
     * and sufficient to determine the system's state. */
    void setGcVelocity(const Eigen::VectorXd &arg_vel)
    { sensors_.dq_ = arg_vel;  }

    /** Prints all the robot info to the screen */
    sBool printInfo();

    /** Inherited from SObject:
     * sBool has_been_init_;
     * std::string name_; */
  };
}

#endif /* SROBOTIO_HPP_ */
