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
/* \file SRobotIOData.hpp
 *
 *  Created on: Jul 22, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 
#ifndef SROBOTIODATA_HPP_
#define SROBOTIODATA_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SForce.hpp>

#include <sutil/CMappedList.hpp>
#include <Eigen/Dense>

#include <string>
#include <vector>

namespace scl
{
  /** Contains generic robot sensor information. These
   * are read from the sensors and/or simulation. */
  struct SRobotSensorData
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

    /** The external forces applied on the robot. The
     * code expects these to be few in number O(n) and
     * appear/disappear relatively rarely:
     *
     * Eg.
     * a) By a user interacting through a gui
     * b) By a physical force on a real robot
     *
     * NOTE TODO : Consider using a CMappedList here
     * instead of the vector. The vector doesn't support
     * removing contacts very efficiently if there are a
     * large number of contacts. CMappedList is much
     * faster at removal/insertion. */
    std::vector<SForce*> forces_external_;

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
    sutil::CMappedList<std::string, SForce> forces_external_transient_;
  };


  /** Contains the controller generated data. These are sent
   * to the actuators. */
  struct SRobotActuatorData
  {
  public:
    /**
     * The control generalized forces (usually torques) to
     * be applied to a robot.
     * Eg.
     * a) By the controller
     * b) By a user interacting through a gui
     */
    Eigen::VectorXd force_gc_commanded_;
  };

  /** Wraps input (sensor) and output (actuator) data for a robot.
   *
   * NOTE : Objects of this struct have the same name as their
   *        parent robot. */
  struct SRobotIOData : public SObject
  {
    /** The degrees of freedom of the robot that these sensors monitor */
    sUInt dof_;

    /** Stores information from the real world (or simulation) */
    SRobotSensorData sensors_;

    /** Stores the controller's and GUI's output to the real
     * world (or simulation). */
    SRobotActuatorData actuators_;

    /** Constructor **/
    SRobotIOData();

    /** Initializes the io data structure */
    sBool init(const std::string& arg_robot_name,
        const sUInt arg_robot_dof);

    /** Joint positions and velocities are necessary
     * and sufficient to determine the system's state. */
    void setJointPositions(const Eigen::VectorXd &arg_pos)
    { sensors_.q_ = arg_pos;  }

    /** Joint positions and velocities are necessary
     * and sufficient to determine the system's state. */
    void setJointVelocities(const Eigen::VectorXd &arg_vel)
    { sensors_.dq_ = arg_vel;  }

    /** Prints all the robot info to the screen */
    sBool printInfo();

    /** Inherited from SObject:
     * sBool has_been_init_;
     * std::string name_; */
  };
}

#endif /* SROBOTIODATA_HPP_ */
