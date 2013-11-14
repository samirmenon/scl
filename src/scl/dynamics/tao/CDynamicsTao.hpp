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
/* \file CDynamicsTao.hpp
 *
 *  Created on: Aug 23, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CDYNAMICSTAO_HPP_
#define CDYNAMICSTAO_HPP_

#include <scl/DataTypes.hpp>
#include <scl/dynamics/CDynamicsBase.hpp>
#include <scl/dynamics/tao/jspace/State.hpp>
#include <scl/dynamics/tao/tao/dynamics/taoNode.h>
#include <scl/dynamics/tao/tao/dynamics/taoJoint.h>
#include <scl/dynamics/tao/tao/dynamics/taoDynamics.h>

#include <Eigen/Dense>

#include <string>


class taoDNode;

namespace jspace {
  struct STaoTreeInfo;
  class Model;
}

namespace scl
{

  class SGcModel;
  struct SRobotSensors;

  /** Computes the dynamics of a robot model
   * using the TAO dynamics engine */
  class CDynamicsTao : public CDynamicsBase
  {
  public:
    /* *******************************************************************
     *                      Computational functions.
     * ******************************************************************* */
    /** BUGGY => DEPRACATED!!! */
    virtual bool computeGCModel(
        SRobotSensors const * arg_sensor_data,
        SGcModel * arg_gc_model)
    { return false; }

    /** Gives an id for a link name.
     *
     * Enables using calculateJacobain without:
     * 1. Storing any dynamic-engine specific objects in
     *    the controller.
     * 2. Using inefficient repeated string based
     *    lookup (usually with maps)
     */
    taoDNode * getTaoIdForLink(std::string arg_link_name);

    /** Integrates the robot's state.
     * Uses the given applied forces, torques, positions and velocities
     * and its internal dynamic model to compute
     * new positions and velocities.
     *
     * This version performs the entire set of operations on
     * the SRobotIO data structure.
     *
     * Reads from, and updates:
     * arg_inputs_.sensors_.q_
     * arg_inputs_.sensors_.dq_
     * arg_inputs_.sensors_.ddq_
     *
     * Reads from:
     * arg_inputs_.actuators_.forces_
     * arg_inputs_.actuators_.force_gc_commanded_
     */
    virtual sBool integrate(
        /** The existing generalized coordinates, velocities and
         * accelerations + The generalized forces + task (euclidean)
         * forces and the list of contact points and links. */
        SRobotIO& arg_inputs,
        /** The time across which the system should integrate the
         * dynamics.
         *
         * Tao uses a forward euler integrator and uses this as the
         * dt so set it to a small value. */
        const sFloat arg_time_interval);

    /* *******************************************************************
     *                      Dynamics State functions.
     * ******************************************************************* */
    /** Sets the generalized coordinates
     *
     * TODO : This dynamics engine implementation should be stateless.
     * This present system is because the intermediate jspace implementation
     * has an extra data layer. Should be cleaned up in the future. */
    sBool setGeneralizedCoordinates(Eigen::VectorXd &arg_q);

    /** Sets the generalized velocities
     *
     * TODO : This dynamics engine implementation should be stateless.
     * This present system is because the intermediate jspace implementation
     * has an extra data layer. Should be cleaned up in the future. */
    sBool setGeneralizedVelocities(Eigen::VectorXd &arg_dq);

    /** Sets the external generalized forces
     *
     * TODO : This dynamics engine implementation should be stateless.
     * This present system is because the intermediate jspace implementation
     * has an extra data layer. Should be cleaned up in the future. */
    sBool setGeneralizedForces(Eigen::VectorXd &arg_fgc);

    /* *******************************************************************
     *                      Initialization functions.
     * ******************************************************************* */
    /** Default constructor sets the initialization state to false */
    CDynamicsTao();
    virtual ~CDynamicsTao();

    /** Initialize the CDynamicsTao with the TAO trees it needs in
     * order to update the kinematics and/or dynamics of a robot.
     *
     * Accesses the database.
     *
     * \return True if everything went according to plan. */
    virtual bool init(const SRobotParsed& arg_robot_data);

  private:
    /** The robot's name */
    std::string robot_name_;

    /** Store pointers to the tao trees */
    taoNodeRoot *tao_tree_q_root_, *tao_tree_q_dq_root_;

    /** The joint space model, a helper class that wraps around tao
     * and computes the robot's dynamics. */
    jspace::Model * model_;

    /** The robot's degrees of freedom */
    size_t ndof_;

    /** The current state of the robot */
    jspace::State state_;

    /** The gravity acting on the robot */
    Eigen::Vector3d gravity_;
  };

}

#endif /* CDYNAMICSTAO_HPP_ */
