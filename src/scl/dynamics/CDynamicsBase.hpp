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
/* \file CDynamicsBase.hpp
 *
 *  Created on: Aug 23, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CDYNAMICSBASE_HPP_
#define CDYNAMICSBASE_HPP_

#include <string>
#include <Eigen/Dense>

#include <scl/DataTypes.hpp>

#include <scl/data_structs/SRobotIOData.hpp>
#include <scl/control/data_structs/SGcModel.hpp>
#include <scl/dynamics/SDynamicsState.hpp>

namespace scl {

/** A base class for different dynamics implementations.
 *
 * Using any controller requires supporting this api with
 * a dynamics engine implementation.
 *
 * All dynamics implementations must try to be stateless.
 * Ie. After one initial configuration that specifies the
 * dynamic model's parameters, they must not store any more
 * data. Pass in the model configuration as an argument
 * and return the new state and/or dynamics matrices.
 */
class CDynamicsBase
{
public:
  /* *******************************************************************
   *                      Computational functions.
   * ******************************************************************* */
  /** Updates the joint space model matrices
   * (Everything in SGcModel)
   */
  virtual sBool updateModelMatrices(/**
          This is where there current robot state is read when
          updateModelMatrices() is called. Contains q, dq, ddq,
          and a vector of external forces.*/
             SRobotSensorData const * arg_io_data,
             /** Pointer to the joint-space model structure for the
          robot. This is the place where the matrices reside
          that will get updated when calling updateModelMatrices().
          Contains Mass matrix, Inverse Mass matrix, centrifugal/
          coriolis force vector, and gravity vector.*/
             SGcModel * arg_gc_model)=0;

  /** Calculates a Transformation Matrix for some link on the robot to which
   * this dynamics object is assigned.
   *
   * The Transformation Matrix performs the operation:
   *
   *           x_global_coords = T * x_link_coords
   *
   * Uses id based link lookup. The dynamics implementation should
   * support this (maintain a map or something).
   */
  virtual sBool calculateTransformationMatrix(
      /** The link at which the transformation matrix is to be calculated */
      const void* arg_link_id,
      /** The transformation matrix will be saved here. */
      Eigen::Affine3d& arg_T)=0;

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link
   *
   * Uses id based link lookup. The dynamics implementation should
   * support this (maintain a map or something).
   */
  virtual sBool calculateJacobian(
      /** The link at which the Jacobian is to be calculated */
      const void* arg_link_id,
      /** The offset from the link's frame (in global coordinates). */
      const Eigen::VectorXd& arg_pos_global,
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J)=0;

  /** Calculates the collision forces for the robot to which this dynamics
   * object is assigned.
   *
   * Uses std::string based force lookup. The dynamics implementation should
   * support this (maintain a map or something).
   *
   * NOTE : The dynamics engine may delete forces from the passed mapped list
   * at will. Do not store pointers to them at random places in your code.
   */
  virtual sBool calculateContactExternalForces(/**
          This is where the simulator will store the contact
          forces. It may use the std::string to identify when
          to remove or re-add forces.*/
          sutil::CMappedList<std::string, SForce> & arg_contact_forces)
  {   /** As of now, this is optional for dynamics engines */ return false; }

  /** Integrates the robot's state.
   * Uses the given applied forces, torques, positions and velocities
   * and its internal dynamic model to compute
   * new positions and velocities.
   *
   * This version performs the entire set of operations on
   * the SRobotIOData data structure.
   *
   * Reads from, and updates:
   * arg_inputs_.sensors_.q_
   * arg_inputs_.sensors_.dq_
   * arg_inputs_.sensors_.ddq_
   *
   * Reads from:
   * arg_inputs_.sensors_.forces_external_
   * arg_inputs_.sensors_.forces_external_transient_
   * arg_inputs_.sensors_.force_gc_measured_
   * arg_inputs_.actuators_.force_gc_commanded_
   */
  virtual sBool integrate(
      /** The existing generalized coordinates, velocities and
       * accelerations + The generalized forces + task (euclidean)
       * forces and the list of contact points and links. */
      SRobotIOData& arg_inputs_,
      /** The time across which the system should integrate the
       * dynamics. Could take fixed steps or dynamic ones in between.
       * Up to the integrator implementation. */
      const sFloat arg_time_interval)=0;

  /* *******************************************************************
   *                      Dynamics State functions.
   * ******************************************************************* */
  /** Gets the robot's kinetic energy */
  virtual sFloat getKineticEnergy()=0;

  /** Gets the robot's potential energy */
  virtual sFloat getPotentialEnergy()=0;

  /** Gives an id for a link name.
   *
   * Enables using calculateJacobain without:
   * 1. Storing any dynamic-engine specific objects in
   *    the controller.
   * 2. Using inefficient repeated string based
   *    lookup (usually with maps)
   */
  virtual const void* getIdForLink(std::string arg_link_name)=0;

  /* *******************************************************************
   *                      Initialization functions.
   * ******************************************************************* */
  /** Default constructor sets the initialization state to false */
  CDynamicsBase() : has_been_init_(false){}

  /** Default destructor does nothing */
  virtual ~CDynamicsBase(){}

  /** Initializes the dynamics to be computed for a
   * specific robot.
   *
   * Returns,
   * true : Succeeds in creating a dynamics object
   *
   * false : If it can't find the robot or if the information
   * is inconsistent with what the implementation requires,
   * it returns false
   */
  virtual sBool init(const SRobotParsedData& arg_robot_data)=0;

  /** Initialization state */
  virtual sBool hasBeenInit() {  return has_been_init_;  }

protected:
  /** True if the dynamics object has been initialized for a given
   * robot */
   sBool has_been_init_;
};

}

#endif /* CDYNAMICSBASE_HPP_ */
