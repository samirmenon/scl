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

#include <scl/data_structs/SRobotIO.hpp>
#include <scl/data_structs/SGcModel.hpp>
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
  /** Updates the generalized coordinate model matrices (Everything in SGcModel).
   *
   * This is the most efficient method to access the standard matrices.
   * Computing transformations and Jacobians individually is typically
   * wasteful.
   */
  virtual sBool computeGCModel(/** Current robot state. q, dq, ddq,
            sensed generalized forces and perceived external forces.*/
      const SRobotSensors * arg_sensor_data,
      /** Individual link Jacobians, and composite intertial,
            centrifugal/coriolis gravity estimates.*/
      SGcModel * arg_gc_model)=0;

  /* *******************************************************************
   *                      Coordinate Transformations
   * ******************************************************************* */
  /** Updates the Transformation Matrices for the robot to which
   * this dynamics object is assigned.
   *      x_ancestor_link_coords = arg_link.T_lnk_ * x_link_coords */
  virtual sBool computeTransformsForAllLinks(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  { return false; }

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *        x_parent_link_coords = arg_link.T_lnk_ * x_link_coords
   * Note that the matrix is stored in the passed link data struct.
   */
  virtual sBool computeTransform(
      /** The link at which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  { return false; }

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *      x_ancestor_link_coords = arg_link.T_lnk_ * x_link_coords
   *
   * Note that the local transformation matrices are updated in
   * all the link data structs from arg_link to arg_ancestor.
   *
   * Also note that there are two origins, robot origin, which actually
   * has a node allocated for it. And the global scl origin, for which
   * you should pass in a NULL as ancestor.
   */
  virtual sBool computeTransformToAncestor(
      /** The transformation matrix in which to store the answer */
      Eigen::Affine3d& arg_T,
      /** The link at which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The link up to which the transformation matrix is to be calculated
       * Pass NULL to compute the transform up to the global root. */
      const SRigidBodyDyn* arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  { return false; }

  /* *******************************************************************
   *                      Compute Jacobians
   * ******************************************************************* */
  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *            dx_global_origin = Jx . dq
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link.
   */
  virtual sBool computeJacobianWithTransforms(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated. Transforms
       * are updated in this link and ancestors in the tree. */
      SRigidBodyDyn& arg_link,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local)
  { return false; }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *            dx_global_origin = Jx . dq
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link.
   * ** CONST VERSION : NOTE : The regular version should call this. **
   */
  virtual sBool computeJacobian(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated */
      const SRigidBodyDyn& arg_link,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local) const
  { return false; }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *            dx_ancestor_coords = Jx . dq
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link.
   */
  virtual sBool computeJacobian(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated */
      const SRigidBodyDyn& arg_link,
      /** The link up to which the Jacobian is to be calculated
       * Pass NULL to compute the Jacobian up to the global root. */
      const SRigidBodyDyn* arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local)
  { return false; }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned. ONLY for translation. Ie. size(Jx) = (3,ndof)
   *            dx_ancestor_coords = Jx . dq
   */
  virtual sBool computeJacobianTrans(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated */
      const SRigidBodyDyn& arg_link,
      /** The link up to which the Jacobian is to be calculated
       * Pass NULL to compute the Jacobian up to the global root. */
      const SRigidBodyDyn* arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local)
  { return false; }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned. ONLY for rotation. Ie. size(Jx) = (3,ndof)
   *            dx_ancestor_coords = Jx . dq
   */
  virtual sBool computeJacobianRot(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated */
      const SRigidBodyDyn& arg_link,
      /** The link up to which the Jacobian is to be calculated
       * Pass NULL to compute the Jacobian up to the global root. */
      const SRigidBodyDyn* arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local)
  { return false; }

  /* *******************************************************************
   *                      Integrator functions.
   * ******************************************************************* */

  /** Calculates the collision forces for the robot to which this dynamics
   * object is assigned.
   *
   * Uses std::string based force lookup. The dynamics implementation should
   * support this (maintain a map or something).
   *
   * NOTE : The dynamics engine may delete forces from the passed mapped list
   * at will. Do not store pointers to them at random places in your code.
   */
  virtual sBool computeExternalContacts(/**
          This is where the simulator will store the contact
          forces. It may use the std::string to identify when
          to remove or re-add forces.*/
          sutil::CMappedList<std::string, SForce> & arg_contacts)
  {   /** As of now, this is optional for dynamics engines */ return false; }

  /** Integrates the robot's state.
   * Uses the given applied forces, torques, positions and velocities
   * and its internal dynamic model to compute new positions and velocities.
   * Operates on the SRobotIO data structure.
   *
   * Reads from, and updates:
   *    arg_inputs_.sensors_.q_, dq_, ddq_
   *
   * Reads from:
   *    arg_inputs_.sensors_.forces_external_
   *    arg_inputs_.actuators_.force_gc_commanded_
   */
  virtual sBool integrate(
      /** The existing generalized coordinates, velocities and
       * accelerations + The generalized forces + task (euclidean)
       * forces and the list of contact points and links. */
      SRobotIO& arg_inputs_,
      /** The time across which the system should integrate the
       * dynamics. Could take fixed steps or dynamic ones in between.
       * Up to the integrator implementation. */
      const sFloat arg_time_interval)
  { return false; }

  /* *******************************************************************
   *                      Dynamics State functions.
   * ******************************************************************* */
  /** Gets the robot's kinetic energy */
  virtual sFloat computeEnergyKinetic(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The current generalized velocities. */
      const Eigen::VectorXd& arg_dq)
  { return false; }

  /** Gets the robot's potential energy */
  virtual sFloat computeEnergyPotential(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  { return false; }

  /** Gives an id for a link name.
   *
   * Enables using calculateJacobain without:
   * 1. Storing any dynamic-engine specific objects in
   *    the controller.
   * 2. Using inefficient repeated string based
   *    lookup (usually with maps)
   */
  virtual const void* getIdForLink(std::string arg_link_name)
  { return NULL;  }

  /* *******************************************************************
   *                      Initialization functions.
   * ******************************************************************* */
  /** Default constructor sets the initialization state to false */
  CDynamicsBase() : has_been_init_(false), robot_parsed_data_(NULL){}

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
  virtual sBool init(const SRobotParsed& arg_robot_data)=0;

  /** Initialization state */
  virtual sBool hasBeenInit() {  return has_been_init_;  }


  /* *******************************************************************
   *                      Depracated functions.
   * ******************************************************************* */
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
  virtual sBool computeTransform_Depracated(
      /** The link at which the transformation matrix is to be calculated */
      const void* arg_link_id,
      /** The transformation matrix will be saved here. */
      Eigen::Affine3d& arg_T)
  { return false; }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link
   *
   * Uses id based link lookup. The dynamics implementation should
   * support this (maintain a map or something).
   */
  virtual sBool computeJacobian_Depracated(
      /** The link at which the Jacobian is to be calculated */
      const void* arg_link_id,
      /** The offset from the link's frame (in global coordinates). */
      const Eigen::VectorXd& arg_pos_global,
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J)
  { return false; }

  /** Gets the robot's kinetic energy */
  virtual sFloat getKineticEnergy_Depracated()
  { return false; }

  /** Gets the robot's potential energy */
  virtual sFloat getPotentialEnergy_Depracated()
  { return false; }

protected:
  /** True if the dynamics object has been initialized for a given
   * robot */
  sBool has_been_init_;

  /** A read-only pointer to access parsed data */
  const SRobotParsed* robot_parsed_data_;
};

}

#endif /* CDYNAMICSBASE_HPP_ */
