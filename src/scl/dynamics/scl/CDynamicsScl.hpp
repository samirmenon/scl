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
/*
 * CDynamicsScl.hpp
 *
 *  Created on: Oct 7, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CDYNAMICSSCL_HPP_
#define CDYNAMICSSCL_HPP_

#include <scl/dynamics/CDynamicsBase.hpp>
#include <scl/data_structs/SRigidBodyDyn.hpp>

#include <sutil/CMappedTree.hpp>
#include <string>

namespace scl
{
  /** This class implements dynamics algorithms for control
   * and simulation using the scl library.
   *
   * The primary goal of this implementation---given that there are
   * alternatives---is to enable quick Jacobian and transformation matrix
   * computations.
   *
   * The main feature of this dynamics implementation is that it is
   * truly stateless.
   *
   * It does not (presently) implement an integrator so plan
   * on using an alternative Physics engine (like Tao).
   */
  class CDynamicsScl : public scl::CDynamicsBase
  {
public:
  /* *******************************************************************
   *                      Computational functions.
   * ******************************************************************* */
  /** Updates the joint space model matrices
   * (Everything in SGcModel)
   */
  virtual sBool updateModelMatrices(/** Current robot state. q, dq, ddq,
            sensed generalized forces and perceived external forces.*/
      const SRobotSensorData * arg_sensor_data,
      /** Individual link Jacobians, and composite intertial,
            centrifugal/coriolis gravity estimates.*/
      SGcModel * arg_gc_model);

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *
   * The Transformation Matrix is specified by a link and an offset
   * (in task space dimensions)from that link and is given by:
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
      Eigen::Affine3d& arg_T)
  { return false; }

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *        x_parent_link_coords = arg_link.T_lnk_ * x_link_coords
   * Note that the matrix is stored in the passed link data struct.
   */
  virtual sBool calculateTransformationMatrixForLink(
      /** The link at which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q);

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
  virtual sBool calculateTransformationMatrixForLink(
      /** The transformation matrix in which to store the answer */
      Eigen::Affine3d& arg_T,
      /** The link at which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The link up to which the transformation matrix is to be calculated
       * Pass NULL to compute the transform up to the global root. */
      const SRigidBodyDyn* arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q);

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
      Eigen::MatrixXd& arg_J)
  { return false; }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *                dx = Jx . dq
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link.
   */
  sBool calculateJacobian(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The link up to which the Jacobian is to be calculated
       * Pass NULL to compute the Jacobian up to the global root. */
      const SRigidBodyDyn* arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local,
      /** Whether to recompute the transformations up to the ancestor
       * Default = true. Set to false to speed things up.*/
      const bool arg_recompute_transforms=true);

  /** Not supported. */
  virtual sBool calculateContactExternalForces(
          sutil::CMappedList<std::string, SForce> & arg_contact_forces)
  { return false; }

  /** Not supported. */
  virtual sBool integrate(SRobotIOData& arg_inputs_,
      const sFloat arg_time_interval)
  { return false; }

  /* *******************************************************************
   *                      Dynamics State functions.
   * ******************************************************************* */
  /** Gets the robot's kinetic energy */
  virtual sFloat getKineticEnergy()
  { return false; }

  /** Gets the robot's potential energy */
  virtual sFloat getPotentialEnergy()
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
  { return NULL; }

  /* *******************************************************************
   *                      Initialization functions.
   * ******************************************************************* */
  /** Default constructor sets the initialization state to false */
  CDynamicsScl() : CDynamicsBase() { }

  /** Default destructor does nothing */
  virtual ~CDynamicsScl(){}

  /** Initializes the dynamics to be computed for a specific robot.
   *  Returns:
   *     true : Succeeds in creating a dynamics object
   *
   *     false : If it can't find the robot or if the information
   *             is inconsistent with what the implementation requires,
   *             it returns false
   */
  virtual sBool init(const SRobotParsedData& arg_robot_data);
};

} /* namespace scl */
#endif /* CDYNAMICSSCL_HPP_ */
