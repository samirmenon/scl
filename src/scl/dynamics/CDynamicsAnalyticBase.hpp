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
/* \file CDynamicsAnalyticBase.hpp
 *
 *  Created on: Sep 3, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CDYNAMICSANALYTICBASE_HPP_
#define CDYNAMICSANALYTICBASE_HPP_

#include <scl/DataTypes.hpp>

#include <scl/data_structs/SRobotIO.hpp>
#include <scl/data_structs/SGcModel.hpp>

#include <Eigen/Dense>
#include <string>

namespace scl {

/** A base class for an analytic dynamics implementation for an
 * actuated articulated body (a robot).
 *
 * All dynamics implementations must try to be stateless.
 * Ie. After one initial configuration that specifies the
 * dynamic model's parameters, they must not store any more
 * data. Pass in the model configuration as an argument
 * and return the new state and/or dynamics matrices.
 */
class CDynamicsAnalyticBase {
public:
  /** Updates the generalized coordinate model matrices (Everything in SGcModel).
   *
   * This is the most efficient method to access the standard matrices.
   * Computing transformations and Jacobians individually is typically
   * wasteful.
   */
  virtual sBool computeGCModel(
      /** The generalized coordinates */
      const Eigen::VectorXd &arg_q,
      /** All individual dynamics matrices will be saved here. */
      SGcModel& arg_gc_model)=0;

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *
   * The Transformation Matrix is specified by a link and an offset
   * (in task space dimensions)from that link and is given by:
   *
   *           x_ancestor_frame_coords = T * x_link_coords
   *
   * Uses id based link lookup. The dynamics implementation should
   * support this (maintain a map or something).
   */
  virtual sBool computeTransformationMatrix(
      /** The generalized coordinates */
      const Eigen::VectorXd &arg_q,
      /** The link at which the transformation matrix is to be calculated */
      sInt arg_link_id,
      /** The link up to which the transformation matrix is to be calculated */
      sUInt arg_ancestor_link_id,
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
  virtual sBool computeJacobian(
      /** The generalized coordinates */
      const Eigen::VectorXd &arg_q,
      /** The link at which the Jacobian is to be calculated */
      sInt arg_link_id,
      /** The offset from the link's frame (in local coordinates). */
      const Eigen::VectorXd& arg_pos_local,
      /** The Jacobian will be saved here. */
      Eigen::MatrixXd& arg_J)=0;

  /* **************************************************************
   *                   Data access functions
   * ************************************************************** */
  /** Gives an id for a link name.
   *
   * Useful because:
   * 1. Allows storing any dynamic-engine specific objects in the controller.
   * 2. Avoids using inefficient repeated string based lookup (usually with maps)
   */
  virtual sUInt getIdForLink(std::string arg_link_name)=0;

  /* **************************************************************
   *                   Initialization functions
   * ************************************************************** */
  /** Default constructor sets the initialization state to false */
  CDynamicsAnalyticBase() : has_been_init_(false){}

  /** Default destructor does nothing */
  virtual ~CDynamicsAnalyticBase(){}

  /** Initializes the dynamics to be computed for a specific robot.
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

  /* **************************************************************
   *                            Data
   * ************************************************************** */
protected:
  /** True if the dynamics object has been initialized for a given
   * robot */
  sBool has_been_init_;
};

}

#endif /* CDYNAMICSANALYTICBASE_HPP_ */
