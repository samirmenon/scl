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
 * \file SGcModel.hpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SGCMODEL_HPP_
#define SGCMODEL_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SRobotParsedData.hpp>

#include <Eigen/Dense>

#include <vector>

namespace scl
{
  /** A data structure to store the joint space model.
   * This model serves as the foundation for all tasks
   * to compute their own models (mass, coriolis/centrifugal
   * and gravity).
   *
   * Each controller data structure contains an object of
   * this type.   */
  class SGcModel
  {
  public:
    // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
    // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** A: Mass matrix */
    Eigen::MatrixXd A_;

    /** Ainv : Mass matrix inverse */
    Eigen::MatrixXd Ainv_;

    /** b : Coriolis+centrifugal torque vector */
    Eigen::VectorXd b_;

    /** g : Gravity torque vector */
    Eigen::VectorXd g_;

    /** com : Center of mass vector */
    Eigen::Vector3d pos_com_;

    /** m : Mass of the robot in Euclidean coords */
    sFloat mass_;

    /** J_com_ : The center of mass Jacobians of the articulated body */
    class SCOMInfo{
    public:
      // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
      // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /** The Jacobian relating center of mass velocities in
       * the origin frame to the generalized velocities.
       *     dx_o_ = J_com_ * dq_
       */
      Eigen::MatrixXd J_com_;

      /** The transformation matrix from the center of mass
       * to the origin frame:
       *     x_o_ = T_com_ * x_com_; */
      Eigen::Affine3d T_com_;

      /** The data structure pointing to the static link information */
      const SRobotLink* link_ds_;

      /** The link's name.
       * This is redundant, but primarily for error checks,
       * since external libraries might not maintain the same
       * numeric ordering scheme used in scl.
       *
       * The errors primarily arise because of an inconsistent
       * initialization sequence between the two, which would lead
       * to mixed link ids. */
      std::string name_;

      /** The dynamics engine's id.
       * This, again, is redundant like the name. But simplifies
       * and speeds up error checks. */
      const void* link_dynamic_id_;
    };
    std::vector<SCOMInfo> coms_;

    /** Constructor does nothing */
    SGcModel();

    /** Initialization function sets up the matrix sizes */
    sBool init(const sUInt arg_robot_dof);
  };
}

#endif /* SGCMODEL_HPP_ */
