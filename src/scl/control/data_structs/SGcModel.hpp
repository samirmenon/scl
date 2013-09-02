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
    /* *********************************************************************
     *                                 Data
     * ********************************************************************* */
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

    /** The generalized coordinates and velocity at the time
     * the model was last updated. Since the dynamics are completely
     * described by the generalized coordinates and velocities, we
     * don't need to store any higher derivatives.
     *
     * NOTE : These may not match the actual generalized coordinates
     * and velocity due to time-delay introduced while computing
     * the model. Typically, this is irrelevant, but it might start
     * to be substantial when the dynamics are computed at a lower
     * rate or with human-like delays. */
    Eigen::VectorXd q_,dq_;

    /** com : Center of mass vector */
    Eigen::Vector3d pos_com_;

    /** m : Mass of the robot in Euclidean coords */
    sFloat mass_;

    /** SRigidBodyDyn : A class that contains the dynamic information pertaining to a rigid body.
     * This is to cache matrices that might be used in different control scenarios.
     * Defined outside the class for simplicity*/
    class SRigidBodyDyn;
    /** A vector of the dynamics information for all the rigid bodies in the
     * articulated body */
    std::vector<SRigidBodyDyn> link_ds_;

    /* *********************************************************************
     *                      Initialization functions
     * ********************************************************************* */
    /** Constructor does nothing */
    SGcModel();

    /** Initialization function sets up the matrix sizes */
    sBool init(const sUInt arg_robot_dof);
  };

  /** SRigidBodyDyn : A class that contains the dynamic information pertaining to a rigid body.
   * This is to cache matrices that might be used in different control scenarios.
   *
   * NOTE : It is typically inefficient to store all of these since O(n) algorithms exist
   * to compute most control parameters. However, caching them greatly simplifies the control
   * code. Moreover, caching may also lead to some performance gains when the dynamics is
   * computed at a lower rate and/or when the information here is reused over and over
   * in complex controllers.*/
  class SGcModel::SRigidBodyDyn{
  public:
    // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
    // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** The Jacobian relating center of mass velocities in
     * the origin frame to the generalized velocities.
     *     dx_o_ = J_com_ * dq_
     */
    Eigen::MatrixXd J_com_;

    /** The transformation matrix from the rigid body's frame
     * to the origin frame:
     *     x_o_com_ = T_o_lnk_ * x_com_; */
    Eigen::Affine3d T_o_lnk_;

    /** The transformation matrix from the rigid body's frame
     * to the parent frame:
     *     x_parent_frame_com_ = T_lnk_ * x_com_; */
    Eigen::Affine3d T_lnk_;

    /** The link's name.
     * This is redundant, but primarily for error checks,
     * since external libraries might not maintain the same
     * numeric ordering scheme used in scl.
     *
     * The errors primarily arise because of an inconsistent
     * initialization sequence between the two, which would lead
     * to mixed link ids. */
    std::string name_;

    /** The data structure pointing to the static link information */
    const SRigidBody* link_ds_;

    /** The dynamics engine's id.
     * This, again, is redundant like the name. But simplifies
     * and speeds up error checks. */
    const void* link_dynamic_id_;
  };
}

#endif /* SGCMODEL_HPP_ */
