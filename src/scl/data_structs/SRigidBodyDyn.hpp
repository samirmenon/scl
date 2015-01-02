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
/* \file SRigidBodyDyn.hpp
 *
 *  Created on: Oct 7, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SRIGIDBODYDYN_HPP_
#define SRIGIDBODYDYN_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SRigidBody.hpp>

#include <Eigen/Core>

namespace scl
{
  /** SRigidBodyDyn : A class that contains the dynamic information pertaining to a rigid body.
   * This is to cache matrices that might be used in different control scenarios.
   *
   * NOTE : It is typically inefficient to store all of these since O(n) algorithms exist
   * to compute most control parameters. However, caching them greatly simplifies the control
   * code. Moreover, caching may also lead to some performance gains when the dynamics is
   * computed at a lower rate and/or when the information here is reused over and over
   * in complex controllers.
   *
   * To be consistent with our tree convention, this contains:
   * a) std::string name_;
   * b) std::string parent_name_;
   * c) SRigidBodyDyn* parent_addr_;
   * d) std::vector<SRigidBodyDyn*> child_addrs_;*/
  class SRigidBodyDyn : public SObject
  {
  public:
    // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
    // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** The data structure pointing to the static link information */
    const SRigidBody* link_ds_;

    /** The Jacobian relating center of mass velocities in
     * the origin frame to the generalized velocities.
     *     dx_o_ = J_com_ * dq_
     */
    Eigen::MatrixXd J_com_;

    //****************************************************************************************
    // For using normal vectors
    // NOTE : This will soon be obsolete!!!

    /** The transformation matrix from the rigid body's frame
     * to the origin frame:
     *     x_o_com_ = T_o_lnk_ * x_com_; */
    Eigen::Affine3d T_o_lnk_;

    /** The transformation matrix from the rigid body's frame
     * to the parent frame:
     *     x_parent_frame_com_ = T_lnk_ * x_com_; */
    Eigen::Affine3d T_lnk_;

    /** The generalized coordinate values at which the local transform
     * was updated. (Avoids recomputation when un-necessary).
     * NOTE : Only supported by scl dynamics as of now. */
    sFloat q_T_;

    //****************************************************************************************
    // For using spatial vectors (and multi-dof joints)
    /** The generalized coordinate values are now a vector (multi-dof or quaternion joints) */
    Eigen::VectorXd sp_q_T_, sp_dq_T_;

    /** Spatial inertia of joint */
    sSpatialXForm sp_inertia_;

    /** spatial velocity  for rigid body in the articulate body */
    Eigen::MatrixXd spatial_velocity_;

    /** spatial acceleration for rigid body in the articulate body */
    Eigen::MatrixXd spatial_acceleration_;

    /** spatial force for the rigid body in the articulate body */
    Eigen::MatrixXd spatial_force_;


    /** The transformation matrix within the link */
    sSpatialXForm sp_X_within_link_;

    /** The transformation matrix from the rigid body's frame
     * to the origin frame:
     *     x_o_com_ = X_o_lnk_ * x_com_; */
    sSpatialXForm sp_X_o_lnk_;

    /** The transformation matrix from the rigid body's frame
     * to the parent frame:
     *     x_parent_frame_com_ = T_lnk_ * x_com_; */
    sutil::CMappedList<std::string,sSpatialXForm> sp_X_joint_;

    Eigen::MatrixXd sp_S_joint_;     ///< Column vectors correspond to spatial directions of motion
    Eigen::MatrixXd sp_Sorth_joint_; ///< Column vectors correspond to spatial directions of constraint

    //****************************************************************************************
    //Robot Branching Structure data:
    // (Spanning) Tree structure information: (Enables manual tree parsing)
    std::string parent_name_;
    SRigidBodyDyn* parent_addr_;
    std::vector<SRigidBodyDyn*> child_addrs_;

    // Graph structure information: (Enables manual graph parsing)
    std::vector<SRigidBodyDyn*> gr_parent_names_;
    std::vector<SRigidBodyDyn*> gr_parent_addrs_;
    std::vector<SRigidBodyDyn*> gr_child_addrs_;

    //****************************************************************************************
    /** Constructor : Sets stuff to NaN/NULL */
    SRigidBodyDyn() : SObject("SRigidBodyDyn"),
        link_ds_(S_NULL), q_T_(std::numeric_limits<sFloat>::quiet_NaN()),
        parent_name_(""), parent_addr_(S_NULL) {}
  };

} /* namespace scl */
#endif /* SRIGIDBODYDYN_HPP_ */
