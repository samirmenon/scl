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
  class SRigidBodyDyn
  {
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

    /** The data structure pointing to the static link information */
    const SRigidBody* link_ds_;

    /** The dynamics engine's id.
     * This, again, is redundant like the name. But simplifies
     * and speeds up error checks.
     *
     * NOTE TODO : Delete this. This shouldn't be here. It's purpose
     * was to primarily update a link quickly. But dynamics implementations
     * either cache data or recompute a lot of it, which removes the
     * potential efficiency gained here. */
    const void* link_dynamic_id_;

    /** Requirements to create a mapped tree of objects */
    /** The link's name and its parent. For the tree structure. */
    std::string name_, parent_name_;
    /** This is automatically created by the map */
    SRigidBodyDyn* parent_addr_;
    /** This is automatically created by the map */
    std::vector<SRigidBodyDyn*> child_addrs_;
  };

} /* namespace scl */
#endif /* SRIGIDBODYDYN_HPP_ */
