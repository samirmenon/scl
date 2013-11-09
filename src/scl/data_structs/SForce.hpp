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
/* \file SForce.hpp
 *
 *  Created on: Mar 23, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SFORCE_HPP_
#define SFORCE_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SRigidBodyDyn.hpp>

#include <Eigen/Dense>
#include <string>

namespace scl
{
  /** Defines a task-space force acting on a robot. */
  struct SForce : public SObject
  {
    // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
    // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Its parent robot */
    std::string robot_name_;

    /** The link on which this force acts */
    std::string link_name_;

    /** A pointer to the dynamic engine's object for this link */
    const SRigidBodyDyn *rbd_;

    /** The force vector */
    Eigen::Vector3d force_;

    /** The point (wrt the link's coordinate frame) where the force acts */
    Eigen::Vector3d pos_;

    /** The Jacobian of the point.
     * Useful for converting the applied force
     * into a set of generalized forces. Ie. J' * f
     *
     * NOTE : This must be an [ndof x 3] sized Matrix to
     * translate an [fx, fy, fz] force vector into joint torques.
     */
    Eigen::MatrixXd J_;

    //Constructor
    SForce(): SObject("SForce"), rbd_(NULL){}
  };
}

#endif /* SFORCE_HPP_ */
