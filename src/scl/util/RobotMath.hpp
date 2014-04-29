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
/* \file RobotMath.hpp
 *
 *  Created on: Aug 4, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

/** \file RobotMath.hpp */

#ifndef ROBOTMATH_HPP_
#define ROBOTMATH_HPP_

#include <scl/DataTypes.hpp>
#include <Eigen/Dense>
#include <math.h>

namespace scl
{
  /** Creates a transformation matrix from a given joint specification
   * and a generalized coordinate. */
  sBool sclTransform(Eigen::Affine3d &arg_T, const Eigen::Vector3d &arg_offset,
      const Eigen::Quaterniond &arg_ori_in_parent,
      const sFloat arg_q, const EJointType arg_jtype);

  /**
   * Creates a transformation matrix given a set of dh parameters
   * z = axis or rotation. x = joint(i-1) to joint(i). y = z cross x
   * SI units : radian, meters
   * alpha    : Angle about common normal (Z_n-1 to Z_n)
   * a (or r) : Length of common normal (Z_n-1 to Z_n = radius)
   * d        : Offset along z to common normal
   * theta    : Angle about prev z, from old x to new x.
   */
  void dh2TransformationMatrix(Eigen::Matrix4d &arg_mat, const sFloat &alpha,
      const sFloat & a, const sFloat & d, const sFloat & theta);

  /**
   * Converts a quaternion into an axis-angle (x,y,z,theta)
   * representation.
   *
   * At axis-angle's two singularities (pi and 0),
   * the passed vector4 will be set to all zeros.
   *
   * angle = 2 * acos(qw)
   * x = qx / sqrt(1-qw*qw)
   * y = qy / sqrt(1-qw*qw)
   * z = qz / sqrt(1-qw*qw)
   *
   * http://www.euclideanspace.com/maths/geometry/rotations/
   * conversions/quaternionToAngle/index.htm
   */
  sBool quat2axisangle(const Eigen::Quaternion<sFloat> & arg_q,
      Eigen::Vector4d & arg_aa);
}

#endif /* ROBOTMATH_HPP_ */
