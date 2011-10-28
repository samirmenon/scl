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

#include <Eigen/Dense>
#include <math.h>

namespace scl
{

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
      const sFloat & a, const sFloat & d, const sFloat & theta)
  {
    sFloat st = sin(theta);
    sFloat ct = cos(theta);
    sFloat sa = sin(alpha);
    sFloat ca = cos(alpha);

    arg_mat<<ct,     -1*st,   0,      a,
             st*ca,  ct*ca,  -1*sa,  -1*sa*d,
             st*sa,  ct*sa,   ca,     ca*d,
             0,      0,       0,      1;
  }

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
  void quat2axisangle(const Eigen::Quaternion<sFloat> & arg_q,
      Eigen::Vector4d & arg_aa)
  {
    //1. Normalize the quaternion
    if (arg_q.norm() > 1)
    { arg_q.normalize(); }

    arg_aa[3] = 2 * acos(arg_q[3]);

    sFloat s = sqrt(1- (arg_q[3]*arg_q[3]));

    if (s < 0.001)
    {// If s close to zero then direction of axis not important
      arg_aa[0] = arg_q[0];
      arg_aa[1] = arg_q[1];
      arg_aa[2] = arg_q[2];
    }
    else
    {
      arg_aa[0] = arg_q[0]/s;
      arg_aa[1] = arg_q[1]/s;
      arg_aa[2] = arg_q[2]/s;
    }
 }
}

#endif /* ROBOTMATH_HPP_ */
