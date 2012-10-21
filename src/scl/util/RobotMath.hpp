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
  inline void dh2TransformationMatrix(Eigen::Matrix4d &arg_mat, const sFloat &alpha,
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
   * To check the math, please look at:
   * http://www.euclideanspace.com/maths/geometry/rotations/
   * conversions/quaternionToAngle/index.htm
   */
  inline void quat2axisangle(Eigen::Quaternion<sFloat> & arg_q,
      Eigen::Vector4d & arg_aa)
  {
    //1. Normalize the quaternion
    if (arg_q.norm() > 1)
    { arg_q.normalize(); }

    arg_aa[3] = 2 * acos(arg_q.w());

    sFloat s = sqrt(1- (arg_q.w()*arg_q.w()));

    if (s < 0.001)
    {// If s close to zero then direction of axis not important
      arg_aa[0] = arg_q.x();
      arg_aa[1] = arg_q.y();
      arg_aa[2] = arg_q.z();
    }
    else
    {
      arg_aa[0] = arg_q.x()/s;
      arg_aa[1] = arg_q.y()/s;
      arg_aa[2] = arg_q.z()/s;
    }
 }

  /** Converts an XYZ euler angle to a 4D vector WXYZ quaternion */
  inline void eulerAngleXYZToQuat(
      const Eigen::Vector3d & arg_euler_angle_xyz,
      Eigen::Quaternion<sFloat> & ret_quat)
  {
    // Create an affine transformation from the xyz euler angles
    Eigen::Affine3d T;
    T = Eigen::AngleAxisd(arg_euler_angle_xyz(0), Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(arg_euler_angle_xyz(1), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(arg_euler_angle_xyz(2), Eigen::Vector3d::UnitZ());

    // Find the quaternion corresponding to the rotation part of the transform
    ret_quat = T.rotation();
  }

  /** Returns the difference between two quaternions
   * as an Euler angle (XYZ).
   *
   * Angular diff(Euler) = AngleToSubFrom(Quat) - AngleToSub(Quat)
   */
  inline void quatDiffToEulerAngleXYZ(
      const Eigen::Quaternion<sFloat> &arg_quat_to_sub_from,
      const Eigen::Quaternion<sFloat> &arg_quat_to_sub,
      Eigen::Vector3d &ret_ori_dff_euler)
  {
    //Need to convert the quaternion into a difference matrix
    //in order to compute the delta angle.
    Eigen::Matrix<sFloat,3,4> tmp_quat_diff_matrix;
    tmp_quat_diff_matrix(0,0) = -arg_quat_to_sub.x();
    tmp_quat_diff_matrix(0,1) = arg_quat_to_sub.w();
    tmp_quat_diff_matrix(0,2) = -arg_quat_to_sub.z();
    tmp_quat_diff_matrix(0,3) = arg_quat_to_sub.y();

    tmp_quat_diff_matrix(1,0) = -arg_quat_to_sub.y();
    tmp_quat_diff_matrix(1,1) = arg_quat_to_sub.z();
    tmp_quat_diff_matrix(1,2) = arg_quat_to_sub.w();
    tmp_quat_diff_matrix(1,3) = -arg_quat_to_sub.x();

    tmp_quat_diff_matrix(2,0) = -arg_quat_to_sub.z();
    tmp_quat_diff_matrix(2,1) = -arg_quat_to_sub.y();
    tmp_quat_diff_matrix(2,2) = arg_quat_to_sub.x();
    tmp_quat_diff_matrix(2,3) = arg_quat_to_sub.w();

    Eigen::Vector4d tmp_quat;
    tmp_quat<<arg_quat_to_sub_from.w(),
        arg_quat_to_sub_from.x(),
        arg_quat_to_sub_from.y(),
        arg_quat_to_sub_from.z();

    //Compute the quaternion difference with the matrix multiply and
    //return the value
    ret_ori_dff_euler = -2 * tmp_quat_diff_matrix * tmp_quat;
  }
}

#endif /* ROBOTMATH_HPP_ */
