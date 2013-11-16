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
/* \file RobotMath.cpp
 *
 *  Created on: Nov 16, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "RobotMath.hpp"

namespace scl
{
  /** Creates a transformation matrix from a given joint specification
   * and a generalized coordinate. */
  sBool sclTransform(Eigen::Affine3d &arg_T, const Eigen::Vector3d &arg_offset,
      const Eigen::Quaterniond &arg_ori_in_parent,
      const sFloat arg_q, const sJointType arg_jtype)
  {
    bool flag = true;

    // Set up the transform depending on the joint type.
    switch(arg_jtype)
    {
      case JOINT_TYPE_PRISMATIC_X:
        arg_T.setIdentity();
        arg_T.rotate(arg_ori_in_parent);
        arg_T.translation() = arg_offset;
        arg_T.translation()(0) += arg_q;
        break;
      case JOINT_TYPE_PRISMATIC_Y:
        arg_T.setIdentity();
        arg_T.rotate(arg_ori_in_parent);
        arg_T.translation() = arg_offset;
        arg_T.translation()(1) += arg_q;
        break;
      case JOINT_TYPE_PRISMATIC_Z:
        arg_T.setIdentity();
        arg_T.rotate(arg_ori_in_parent);
        arg_T.translation() = arg_offset;
        arg_T.translation()(2) += arg_q;
        break;
      case JOINT_TYPE_REVOLUTE_X:
        arg_T.setIdentity();
        arg_T.rotate(arg_ori_in_parent);
        arg_T.translation() = arg_offset;
        arg_T.rotate(Eigen::AngleAxisd(arg_q, Eigen::Vector3d::UnitX()));
        break;
      case JOINT_TYPE_REVOLUTE_Y:
        arg_T.setIdentity();
        arg_T.rotate(arg_ori_in_parent);
        arg_T.translation() = arg_offset;
        arg_T.rotate(Eigen::AngleAxisd(arg_q, Eigen::Vector3d::UnitY()));
        break;
      case JOINT_TYPE_REVOLUTE_Z:
        arg_T.setIdentity();
        arg_T.rotate(arg_ori_in_parent);
        arg_T.translation() = arg_offset;
        arg_T.rotate(Eigen::AngleAxisd(arg_q, Eigen::Vector3d::UnitZ()));
        break;
      default:
        flag = false;
        break;
    }
    return flag;
  }

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
  sBool quat2axisangle(const Eigen::Quaternion<sFloat> & arg_q,
      Eigen::Vector4d & arg_aa)
  {
    //Error if the quaternion isn't normalized
    if( fabs(arg_q.norm() - 1) > SCL_MINIMUM_POSITION_CHANGE)
    { return false; }

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

    return true;
 }
}
