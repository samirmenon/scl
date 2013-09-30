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
 * CDynamicsAnalyticRPP.cpp
 *
 *  Created on: Sep 11, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CDynamicsAnalyticRPP.hpp"

#include <math.h>
#include <iostream>

namespace scl
{
  void CDynamicsAnalyticRPP::computeTOrg_0(const Eigen::VectorXd &arg_q,
      Eigen::Affine3d& arg_T)
  {
    Eigen::Matrix4d tmp;
    tmp.setIdentity();

    tmp(0,0) = cos(arg_q(0));    tmp(0,1) = -sin(arg_q(0));   tmp(0,2) = 0;     tmp(0,3) = pos_root_in_global_org_(0);
    tmp(1,0) = sin(arg_q(0));    tmp(1,1) = cos(arg_q(0));    tmp(1,2) = 0;     tmp(1,3) = pos_root_in_global_org_(1);
    tmp(2,0) = 0;                tmp(2,1) = 0;                tmp(2,2) = 1;     tmp(2,3) = pos_root_in_global_org_(2)+1;
    tmp(3,0) = 0;                tmp(3,1) = 0;                tmp(3,2) = 0;     tmp(3,3) = 1;

    arg_T.matrix() = tmp;
  }

  void CDynamicsAnalyticRPP::computeT0_1(const Eigen::VectorXd &arg_q,
      Eigen::Affine3d& arg_T)
  {
    Eigen::Matrix4d tmp;
    tmp.setIdentity();

    tmp(0,0) = 1;             tmp(0,1) = 0;             tmp(0,2) = 0;     tmp(0,3) = 0;
    tmp(1,0) = 0;             tmp(1,1) = 1;             tmp(1,2) = 0;     tmp(1,3) = 0.5+arg_q(1);
    tmp(2,0) = 0;             tmp(2,1) = 0;             tmp(2,2) = 1;     tmp(2,3) = 0;
    tmp(3,0) = 0;             tmp(3,1) = 0;             tmp(3,2) = 0;     tmp(3,3) = 1;

    arg_T.matrix() = tmp;
  }

  void CDynamicsAnalyticRPP::computeT1_2(const Eigen::VectorXd &arg_q,
      Eigen::Affine3d& arg_T)
  {
    Eigen::Matrix4d tmp;
    tmp.setIdentity();

    tmp(0,0) = 1;             tmp(0,1) = 0;             tmp(0,2) = 0;     tmp(0,3) = 0;
    tmp(1,0) = 0;             tmp(1,1) = 1;             tmp(1,2) = 0;     tmp(1,3) = 0;
    tmp(2,0) = 0;             tmp(2,1) = 0;             tmp(2,2) = 1;     tmp(2,3) = -0.2+arg_q(2);
    tmp(3,0) = 0;             tmp(3,1) = 0;             tmp(3,2) = 0;     tmp(3,3) = 1;

    arg_T.matrix() = tmp;
  }

  bool CDynamicsAnalyticRPP::computeJcom(
      const Eigen::VectorXd &arg_q, unsigned int arg_link_id,
      Eigen::MatrixXd& arg_J)
  {
    bool flag = true;
    arg_J.setZero(6,3);//Reset

    switch(arg_link_id)
    {
      case 0:
        arg_J(0,0) = 0;             arg_J(0,1) = 0;             arg_J(0,2) = 0;
        arg_J(1,0) = 0;             arg_J(1,1) = 0;             arg_J(1,2) = 0;
        arg_J(2,0) = 0;             arg_J(2,1) = 0;             arg_J(2,2) = 0;
        arg_J(3,0) = 0;             arg_J(3,1) = 0;             arg_J(3,2) = 0;
        arg_J(4,0) = 0;             arg_J(4,1) = 0;             arg_J(4,2) = 0;
        arg_J(5,0) = 1;             arg_J(5,1) = 0;             arg_J(5,2) = 0;
        break;

      case 1:
        arg_J(0,0) = 0.3625*cos(arg_q(0)) - (.5+arg_q(1)) * cos(arg_q(0)); arg_J(0,1) = -sin(arg_q(0)); arg_J(0,2) = 0;
        arg_J(0,0) = 0.3625*sin(arg_q(0)) - (.5+arg_q(1)) * sin(arg_q(0)); arg_J(0,1) = cos(arg_q(0)); arg_J(0,2) = 0;
        arg_J(2,0) = 0;             arg_J(2,1) = 0;             arg_J(2,2) = 0;
        arg_J(3,0) = 0;             arg_J(3,1) = 0;             arg_J(3,2) = 0;
        arg_J(4,0) = 0;             arg_J(4,1) = 0;             arg_J(4,2) = 0;
        arg_J(5,0) = 1;             arg_J(5,1) = 0;             arg_J(5,2) = 0;
        break;

      case 2:
        arg_J(0,0) = 0.3625*cos(arg_q(0)) - (.5+arg_q(1)) * cos(arg_q(0)); arg_J(0,1) = -sin(arg_q(0)); arg_J(0,2) = 0;
        arg_J(0,0) = 0.3625*sin(arg_q(0)) - (.5+arg_q(1)) * sin(arg_q(0)); arg_J(0,1) = cos(arg_q(0)); arg_J(0,2) = 0;
        arg_J(2,0) = 0;             arg_J(2,1) = 0;             arg_J(2,2) = 1;
        arg_J(3,0) = 0;             arg_J(3,1) = 0;             arg_J(3,2) = 0;
        arg_J(4,0) = 0;             arg_J(4,1) = 0;             arg_J(4,2) = 0;
        arg_J(5,0) = 1;             arg_J(5,1) = 0;             arg_J(5,2) = 0;
        break;

      default:
        flag = false;
        break;
    }
    return flag;
  }

  /** Uses following inertial properties.
   * Subscript[m, 0] = 10; Subscript[Ixx, 0] = 5; Subscript[Iyy, 0] = 5; Subscript[Izz, 0] = 2;
     Subscript[Ixy, 0] = 0; Subscript[Ixz, 0] = 0; Subscript[Iyz, 0] = 0;
     Subscript[m, 1] = 5; Subscript[Ixx, 1] = 3; Subscript[Iyy, 1] = 3; Subscript[Izz, 1] = 1;
     Subscript[Ixy, 1] = 0; Subscript[Ixz, 1] = 0; Subscript[Iyz, 1] = 0;
     Subscript[m, 2] = 3; Subscript[Ixx, 2] = 2; Subscript[Iyy, 2] = 2; Subscript[Izz, 2] = 0.5;
     Subscript[Ixy, 2] = 0; Subscript[Ixz, 2] = 0; Subscript[Iyz, 2] = 0;

   * Uses following com positions:
   * Subscript[comPos, 0] = (0,0,-0.5)
   * Subscript[comPos, 1] = (0,-.3625,0)
   * Subscript[comPos, 2] = (0 0 .325)
   */
  bool CDynamicsAnalyticRPP::computeMgc(const Eigen::VectorXd &arg_q,
          Eigen::MatrixXd& arg_Mgc)
  {
    arg_Mgc.setZero(3,3);

    arg_Mgc(0,0) = 4.34453 + arg_q(1) * (4.375 + 8*arg_q(1));
    arg_Mgc(1,1) = 8.0;
    arg_Mgc(0,0) = 3.0;

    return true;
  }


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
  sBool CDynamicsAnalyticRPP::calculateTransformationMatrix(
      /** The generalized coordinates */
      const Eigen::VectorXd &arg_q,
      /** The link at which the transformation matrix is to be calculated */
      sInt arg_link_id,
      /** The link up to which the transformation matrix is to be calculated */
      sUInt arg_ancestor_link_id,
      /** The transformation matrix will be saved here. */
      Eigen::Affine3d& arg_T)
  {
    bool flag = true;
    Eigen::Affine3d tmp;

    // Reset the affine transform.
    arg_T.setIdentity();
    switch(arg_link_id)
    {
      case 0:
        if(arg_ancestor_link_id == -1)//Ancestor must be ground
        { computeTOrg_0(arg_q, arg_T);  }
        else
        { flag = false; }
        break;

      case 1:
        if(arg_ancestor_link_id == 0)
        {  computeT0_1(arg_q, arg_T); }
        else if(arg_ancestor_link_id == -1)//Ancestor must be ground
        {
          computeTOrg_0(arg_q, tmp);
          computeT0_1(arg_q, arg_T);
          arg_T = (tmp * arg_T);
        }
        else
        { flag = false; }
        break;

      case 2:
        if(arg_ancestor_link_id == 1)
        {  computeT1_2(arg_q, arg_T); }
        else if(arg_ancestor_link_id == 0)//Ancestor must be ground
        {
          computeT0_1(arg_q, tmp);
          computeT1_2(arg_q, arg_T);
          arg_T = (arg_T * tmp);
        }
        else if(arg_ancestor_link_id == - 1)//Ancestor must be ground
        {
          computeTOrg_0(arg_q, tmp);
          computeT0_1(arg_q, arg_T);
          arg_T = (tmp * arg_T);
          computeT1_2(arg_q, tmp);
          arg_T = (arg_T * tmp);
        }
        else
        { flag = false; }
        break;

      default:
        flag = false;
        break;
    }
    return flag;
  }


  sBool CDynamicsAnalyticRPP::init(const SRobotParsedData& arg_robot_data)
  {
    const SRigidBody * tmp = arg_robot_data.robot_br_rep_.getRootNodeConst();
    if(NULL == tmp)
    {
      std::cout<<"\nCDynamicsAnalyticRPP::init() : Error. Couldn't find robot root node";
      return false;
    }
    pos_root_in_global_org_ = tmp->pos_in_parent_;
    has_been_init_ = true;
    return true;
  }

} /* namespace scl */
