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
 * CDynamicsScl.cpp
 *
 *  Created on: Oct 7, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CDynamicsScl.hpp"

#include <scl/util/RobotMath.hpp>

#include <iostream>
#include <stdexcept>

namespace scl
{

  /** Updates the joint space model matrices
   * (Everything in SGcModel)
   */
  sBool CDynamicsScl::updateModelMatrices(/** Current robot state. q, dq, ddq,
              sensed generalized forces and perceived external forces.*/
        const SRobotSensorData * arg_sensor_data,
        /** Individual link Jacobians, and composite intertial,
              centrifugal/coriolis gravity estimates.*/
        SGcModel * arg_gc_model)
  {
    bool flag = true;
    const Eigen::VectorXd &q = arg_sensor_data->q_;
    const Eigen::VectorXd &dq = arg_sensor_data->dq_;

    //1. Update the transformation matrices. Everything else depends on it.
    sutil::CMappedTree<std::string, SRigidBodyDyn>::iterator it,ite;
    for(it = arg_gc_model->link_ds_.begin(), ite = arg_gc_model->link_ds_.end();
        it!=ite; ++it)
    {// Loop over all the links.
      SRigidBodyDyn &rbd = *it;
      const SRigidBody &rb = *(it->link_ds_);
      int i = rbd.link_ds_->link_id_;
      flag = flag && sclTransform(rbd.T_lnk_, rb.pos_in_parent_, q(i), rb.joint_type_);
    }
    if(false == flag)
    { return false; }

    //2. Update the com Jacobians.
    for(it = arg_gc_model->link_ds_.begin(), ite = arg_gc_model->link_ds_.end();
        it!=ite; ++it)
    {// Loop over all the links.
      SRigidBodyDyn &rbd = *it;
      const SRigidBody &rb = *(it->link_ds_);
      int i = rbd.link_ds_->link_id_;

      //For each link, set the Jacobian to zero. Then iterate over parents
      //and set each column of J to its correct value
      const SRigidBodyDyn* tmp = &rbd;
      while( (NULL != tmp) && (false == tmp->link_ds_->is_root_) )
      {//Abort if root
        //Get the id to fill the Jacobian column
        int icol = tmp->link_ds_->link_id_;
        //J_Col = R_o_lnk * dtheta;
        switch(tmp->link_ds_->joint_type_)
        {
          case JOINT_TYPE_REVOLUTE_X:
            rbd.J_com_.col(icol) = tmp->T_o_lnk_.rotation() * Eigen::Vector3d::UnitX();
            break;

          case JOINT_TYPE_REVOLUTE_Y:
            rbd.J_com_.col(icol) = tmp->T_o_lnk_.rotation() * Eigen::Vector3d::UnitY();
            break;

          case JOINT_TYPE_REVOLUTE_Z:
            rbd.J_com_.col(icol) = tmp->T_o_lnk_.rotation() * Eigen::Vector3d::UnitZ();
            break;
          default:
            break;
        }
      }
    }
    return true;
  }

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *        x_parent_link_coords = arg_link.T_lnk_ * x_link_coords
   * Note that the matrix is stored in the passed link data struct.
   */
  sBool CDynamicsScl::calculateTransformationMatrixForLink(
      /** The link at which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The transformation matrix will be saved here. */
      const Eigen::VectorXd& arg_q,
      /** Include the offset from global origin to robot's origin */
      const bool arg_flag_include_origin_offset)
  {
    bool flag;

    //The matrix is already up to date. Just return.
    // Update if q_T_ is NaN (never initialized before).
    if( fabs(arg_q(arg_link.link_ds_->link_id_) - arg_link.q_T_) <
        std::numeric_limits<sFloat>::epsilon() && false == std::isnan(arg_link.q_T_))
    { return true;  }

    // For origin (root) nodes, include the global origin translation/rotation if required.
    // NOTE : This allows us to provide a global origin + robot-specific origin(s).
    // By doing so, we don't have to manually change the robot's origin specification
    // when we use the same config file multiple times to create multiple instances
    // of the same robot. Ie. This is important to support a "root" node in the *Cfg.xml
    // with a constant offset from it's parent..
    if(arg_flag_include_origin_offset && arg_link.parent_addr_->link_ds_->is_root_)
    {
      flag = sclTransform(arg_link.T_lnk_,
          arg_link.link_ds_->pos_in_parent_ + arg_link.parent_addr_->link_ds_->pos_in_parent_,
          arg_q(arg_link.link_ds_->link_id_), arg_link.link_ds_->joint_type_);

      arg_link.T_lnk_.rotate(arg_link.parent_addr_->link_ds_->ori_parent_quat_);
    }
    else
    { // For all other nodes, just use the position in parent.
      flag = sclTransform(arg_link.T_lnk_, arg_link.link_ds_->pos_in_parent_,
          arg_q(arg_link.link_ds_->link_id_), arg_link.link_ds_->joint_type_);
    }

    // If successful, update the gc for this transformation matrix update.
    if(flag)
    { arg_link.q_T_ = arg_q(arg_link.link_ds_->link_id_); }

    return flag;
  }

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *      x_ancestor_link_coords = arg_link.T_lnk_ * x_link_coords
   * Note that the local transformation matrices are updated in
   * all the link data structs from arg_link to arg_ancestor.
   */
  sBool CDynamicsScl::calculateTransformationMatrixForLink(
      /** The link at which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The link up to which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** Include the offset from global origin to robot's origin */
      const bool arg_flag_include_origin_offset)
  {
    return false;
  }


  /** Initializes the dynamics to be computed for a specific robot.
   *  Returns:
   *     true : Always.
   *     false : If something is seriously wrong
   */
  sBool CDynamicsScl::init(const SRobotParsedData& arg_robot_data)
  {
    try
    {
      robot_parsed_data_ = & arg_robot_data;
      if(NULL == robot_parsed_data_)
      { throw(std::runtime_error("Got null pointer while referring to object. Should never happen."));  }
      has_been_init_ = true;
    }
    catch(std::exception& e)
    { std::cerr<<"\n"<<e.what();  }
    return has_been_init_;
  }

} /* namespace scl */
