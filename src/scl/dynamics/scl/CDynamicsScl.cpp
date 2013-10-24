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
      const Eigen::VectorXd& arg_q)
  {
    bool flag;

    //The matrix is already up to date. Just return.
    // Update if q_T_ is NaN (never initialized before).
    if(false == arg_link.link_ds_->is_root_){
      //Not root, and gc didn't change, and has been computed atleast once before.
      if( fabs(arg_q(arg_link.link_ds_->link_id_) - arg_link.q_T_) <
          std::numeric_limits<sFloat>::epsilon() &&
          false == std::isnan(arg_link.q_T_))
      { return true;  }
    }
    else{
#ifdef DEBUG
      //The root link never changes after it has been set. And it should have been set
      //at the time the rigid body dyn setup was initialized.
      if(true == std::isnan(arg_link.q_T_))
      { return false;  }
#endif
      return true;
    }

    // Compute the transformation matrix.
    flag = sclTransform(arg_link.T_lnk_, arg_link.link_ds_->pos_in_parent_,
        arg_q(arg_link.link_ds_->link_id_), arg_link.link_ds_->joint_type_);

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
      /** The transformation matrix in which to store the answer */
      Eigen::Affine3d& arg_T,
      /** The link at which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The link up to which the transformation matrix is to be calculated.
       * Pass NULL to compute the transform up to the global root. */
      const SRigidBodyDyn* arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  {
    bool flag = true;

    //First compute all the transformations from the link to its ancestor.
    SRigidBodyDyn *rbd = &arg_link;

    //Reset transform to identity. Good if ancestor is node itself
    arg_T.setIdentity();

    //Walk up the tree.
    while(rbd != arg_ancestor)
    {//Keep going till you reach the ancestor
      flag = calculateTransformationMatrixForLink(*rbd, arg_q);
      arg_T = rbd->T_lnk_ * arg_T;
      rbd = rbd->parent_addr_;
    }

    if(S_NULL == arg_ancestor && false == arg_link.link_ds_->is_root_)//Updated link to origin.
    {
      arg_link.T_o_lnk_ = arg_T;
      arg_link.q_T_o_ = arg_q(arg_link.link_ds_->link_id_);
    }

    return flag;
  }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *                dx = Jx . dq
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link.
   */
  sBool CDynamicsScl::calculateJacobian(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The link up to which the Jacobian is to be calculated
       * Pass NULL to compute the Jacobian up to the global root. */
      const SRigidBodyDyn* arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local,
      /** Whether to recompute the transformations up to the ancestor
       * Default = true. Set to false to speed things up.*/
      const bool arg_recompute_transforms)
  {
    bool flag = true;

    //To efficiently walk up the tree to the node's ancestor.
    SRigidBodyDyn *rbd = &arg_link;

    if(arg_recompute_transforms)
    {//Recompute the transformation matrices.
      Eigen::Affine3d Ttmp;
      //Walk up the tree.
      while(flag && rbd != arg_ancestor)
      {//Keep going till you reach the ancestor
        //Gets global transforms for all links.
        flag = calculateTransformationMatrixForLink(Ttmp,*rbd,NULL,arg_q);
        if(false == flag)
        {
          std::cerr<<"\nCDynamicsScl::calculateJacobian() : Could not compute transforms.";
          return false;
        }
        rbd = rbd->parent_addr_;
      }
      //Reset the pointer to the current node for further processing
      rbd = &arg_link;
    }

    //Reset Jacobian to zero. Good if ancestor is node itself
    arg_J.setZero(6,arg_q.rows());

    //Temporary transform to track position up to the present point.
    Eigen::Affine3d T_to_joint;
    T_to_joint.setIdentity();
    Eigen::Vector3d pos_wrt_joint, axis_global_frame;

    //Walk up the tree.
    const SRigidBodyDyn* up_link;
    //NOTE TODO: This avoids reaching global root. J might be incorrect for
    // robot roots that are at a different orientation than the global root.
    if(S_NULL == arg_ancestor)
    { up_link = arg_ancestor; }
    else
    { up_link = arg_ancestor->parent_addr_; }
    while(flag && rbd != up_link)
    {//Keep going till you reach the ancestor
      const int i = rbd->link_ds_->link_id_;
      switch(rbd->link_ds_->joint_type_)
      {
        case JOINT_TYPE_PRISMATIC_X:
          arg_J.block(0,i,3,1) = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitX();
          break;
        case JOINT_TYPE_PRISMATIC_Y:
          arg_J.block(0,i,3,1) = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitY();
          break;
        case JOINT_TYPE_PRISMATIC_Z:
          arg_J.block(0,i,3,1) = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitZ();
          break;
        case JOINT_TYPE_REVOLUTE_X:
          arg_J(3,i) = 1;
          pos_wrt_joint = rbd->T_o_lnk_.rotation()*T_to_joint * arg_pos_local;
          axis_global_frame = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitX();
          arg_J.block(0,i,3,1) = axis_global_frame.cross(pos_wrt_joint);
          break;
        case JOINT_TYPE_REVOLUTE_Y:
          arg_J(4,i) = 1;
          pos_wrt_joint = rbd->T_o_lnk_.rotation()*T_to_joint * arg_pos_local;
          axis_global_frame = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitY();
          arg_J.block(0,i,3,1) = axis_global_frame.cross(pos_wrt_joint);
          break;
        case JOINT_TYPE_REVOLUTE_Z:
          arg_J(5,i) = 1;
          pos_wrt_joint = rbd->T_o_lnk_.rotation()*T_to_joint * arg_pos_local;
          axis_global_frame = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitZ();
          arg_J.block(0,i,3,1) = axis_global_frame.cross(pos_wrt_joint);
          break;
        default:
          if(rbd->link_ds_->is_root_)//Root node has no joint.
          { break;  }
          flag = false;
          break;
      }
      //Move the transform one frame back.
      if(S_NULL != rbd)//Exit when reached global origin
      { T_to_joint = rbd->T_lnk_ * T_to_joint;  }
      //Move up one link.
      rbd = rbd->parent_addr_;
    }

    return flag;
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
