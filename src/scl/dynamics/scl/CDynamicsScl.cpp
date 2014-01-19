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
  sBool CDynamicsScl::computeGCModel(/** Current robot state. q, dq, ddq,
              sensed generalized forces and perceived external forces.*/
        const SRobotSensors * arg_sensor_data,
        /** Individual link Jacobians, and composite intertial,
              centrifugal/coriolis gravity estimates.*/
        SGcModel * arg_gc_model)
  {
    bool flag = true;
    const Eigen::VectorXd &q = arg_sensor_data->q_;
    const Eigen::VectorXd &dq = arg_sensor_data->dq_;
    sutil::CMappedTree<std::string, SRigidBodyDyn> &rbtree = arg_gc_model->rbdyn_tree_;

    //0. Update the coordinates
    arg_gc_model->q_ = q;
    arg_gc_model->dq_ = dq;

    //1. Update the transformation matrices. Everything else depends on it.
    flag = flag && computeTransformsForAllLinks(rbtree, q);

    //2. Update the com Jacobians.
    flag = flag && computeJacobianComForAllLinks(arg_gc_model->rbdyn_tree_,arg_sensor_data->q_);

    //3. Update generalized inertia and its inverse
    flag = flag && computeInertiaGC(arg_gc_model->M_gc_, arg_gc_model->rbdyn_tree_, arg_sensor_data->q_);
    arg_gc_model->M_gc_inv_ = arg_gc_model->M_gc_.inverse(); //A is always invertible.

    //4. Update b_
    //arg_gc_model->b_

    //5. Update the generalized gravity force
    flag = flag && computeForceGravityGC(arg_gc_model->force_gc_grav_, arg_gc_model->rbdyn_tree_, arg_sensor_data->q_);

    return flag;
  }

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *        x_parent_link_coords = arg_link.T_lnk_ * x_link_coords
   * Note that the matrix is stored in the passed link data struct.
   */
  sBool CDynamicsScl::computeTransform(
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
        arg_link.link_ds_->ori_parent_quat_,
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
  sBool CDynamicsScl::computeTransformToAncestor(
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
      flag = computeTransform(*rbd, arg_q);
      arg_T = rbd->T_lnk_ * arg_T;
      rbd = rbd->parent_addr_;
    }

    if(S_NULL == arg_ancestor && false == arg_link.link_ds_->is_root_)//Updated link to origin.
    { arg_link.T_o_lnk_ = arg_T; }

    return flag;
  }

  /** Updates the Transformation Matrices for the robot to which
   * this dynamics object is assigned.
   *      x_ancestor_link_coords = arg_link.T_lnk_ * x_link_coords */
  sBool CDynamicsScl::computeTransformsForAllLinks(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  {
    bool flag = true;
    sutil::CMappedTree<std::string, SRigidBodyDyn>::iterator it,ite;

    //1. First update the link transforms
    for(it = arg_tree.begin(), ite = arg_tree.end(); it!=ite; ++it)
    { flag = flag && computeTransform(*it, arg_q); }

    //2. Next update the origin transforms
    for(it = arg_tree.begin(), ite = arg_tree.end(); it!=ite; ++it)
    {
      it->T_o_lnk_ = it->T_lnk_;
      Eigen::Affine3d &To = it->T_o_lnk_;
      SRigidBodyDyn *rbd = it->parent_addr_;
      while(rbd != NULL)
      {
        To = rbd->T_lnk_ * To;
        rbd = rbd->parent_addr_;
      }

#ifdef SCL_PRINT_INFO_MESSAGES
      std::cout<<"\n\nLink:"<<it->name_
          <<"\n\tTlnk:\n"<<it->T_lnk_.matrix()
          <<"\n\tOriParent:\n"<<it->link_ds_->ori_parent_quat_.toRotationMatrix()
          <<"\n\tTOlnk:\n"<<it->T_o_lnk_.matrix();
#endif
    }
    return flag;
  }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *                dx = Jx . dq
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link.
   */
  sBool CDynamicsScl::computeJacobianWithTransforms(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local)
  {
    bool flag = true;

    //Walk up the tree to the node's ancestor, compute all Transforms
    SRigidBodyDyn *rbd = &arg_link;
    Eigen::Affine3d Ttmp; // NOTE TODO: Should have a function that doesn't need a temp matrix
    while(flag && rbd != S_NULL)
    {//Gets global transforms for all links.
      flag = flag && computeTransformToAncestor(Ttmp,*rbd,NULL,arg_q);
      rbd = rbd->parent_addr_;
    }
    // Now compute the Jacobian
    flag = flag && computeJacobian(arg_J, arg_link, arg_q, arg_pos_local);

    return flag;
  }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *            dx_global_origin = Jx . dq
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link.
   * ** CONST VERSION **
   */
  sBool CDynamicsScl::computeJacobian(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated */
      const SRigidBodyDyn& arg_link,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local) const
  {
    bool flag = true;

    //To efficiently walk up the tree to the node's ancestor.
    const SRigidBodyDyn *rbd = &arg_link;

    //Reset Jacobian to zero. Good if ancestor is node itself
    arg_J.setZero(6,arg_q.rows());

    //Temporary transform to track position up to the present point.
    Eigen::Affine3d T_to_joint;
    T_to_joint.setIdentity();
    Eigen::Vector3d pos_wrt_joint_global_frame, axis_global_frame;

    //Walk up the tree.
    while(flag && rbd != NULL)
    {//Keep going till you reach the ancestor
      const int i = rbd->link_ds_->link_id_;
      switch(rbd->link_ds_->joint_type_)
      {
        /** For linear joints:
         * Jv => dq = The linear velocity vector simply needs to be rotated to the global frame
         * Jω => Zero. There is no contribution to the angular velocity.
         */
        case JOINT_TYPE_PRISMATIC_X:
          //Jv
          arg_J.block(0,i,3,1) = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitX();
          break;
        case JOINT_TYPE_PRISMATIC_Y:
          //Jv
          arg_J.block(0,i,3,1) = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitY();
          break;
        case JOINT_TYPE_PRISMATIC_Z:
          //Jv
          arg_J.block(0,i,3,1) = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitZ();
          break;
        /** For rotational joints:
         * Jv => cross product of operational point and angular velocity at the given dof.
         * Jω => The angular velocity vector simply needs to be rotated to the global frame.
         */
        case JOINT_TYPE_REVOLUTE_X:
          //Jv
          pos_wrt_joint_global_frame = rbd->T_o_lnk_.rotation()*T_to_joint * arg_pos_local;
          axis_global_frame = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitX();
          arg_J.block(0,i,3,1) = axis_global_frame.cross(pos_wrt_joint_global_frame);
          //Jω
          arg_J.block(3,i,3,1) = axis_global_frame;
          break;
        case JOINT_TYPE_REVOLUTE_Y:
          //Jv
          pos_wrt_joint_global_frame = rbd->T_o_lnk_.rotation()*T_to_joint * arg_pos_local;
          axis_global_frame = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitY();
          arg_J.block(0,i,3,1) = axis_global_frame.cross(pos_wrt_joint_global_frame);
          //Jω
          arg_J.block(3,i,3,1) = axis_global_frame;
          break;
        case JOINT_TYPE_REVOLUTE_Z:
          //Jv
          pos_wrt_joint_global_frame = rbd->T_o_lnk_.rotation()*T_to_joint * arg_pos_local;
          axis_global_frame = rbd->T_o_lnk_.rotation()*Eigen::Vector3d::UnitZ();
          arg_J.block(0,i,3,1) = axis_global_frame.cross(pos_wrt_joint_global_frame);
          //Jω
          arg_J.block(3,i,3,1) = axis_global_frame;
          break;
        default:
          if(rbd->link_ds_->is_root_)//Root node has no joint.
          { break;  }
          flag = false;
          break;
      }//End of switch
      T_to_joint = rbd->T_lnk_ * T_to_joint; //Move the transform one frame back.
      rbd = rbd->parent_addr_;               //Move up one link.
    }//End of while loop moving up the rigid body dyn tree

    return flag;
  }


  /** Gets the robot's kinetic energy */
  sFloat CDynamicsScl::computeEnergyKinetic(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The current generalized velocities. */
      const Eigen::VectorXd& arg_dq)
  {
    if(false == has_been_init_) { return std::numeric_limits<sFloat>::quiet_NaN(); }
#ifdef DEBUG
    if(arg_q.rows() != static_cast<int>(arg_tree.size())-1 || arg_q.rows() != arg_dq.rows())
    { return std::numeric_limits<sFloat>::quiet_NaN(); }
#endif

    bool flag; sFloat ret_ke = 0.0;
    int dof = arg_q.rows();

    // Update transformations for computing the Jacobians, which are used to compute KE
    flag = computeTransformsForAllLinks(arg_tree, arg_q);
    flag = flag && computeJacobianComForAllLinks(arg_tree,arg_q);
    if(false == flag){ return std::numeric_limits<sFloat>::quiet_NaN(); }

    // Compute kinetic energy
    sutil::CMappedTree<std::string, SRigidBodyDyn>::iterator it,ite;
    for(it = arg_tree.begin(), ite = arg_tree.end(); it!=ite; ++it)
    {// Walk over all links.
      if(it->link_ds_->is_root_){ continue;  }//Root doesn't move
      // Kinetic energy at the link due to translation.
      ret_ke += it->link_ds_->mass_ * (arg_dq.transpose() * it->J_com_.block(0,0,3,dof).transpose())
                  * (it->J_com_.block(0,0,3,dof) * arg_dq);
      // Kinetic energy at the link due to rotation.
      ret_ke += (arg_dq.transpose() * it->J_com_.block(3,0,3,dof).transpose())
                  * it->link_ds_->inertia_
                  * (it->J_com_.block(3,0,3,dof)*arg_dq);
    }

    return ret_ke;
  }

  /** Gets the robot's potential energy */
  sFloat CDynamicsScl::computeEnergyPotential(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  {
    if(false == has_been_init_) { return std::numeric_limits<sFloat>::quiet_NaN(); }
#ifdef DEBUG
    if(arg_q.rows() != static_cast<int>(arg_tree.size())-1)
    { return std::numeric_limits<sFloat>::quiet_NaN(); }
#endif

    sFloat ret_pe = 0.0;

    // Update transformations, which are necessary to find the com vectors in Origin coords
    if(false == computeTransformsForAllLinks(arg_tree, arg_q))
    { return std::numeric_limits<sFloat>::quiet_NaN(); }

    sutil::CMappedTree<std::string, SRigidBodyDyn>::iterator it,ite;
    for(it = arg_tree.begin(), ite = arg_tree.end(); it!=ite; ++it)
    {
      if(it->link_ds_->is_root_){ continue;  }//Root doesn't experience gravity
      ret_pe += it->link_ds_->mass_ * static_cast<double>((it->T_o_lnk_ * it->link_ds_->com_).transpose()
          * robot_parsed_data_->gravity_);
    }

    return ret_pe;
  }


  /** Initializes the dynamics to be computed for a specific robot.
   *  Returns:
   *     true : Always.
   *     false : If something is seriously wrong
   */
  sBool CDynamicsScl::init(const SRobotParsed& arg_robot_data)
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
