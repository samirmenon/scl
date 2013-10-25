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
 * CDynamicsScl.hpp
 *
 *  Created on: Oct 7, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CDYNAMICSSCL_HPP_
#define CDYNAMICSSCL_HPP_

#include <scl/dynamics/CDynamicsBase.hpp>
#include <scl/data_structs/SRigidBodyDyn.hpp>

#include <sutil/CMappedTree.hpp>
#include <string>

namespace scl
{
  /** This class implements dynamics algorithms for control
   * and simulation using the scl library.
   *
   * The primary goal of this implementation---given that there are
   * alternatives---is to enable quick Jacobian and transformation matrix
   * computations.
   *
   * The main feature of this dynamics implementation is that it is
   * truly stateless.
   *
   * It does not (presently) implement an integrator so plan
   * on using an alternative Physics engine (like Tao).
   */
  class CDynamicsScl : public scl::CDynamicsBase
  {
public:
  /* *******************************************************************
   *                      Overall Computational function.
   * ******************************************************************* */
    /** Updates the joint space model matrices (Everything in SGcModel).
     *
     * This is the most efficient method to access the standard matrices.
     * Computing transformations and Jacobians individually is typically
     * wasteful.
     */
  virtual sBool computeGCModel(/** Current robot state. q, dq, ddq,
            sensed generalized forces and perceived external forces.*/
      const SRobotSensorData * arg_sensor_data,
      /** Individual link Jacobians, and composite intertial,
          centrifugal/coriolis gravity estimates. */
      SGcModel * arg_gc_model);

  /* *******************************************************************
   *                      Coordinate Transformations
   * ******************************************************************* */
  /** Updates the Transformation Matrices for the robot to which
   * this dynamics object is assigned.
   *      x_ancestor_link_coords = arg_link.T_lnk_ * x_link_coords */
  inline virtual sBool computeTransformsForAllLinks(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q);

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *        x_parent_link_coords = arg_link.T_lnk_ * x_link_coords
   * Note that the matrix is stored in the passed link data struct.
   */
  virtual sBool computeTransform(
      /** The link at which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q);

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *      x_ancestor_link_coords = arg_link.T_lnk_ * x_link_coords
   *
   * Note that the local transformation matrices are updated in
   * all the link data structs from arg_link to arg_ancestor.
   *
   * Also note that there are two origins, robot origin, which actually
   * has a node allocated for it. And the global scl origin, for which
   * you should pass in a NULL as ancestor.
   */
  virtual sBool computeTransformToAncestor(
      /** The transformation matrix in which to store the answer */
      Eigen::Affine3d& arg_T,
      /** The link at which the transformation matrix is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The link up to which the transformation matrix is to be calculated
       * Pass NULL to compute the transform up to the global root. */
      const SRigidBodyDyn* arg_ancestor,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q);

  /* *******************************************************************
   *                      Coordinate Jacobians
   * ******************************************************************* */
  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *            dx_global_origin = Jx . dq
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link.
   */
  sBool computeJacobian(
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J,
      /** The link at which the Jacobian is to be calculated */
      SRigidBodyDyn& arg_link,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The offset from the link's frame (in link coordinates). */
      const Eigen::Vector3d& arg_pos_local,
      /** Whether to recompute the transformations up to the ancestor
       * Default = true. Set to false to speed things up.*/
      const bool arg_recompute_transforms=true);

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *            dx_ancestor_coords = Jx . dq
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link.
   */
  sBool computeJacobian(
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
      const bool arg_recompute_transforms=true)
  { return false; }

  /** Updates the center of mass Jacobians for the robot  to which
   * this dynamics object is assigned.
   *      dx_com_origin_coords = tree_link.J_com_ * dq */
  inline sBool computeJacobianComForAllLinks(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  {
    if(false == has_been_init_){  return false; }
    bool flag = true; sutil::CMappedTree<std::string, SRigidBodyDyn>::iterator it,ite;
    //Update the com Jacobians.
    for(it = arg_tree.begin(), ite = arg_tree.end(); it!=ite; ++it)
    { flag = flag && computeJacobian(it->J_com_,*it, arg_q, it->link_ds_->com_,false); }
    return flag;
  }

  /* *******************************************************************
   *                      Dynamics Helper functions.
   * ******************************************************************* */
  /** Updates the generalized inertia for the robot  to which
   * this dynamics object is assigned.
   *      Mgc = sum_link_i [ J_com_' * M_com_i * J_com_] */
  inline sBool computeInertiaGC(Eigen::MatrixXd &ret_Mgc,
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  {
    if(false == has_been_init_){  return false; }
    bool flag = true; sutil::CMappedTree<std::string, SRigidBodyDyn>::iterator it,ite;
    int dof = robot_parsed_data_->dof_;
    ret_Mgc.setZero(dof, dof);//Set the generalized inertia to zero.
    for(it = arg_tree.begin(), ite = arg_tree.end(); it!=ite; ++it)
    {//Compute each link's contribution to the overall Mgc
      if(it->link_ds_->is_root_){ continue;  }//Root doesn't move
      ret_Mgc += it->link_ds_->mass_ * (it->J_com_.block(0,0,3,dof).transpose() * it->J_com_.block(0,0,3,dof));
      ret_Mgc += it->J_com_.block(3,0,3,dof).transpose() * it->link_ds_->inertia_ * it->J_com_.block(3,0,3,dof);
    }
    return flag;
  }

  /** Updates the center of mass Jacobians for the robot  to which
   * this dynamics object is assigned.
   *      dx_com_origin_coords = tree_link.J_com_ * dq */
  inline sBool computeForceGravityGC(Eigen::VectorXd &ret_FgravGC,
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q)
  {
    if(false == has_been_init_){  return false; }
    bool flag = true; sutil::CMappedTree<std::string, SRigidBodyDyn>::iterator it,ite;
    //Update the generalized gravity force vector.
    int dof = robot_parsed_data_->dof_;
    ret_FgravGC.setZero(dof);
    for(it = arg_tree.begin(), ite = arg_tree.end(); it!=ite; ++it)
    {
      if(it->link_ds_->is_root_){ continue;  }//Root doesn't experience gravity
      ret_FgravGC += it->J_com_.transpose() * robot_parsed_data_->gravity_;
    }
    return flag;
  }

  /* *******************************************************************
   *                      Dynamics State functions.
   * ******************************************************************* */
  /** Gets the robot's kinetic energy */
  virtual sFloat computeEnergyKinetic(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q,
      /** The current generalized velocities. */
      const Eigen::VectorXd& arg_dq);

  /** Gets the robot's potential energy */
  virtual sFloat computeEnergyPotential(
      /** The tree for which the transformation matrices are to be updated */
      sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree,
      /** The current generalized coordinates. */
      const Eigen::VectorXd& arg_q);

  /* *******************************************************************
   *                      Initialization functions.
   * ******************************************************************* */
  /** Default constructor sets the initialization state to false */
  CDynamicsScl() : CDynamicsBase() { }

  /** Default destructor does nothing */
  virtual ~CDynamicsScl(){}

  /** Initializes the dynamics to be computed for a specific robot.
   *  Returns:
   *     true : Succeeds in creating a dynamics object
   *
   *     false : If it can't find the robot or if the information
   *             is inconsistent with what the implementation requires,
   *             it returns false
   */
  virtual sBool init(const SRobotParsedData& arg_robot_data);
};

} /* namespace scl */
#endif /* CDYNAMICSSCL_HPP_ */
