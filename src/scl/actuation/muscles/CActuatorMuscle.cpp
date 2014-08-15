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
 * \file CActuatorMuscle.cpp
 *
 *  Created on: Jul 21, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CActuatorMuscle.hpp"

#include <iostream>
#include <stdexcept>

namespace scl
{
  /** Default constructor. Sets stuff to NULL. */
  CActuatorMuscle::CActuatorMuscle() :
      robot_(NULL),
      msys_(NULL), muscle_(NULL), dynamics_(NULL)
  {}

  /** Initializes the actuator. This involves determining whether the muscle
   * matches the robot etc. It also sets up the Jacobians to be computed etc.
   */
  sBool CActuatorMuscle::init(const std::string& arg_name,
      const SRobotParsed *arg_robot,
      const SActuatorSetMuscleParsed *arg_msys,
      const sutil::CMappedList<std::string,SRigidBodyDyn> &arg_rbdtree,
      CDynamicsBase *arg_dynamics)
  {
    bool flag;
    try
    {
      //Check pointers.
      if(NULL==arg_robot)
      { throw(std::runtime_error("Passed NULL robot parsed data struct")); }
      if(NULL==arg_msys)
      { throw(std::runtime_error("Passed NULL muscle spec data struct")); }
      if(false==arg_msys->hasBeenInit())
      { throw(std::runtime_error("Passed unintialized muscle spec data struct")); }
      if(NULL==arg_dynamics)
      { throw(std::runtime_error("Passed NULL robot dynamics object")); }

      //Set the muscle name
      data_.name_ = arg_name;

      //Save pointers
      robot_ = arg_robot;
      msys_ = arg_msys;
      dynamics_ = arg_dynamics;

      //Check that the muscle exists
      muscle_ = arg_msys->muscles_.at_const(arg_name);
      if(NULL==muscle_)
      { throw(std::runtime_error("Could not find muscle in provided muscle system")); }

      //Check whether the robot has the correct links for this muscle
      // We don't use iterators here because muscle points are ordered by index
      int n_pts = muscle_->points_.size();

      //Clear any existing data.
      flag = data_.via_point_set_.clear();
      if(false==flag)
      { throw(std::runtime_error("Could not clear existing muscle via points")); }

      // We only iterate to the second last point because we are concerned with successive
      // pairs. So reaching the second last will test for all possible pairs
      for(int i=0; i<n_pts-1; ++i)
      {
        int tmp_child_link_id=-1;

        const SMusclePointParsed& tmp_pt_0 = muscle_->points_[i]; //tmp reference
        const SMusclePointParsed& tmp_pt_1 = muscle_->points_[i+1]; //tmp reference

        if(tmp_pt_0.parent_link_ == tmp_pt_1.parent_link_)
        { continue; } //Bot points are connected to same link. Don't contribute to J. Ignore.

        //Test that the parent links for both muscle attachment points exist in the robot
        const SRigidBody *tmp_rb_0 = arg_robot->rb_tree_.at_const(tmp_pt_0.parent_link_);
        if(NULL == tmp_rb_0)
        { throw(std::runtime_error(std::string("Could not find parent link for muscle: ")+tmp_pt_0.parent_link_)); }

        const SRigidBody *tmp_rb_1 = arg_robot->rb_tree_.at_const(tmp_pt_1.parent_link_);
        if(NULL == tmp_rb_1)
        { throw(std::runtime_error(std::string("Could not find parent link for muscle: ")+tmp_pt_1.parent_link_)); }

        //Check which of the two links is the parent (useful for determining the dynamics -> getIdForLink())
        if(arg_robot->rb_tree_.isAncestor(tmp_rb_0, tmp_rb_1))
        { tmp_child_link_id = 0;  }
        else if(arg_robot->rb_tree_.isAncestor(tmp_rb_1, tmp_rb_0))
        { tmp_child_link_id = 1;  }
        else
        { throw(std::runtime_error(std::string("Parent links for successive muscle via points disconnected at: ")+arg_name)); }

        // This set of via points contributes to muscle length changes.
        // Now actually set via point set data struct properties.
        SActuatorMuscle::SViaPointSet* pt_set = data_.via_point_set_.create(i);
        if(NULL == pt_set)
        { throw(std::runtime_error("Could not find created via point in the mapped list")); }

        // Set the parent links
        pt_set->parent_link_0_ = tmp_pt_0.parent_link_;
        pt_set->parent_link_1_ = tmp_pt_1.parent_link_;

        pt_set->pos_in_parent_0_ = tmp_pt_0.pos_in_parent_;
        pt_set->pos_in_parent_1_ = tmp_pt_1.pos_in_parent_;

        //Indicate which link in the pair corresponds to the child.
        pt_set->child_link_id_ = tmp_child_link_id;

        // For root links, the via point positions are constant
        if(tmp_rb_0->is_root_){
          pt_set->is_root_0_ = true;
          pt_set->x_glob_0_ = pt_set->pos_in_parent_0_+tmp_rb_0->pos_in_parent_;
          pt_set->J_0_.setZero(3, robot_->dof_);
        }
        if(tmp_rb_1->is_root_){
          pt_set->is_root_1_ = true;
          pt_set->x_glob_1_ = pt_set->pos_in_parent_1_+tmp_rb_1->pos_in_parent_;
          pt_set->J_0_.setZero(3, robot_->dof_);
        }
#ifdef DEBUG
        if(tmp_rb_0->is_root_ && tmp_rb_1->is_root_)
        { throw(std::runtime_error(std::string("Both point set parent links are root nodes: ")+tmp_pt_0.parent_link_+
            std::string(", ")+ tmp_pt_1.parent_link_)); }
#endif

        // Get the dynamic objects (used to compute the Jacobian later).
        pt_set->rigid_body_dyn_0_ = arg_rbdtree.at_const(pt_set->parent_link_0_);
        if(S_NULL == pt_set->rigid_body_dyn_0_)
        { throw(std::runtime_error(std::string("Could not find link dyn object: ") + pt_set->parent_link_0_)); }
        pt_set->rigid_body_dyn_1_ = arg_rbdtree.at_const(pt_set->parent_link_1_);
        if(S_NULL == pt_set->rigid_body_dyn_1_)
        { throw(std::runtime_error(std::string("Could not find link dyn object: ") + pt_set->parent_link_1_)); }

        const SRigidBody* child = NULL, *parent = NULL;
        if(0 == pt_set->child_link_id_)
        {
          child = pt_set->rigid_body_dyn_0_->link_ds_;
          parent = pt_set->rigid_body_dyn_1_->link_ds_;
        }
        else if(1 == pt_set->child_link_id_)
        {
          child = pt_set->rigid_body_dyn_1_->link_ds_;
          parent = pt_set->rigid_body_dyn_0_->link_ds_;
        }

        if(NULL == child || NULL == parent)
        { throw(std::runtime_error("Rigid body pointer for child or parent link is null in the rbd tree")); }

        while(parent != child)
        {// Indicate which gcs are going to be modified by the joint between the parent and child link.
          pt_set->scl_gc_id_.push_back(child->link_id_);
          child = child->parent_addr_;
        }
      }// End of for loop: At this point of time, we have all the relevant point sets in data_.via_point_set_

      data_.has_been_init_ = true;
    }
    catch(std::exception &e)
    {
      std::cout<<"\nCActuatorMuscle::init() : Failed. "<<e.what();
      data_.has_been_init_ = false;
    }
    return data_.has_been_init_;
  }

  /** Has this actuator been initialized */
  sBool CActuatorMuscle::hasBeenInit()
  {
#ifdef DEBUG
    if(NULL == robot_) {  data_.has_been_init_ = false; return false; }
    if(NULL == msys_) {  data_.has_been_init_ = false; return false; }
    if(false == msys_->has_been_init_) {  data_.has_been_init_ = false; return false; }
    if(NULL == muscle_) {  data_.has_been_init_ = false; return false; }
    if(NULL == dynamics_) {  data_.has_been_init_ = false; return false; }
#endif

    return data_.has_been_init_;
  }


  /** Some actuator sets don't directly actuate the generalized coordinates
   * and require a Jacobian to compute their contribution to the generalized
   * forces.
   *
   * Each actuator instance must implement this. */
  sBool CActuatorMuscle::computeJacobian(
      const Eigen::VectorXd &arg_q,
      Eigen::VectorXd& ret_J)
  {//This function doesn't use std::exceptions (for speed).
    if(false == hasBeenInit()) { return data_.has_been_init_; }

#ifdef DEBUG
    if(arg_q.rows() != robot_->dof_)
    {
      std::cerr<<"\nCActuatorMuscle::computeJacobian("<<data_.name_<<") : Passed q and dq have different sizes.";
      return false;
    }
#endif

    //Compute the change in length at this point.
    bool flag = true;

    //Zero the Jacobian. And get the latest gc configuration.
    ret_J.setZero(robot_->dof_);

    //1. Iterate over all gc spanning muscle via-points.
    sutil::CMappedList<sUInt,SActuatorMuscle::SViaPointSet>::iterator it,ite;
    for(it = data_.via_point_set_.begin(), ite = data_.via_point_set_.end(); it!=ite; ++it)
    {
      SActuatorMuscle::SViaPointSet& tmp_pt_set = *it;

      //1.a: Set up vars to find position of the via points
      // NOTE TODO : This is inefficient. Shouldn't require this. Improve dynamics engine
      // to use direct vectors without re-doing all this.

      // The root never moves. x_glob_0_ is constant (computed at init) and J = all zeros.
      if(false == tmp_pt_set.is_root_0_){
        //1.b.0: Compute point offset in global coords
        tmp_pt_set.x_glob_0_ = tmp_pt_set.rigid_body_dyn_0_->T_o_lnk_ * tmp_pt_set.pos_in_parent_0_;

        //1.c.0: Compute Jacobians at the via points.
        flag = flag && dynamics_->computeJacobian(tmp_pt_set.J_0_, *tmp_pt_set.rigid_body_dyn_0_,
            arg_q, tmp_pt_set.pos_in_parent_0_);
        //Use the position jacobian only. This is a point task.
        tmp_pt_set.J_0_ = tmp_pt_set.J_0_.block(0,0,3,robot_->dof_);
      }

      if(false == tmp_pt_set.is_root_1_){
        //1.b.1: Compute point offset in global coords
        tmp_pt_set.x_glob_1_ = tmp_pt_set.rigid_body_dyn_1_->T_o_lnk_ * tmp_pt_set.pos_in_parent_1_;

        //1.c.1: Compute Jacobians at the via points.
        flag = flag && dynamics_->computeJacobian(tmp_pt_set.J_1_, *tmp_pt_set.rigid_body_dyn_1_,
            arg_q, tmp_pt_set.pos_in_parent_1_);
        //Use the position jacobian only. This is a point task.
        tmp_pt_set.J_1_ = tmp_pt_set.J_1_.block(0,0,3,robot_->dof_);
      }

      tmp_pt_set.x_glob_delta_ = tmp_pt_set.x_glob_0_ - tmp_pt_set.x_glob_1_;
      if(false == flag){  return false; }

#ifdef SCL_PRINT_INFO_MESSAGES
      std::cout<<"\nMuscle gc point diffs: \n"<<tmp_pt_set.x_glob_delta_.transpose();
#endif

      //2. Iterate over all the gcs between a set of via-points. Each gc's col will be populated.
      // The contribution by the others is zero.
      std::vector<sUInt>::const_iterator its, itse;
      for(its = tmp_pt_set.scl_gc_id_.begin(), itse = tmp_pt_set.scl_gc_id_.end();
          its != itse; ++its)
      {
        sUInt jcol = *its;// The column of the Jacobian to fill..
        double grad_gc;

        //2.a: Compute the instantaneous muscle section length change due to the gc
        /*   = δ/δq ( |p0 - p1| )
         *   = δ/δq ( sqrt( (p0-p1)' * (p0-p1) ) )
         *   = 1/(2 * sqrt( (p0-p1)' * (p0-p1) ) ) * δ/δq ( (p0-p1)' * (p0-p1) )
         *   = 1/(2 * sqrt( (p0-p1)' * (p0-p1) ) ) * 2* (p0-p1)' δ/δq (p0-p1)
         *   = 1/sqrt( (p0-p1)' * (p0-p1) ) * (p0-p1)' ( δ/δq (p0) - δ/δq (p1) )
         *   = 1/d.norm() * d' ( δ/δq (p0) - δ/δq (p1) ); where d = p0 - p1 */
        // Using the above formula:
        grad_gc = tmp_pt_set.x_glob_delta_.norm();

        // Avoid numerical precision errors
        if(std::numeric_limits<sFloat>::min() > fabs(grad_gc))
        { grad_gc = 0.0;  }
        else
        {
          grad_gc = 1/grad_gc;
          grad_gc *= tmp_pt_set.x_glob_delta_.transpose() * (tmp_pt_set.J_0_.col(jcol) - tmp_pt_set.J_1_.col(jcol));
        }

        //2.b: Compute the muscle's length gradient wrt the gc
        ret_J(jcol) = grad_gc;

#ifdef SCL_PRINT_INFO_MESSAGES
        std::cout<<"["<<jcol<<". x:"<<tmp_pt_set.x_glob_delta_.transpose()<<". n: "<<tmp_pt_set.x_glob_delta_.norm()
            <<". grad:"<<grad_gc<<"] ";
#endif
      }
#ifdef SCL_PRINT_INFO_MESSAGES
      std::cout<<"\nJ0: \n"<<tmp_pt_set.J_0_;
      std::cout<<"\nJ1: \n"<<tmp_pt_set.J_1_;
#endif
    }

    return true;
  }

} /* namespace scl */
