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
  SActuatorMuscle::SViaPointSet::SViaPointSet() :
    parent_link_0_(""), parent_link_1_(""),
    dynamics_link_id_0_(NULL), dynamics_link_id_1_(NULL),
    child_link_id_(-1)
  {}

  SActuatorMuscle::SActuatorMuscle() : SObject("SActuatorMuscle")
  {}

  /** Default constructor. Sets stuff to NULL. */
  CActuatorMuscle::CActuatorMuscle() :
      robot_(NULL), robot_io_ds_(NULL),
      msys_(NULL), muscle_(NULL), dynamics_(NULL)
  {}

  /** Initializes the actuator. This involves determining whether the muscle
   * matches the robot etc. It also sets up the Jacobians to be computed etc.
   */
  sBool CActuatorMuscle::init(const std::string& arg_name,
      const SRobotParsedData *arg_robot,
      const SRobotIOData *arg_rob_io_ds,
      const SMuscleSystem *arg_msys,
      CDynamicsBase *arg_dynamics)
  {
    bool flag;
    try
    {
      //Check pointers.
      if(NULL==arg_robot)
      { throw(std::runtime_error("Passed NULL robot parsed data struct")); }
      if(NULL==arg_rob_io_ds)
      { throw(std::runtime_error("Passed NULL robot io data struct")); }
      if(NULL==arg_msys)
      { throw(std::runtime_error("Passed NULL muscle spec data struct")); }
      if(NULL==arg_dynamics)
      { throw(std::runtime_error("Passed NULL robot dynamics object")); }

      //Set the muscle name
      data_.name_ = arg_name;

      //Save pointers
      robot_ = arg_robot;
      robot_io_ds_ = arg_rob_io_ds;
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
        bool tmp_parent_link=-1;

        const SMusclePoint& tmp_pt_0 = muscle_->points_[i]; //tmp reference
        const SMusclePoint& tmp_pt_1 = muscle_->points_[i+1]; //tmp reference

        //Test that the parent links exist in the robot
        const SRigidBody *tmp_rb_0 = arg_robot->robot_br_rep_.at_const(tmp_pt_0.parent_link_);
        if(NULL == tmp_rb_0)
        { throw(std::runtime_error(std::string("Could not find parent link for muscle: ")+tmp_pt_0.parent_link_)); }
        //Check which of the two links is the parent (useful for determining the dynamics -> getIdForLink())
        if(tmp_pt_1.parent_link_ == tmp_rb_0->parent_name_)
        { tmp_parent_link = 1;  }

        const SRigidBody *tmp_rb_1 = arg_robot->robot_br_rep_.at_const(tmp_pt_1.parent_link_);
        if(NULL == tmp_rb_1)
        { throw(std::runtime_error(std::string("Could not find parent link for muscle: ")+tmp_pt_1.parent_link_)); }
        //Check which of the two links is the parent (useful for determining the dynamics -> getIdForLink())
        if(tmp_pt_0.parent_link_ == tmp_rb_1->parent_name_)
        { tmp_parent_link = 0;  }

        if(-1==tmp_parent_link)
        { throw(std::runtime_error(std::string("Parent links for successive muscle via points disconnected at: ")+arg_name)); }

        // **********************
        //Detect link transition.
        if(tmp_pt_0.parent_link_ != tmp_pt_1.parent_link_)
        {// This set of via points contributes to muscle length changes. Use it.
          SActuatorMuscle::SViaPointSet* pt_set = data_.via_point_set_.create(i);
          if(NULL == pt_set)
          { throw(std::runtime_error("Could not find create via point in the mapped list")); }

          // Set the parent links
          pt_set->parent_link_0_ = tmp_pt_0.parent_link_;
          pt_set->parent_link_1_ = tmp_pt_1.parent_link_;

          pt_set->position_in_parent_0_ = tmp_pt_0.point_;
          pt_set->position_in_parent_1_ = tmp_pt_1.point_;

          pt_set->dynamics_link_id_0_ = dynamics_->getIdForLink(tmp_pt_0.parent_link_);
          pt_set->dynamics_link_id_1_ = dynamics_->getIdForLink(tmp_pt_1.parent_link_);

          // NOTE TODO : This is a quirk of tao. Each link has one dof.
          // Can be improved later. Generalize for different dynamics engines.
          // For spherical coordinates, for instance, 3 gc ids should be returned.
          if(0 == tmp_parent_link)
          {
            pt_set->child_link_id_ = 1;
            pt_set->scl_gc_id_.push_back(tmp_rb_1->link_id_);
          }
          else
          {
            pt_set->child_link_id_ = 0;
            pt_set->scl_gc_id_.push_back(tmp_rb_0->link_id_);
          }
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
    if(NULL == robot_) {  data_.has_been_init_ = false; return false; }
    if(NULL == robot_io_ds_) {  data_.has_been_init_ = false; return false; }
    if(NULL == msys_) {  data_.has_been_init_ = false; return false; }
    if(NULL == muscle_) {  data_.has_been_init_ = false; return false; }
    if(NULL == dynamics_) {  data_.has_been_init_ = false; return false; }

    return data_.has_been_init_;
  }


  /** Some actuator sets don't directly actuate the generalized coordinates
   * and require a Jacobian to compute their contribution to the generalized
   * forces.
   *
   * Each actuator instance must implement this. */
  sBool CActuatorMuscle::computeJacobian(Eigen::VectorXd& ret_J)
  {//This function doesn't use std::exceptions (for speed).
    if(false == data_.has_been_init_) { return data_.has_been_init_; }

    //Compute the change in length at this point.
    bool flag = true;

    //Zero the Jacobian. And get the latest gc configuration.
    ret_J.Zero(robot_->dof_);
    data_.dq_curr_ = robot_io_ds_->sensors_.dq_;

    //1. Iterate over all gc spanning muscle via-points.
    sutil::CMappedList<sUInt,SActuatorMuscle::SViaPointSet>::iterator it,ite;
    for(it = data_.via_point_set_.begin(), ite = data_.via_point_set_.end(); it!=ite; ++it)
    {
      SActuatorMuscle::SViaPointSet& tmp_pt_set = *it;

      //1.a: Set up vars to find position of the via points
      // NOTE TODO : This is inefficient. Shouldn't require this. Improve dynamics engine
      // to use direct vectors without re-doing all this.
      Eigen::Affine3d T;

      //1.b.0: Compute point offset in global coords
      flag = dynamics_->calculateTransformationMatrix(tmp_pt_set.dynamics_link_id_0_,T);
      tmp_pt_set.x_glob_0_ = T * tmp_pt_set.position_in_parent_0_;

      //1.b.1: Compute point offset in global coords
      flag = flag && dynamics_->calculateTransformationMatrix(tmp_pt_set.dynamics_link_id_1_,T);
      tmp_pt_set.x_glob_1_ = T * tmp_pt_set.position_in_parent_1_;

      //1.c.0: Compute Jacobians at the via points.
      flag = flag && dynamics_->calculateJacobian(tmp_pt_set.dynamics_link_id_0_, tmp_pt_set.x_glob_0_, tmp_pt_set.J_0_);
      //Use the position jacobian only. This is a point task.
      tmp_pt_set.J_0_ = tmp_pt_set.J_0_.block(0,0,3,robot_->dof_);

      //1.c.1: Compute Jacobians at the via points.
      flag = flag && dynamics_->calculateJacobian(tmp_pt_set.dynamics_link_id_1_, tmp_pt_set.x_glob_1_, tmp_pt_set.J_1_);
      //Use the position jacobian only. This is a point task.
      tmp_pt_set.J_1_ = tmp_pt_set.J_1_.block(0,0,3,robot_->dof_);

      if(false == flag){  return false; }

      //2. Iterate over all the gcs between a set of via-points. Each gc's col will be populated.
      std::vector<sUInt>::const_iterator its, itse;
      for(its = tmp_pt_set.scl_gc_id_.begin(), itse = tmp_pt_set.scl_gc_id_.end();
          its != itse; ++its)
      {
        sUInt jcol = *its;// The column of the Jacobian to fill..
        double grad_gc;

        //2.a: Compute the instantaneous velocity due to the gc
        tmp_pt_set.dx_glob_0_ = tmp_pt_set.J_0_.col(jcol) * data_.dq_curr_(jcol);
        tmp_pt_set.dx_glob_1_ = tmp_pt_set.J_1_.col(jcol) * data_.dq_curr_(jcol);

        grad_gc = (tmp_pt_set.x_glob_1_+tmp_pt_set.dx_glob_1_-
                      tmp_pt_set.x_glob_0_-tmp_pt_set.dx_glob_0_).norm()
            - (tmp_pt_set.x_glob_1_-tmp_pt_set.x_glob_0_).norm();

        //2.b: Compute the muscle's length gradient wrt the gc
        ret_J(jcol) = grad_gc;
      }
    }

    return true;
  }

} /* namespace scl */
