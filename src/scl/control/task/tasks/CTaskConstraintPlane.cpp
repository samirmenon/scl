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
/* \file CTaskConstraintPlane.cpp
 *
 *  Created on: Nov 02, 2015
 *
 *  Copyright (C) 2015
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <control/task/tasks/CTaskConstraintPlane.hpp>
#include <scl/control/task/tasks/data_structs/STaskConstraintPlane.hpp>

#include <stdio.h>
#include <iostream>
#include <stdexcept>
#include <sstream>

#ifdef DEBUG
#include <cassert>
#endif

#include <Eigen/Dense>

//Don't always use it. Read comments in the model update function
#include <Eigen/SVD>

namespace scl
{

  CTaskConstraintPlane::CTaskConstraintPlane() :
      CTaskBase(),
      data_(S_NULL)
  { }

  //************************
  // Inherited stuff
  //************************
  bool CTaskConstraintPlane::init(STaskBase* arg_task_data,
      CDynamicsBase* arg_dynamics)
  {
    try
    {
      if(S_NULL == arg_task_data)
      { throw(std::runtime_error("Passed a null task data structure"));  }

      if(false == arg_task_data->has_been_init_)
      { throw(std::runtime_error("Passed an uninitialized task data structure"));  }

      if(S_NULL == arg_dynamics)
      { throw(std::runtime_error("Passed a null dynamics object"));  }

      if(false == arg_dynamics->hasBeenInit())
      { throw(std::runtime_error("Passed an uninitialized dynamics object"));  }

      data_ = dynamic_cast<STaskConstraintPlane*>(arg_task_data);

      dynamics_ = arg_dynamics;
      data_->rbd_ = data_->gc_model_->rbdyn_tree_.at_const(data_->link_name_);
      if(S_NULL == data_->rbd_)
      { throw(std::runtime_error("Couldn't find link dynamics object")); }

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCTaskConstraintPlane::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  STaskBase* CTaskConstraintPlane::getTaskData()
  { return data_; }

  /** Sets the current goal position */
  bool CTaskConstraintPlane::setGoalPlane(
      const Eigen::Vector3d & arg_p0,const Eigen::Vector3d & arg_p1,
      const Eigen::Vector3d & arg_p2, const Eigen::Vector3d & arg_free_point,
      const double arg_stiffness)
  {
    bool flag = computePlaneCoefficients(data_->p0_, data_->p1_, data_->p2_, data_->a_, data_->b_, data_->c_, data_->d_);
    return flag;
  }

  void CTaskConstraintPlane::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


bool CTaskConstraintPlane::computeServo(const SRobotSensors* arg_sensors)
{
  bool flag = true;
#ifdef DEBUG
  assert(has_been_init_);
  assert(S_NULL!=dynamics_);
#endif
  if(data_->has_been_init_)
  {
    //Step 1: Find position of the op_point
    data_->x_ = data_->rbd_->T_o_lnk_ * data_->pos_in_parent_;

    //Now find the distance it has advanced into the constraint plane.
    double dist = data_->mul_dist_ * computePlanePointDistance(data_->a_, data_->b_, data_->c_, data_->d_, data_->x_);

    if(dist > 0)
    {// The constraint is inactive
      data_->force_task_ *= 0;
      data_->force_gc_ *= 0;
      data_->is_active_ = false;
      return true;
    }

    // The constraint is now active..
    data_->is_active_ = true;

    // Find the plane normal
    tmp1(0) = data_->a_;
    tmp1(1) = data_->b_;
    tmp1(2) = data_->c_;
    if(tmp1.norm() < std::numeric_limits<float>::min())
    {
      std::cout<<"\n CTaskConstraintPlane::computeServo() : ERROR. Plane equation has zero norm normal vector";
      return false;
    }
    tmp1 /= tmp1.norm();

    //Compute the compute the constraint penetration force
    data_->force_task_ = data_->mul_dist_ * data_->stiffness_ * tmp1;

    // T = J' F
    data_->force_gc_ = data_->J_.transpose() * data_->force_task_;
  }
  else
  { return false; }

  return flag;
}

/** Computes the dynamics (task model)
 * Assumes that the data_->model_.gc_model_ has been updated. */
bool CTaskConstraintPlane::computeModel(const SRobotSensors* arg_sensors)
{
#ifdef DEBUG
  assert(has_been_init_);
  assert(data_->has_been_init_);
  assert(S_NULL!=data_->rbd_);
  assert(S_NULL!=dynamics_);
#endif
  if(data_->has_been_init_)
  {
    bool flag = true;

    flag = flag && dynamics_->computeJacobian(data_->J_6_,*(data_->rbd_),
        arg_sensors->q_,data_->pos_in_parent_);

    //Use the position jacobian only. This is an op-point task.
    data_->J_ = data_->J_6_.block(0,0,3,data_->robot_->dof_);

#ifdef SCL_PRINT_INFO_MESSAGES
    std::cout<<"\n\tTo_lnk: \n"<<data_->rbd_->T_o_lnk_.matrix()
              <<"\n\tPosInPar: "<<data_->pos_in_parent_.transpose()
              <<"\n\t       X: "<<data_->x_.transpose()
              <<"\n\t   Ftask: "<<data_->force_task_.transpose()
              <<"\n\t Ftaskgc: "<<data_->force_gc_.transpose();
#endif

    // Don't need to eat up range space (this is an external force).
    data_->null_space_.setIdentity(data_->robot_->dof_,data_->robot_->dof_);

    return flag;
  }
  else
  { return false; }
}

}
