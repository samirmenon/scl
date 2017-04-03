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
/* \file CTaskConstraintPlane.hpp
 *
 *  Created on: Nov 02, 2015
 *
 *  Copyright (C) 2015
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTASKCONSTRAINTPLANE_HPP_
#define CTASKCONSTRAINTPLANE_HPP_

#include <scl/DataTypes.hpp>

#include "data_structs/STaskConstraintPlane.hpp"

#include <scl/control/task/CTaskBase.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <string>
#include <vector>

namespace scl
{
  /**
   * Computes the operational space  forces for a single
   * 3-d (x,y,z) goal point planar high stiffness penetration
   * constraint
   *
   * Simulates contact with a (stiff) plane
   */
class CTaskConstraintPlane : public scl::CTaskBase
{
public:
  /********************************
   * CTaskBase API
   *********************************/
  /** Computes the task torques */
  virtual bool computeServo(const SRobotSensors* arg_sensors);

  /** Computes the dynamics (task model)
   * Assumes that the data_->model_.gc_model_ has been updated. */
  virtual bool computeModel(const SRobotSensors* arg_sensors);

  /* **************************************************************
   *                   Status Get/Set Functions
   * ************************************************************** */
  /** Return this task controller's task data structure.*/
  virtual STaskBase* getTaskData();

  /** Sets the current goal plane. If the task position is on the "arg_free_point" side of the plane
   * the task will apply zero force. Else it will apply a large force.
   *
   * The plane is defined by three points and a stiffness*/
  virtual bool setGoalPlane(const Eigen::Vector3d & arg_p0,const Eigen::Vector3d & arg_p1,
      const Eigen::Vector3d & arg_p2, const Eigen::Vector3d & arg_free_point,
      const double arg_stiffness);

  /** Gets the data struct. */
  const STaskConstraintPlane& getData() const
  { return *data_; }

  /* *******************************
   * CTaskConstraintPlane specific functions
   ******************************** */
  /** Whether the constraint is active (is applying a force). */
  sBool isActive() {return data_->is_active_;}

  /* *******************************
   * Initialization specific functions
   ******************************** */
  /** Default constructor : Does nothing   */
  CTaskConstraintPlane();

  /** Default destructor : Does nothing.   */
  virtual ~CTaskConstraintPlane(){}

  /** Initializes the task object. Required to set output
   * gc force dofs */
  virtual bool init(STaskBase* arg_task_data,
      CDynamicsBase* arg_dynamics);

  /** Resets the task by removing its data.
   * NOTE : Does not deallocate its data structure*/
  virtual void reset();

protected:
  /** The actual data structure for this computational object */
  STaskConstraintPlane* data_;

  /** Temporary variables */
  Eigen::Vector3d tmp1,tmp2;
};

}

#endif /* CTASKCONSTRAINTPLANE_HPP_ */
