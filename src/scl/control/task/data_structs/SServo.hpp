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
/* \file SServo.hpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SSERVO_HPP_
#define SSERVO_HPP_

#include <vector>

#include <scl/DataTypes.hpp>

#include <scl/control/task/data_structs/STaskBase.hpp>

#include <sutil/CMappedMultiLevelList.hpp>

namespace scl
{
  /** A data structure to store the main servo loop's data.
   *  The main servo loop accesses the current torques to be
   *  applied from each task, computes the range spaces of the
   *  tasks and projects the torques through them to obtain the
   *  combined robot control force in generalized coordinates
   *  (usually joint torques).
   *
   *  Ie. Servo's job (pseudocode):
   *  force_gc_.zero();
   *  for(all-task-data){
   *    compute-range-space(task_data_);
   *    force_gc_+=task_data_->range_space * task_data_->force_gc_;
   *  }
   *
   *  See docs/ControllerDesign.eps for an overview of how
   *  the controller works */
  class SServo : public SObject
  {
  public:
    /** Servo forces to be sent to the robot directly.
     * The purpose of a controller is to compute these */
    Eigen::VectorXd force_gc_;

    /** For the task torques and range spaces */
    sutil::CMappedMultiLevelList<std::string, STaskBase*>* task_data_;

    /** The robot's branching structure */
    const SRobotParsed* robot_;

    /** The default constructor does nothing */
    SServo();

    /** Initializes the servo and sets has_been_init_ to true */
    sBool init(const SRobotParsed* arg_robot_ds,
        sutil::CMappedMultiLevelList<std::string, STaskBase*>* arg_task_ds);
  };
}

#endif /* SSERVO_HPP_ */
