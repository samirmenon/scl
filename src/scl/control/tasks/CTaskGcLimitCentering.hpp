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
/* \file CTaskGcLimitCentering.hpp
 *
 *  Created on: Mar 15, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SRC_SCL_CONTROL_TASKS_CTASKGCLIMITCENTERING_HPP_
#define SRC_SCL_CONTROL_TASKS_CTASKGCLIMITCENTERING_HPP_

#include <scl/control/task/CTaskBase.hpp>

#include <scl/control/task/tasks/data_structs/STaskGcLimitCentering.hpp>

namespace scl
{

  class CTaskGcLimitCentering : public scl::CTaskBase
  {
  public:
    /********************************
     * CTaskBase API
     *********************************/
    /** Initializes the task object. Required to set output
     * gc force dofs */
    virtual bool init(STaskBase* arg_task_data,
        CDynamicsBase* arg_dynamics);

    /** Return this task controller's task data structure.*/
    virtual STaskBase* getTaskData();

    /** Resets the task by removing its data.
     * NOTE : Does not deallocate its data structure*/
    virtual void reset();

    /** Computes the gc forces that resist gc velocity */
    virtual bool computeServo(const SRobotSensors* arg_sensors);

    /** Sets the null space for the next level to zero. Ie.
     * any task below this one in the hierarchy is ignored. */
    virtual bool computeModel(const SRobotSensors* arg_sensors);

    /********************************
     * CTaskGcLimitCentering specific functions
     *********************************/
    /** Sets the current goal position */
    virtual bool setGoalPos(const Eigen::VectorXd & arg_goal);

    /** Gets the current goal position. Returns false if not supported by task. */
    virtual bool getGoalPos(Eigen::VectorXd & arg_goal) const
    { arg_goal = data_->q_goal_; return true; }

    /** Gets the current position. Returns false if not supported by task. */
    virtual bool getPos(Eigen::VectorXd & arg_pos) const
    { arg_pos = data_->q_; return true; }

    /** Whether the task has achieved its goal position. */
    sBool achievedGoalPos();

    /********************************
     * Initialization specific functions
     *********************************/

    /** Default constructor : Does nothing   */
    CTaskGcLimitCentering();

    /** Default destructor : Does nothing.   */
    virtual ~CTaskGcLimitCentering(){}

  private:
    STaskGcLimitCentering* data_;
  };

}

#endif /* SRC_SCL_CONTROL_TASKS_CTASKGCLIMITCENTERING_HPP_ */
