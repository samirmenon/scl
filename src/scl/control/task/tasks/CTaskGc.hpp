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
/* \file CTaskGc.hpp
 *
 *  Created on: Jul 23, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTASKGC_HPP_
#define CTASKGC_HPP_

#include <scl/control/task/CTaskBase.hpp>

#include <scl/control/task/tasks/data_structs/STaskGc.hpp>

namespace scl
{

  class CTaskGc : public scl::CTaskBase
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
    virtual bool computeServo(const SRobotSensorData* arg_sensors);

    /** Sets the null space for the next level to zero. Ie.
     * any task below this one in the hierarchy is ignored. */
    virtual bool computeModel();

    /********************************
     * CTaskGc specific functions
     *********************************/
    /** Sets the current goal position */
    inline void setGoal(const Eigen::VectorXd & arg_gc_goal)
    { data_->q_goal_ = arg_gc_goal;  }

    /** Sets the current goal velocity */
    inline void setGoalVel(const Eigen::VectorXd & arg_dgc_goal)
    { data_->dq_goal_ = arg_dgc_goal;  }

    /** Sets the current goal acceleration */
    inline void setGoalAcc(const Eigen::VectorXd & arg_ddgc_goal)
    { data_->ddq_goal_ = arg_ddgc_goal;  }

    /** Whether the task has achieved its goal position. */
    sBool achievedGoalPos();

    /********************************
     * Initialization specific functions
     *********************************/

    /** Default constructor : Does nothing   */
    CTaskGc();

    /** Default destructor : Does nothing.   */
    virtual ~CTaskGc(){}

  private:
    STaskGc* data_;
  };

}

#endif /* CTASKGC_HPP_ */
