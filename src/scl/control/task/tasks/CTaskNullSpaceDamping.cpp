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
/* \file CTaskNullSpaceDamping.cpp
 *
 *  Created on: Apr 4, 2011
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/control/task/tasks/CTaskNullSpaceDamping.hpp>

#include <iostream>
#include <stdexcept>

namespace scl
{

  CTaskNullSpaceDamping::CTaskNullSpaceDamping() : CTaskBase(), data_(S_NULL)
  { }

  //************************
  // Inherited stuff
  //************************
  bool CTaskNullSpaceDamping::init(STaskBase* arg_task_data,
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

      data_ = dynamic_cast<STaskNullSpaceDamping*>(arg_task_data);
      dynamics_ = arg_dynamics;

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCTaskNullSpaceDamping::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  STaskBase* CTaskNullSpaceDamping::getTaskData()
  { return data_; }

  void CTaskNullSpaceDamping::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


  bool CTaskNullSpaceDamping::computeServo(const SRobotSensorData* arg_sensors)
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      // T = -kv * dq (velocity damping)
      data_->force_task_ = data_->kv_.array() * arg_sensors->dq_.array();

      data_->force_task_ = data_->force_task_.array().min(data_->force_task_max_.array());//Min of self and max
      data_->force_task_ = data_->force_task_.array().max(data_->force_task_min_.array());//Max of self and min

      data_->force_gc_ = -1* data_->gc_model_->A_ * data_->force_task_;
      return true;
    }
    else
    { return false; }
  }


  /** Sets the null space for the next level to zero. Ie.
   * any task below this one in the hierarchy is ignored. */
  bool CTaskNullSpaceDamping::computeModel()
  {
    if(data_->has_been_init_)
    {
      //Leaves nothing for other tasks. Uses up the entire remaining range space.
      data_->null_space_.setZero(data_->robot_->dof_,data_->robot_->dof_);
      return true;
    }
    else
    { return false; }
  }

}
