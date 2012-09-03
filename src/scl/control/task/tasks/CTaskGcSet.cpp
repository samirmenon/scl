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
/* \file CTaskGcSet.cpp
 *
 *  Created on: Aug 29, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CTaskGcSet.hpp"

#include <iostream>
#include <stdexcept>

namespace scl
{

  CTaskGcSet::CTaskGcSet() : CTaskBase(), data_(S_NULL)
  { }

  //************************
  // Inherited stuff
  //************************
  bool CTaskGcSet::init(STaskBase* arg_task_data,
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

      data_ = dynamic_cast<STaskGcSet*>(arg_task_data);
      dynamics_ = arg_dynamics;

      //Uses up the range space for its generalized coordinates. Ie. NS = 0 for them
      data_->jacobian_.setZero(data_->dof_task_,data_->robot_->dof_);
      for(sUInt i=0; i<data_->dof_task_; ++i)
      { data_->jacobian_(i,data_->q_sel_[i]) = 1.0;  }

      data_->null_space_.setIdentity(data_->robot_->dof_,data_->robot_->dof_);
      for(sUInt i=0; i<data_->dof_task_; ++i)
      { data_->null_space_(data_->q_sel_[i],data_->q_sel_[i]) = 0.0;  }

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCTaskGc::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  STaskBase* CTaskGcSet::getTaskData()
  { return data_; }

  void CTaskGcSet::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


  bool CTaskGcSet::computeServo(const SRobotSensorData* arg_sensors)
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      data_->force_gc_.array().setZero();

      for(sUInt i=0; i<data_->dof_task_; ++i)
      {
#ifdef DEBUG
        //Test if the selection indices are correctly assigned
        if(data_->robot_->dof_ < data_->q_sel_[i])
        {
          std::cout<<"\nCTaskGcSet::computeServo() : Selection index ("
              <<data_->q_sel_[i]<<")is greater than robot's dof ("
              <<data_->robot_->dof_<<")";
          return false;
        }
#endif
        //Compute the servo torques
        sFloat tmp1, tmp2;

        tmp1 = (data_->q_goal_[i] - arg_sensors->q_[data_->q_sel_[i]]);
        tmp1 =  data_->kp_[i] * tmp1;

        tmp2 = (data_->dq_goal_[i] - arg_sensors->dq_[data_->q_sel_[i]]);
        tmp2 = data_->kv_[i] * tmp2;

        //Obtain force to be applied to a unit mass floating about
        //in space (ie. A dynamically decoupled mass).
        data_->force_task_[i] = data_->ddq_goal_[i] + tmp2 + tmp1;

        //Min of self and max
        if(data_->force_task_[i] > data_->force_task_max_[i])
        {data_->force_task_[i] = data_->force_task_max_[i]; }

        //Max of self and min
        if(data_->force_task_[i] < data_->force_task_min_[i])
        {data_->force_task_[i] = data_->force_task_min_[i]; }

        data_->force_gc_[data_->q_sel_[i]] =  data_->force_task_[i] + data_->gc_model_->g_[data_->q_sel_[i]];
      }

      return true;
    }
    else
    { return false; }
  }


  /** Sets the null space for the next level. Ie.
   * any task below this one in the hierarchy can't control
   * the joints that this task controls. */
  bool CTaskGcSet::computeModel()
  {
    if(data_->has_been_init_)
    { return true;  }
    else
    { return false; }
  }

}
