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
/* \file STaskOpPosPIDA1OrderInfTime.cpp
 *
 *  Created on: Jan 1, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "STaskOpPosPIDA1OrderInfTime.hpp"

#include <stdexcept>
#include <iostream>

namespace scl
{

  //0.5cm spatial resolution
#define SCL_COPPTTASK_SPATIAL_RESOLUTION 0.005
#define SCL_COPPTTASK_TASK_DOF 3

  STaskOpPosPIDA1OrderInfTime::STaskOpPosPIDA1OrderInfTime() : STaskBase(),
      link_name_(""),
      link_ds_(S_NULL),
      spatial_resolution_(SCL_COPPTTASK_SPATIAL_RESOLUTION),
      link_dynamic_id_(S_NULL),
      integral_gain_time_pre_(-1),integral_gain_time_curr_(-1),
      integral_gain_time_constt_(-1), integral_gain_time_max_(-1)
  { }

  STaskOpPosPIDA1OrderInfTime::~STaskOpPosPIDA1OrderInfTime()
  { }


  bool STaskOpPosPIDA1OrderInfTime::initTaskParams()
  {
    try
    {
      if(3!=dof_task_)//This is a position based op point task
      { throw(std::runtime_error("Operational point tasks MUST have 3 dofs (xyz translation at a point)."));  }

      /** Extract the extra params. Check that we have all of them. */
      bool contains_plink = false, contains_posinp = false,
          contains_int_tau = false, contains_int_tmax = false;

      std::vector<scl::sString2>::const_iterator it,ite;
      for(it = task_nonstd_params_.begin(), ite = task_nonstd_params_.end();
          it!=ite;++it)
      {
        const sString2& param = *it;
        if(param.data_[0] == std::string("parent_link"))
        {
          link_name_ = param.data_[1];

          contains_plink = true;
        }
        else if(param.data_[0] == std::string("pos_in_parent"))
        {
          std::stringstream ss(param.data_[1]);
          ss>> pos_in_parent_(0);
          ss>> pos_in_parent_(1);
          ss>> pos_in_parent_(2);

          contains_posinp = true;
        }
        else if(param.data_[0] == std::string("integral_time_constt"))
        {
          std::stringstream ss(param.data_[1]);
          ss>> integral_gain_time_constt_;

          contains_int_tau = true;
        }
        else if(param.data_[0] == std::string("integral_time_max"))
        {
          std::stringstream ss(param.data_[1]);
          ss>> integral_gain_time_max_;

          contains_int_tmax = true;
        }
      }

      //Error checks
      if(false == contains_plink)
      { throw(std::runtime_error("Task's nonstandard params do not contain a parent link name."));  }

      if(false == contains_posinp)
      { throw(std::runtime_error("Task's nonstandard params do not contain a pos in parent."));  }

      if(false == contains_int_tau)
      { throw(std::runtime_error("Task's nonstandard params do not contain integral_time_constt."));  }

      if(false == contains_int_tmax)
      { throw(std::runtime_error("Task's nonstandard params do not contain integral_time_max."));  }

      if(0>=link_name_.size())
      { throw(std::runtime_error("Parent link's name is too short."));  }

      link_ds_ = dynamic_cast<const SRigidBody *>(robot_->robot_tree_.at_const(link_name_));
      if(S_NULL == link_ds_)
      { throw(std::runtime_error("Could not find the parent link in the parsed robot data structure"));  }

      // Initialize the integral gains
      integral_gain_time_pre_ = -1;
      integral_gain_time_curr_ = -1;
      integral_force_.setZero(dof_task_);

      //Set task space vector sizes stuff to zero
      x_.setZero(dof_task_);
      dx_.setZero(dof_task_);
      ddx_.setZero(dof_task_);

      x_goal_.setZero(dof_task_);
      dx_goal_.setZero(dof_task_);
      ddx_goal_.setZero(dof_task_);

      force_task_.setZero(dof_task_);
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSTaskOpPosPID::init("<<link_name_<<") : "<<e.what();
      return false;
    }
    return true;
  }
}
