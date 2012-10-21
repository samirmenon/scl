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
/* \file SOpRotationQuatTask.cpp
 *
 *  Created on: Sep 14, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/control/task/tasks/data_structs/SOpRotationQuatTask.hpp>

#include <stdexcept>
#include <iostream>

namespace scl
{

#define SCL_OPROTATION_TASK_SPATIAL_RESOLUTION 0.01
#define SCL_OPROTATION_TASK_DOF 3

  SOpRotationQuatTask::SOpRotationQuatTask() : STaskBase(),
      ori_quat_(Eigen::Quaterniond::Identity()),
      ori_eulerang_goal_(Eigen::Vector3d::Zero()),
      ori_quat_goal_(Eigen::Quaterniond::Identity()),
      pos_in_parent_(Eigen::Vector3d::Zero()),
      link_name_(""),
      link_ds_(S_NULL),
      spatial_resolution_(SCL_OPROTATION_TASK_SPATIAL_RESOLUTION),
      link_dynamic_id_(S_NULL)
  { }

  SOpRotationQuatTask::~SOpRotationQuatTask()
  { }

  /** 1. Initializes the task specific data members.
   *
   * 2. Parses non standard task parameters,
   * which are stored in STaskBase::task_nonstd_params_.
   * Namely:
   *  (a) parent link name
   *  (b) pos in parent.*/
  bool SOpRotationQuatTask::initTaskParams()
  {
    try
    {
      /** Extract the extra params from the command line args */
      std::string parent_link_name;
      Eigen::Vector3d pos_in_parent;

      bool contains_plink = false, contains_posinp = false;

      std::vector<scl::sString2>::const_iterator it,ite;
      for(it = task_nonstd_params_.begin(), ite = task_nonstd_params_.end();
          it!=ite;++it)
      {
        const sString2& param = *it;
        if(param.data_[0] == std::string("parent_link"))
        {
          parent_link_name = param.data_[1];
          contains_plink = true;
        }
        else if(param.data_[0] == std::string("pos_in_parent"))
        {
          std::stringstream ss(param.data_[1]);
          ss>> pos_in_parent[0];
          ss>> pos_in_parent[1];
          ss>> pos_in_parent[2];

          contains_posinp = true;
        }
      }

      //Error checks : The config file task specific args should contain a parent name and pos in parent
      if(false == contains_plink)
      { throw(std::runtime_error("Task's nonstandard params do not contain a parent link name."));  }

      if(false == contains_posinp)
      { throw(std::runtime_error("Task's nonstandard params do not contain a pos in parent."));  }

      if(0>=parent_link_name.size())
      { throw(std::runtime_error("Parent link's name is too short."));  }

      link_name_ = parent_link_name;

      link_ds_ = robot_->robot_br_rep_.at_const(link_name_);
      if(S_NULL == link_ds_)
      { throw(std::runtime_error("Could not find the parent link in the parsed robot data structure"));  }

      //Initalize the task data structure.
      pos_in_parent_ = pos_in_parent;

      //Set task space quaternions to identity and vector sizes to zero
      ori_quat_ = Eigen::Quaterniond::Identity();
      ori_eulerang_goal_ = Eigen::Vector3d::Zero();
      ori_quat_goal_ = Eigen::Quaterniond::Identity();
      J_.setZero(3,robot_->dof_);
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSOpRotationQuatTask::init() : "<<e.what();
      return false;
    }
    return true;
  }
}
