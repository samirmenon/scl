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
/* \file STaskGc.cpp
 *
 *  Created on: Jan 1, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "STaskGc.hpp"

#include <stdexcept>
#include <iostream>

namespace scl
{

//5mm or 0.005 rad resolution
#define SCL_GCTASK_SPATIAL_RESOLUTION 0.005

  STaskGc::STaskGc() : STaskBase("STaskGc"),
    spatial_resolution_(SCL_GCTASK_SPATIAL_RESOLUTION),
    flag_compute_gravity_(true),
    flag_compute_cc_forces_(false),
    flag_compute_inertia_(true)
  { }

  STaskGc::~STaskGc()
  { }

  bool STaskGc::initTaskParams()
  {
    try
    {
      //Extra flags..
      flag_compute_gravity_ = true;
      flag_compute_cc_forces_ = false;
      flag_compute_inertia_ = true;

      /** Extract the extra params */
      std::string parent_link_name;
      Eigen::Vector3d pos_in_parent;

      std::vector<scl::sString2>::const_iterator it,ite;
      for(it = task_nonstd_params_.begin(), ite = task_nonstd_params_.end();
          it!=ite;++it)
      {
        const sString2& param = *it;
        if(param.data_[0] == std::string("flag_compute_gravity"))
        {//Flags are optional, so we don't need to check them with contains_
          std::stringstream ss(param.data_[1]);
          int tmp;
          ss>>tmp;
          if(tmp == 0)  { flag_compute_gravity_ = false; }
          else  { flag_compute_gravity_ = true; }
        }
        else if(param.data_[0] == std::string("flag_compute_cc_forces"))
        {//Flags are optional, so we don't need to check them with contains_
          std::stringstream ss(param.data_[1]);
          int tmp;
          ss>>tmp;
          if(tmp == 0)  { flag_compute_cc_forces_ = false; }
          else  { flag_compute_cc_forces_ = true; }
        }
        else if(param.data_[0] == std::string("flag_compute_inertia"))
        {//Flags are optional, so we don't need to check them with contains_
          std::stringstream ss(param.data_[1]);
          int tmp;
          ss>>tmp;
          if(tmp == 0)  { flag_compute_inertia_ = false; }
          else  { flag_compute_inertia_ = true; }
        }
      }

      // To test whether the goal position has been achieved.
      spatial_resolution_ = SCL_GCTASK_SPATIAL_RESOLUTION;

      // Initialize the gc vectors to the correct sizes
      q_.setZero(dof_task_);
      dq_.setZero(dof_task_);
      ddq_.setZero(dof_task_);

      q_goal_.setZero(dof_task_);
      dq_goal_.setZero(dof_task_);
      ddq_goal_.setZero(dof_task_);

      force_gc_.setZero(dof_task_);
      force_task_.setZero(dof_task_);
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSTaskGc::init() : "<<e.what();
      return false;
    }
    return true;
  }
}
