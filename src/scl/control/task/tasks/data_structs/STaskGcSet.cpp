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
/* \file STaskGcSetSet.hpp
 *
 *  Created on: Aug 29, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "STaskGcSet.hpp"

#include <stdexcept>
#include <iostream>

namespace scl
{

//5mm or 0.005 rad resolution
#define SCL_GCTASKSET_SPATIAL_RESOLUTION 0.005

  STaskGcSet::STaskGcSet()
  { }

  STaskGcSet::~STaskGcSet()
  { }

  bool STaskGcSet::initTaskParams()
  {
    try
    {
      /** Resize the vectors */
      q_sel_.setZero(dof_task_);
      q_goal_.setZero(dof_task_);
      dq_goal_.setZero(dof_task_);
      ddq_goal_.setZero(dof_task_);

      spatial_resolution_ = SCL_GCTASKSET_SPATIAL_RESOLUTION;

      /** Extract the extra params */
      bool contains_gc_names = false, contains_gc_setpts = false;

      std::vector<scl::sString2>::const_iterator it,ite;
      for(it = task_nonstd_params_.begin(), ite = task_nonstd_params_.end();
          it!=ite;++it)
      {
        const sString2& param = *it;
        if(param.data_[0] == std::string("gc_set_names"))
        {
          std::stringstream ss(param.data_[1]);
          for(sUInt i=0; i<dof_task_; ++i)
          {
            std::string s;
            ss>>s;
            q_sel_names_.push_back(s);

            SRigidBody* tlnk = robot_->robot_br_rep_.at(s);
            if(S_NULL == tlnk)
            { throw(std::runtime_error(s + std::string(" -- Passed a link that doesn't exist.")));  }

            q_sel_[i] = tlnk->link_id_;
          }

          contains_gc_names = true;
        }
        else if(param.data_[0] == std::string("gc_set_pos_des"))
        {
          std::stringstream ss(param.data_[1]);
          for(sUInt i=0; i<dof_task_; ++i)
          { ss>> q_goal_[i];  }

          contains_gc_setpts = true;
        }
      }

      //Error checks
      if(false == contains_gc_names)
      { throw(std::runtime_error("Task's nonstandard params do not contain gen coord set's names."));  }

      if(false == contains_gc_setpts)
      { throw(std::runtime_error("Task's nonstandard params do not contain gen coord set's desired positions."));  }
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSTaskGcSet::init() : "<<e.what();
      return false;
    }
    return true;
  }
} /* namespace scl */
