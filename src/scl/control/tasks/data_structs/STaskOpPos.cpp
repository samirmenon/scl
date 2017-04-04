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
/* \file STaskOpPos.cpp
 *
 *  Created on: Jan 1, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "STaskOpPos.hpp"

#include <scl/util/EigenExtensions.hpp>

#include <stdexcept>
#include <iostream>

namespace scl
{
  namespace tasks
  {
    bool STaskOpPos::initTaskSubclass(
        const sutil::CMappedList<std::string, std::string>& arg_params)
    {
      try
      {
        if(3!=dof_task_)//This is a position based op point task
        { throw(std::runtime_error("Operational point tasks MUST have 3 dofs (xyz translation at a point)."));  }

        // Set defaults
        flag_compute_op_gravity_ = flag_defaults_[0];
        flag_compute_op_cc_forces_ = flag_defaults_[1];
        flag_compute_op_inertia_ = flag_defaults_[2];

        /** Extract the extra params */
        const std::string *tmp_p;
        tmp_p = arg_params.at_const("parent_link"); if(NULL == tmp_p) { throw(std::runtime_error("Could not find field in string map: parent_link")); }
        link_name_ = *tmp_p;
        if(0>=link_name_.size())
        { throw(std::runtime_error("Parent link's name is too short."));  }

        tmp_p = arg_params.at_const("pos_in_parent"); if(NULL == tmp_p) { throw(std::runtime_error("Could not find field in string map: pos_in_parent")); }
        if(false == scl_util::eigenVectorFromString(pos_in_parent_, *tmp_p, 3))
        { throw(std::runtime_error("Ill formatted key value: pos_in_parent")); }

        // Parse optional params : Issues warning if they don't exist...
        {
          tmp_p = arg_params.at_const("flag_compute_op_gravity");
          if(NULL == tmp_p) { std::cout<<"\n STaskOpPos::init ["<<name_<<"] : WARNING : Using default for: flag_compute_op_gravity"; goto LABEL_NEXT; }

          std::stringstream ss(*tmp_p);
          int tmp; ss>>tmp;
          tmp == 0 ? flag_compute_op_gravity_ = false : flag_compute_op_gravity_ = true;
        }
        LABEL_NEXT:

        {
          tmp_p = arg_params.at_const("flag_compute_op_cc_forces");
          if(NULL == tmp_p) { std::cout<<"\n STaskOpPos::init ["<<name_<<"] : WARNING : Using default for: flag_compute_op_cc_forces"; goto LABEL_NEXT2; }

          std::stringstream ss(*tmp_p);
          int tmp; ss>>tmp;
          tmp == 0 ? flag_compute_op_cc_forces_ = false : flag_compute_op_cc_forces_ = true;
        }
        LABEL_NEXT2:

        {
          tmp_p = arg_params.at_const("flag_compute_op_inertia");
          if(NULL == tmp_p) { std::cout<<"\n STaskOpPos::init ["<<name_<<"] : WARNING : Using default for: flag_compute_op_inertia"; goto LABEL_NEXT3; }

          std::stringstream ss(*tmp_p);
          int tmp; ss>>tmp;
          tmp == 0 ? flag_compute_op_inertia_ = false : flag_compute_op_inertia_ = true;
        }
        LABEL_NEXT3:

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
        std::cerr<<"\nSTaskOpPos::init() : "<<e.what();
        return false;
      }
      return true;
    }
  }
}

