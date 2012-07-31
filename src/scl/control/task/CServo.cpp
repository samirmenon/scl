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
/* \file CServo.cpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 *
 *  Computes the main servo loop's data.
 *  The main servo loop accesses the current torques to be
 *  applied from each task, computes the range spaces of the
 *  tasks and projects the torques through them to obtain the
 *  combined robot torque.
 */
#include "CServo.hpp"

#include <iostream>
#include <string>
#include <stdexcept>

namespace scl {

  bool CServo::init(const std::string &arg_ctrl_name, SServo* arg_data)
  {
    try
    {
      if(0>=arg_ctrl_name.size())
      { throw(std::runtime_error("Passed controller name is too short")); }

      if(S_NULL==arg_data)
      { throw(std::runtime_error("Passed a NULL servo data structure pointer")); }

      if(false == arg_data->has_been_init_)
      { throw(std::runtime_error("Passed an uninitialized servo data structure")); }

      if(S_NULL==arg_data->task_data_)
      { throw(std::runtime_error("Servo's task-data-struct pointer is NULL")); }

      data_ = arg_data;
      name_ = arg_ctrl_name;
      has_been_init_ = true;
    }
    catch(std::exception &e)
    {
      std::cout<<"\nCServo::init() : Failed. "<<e.what();
      has_been_init_ = false;
      reset();
    }
    return has_been_init_;
  }


  void CServo::reset()
  {
    data_ = S_NULL;
    has_been_init_ = false;
  }

  bool CServo::computeControlForces()
  {
    //Compute the command generalized coordinate forces
    if(data_->task_data_->size()<=0)
    { return false; }
    if(data_->task_data_->size()==1)
    {
      STaskBase* ds = *(data_->task_data_->begin());
      data_->force_gc_ = ds->force_gc_;
    }
    else
    {//Multiple tasks
      data_->force_gc_.setZero(data_->force_gc_.size());

      sutil::CMappedMultiLevelList<std::basic_string<char>, scl::STaskBase*>::iterator it, ite;
      for(it = data_->task_data_->begin(), ite = data_->task_data_->end(); it!=ite; ++it)
      {
        STaskBase* ds = *it;
        if(ds->has_been_activated_)
        { data_->force_gc_ += ds->range_space_ * ds->force_gc_; }
      }
    }
    return true;
  }
}
