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
/*
 * \file SGcController.cpp
 *
 *  Created on: Dec 29, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "SGcController.hpp"

#include <vector>
#include <list>
#include <string>
#include <stdexcept>
#include <iostream>


namespace scl
{

  SGcController::SGcController() : SControllerBase(),
      integral_gain_time_pre_(-1), integral_gain_time_curr_(-1),
      integral_gain_time_constt_(-1), integral_gain_time_max_(-1)
  {}

  bool SGcController::init(const std::string & arg_controller_name,
      SRobotParsedData* arg_robot_ds,
      SRobotIOData* arg_robot_io_ds,
      /* The remaining variables initialize the gc controller */
      const Eigen::VectorXd & arg_kp,
      const Eigen::VectorXd & arg_kv,
      const Eigen::VectorXd & arg_ka,
      const Eigen::VectorXd & arg_ki,
      const Eigen::VectorXd & arg_fgc_max,
      const Eigen::VectorXd & arg_fgc_min,
      const sFloat arg_integral_gain_time_constt,
      const sFloat arg_integral_gain_time_max)
  {
    bool flag;
    try
    {
      flag = SControllerBase::init(arg_controller_name,arg_robot_ds,arg_robot_io_ds);
      if(false == flag)
      { throw(std::runtime_error("Failed to initialize gc controller data structure")); }

      sUInt dof = arg_robot_ds->dof_;

      if((1!=arg_kp.size()) && (dof!=(sUInt)arg_kp.size()))
      { throw(std::runtime_error("Proportional gain vector (kp) size should be 1 or robot-dof")); }

      if((1!=arg_kv.size()) && (dof!=(sUInt)arg_kv.size()))
      { throw(std::runtime_error("Velocity gain vector (kv) size should be 1 or robot-dof")); }

      if((1!=arg_ka.size()) && (dof!=(sUInt)arg_ka.size()))
      { throw(std::runtime_error("Acceleration gain vector (ka) size should be 1 or robot-dof")); }

      if((1!=arg_ki.size()) && (dof!=(sUInt)arg_ki.size()))
      { throw(std::runtime_error("Integral gain vector (ki) size should be 1 or robot-dof")); }

      if((1!=arg_fgc_max.size()) && (dof!=(sUInt)arg_fgc_max.size()))
      { throw(std::runtime_error("Maximum gc-force vector size should be 1 or robot-dof")); }

      if((1!=arg_fgc_min.size()) && (dof!=(sUInt)arg_fgc_min.size()))
      { throw(std::runtime_error("Minimum gc-force vector size should be 1 or robot-dof")); }

      if(0>=arg_integral_gain_time_constt)
      { throw(std::runtime_error("Integral gain time constant must be greater than 0")); }

      if(0>=arg_integral_gain_time_max)
      { throw(std::runtime_error("Integral gain max interval time must be greater than 0")); }

      if(1==arg_kp.size())
      {kp_.setConstant(dof,arg_kp(0));}
      else{ kp_ = arg_kp;  }

      if(1==arg_kv.size())
      {kv_.setConstant(dof,arg_kv(0));}
      else{ kv_ = arg_kv;  }

      if(1==arg_ka.size())
      {ka_.setConstant(dof,arg_ka(0));}
      else{ ka_ = arg_ka;  }

      if(1==arg_ki.size())
      {ki_.setConstant(dof,arg_ki(0));}
      else{ ki_ = arg_ki;  }

      if(1==arg_fgc_max.size())
      {force_gc_max_.setConstant(dof,arg_fgc_max(0));}
      else{ force_gc_max_ = arg_fgc_max;  }

      if(1==arg_fgc_min.size())
      {force_gc_min_.setConstant(dof,arg_fgc_min(0));}
      else{ force_gc_min_ = arg_fgc_min;  }

      for(int i=0;i<static_cast<int>(dof);i++)
      {
        if(force_gc_min_(i)>= force_gc_max_(i))
        { throw(std::runtime_error("Specified minimum gc-force exceeds specified maximum gc-force.")); }
      }

      // Integral gain will get activated the first time
      integral_gain_time_constt_ = arg_integral_gain_time_constt;
      integral_gain_time_max_ = arg_integral_gain_time_max;

      integral_gain_time_curr_ = -1;
      integral_gain_time_pre_ = -1;

      integral_force_.setZero(dof);

      des_force_gc_.setZero(dof);
      des_q_.setZero(dof);
      des_dq_.setZero(dof);
      des_ddq_.setZero(dof);

      type_ = "SGcController";

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSGcController::init() : "<<e.what();

      //Even if it was half-initialized, it might be in an invalid state now.
      //So, we will keep the past values for error handling / debugging.
      has_been_init_ = false;
    }
    return has_been_init_;
  }
}
