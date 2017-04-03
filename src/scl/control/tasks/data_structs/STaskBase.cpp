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
/* \file STaskBase.cpp
 *
 *  Created on: Dec 25, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "STaskBase.hpp"

#include <stdexcept>
#include <iostream>


namespace scl
{
  namespace tasks
  {
    bool STaskBase::init(
        const std::string & arg_name,
        const std::string & arg_type,
        const sUInt arg_priority,
        /** 0  task dof means a gc task. Ie. full dofs */
        const scl::sUInt arg_task_dof,
        const SRobotParsed* arg_robot_ds,
        /* The remaining variables initialize model_ and servo_ */
        const SGcModel* arg_gc_model,
        const Eigen::VectorXd & arg_kp,
        const Eigen::VectorXd & arg_kv,
        const Eigen::VectorXd & arg_ka,
        const Eigen::VectorXd & arg_ki,
        const Eigen::VectorXd & arg_ftask_max,
        const Eigen::VectorXd & arg_ftask_min,
        /** These are ignored during STaskBase initialization.
         * However, subclasses may choose to use them and/or
         * require various values */
        const std::vector<scl::sString2>& arg_nonstd_params)
    {
      bool flag;
      try
      {
        if(1>arg_name.size())
        { throw(std::runtime_error("Task name is too short")); }

        if(1>arg_type.size())
        { throw(std::runtime_error("Type name is too short")); }

        if(S_NULL==arg_robot_ds)
        { throw(std::runtime_error("Passed a NULL parent-robot data structure")); }

        if(0==arg_robot_ds->dof_)
        { throw(std::runtime_error("Passed a parent-robot data structure with 0 dof")); }

        if(1>arg_robot_ds->name_.size())
        { throw(std::runtime_error("Passed parent-robot's name is too short")); }

        if(false == arg_robot_ds->has_been_init_)
        { throw(std::runtime_error("Passed uninitialized parent-robot's data structure")); }

        if(S_NULL==arg_gc_model)
        { throw(std::runtime_error("Passed a NULL generalized coordinate model")); }

        //We have an extra layer of checks here since the gc model is already tested by the S*Controller data
        //structure. And since it is a member object of one of those, it doesn't inherit from SObject and thus
        //doesn't have its own init function. So, unfortunately, to preserve the convention, we can't simply use
        //has_been_init_
        if((sUInt)arg_gc_model->M_gc_.rows()!=arg_robot_ds->dof_)
        { throw(std::runtime_error("Generalized coordinate mass matrix rows don't match the robot's dofs")); }

        if(arg_gc_model->M_gc_.rows()!=arg_gc_model->M_gc_.cols())
        { throw(std::runtime_error("Generalized coordinate mass matrix is not square")); }

        if((sUInt)arg_gc_model->M_gc_inv_.rows()!=arg_robot_ds->dof_)
        { throw(std::runtime_error("Generalized coordinate mass matrix inverse rows don't match the robot's dofs")); }

        if(arg_gc_model->M_gc_inv_.rows()!=arg_gc_model->M_gc_inv_.cols())
        { throw(std::runtime_error("Generalized coordinate mass matrix inverse is not square")); }

        if((sUInt)arg_gc_model->force_gc_cc_.size()!=arg_robot_ds->dof_)
        { throw(std::runtime_error("Centrifugal-coriolis force vector doesn't match the robot's dofs")); }

        if((sUInt)arg_gc_model->force_gc_grav_.size()!=arg_robot_ds->dof_)
        { throw(std::runtime_error("Gravity force vector doesn't match the robot's dofs")); }


        //Test whether the parameters for the task servo loop are fine.
        if((1!=arg_kp.size()) && (arg_task_dof!=(sUInt)arg_kp.size()))
        { throw(std::runtime_error("Proportional gain vector (kp) size should be 1 or task-dof")); }

        if((1!=arg_kv.size()) && (arg_task_dof!=(sUInt)arg_kv.size()))
        { throw(std::runtime_error("Velocity gain vector (kv) size should be 1 or task-dof")); }

        if((1!=arg_ka.size()) && (arg_task_dof!=(sUInt)arg_ka.size()))
        { throw(std::runtime_error("Velocity gain vector (ka) size should be 1 or task-dof")); }

        if((1!=arg_ki.size()) && (arg_task_dof!=(sUInt)arg_ki.size()))
        { throw(std::runtime_error("Integral gain vector (ki) size should be 1 or task-dof")); }

        if((1!=arg_ftask_max.size()) && (arg_task_dof!=(sUInt)arg_ftask_max.size()))
        { throw(std::runtime_error("Maximum task-force vector size should be 1 or task-dof")); }

        if((1!=arg_ftask_min.size()) && (arg_task_dof!=(sUInt)arg_ftask_min.size()))
        { throw(std::runtime_error("Minimum task-force vector size should be 1 or task-dof")); }

        //Start initializing the task
        name_ = arg_name;
        type_task_ = arg_type;
        priority_ = arg_priority;
        if(0 == arg_task_dof)//Generalized coordinate controller
        { dof_task_ = arg_robot_ds->dof_; }
        else
        { dof_task_ = arg_task_dof; }
        robot_ = arg_robot_ds;
        gc_model_ = arg_gc_model;

        //Set up the dynamics model
        J_.setZero(dof_task_,robot_->dof_);
        J_6_.setZero(6,robot_->dof_);
        J_dyn_inv_.setZero(robot_->dof_,dof_task_);
        null_space_.setZero(robot_->dof_,robot_->dof_);
        M_task_.setZero(dof_task_,dof_task_);
        M_task_inv_.setZero(dof_task_,dof_task_);
        force_task_cc_.setZero(dof_task_,1);
        force_task_grav_.setZero(dof_task_,1);

        //Set up the servo
        force_task_.setZero(dof_task_);
        force_gc_.setZero(robot_->dof_);
        range_space_.setZero(robot_->dof_,robot_->dof_);//Can't do anything till this is determined

        if(1==arg_kp.size())
        {kp_.setConstant(dof_task_,arg_kp(0));}
        else{ kp_ = arg_kp;  }

        if(1==arg_kv.size())
        {kv_.setConstant(dof_task_,arg_kv(0));}
        else{ kv_ = arg_kv;  }

        if(1==arg_ka.size())
        {ka_.setConstant(dof_task_,arg_ka(0));}
        else{ ka_ = arg_ka;  }

        if(1==arg_ki.size())
        {ki_.setConstant(dof_task_,arg_ki(0));}
        else{ ki_ = arg_ki;  }

        if(1==arg_ftask_max.size())
        {force_task_max_.setConstant(dof_task_,arg_ftask_max(0));}
        else{ force_task_max_ = arg_ftask_max;  }

        if(1==arg_ftask_min.size())
        {force_task_min_.setConstant(dof_task_,arg_ftask_min(0));}
        else{ force_task_min_ = arg_ftask_min;  }

        for(int i=0;i<static_cast<int>(dof_task_);i++)
        {
          if(force_task_min_(i)>= force_task_max_(i))
          { throw(std::runtime_error("Specified minimum task-force exceeds specified maximum task force.")); }
        }

        //Store the nonstandard params
        task_nonstd_params_ = arg_nonstd_params;

        flag = initTaskParams();
        if(false == flag)
        { throw(std::runtime_error("Could not initialize the non standard task parameters. \nTODO : Subclass STaskBase, implement your task data structure, and make the function return true.")); }

        has_been_init_ = true;
      }
      catch(std::exception& e)
      {
        std::cerr<<"\nSTaskBase::init() : "<<e.what();

        //Even if it was initialized earlier, it might be in an invalid state now.
        //We will, however. keep the past values for error handling / debugging.
        has_been_init_ = false;
      }
      return has_been_init_;
    }
  }
}
