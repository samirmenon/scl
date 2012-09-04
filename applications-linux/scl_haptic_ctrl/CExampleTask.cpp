/* Copyright (C) 2011  Samir Menon, Stanford University

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 * CHapticTask.cpp
 *
 *  Created on: Apr 12, 2011
 *      Author: samir
 */

#include "CHapticTask.hpp"

#include <scl/Singletons.hpp>

#include <sutil/CRegisteredDynamicTypes.hpp>

#include <stdexcept>

namespace scl_app
{
  CHapticTask::CHapticTask() :
      scl::CTaskBase(),
      data_(S_NULL),
      link_dynamic_id_(S_NULL)
  {}

  CHapticTask::~CHapticTask()
  {}

  bool CHapticTask::computeServo(const scl::SRobotSensorData* arg_sensors)
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      //TODO : COMPUTE SERVO GENERALIZED FORCES
      //data_->force_gc_ = ?
      return true;
    }
    else
    { return false; }
  }


  bool CHapticTask::computeModel()
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      //TODO : COMPUTE DYNAMIC MODEL (if not a generalized coordinate task)
      //TODO : COMPUTE NULL SPACE
      //data_->null_space_ = ?
      return true;
    }
    else
    { return false; }
  }

  scl::STaskBase* CHapticTask::getTaskData()
  { return data_; }

  bool CHapticTask::init(scl::STaskBase* arg_task_data,
      scl::CDynamicsBase* arg_dynamics)
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

      data_ = dynamic_cast<SHapticTask*>(arg_task_data);
      dynamics_ = arg_dynamics;

      link_dynamic_id_ = dynamics_->getIdForLink("end-effector");
      if(S_NULL == link_dynamic_id_)
      { throw(std::runtime_error("The link dynamic id for this operational point's parent link is NULL. Probably due to un-initialized dynamics.")); }

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCHapticTask::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  void CHapticTask::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


  /*******************************************
            Dynamic Type : CHapticTask

   NOTE : To enable dynamic typing for tasks, you
   must define the types for the "CTaskName" computation
   object AND the "STaskName" data structure. THIS IS NECESSARY.

   Why? So that you have a quick and easy way to specify
   custom xml parameters in the *Cfg.xml file.
   *******************************************/
  scl::sBool registerHapticTaskType()
  {
    bool flag;
    try
    {
      sutil::CDynamicType<std::string,scl_app::CHapticTask> typeCHapticTask(std::string("CHapticTask"));
      flag = typeCHapticTask.registerType();
      if(false == flag) {throw(std::runtime_error("Could not register type CHapticTask"));}

      sutil::CDynamicType<std::string,scl_app::SHapticTask> typeSHapticTask(std::string("SHapticTask"));
      flag = typeSHapticTask.registerType();
      if(false == flag) {throw(std::runtime_error("Could not register type SHapticTask"));}

#ifdef DEBUG
      std::cout<<"\nregisterHapticTaskType() : Registered my cool task with the database";
#endif
    }
    catch (std::exception& e)
    {
      std::cout<<"\nregisterHapticTaskType() : Error : "<<e.what();
      return false;
    }
    return true;
  }
}
