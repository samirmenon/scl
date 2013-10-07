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
/* \file CTaskController.cpp
 *
 *  Created on: Jul 21, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CTaskController.hpp"

#include <sutil/CRegisteredDynamicTypes.hpp>

#include <iostream>
#include <stdexcept>
#include <sstream>

namespace scl
{
  /**********************************************
   *               INITIALIZATION
   * ********************************************/
  CTaskController::CTaskController() : CControllerBase()
  {
    active_task_ = S_NULL;
    data_= S_NULL;
    dynamics_ = S_NULL;
    task_count_ = 0;
    task_non_ctrl_count_ = 0;
  }

  sBool CTaskController::init(SControllerBase* arg_data,
      scl::CDynamicsBase* arg_dynamics)
  {
    try
    {
      //Reset the computational object (remove all the associated data).
      reset();

      // Set up the task controller data stucture
      if(S_NULL==arg_data)
      { throw(std::runtime_error("NULL data structure passed."));  }
      if(false==arg_data->has_been_init_)
      { throw(std::runtime_error("Uninitialized data structure passed."));  }
      //This ensures that the passed data was properly initialized for task controllers.
      data_ = dynamic_cast<STaskController*>(arg_data);

      // Set up the dynamics
      if(NULL==arg_dynamics)
      { throw(std::runtime_error("NULL dynamics object passed."));  }
      if(false==arg_dynamics->hasBeenInit())
      { throw(std::runtime_error("Uninitialized dynamics object passed."));  }
      dynamics_ = arg_dynamics;

      // Set up the center of mass properties of the robot
      data_->gc_model_.mass_ = 0.0;
      sutil::CMappedTree<std::string, SRigidBodyDyn>::iterator itcom,itcome;
      sutil::CMappedTree<std::string, SRigidBody>::const_iterator itr,itre;
      //Set the center of mass position for each link.
      for(itcom = data_->gc_model_.link_ds_.begin(), itcome =data_->gc_model_.link_ds_.end(),
          itr = data_->robot_->robot_br_rep_.begin(),itre = data_->robot_->robot_br_rep_.end();
          itcom!=itcome; ++itcom,++itr)
      {
        //Note : The root node doesn't move, has infinite mass, and doesn't
        //       have a com jacobian. So skip it.
        while(itr->is_root_) { ++itr; }

        if(itr == itre)
        {// gc and dynamics should have same dof.
          std::stringstream ss;
          ss<<"Inconsistent model. Gc model has more entries ["
              <<data_->gc_model_.link_ds_.size()<<"] than the robot's mapped tree ["
              <<data_->robot_->robot_br_rep_.size()<<"]";
          throw(std::runtime_error(ss.str()));
        }

        itcom->name_ = itr->name_;
        itcom->link_dynamic_id_ = dynamics_->getIdForLink(itcom->name_);
        itcom->link_ds_ = static_cast<const SRigidBody*>( &(*itr) );

        data_->gc_model_.mass_ += itcom->link_ds_->mass_;
      }
      if(itr != itre)
      {// Error check in case the root node is at the end.
        while(itr->is_root_) { ++itr; if(itr == itre) break; }
        if(itr != itre)
        {
          std::stringstream ss;
          ss<<"Inconsistent model. Gc model has less entries ["
              <<data_->gc_model_.link_ds_.size()<<"] than the robot's mapped tree ["
              <<data_->robot_->robot_br_rep_.size()-1<<"]"; //-1 in br_rep_.size to remove root.
          throw(std::runtime_error(ss.str()));
        }
      }

      has_been_init_ = true;

      //Point the servo computational object to the data struct
      //This also initializes the servo data
      sBool flag;
      flag = servo_.init(data_->robot_name_, &(data_->servo_) );
      if(false == flag)
      { throw(std::runtime_error("Couldn't initialize the servo object.")); }

      //Add CTask objects for all the initialized tasks
      //Read Luis Sentis' thesis for a theoretical description of how multiple task levels work.
      std::vector<std::vector<STaskBase**> >::iterator it,ite;
      for(it = data_->tasks_.mlvec_.begin(), ite = data_->tasks_.mlvec_.end();
          it!=ite; ++it)
      {//--------------------For each level--------------------
        std::vector<STaskBase**>::iterator it2, ite2;
        for(it2 = (*it).begin(), ite2 = (*it).end();
            it2!=ite2; ++it2)
        {//----------For every task in the level----------
          CTaskBase* tmp_task;
          STaskBase* tmp_task_ds = *(*it2);
          if(S_NULL == tmp_task_ds)
          { throw(std::runtime_error("Passed SControllerBase object has a NULL task pointer."));  }

          std::string tmp_type = "C" + tmp_task_ds->type_task_; //Computational object of this type.
          int tmp_task_pri = tmp_task_ds->priority_;
              //data_->tasks_.getPriorityLevel(&tmp_task_ds);

          void* obj = S_NULL;
          flag = sutil::CRegisteredDynamicTypes<std::string>::getObjectForType(tmp_type,obj);
          if(false == flag)
          {
            std::stringstream ss;
            ss<<"Dynamic controller type not initialized for task " <<tmp_task_ds->name_<<" of type "
                << tmp_type <<" at level "<<tmp_task_pri;
            std::string err;
            err = ss.str();
            throw(std::runtime_error(err.c_str()));
          }
          tmp_task = reinterpret_cast<CTaskBase*>(obj);

          //Now initialize the task
          flag = tmp_task->init(tmp_task_ds, arg_dynamics);
          if(false == flag)
          {
            std::stringstream ss;
            ss<<"Could not initialize task " <<tmp_task_ds->name_<<" of type "
                << tmp_type <<" at level "<<tmp_task_pri;
            std::string err;
            err = ss.str();
            throw(std::runtime_error(err.c_str()));
          }

          flag = addTask(tmp_task_ds->name_,tmp_task,tmp_task_pri);
          if(false == flag)
          {
            std::stringstream ss;
            ss<<"Could not add task " <<tmp_task_ds->name_<<" of type "
                << tmp_type <<" at level "<<tmp_task_pri;
            std::string err;
            err = ss.str();
            throw(std::runtime_error(err.c_str()));
          }
        }
      }

      has_been_init_ = true; //Can't really do anything with the data structure yet.

    }
    catch(std::exception& e)
    {
      has_been_init_ = false;
      std::cout<<"\nCTaskController::init() Error : "<<e.what();
    }
    return has_been_init_;
  }

  sBool CTaskController::reset()
  {
    //Remove all data references from this task controller.
    //It will require re-initialization after this.

    data_ = S_NULL;
    dynamics_ = S_NULL;

    servo_.reset();

    /**
     * NOTE : This is a rather complicated thing to do.
     *
     * Resetting the tasks can be done in two ways:
     * 1. Simply forget everything about every task. Then there
     * isn't enough information to reconstruct the tasks after that
     * since the CTaskBase* pointers point to some subclass which is lost.
     *
     * 2. Individually reset each task. In this case, the reset tasks
     * need to be seeded with their data structures again which is
     * just as hard to do as (1).
     *
     * For now we'll just do (1)
     */
    tasks_.clear();
    tasks_non_ctrl_.clear();
    task_count_ = 0;

    active_task_ = S_NULL;

    return true;
  }

  sBool CTaskController::addTask(const std::string &arg_task_name,
      CTaskBase* arg_task, const sUInt arg_level)
  {
    sBool flag;
    try
    {
      if(NULL==data_)
      { throw(std::runtime_error("CTaskController not initialized. Can't add task."));  }

      if(NULL==arg_task)
      { throw(std::runtime_error("Passed  a NULL task pointer. Can't do anything with it."));  }

      //Initialize the task's data structure.
      flag = arg_task->hasBeenInit();
      if(false == flag) { throw(std::runtime_error("Passed an un-initialized task."));  }

      scl::CTaskBase** ret = tasks_.create(arg_task_name, arg_task, arg_level);
      if(NULL == ret) { throw(std::runtime_error("Could not create a task computational object."));  }

      //Works best for only one task
      if(0 == task_count_)
      { active_task_ = arg_task;  }

      task_count_++;

      return true;
    }
    catch(std::exception& e)
    { std::cout<<"\nCTaskController::addTask() : Failed. "<<e.what(); }
    return false;
  }

  sBool CTaskController::removeTask(const std::string &arg_task_name)
  {
    bool flag;
    try
    {
      CTaskBase** tmp = tasks_.at(arg_task_name);
      if(S_NULL == tmp)
      { throw(std::runtime_error("Could not find task to delete."));  }

      flag = deactivateTask(arg_task_name);
      if(false == flag)
      { throw(std::runtime_error("Could not deactivate the task. Required before removal."));  }

      flag = tasks_.erase(arg_task_name);
      if(false == flag)
      { throw(std::runtime_error("Could not delete a task computational object."));  }

      //NOTE : Even though the computational object is deleted here, its data is still
      //alive in the database. You can always resurrect this task using the data. It is immortal! ;-)
      delete *tmp; tmp = S_NULL;

      task_count_--;
      if(1== task_count_)
      {
        active_task_ = *(tasks_.begin());
      }
    }
    catch(std::exception& e)
    {
      std::cout<<"\nCTaskController::removeTask() : Failed. "<<e.what();
      return false;
    }
    return true;
  }

  /** Returns the task by this name */
  CTaskBase * CTaskController::getTask(const std::string& arg_name)
  {
    try
    {
      CTaskBase ** ret;
      ret = tasks_.at(arg_name);
      if(S_NULL == ret)
      { throw(std::runtime_error("Task not found in the pile"));  }

      if(S_NULL == *ret)
      { throw(std::runtime_error("NULL task found in the pile"));  }

      return *ret;
    }
    catch(std::exception& e)
    { std::cout<<"\nCTaskController::getTask() : Failed. "<<e.what(); }
    return S_NULL;
  }

  sUInt CTaskController::getNumTasks(const std::string& arg_type) const
  {
    //Since the database is static but the task computational objects might be
    //removed and re-added on the fly, we count the tasks in the computational
    //object map. (Still need to get the ds, coz that's there the task type is
    //stored).
    sutil::CMappedMultiLevelList<std::string, CTaskBase*>::const_iterator it,ite;

    //Loop over all the tasks.
    sUInt ctr = 0;
    for(it = tasks_.begin(), ite = tasks_.end();
        it!=ite;++it)
    {
      STaskBase* t = (*it)->getTaskData();
      if(arg_type == t->type_task_)
      { ctr++;  }
    }

    return ctr;
  }

  /** Enables a task within the controller */
  sBool CTaskController::activateTask(const std::string& arg_task_name)
  {
    try
    {
      CTaskBase** tmp = tasks_.at(arg_task_name);
      if(S_NULL == tmp)
      { throw(std::runtime_error("Could not find task to activate."));  }

      bool flag = (*tmp)->setActivated(true);

      if(flag) { return flag; }
      else
      { throw(std::runtime_error("Could not activate task."));  }
    }
    catch(std::exception& e)
    { std::cout<<"\nCTaskController::activateTask() : Failed. "<<e.what();  }
    return false;
  }

  /** Disables a control task within the controller */
  sBool CTaskController::deactivateTask(const std::string& arg_task_name)
  {
    try
    {
      CTaskBase** tmp = tasks_.at(arg_task_name);
      if(S_NULL == tmp)
      { throw(std::runtime_error("Could not find task to activate."));  }

      bool flag = (*tmp)->setActivated(false);

      if(flag) { return flag; }
      else
      { throw(std::runtime_error("Could not deactivate task."));  }
    }
    catch(std::exception& e)
    { std::cout<<"\nCTaskController::activateTask() : Failed. "<<e.what();  }
    return false;
  }


  /**********************************************
   *               NON CONTROL TASKS
   ***********************************************/
  sBool CTaskController::addNonControlTask(const std::string &arg_task_name,
      CNonControlTaskBase* arg_task)
  {
    sBool flag;
    try
    {
      if(NULL==data_)
      { throw(std::runtime_error("CTaskController not initialized. Can't add task."));  }

      if(NULL==arg_task)
      { throw(std::runtime_error("Passed  a NULL task pointer. Can't do anything with it."));  }

      //Initialize the task's data structure.
      flag = arg_task->hasBeenInit();
      if(false == flag) { throw(std::runtime_error("Passed an un-initialized task."));  }

      //Yes, we add a pointer to the mapped list. It deletes the double pointer later.
      //Look at the destructor of CMappedList (if in doubt).
      //If you didn't understand the above comment, don't worry (or read about memory management).
      scl::CNonControlTaskBase** ret = tasks_non_ctrl_.create(arg_task_name, arg_task);
      if(NULL == ret) { throw(std::runtime_error("Could not create a task computational object."));  }

      task_non_ctrl_count_++;

      return true;
    }
    catch(std::exception& e)
    { std::cout<<"\nCTaskController::addNonControlTask() : Failed. "<<e.what(); }
    return false;
  }

  sBool CTaskController::removeNonControlTask(const std::string &arg_task_name)
  {
    bool flag;
    try
    {
      CNonControlTaskBase** tmp = tasks_non_ctrl_.at(arg_task_name);
      if(S_NULL == tmp)
      { throw(std::runtime_error("Could not find task to delete."));  }

      flag = tasks_non_ctrl_.erase(arg_task_name);
      if(false == flag)
      { throw(std::runtime_error("Could not delete a task computational object."));  }

      delete *tmp; tmp = S_NULL;

      task_non_ctrl_count_--;

      return true;
    }
    catch(std::exception& e)
    { std::cout<<"\nCTaskController::removeNonControlTask() : Failed. "<<e.what();  }
    return false;
  }

  /** Returns the task by this name */
  CNonControlTaskBase* CTaskController::getNonControlTask(const std::string& arg_name)
  {
    try
    {
      CNonControlTaskBase ** ret;
      ret = tasks_non_ctrl_.at(arg_name);
      if(S_NULL == ret)
      { throw(std::runtime_error("Task not found in the pile"));  }

      if(S_NULL == *ret)
      { throw(std::runtime_error("NULL task found in the pile"));  }

      return *ret;
    }
    catch(std::exception& e)
    { std::cout<<"\nCTaskController::getNonControlTask() : Failed. "<<e.what(); }
    return S_NULL;
  }

  /** Enables a task within the controller */
  sBool CTaskController::activateNonControlTask(const std::string& arg_task_name)
  {
    try
    {
      CNonControlTaskBase** tmp = tasks_non_ctrl_.at(arg_task_name);
      if(S_NULL == tmp)
      { throw(std::runtime_error("Could not find task to activate."));  }

      if(!(*tmp)->hasBeenInit())
      { throw(std::runtime_error("Task not intialized. Can't activate."));  }

      (*tmp)->setActivation(true);

      return true;
    }
    catch(std::exception& e)
    { std::cout<<"\nCTaskController::activateNonControlTask() : Failed. "<<e.what();  }
    return false;
  }


  /** Disables a control task within the controller */
  sBool CTaskController::deactivateNonControlTask(const std::string& arg_task_name)
  {
    try
    {
      CNonControlTaskBase** tmp = tasks_non_ctrl_.at(arg_task_name);
      if(S_NULL == tmp)
      { throw(std::runtime_error("Could not find task to deactivate."));  }

      if(!(*tmp)->hasBeenInit())
      { throw(std::runtime_error("Task not intialized. Can't deactivate."));  }

      (*tmp)->setActivation(false);

      return true;
    }
    catch(std::exception& e)
    { std::cout<<"\nCTaskController::deactivateNonControlTask() : Failed. "<<e.what();  }
    return false;
  }

  /**********************************************
   *               COMPUTATION
   ***********************************************/
  sBool CTaskController::computeControlForces()
  {
    //Compute the task torques
    sBool flag=true;
    if(1==task_count_)
    {
      flag = active_task_->computeServo(&(data_->io_data_->sensors_));
      flag = flag && servo_.computeControlForces();

      STaskBase* t = active_task_->getTaskData();
      data_->io_data_->actuators_.force_gc_commanded_ = t->force_gc_;
    }
    else
    {
      sutil::CMappedMultiLevelList<std::basic_string<char>, scl::CTaskBase*>::iterator it, ite;
      for(it = tasks_.begin(), ite = tasks_.end(); it!=ite; ++it)
      {
        CTaskBase* task = *it;
        //Check if the task has been activated
#ifdef DEBUG
        assert(task->getTaskData()->has_been_init_); //Must have been initialized by now
#endif
        if(task->hasBeenActivated())
        { flag = flag && task->computeServo(&(data_->io_data_->sensors_));  }
      }

      //Compute the command torques by filtering the
      //various tasks through their range spaces.
      flag = flag && servo_.computeControlForces();

      data_->io_data_->actuators_.force_gc_commanded_ = data_->servo_.force_gc_;
    }

    return flag;
  }

  sBool CTaskController::computeDynamics()
  {
    sBool flag=true;

    //Update the joint space dynamic matrices
    flag = dynamics_->updateModelMatrices(&(data_->io_data_->sensors_), &(data_->gc_model_));

    // Compute the task space dynamics
    if(0==task_count_)
    { return false; }
    if(1==task_count_)
    { flag = flag && active_task_->computeModel();  }
    else
    {
      sutil::CMappedMultiLevelList<std::basic_string<char>, scl::CTaskBase*>::iterator it, ite;
      for(it = tasks_.begin(), ite = tasks_.end(); it!=ite; ++it)
      {
        CTaskBase* task = *it;
        //Check if the task has been activated
#ifdef DEBUG
        assert(task->getTaskData()->has_been_init_); //Must have been initialized by now
#endif
        if(task->hasBeenActivated())
        { flag = flag && task->computeModel();  }
      }
    }

    //Compute the range spaces for all the tasks.
    flag = flag && computeRangeSpaces();

    return flag;
  }

  /** Computes the non-control tasks : I/O etc..     */
  sBool CTaskController::computeNonControlOperations()
  {
    sBool flag=true;

    // Compute the non control tasks. If none exist, just return true.
    if(0==task_non_ctrl_count_)
    { return true; }
    else
    {
      sutil::CMappedList<std::string, CNonControlTaskBase*>::iterator it, ite;
      for(it = tasks_non_ctrl_.begin(), ite = tasks_non_ctrl_.end(); it!=ite; ++it)
      {
        CNonControlTaskBase* task = *it;
        //Check if the task has been activated
#ifdef DEBUG
        assert(task->hasBeenInit()); //Must have been initialized by now
#endif
        if(task->hasBeenActivated())
        { flag = flag && task->computeTask();  }
      }
    }

    return flag;
  }

  sBool CTaskController::computeRangeSpaces()
  {
    sUInt dof = data_->io_data_->dof_;
    if(1==task_count_)
    {
      STaskBase* tmp = active_task_->getTaskData();
      tmp->range_space_.setIdentity(dof, dof);
      return true;
    }
    else
    {
      Eigen::MatrixXd null_space;//The null space within which each successive level operates
      null_space.setIdentity(dof, dof);//Initially no part of the gen-coords is used up

      sUInt levels = tasks_.getNumPriorityLevels();
      for(sUInt i=0;i<levels;++i)
      {//We directly write into the task data structures.
        std::vector<STaskBase**>* taskvec = data_->tasks_.getSinglePriorityLevel(i);
        std::vector<STaskBase**>::iterator it,ite;

        //This level will use us some of the gen-coord space. So
        //the next level will operate within this level's null space
        Eigen::MatrixXd lvl_null_space;
        lvl_null_space.setIdentity(dof, dof);

        for(it = taskvec->begin(),ite = taskvec->end();it!=ite;++it)
        {
          STaskBase *task_ds = *(*it);
#ifdef DEBUG
          assert(task_ds->has_been_init_); //Must have been initialized by now
#endif
          if(task_ds->has_been_activated_)
          {
            task_ds->range_space_ = null_space;//Set this task's range space to the higher level's null_space
            lvl_null_space *= task_ds->null_space_;//Reduce this level's null space
          }
        }
        //The next level's range space is filtered through this level's null space
        null_space *= lvl_null_space;
      }

      return true;
    }
    return false;
  }
}
