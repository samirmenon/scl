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
/* \file DatabaseUtils.cpp
 *
 *  Created on: Aug 6, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/util/DatabaseUtils.hpp>
#include <scl/Singletons.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <sutil/CRegisteredDynamicTypes.hpp>

#include <vector>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdexcept>

#ifdef DEBUG
//Asserts appear in debug mode when the ifdef is true
#include <cassert>
#endif

using namespace scl;

namespace scl_util
{

  void printRobotLinkTree(const SRigidBody &link, int depth)
  {
    std::cout<<"\n Link: "<<link.name_<<" Robot: <"<<link.robot_name_<<">";
    if(S_NULL==link.parent_addr_)
    { std::cout<<" Parent: <None>";  }
    else
    { std::cout<<" Parent: <"<<link.parent_addr_->name_<<">"; }
    std::cout<<", Id: "<<link.link_id_
              <<", Depth: "<<depth
              <<", Com: "<<(link.com_).transpose()
              <<", Mass: "<<link.mass_
              <<", Inertia: \n"<<link.inertia_
              <<",\nPos in par: "<<link.pos_in_parent_.transpose()
              <<",\nOri in par: \n"<<link.ori_parent_quat_.toRotationMatrix()
              <<std::flush;

    std::vector<SRigidBody*>::const_iterator clink, clinke;
    for(clink = link.child_addrs_.begin(),
        clinke = link.child_addrs_.end();
        clink != clinke; ++clink)
    { printRobotLinkTree( (**clink),depth+1); }

    if(link.child_addrs_.begin() == link.child_addrs_.end())
    { std::cout<<"\nLEAF NODE."<<std::flush; }
  }

  /** In addition to the default initMultiTaskCtrlDsFromParsedTasks(), this function
   * also parses some extra options if they exist.
   *
   * It also helps split the code for the task parsing from the param parsing (potentially
   * easier to read for some).
   *
   * Sometimes parameters may be provided as strings and might require dynamic type
   * resolution. As such, we will parse them here... */
  int initMultiTaskCtrlDsFromParsedTasks(
        const std::vector<scl::STaskBase*> &arg_taskvec,
        const std::vector<scl::SNonControlTaskBase*> &arg_taskvec_nc,
        const std::vector<scl::sString2> &arg_ctrl_params,
        scl::SControllerMultiTask& ret_ctrl)
  {
    bool flag;

    // First parse the tasks..
    // We don't throw here because the overloaded function has the same name and
    // it already prints the associated error..
    flag = initMultiTaskCtrlDsFromParsedTasks(arg_taskvec,arg_taskvec_nc,ret_ctrl);
    if(false == flag){ return false;  }

    // Now parse the text options
    try{
      std::vector<scl::sString2>::const_iterator it,ite;
      for(it = arg_ctrl_params.begin(), ite = arg_ctrl_params.end(); it!=ite; ++it)
      {
        if(it->data_[0] == "must_use_robot")
          if(it->data_[1] != ret_ctrl.robot_->name_)
          {
            throw(std::runtime_error(std::string("Params require robot: ")+it->data_[1]+
              std::string(". Controller init used: ") + ret_ctrl.robot_->name_));
          }
        if(it->data_[0] == "option_servo_to_model_rate")
        {
          std::stringstream ss;
          ss<<it->data_[1];
          ss>>ret_ctrl.servo_to_model_rate_;
        }
      }
      return true;
    }
    catch (std::exception & e)
    { std::cout<<"\ninitMultiTaskCtrlDsFromParsedTasks() : Failed "<<e.what(); }
    return false;
  }

  /** Checks whether dynamic type information is available. If so, it parses
   * tasks into the control data structure
   *
   * NOTE : This would have been part of the core data structure except for the
   * need for dynamic typing, which is useful for parsing user specified tasks. */
  int initMultiTaskCtrlDsFromParsedTasks(
      const std::vector<scl::STaskBase*> &arg_taskvec,
      const std::vector<scl::SNonControlTaskBase*> &arg_taskvec_nc,
      scl::SControllerMultiTask& ret_ctrl)
  {
    bool flag;
    scl::sUInt tasks_parsed=0, nc_tasks_parsed;
    try
    {
      if(false == ret_ctrl.hasBeenInit())
      { throw(std::runtime_error("Controller data structure not intialized"));  }

      //Now add the tasks to the controller sutil::CMappedMultiLevelList
      for(std::vector<scl::STaskBase*>::const_iterator it = arg_taskvec.begin(),
          ite = arg_taskvec.end(); it!=ite; ++it)
      {
        const scl::STaskBase& tmp_task = **it;

        //Use dynamic typing to get the correct task type.
        void *get_task_type=S_NULL;
        flag = sutil::CRegisteredDynamicTypes<std::string>::getObjectForType(std::string("S")+tmp_task.type_task_,get_task_type);
        if(false == flag)
        { throw (std::runtime_error(std::string("S")+tmp_task.type_task_+
            std::string(" -- Unrecognized task type requested.\n * Did you register the task type with the database?\n * If not, see applications-linux/scl_skeleton_code/*.cpp to find out how.")));  }

        //Convert the pointer into a STaskBase*
        scl::STaskBase* tmp_task2add = reinterpret_cast<scl::STaskBase*>(get_task_type);

        flag = tmp_task2add->init(tmp_task.name_, tmp_task.type_task_,
            tmp_task.priority_, tmp_task.dof_task_, ret_ctrl.robot_, ret_ctrl.gc_model_,
            tmp_task.kp_, tmp_task.kv_, tmp_task.ka_, tmp_task.ki_,
            tmp_task.force_task_max_, tmp_task.force_task_min_,
            tmp_task.task_nonstd_params_);
        if(false == flag)
        {
          if(S_NULL!=tmp_task2add) { delete tmp_task2add;  }
          throw (std::runtime_error(std::string("Could not initialize task : ")+ tmp_task.type_task_+ std::string(" : ") + tmp_task.name_));
        }

        //THE JUICE : Add all the tasks into the multi level mapped list!!
        if(S_NULL!=tmp_task2add)
        {
          //NOTE : We have to store a double pointer for tasks because the mapped list
          //creates its own memory.
          scl::STaskBase** tmp = ret_ctrl.tasks_.create(tmp_task2add->name_,
              tmp_task2add,tmp_task2add->priority_);

          //Detailed error reporting.
          if(S_NULL == tmp)
          {
            std::string serr = "Could not create task data structure for : ";
            if(S_NULL!=tmp_task2add)
            { serr = serr + tmp_task2add->name_; delete tmp_task2add; }
            else
            { serr = serr + "NULL task pointer (should not be NULL)"; }
            throw (std::runtime_error(serr.c_str()));
          }

          //Now set the task to point to it's parent.
          if(S_NULL != (*tmp)->parent_controller_)
          { throw(std::runtime_error(std::string("Task already has a parent controller : ") + tmp_task2add->name_));  }
          (*tmp)->setParentController(&ret_ctrl);
        }
        tasks_parsed++;
      }

      if(0==tasks_parsed)
      {  throw (std::runtime_error("Found zero tasks in a task controller."));  }

      //Now parse the non-control tasks
      //Now add the tasks to the controller sutil::CMappedList
      nc_tasks_parsed=0;
      for(std::vector<scl::SNonControlTaskBase*>::const_iterator it = arg_taskvec_nc.begin(),
          ite = arg_taskvec_nc.end(); it!=ite; ++it)
      {
        const scl::SNonControlTaskBase& tmp_task = **it;

        //Use dynamic typing to get the correct task type.
        void *get_task_type=S_NULL;
        flag = sutil::CRegisteredDynamicTypes<std::string>::getObjectForType(std::string("S")+tmp_task.type_task_,get_task_type);
        if(false == flag)
        { throw (std::runtime_error(std::string("S")+tmp_task.type_task_+
            std::string(" -- Unrecognized task type requested.\n * Did you register the task type with the database?\n * If not, see applications-linux/scl_skeleton_code/*.cpp to find out how.")));  }

        //Convert the pointer into a STaskBase*
        scl::SNonControlTaskBase* tmp_task2add = reinterpret_cast<scl::SNonControlTaskBase*>(get_task_type);

        flag = tmp_task2add->init(tmp_task.name_, tmp_task.type_task_,
            tmp_task.task_nonstd_params_);
        if(false == flag)
        {
          std::string serr;
          serr = "Could not initialize non-control task : ";
          serr = serr + tmp_task.type_task_+ std::string(" : ") + tmp_task.name_;
          if(S_NULL!=tmp_task2add)
          { delete tmp_task2add;  }
          throw (std::runtime_error(serr.c_str()));
        }


        //THE JUICE : Add all the tasks into the mapped list!!
        if(S_NULL!=tmp_task2add)
        {
          //NOTE : We have to store a double pointer for tasks because the mapped list
          //creates its own memory.
          scl::SNonControlTaskBase** tmp = ret_ctrl.tasks_non_ctrl_.create(tmp_task2add->name_, tmp_task2add);

          //Detailed error reporting.
          if(S_NULL == tmp)
          {
            std::string serr;
            serr = "Could not create task data structure for : ";
            if(S_NULL!=tmp_task2add)
            {
              serr = serr + tmp_task2add->name_;
              delete tmp_task2add;
            }
            else
            {
              serr = serr + "NULL task pointer (should not be NULL)";
            }
            throw (std::runtime_error(serr.c_str()));
          }

          //Now set the task to point to it's parent.
          if(S_NULL != (*tmp)->parent_controller_)
          { throw(std::runtime_error(std::string("Non control task already has a parent controller : ") + tmp_task2add->name_));  }
          (*tmp)->setParentController(&ret_ctrl);
        }
        nc_tasks_parsed++;
      }
#ifdef DEBUG
      if(0==nc_tasks_parsed)
      { std::cout<<"\nDatabaseUtils::initMultiTaskCtrlDsFromParsedTasks() : WARNING : Did not find any non-control tasks";}
#endif
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nDatabaseUtils::initMultiTaskCtrlDsFromParsedTasks() : "<<ee.what();
      return 0;
    }
    return tasks_parsed;
  }

  /** Initializes a dynamic tree given a static tree for a robot. */
  bool initDynRobotFromParsedRobot(sutil::CMappedTree<std::string, scl::SRigidBodyDyn>& arg_rbd_tree,
      const sutil::CMappedTree<std::string, scl::SRigidBody>& arg_rb_tree)
  {
    bool flag;
    try
    {
      const int ndof = arg_rb_tree.size()-1;
      arg_rbd_tree.clear();

      sutil::CMappedTree<std::string, SRigidBody>::const_iterator it,ite;
      for(it = arg_rb_tree.begin(), ite = arg_rb_tree.end();
          it!=ite; ++it)
      {
        const SRigidBody& rb = *it;
        SRigidBodyDyn *rbd = arg_rbd_tree.create(rb.name_,rb.is_root_);
        if(NULL == rbd)
        { throw(std::runtime_error( std::string("Could not create dyn node: ")+ rb.name_+std::string("for robot: ")+rb.robot_name_ )); }

        rbd->name_ = rb.name_;
        rbd->parent_name_ = rb.parent_name_;
        rbd->link_ds_ = &rb;

        rbd->J_com_.setZero(6, ndof);

        if(rb.is_root_)
        {//The root node doesn't move, so we can already compute the translations.
          rbd->T_o_lnk_.setIdentity();
          rbd->T_o_lnk_.translate(rbd->link_ds_->pos_in_parent_);
          rbd->T_o_lnk_.rotate(rbd->link_ds_->ori_parent_quat_);
          rbd->T_lnk_ = rbd->T_o_lnk_;

          //Default is NaN, which indicates that these values weren't initialized.
          //Set to zero to indicate that they are now initialized. (actual value has no
          //meaning since the root node never moves).
          rbd->q_T_ = 0.0;
        }
        else
        {
          rbd->T_o_lnk_.setIdentity();
          rbd->T_lnk_.setIdentity();
        }
      }

      flag = arg_rbd_tree.linkNodes();
      if(false == flag)
      { throw(std::runtime_error( "Could not link the dynamic nodes into a tree" )); }
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nDatabaseUtils::initDynRobotFromParsedRobot() : "<<ee.what();
      return false;
    }
    return true;
  }

  /** Checks if a muscle system is compatible with a given robot. Looks for
     * both in the database */
    sBool isMuscleCompatWithRobot(const std::string& arg_msys,
        const std::string& arg_robot)
    {
      try
      {
        SDatabase * db = CDatabase::getData();
        if(S_NULL == db) { throw(std::runtime_error("Database not initialized"));  }

        SActuatorSetMuscleParsed *msys = db->s_parser_.muscle_sets_.at(arg_msys);
        if(S_NULL == msys) { throw(std::runtime_error("Could not find muscle system"));  }

        SRobotParsed *rob = db->s_parser_.robots_.at(arg_robot);
        if(S_NULL == rob) { throw(std::runtime_error("Could not find robot"));  }

        //NOTE TODO : Implement this.
      }
      catch(std::exception& ee)
      {
        std::cerr<<"\nDatabaseUtils::isMuscleCompatWithRobot() : "<<ee.what();
        return false;
      }
      return true;
    }
}
