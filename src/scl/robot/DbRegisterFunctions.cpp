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
/* \file SNRegisterFunctions.cpp
 *
 *  Created on: Jul 2, 2010
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

/** \file DbRegisterFunctions.cpp */

#include "DbRegisterFunctions.hpp"

#include <scl/DataTypes.hpp>

#include <scl/Singletons.hpp>

#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/data_structs/SGraphicsParsedData.hpp>
#include <scl/data_structs/SRobotIOData.hpp>

#include <scl/control/data_structs/SControllerBase.hpp>
#include <scl/control/gc/CGcController.hpp>
#include <scl/control/task/CTaskController.hpp>

#include <scl/control/task/data_structs/STaskBase.hpp>

#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>

#include <scl/control/task/tasks/CTaskOpPosNoGravity.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPosNoGravity.hpp>

#include <scl/control/task/tasks/CTaskGc.hpp>
#include <scl/control/task/tasks/data_structs/STaskGc.hpp>

#include <scl/control/task/tasks/CTaskGcSet.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcSet.hpp>

#include <scl/control/task/tasks/CTaskNullSpaceDamping.hpp>
#include <scl/control/task/tasks/data_structs/STaskNullSpaceDamping.hpp>

#include <scl/control/task/tasks/CTaskComPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskComPos.hpp>

#include <sutil/CRegisteredDynamicTypes.hpp>

#include <iostream>
#include <vector>
#include <stdexcept>
#include <map>
#include <string>
#include <utility>

 /* These are a set of functions to register various data objects
 * with the database. */

namespace scl_registry
{

  bool parseEverythingInFile(const std::string &arg_file,
      scl_parser::CParserBase *arg_parser,
      std::vector<std::string>* arg_robots_parsed,
      std::vector<std::string>* arg_graphics_parsed)
  {
    bool flag;
    try
    {
      //1. Verify that the database exists
      if(NULL == scl::CDatabase::getData())
      { throw(std::runtime_error("Database not initialized.")); }

      //2. List the robot specifications in the file and parse them into the database
      std::vector<std::string> robot_names;
      flag = arg_parser->listRobotsInFile(arg_file,robot_names);
      if(false == flag) { throw(std::runtime_error("Could not read robot names from the file"));  }

      std::vector<std::string>::iterator itr, itre;
      for(itr = robot_names.begin(), itre = robot_names.end();itr!=itre;++itr)
      {
        if(S_NULL == scl_registry::parseRobot(arg_file, *itr, arg_parser))
        { throw(std::runtime_error("Could not register robot with the database"));  }
        if(S_NULL!=arg_robots_parsed)//To be returned to caller
        { arg_robots_parsed->push_back(*itr); }
      }

      //3. List and parse graphics
      if(S_NULL!=arg_graphics_parsed)
      {
        std::vector<std::string> graphics_names;
        flag = arg_parser->listGraphicsInFile(arg_file,graphics_names);
        if(false == flag) { throw(std::runtime_error("Could not list graphics names from the file"));  }

        std::vector<std::string>::iterator itg, itge;
        for(itg = graphics_names.begin(), itge = graphics_names.end();itg!=itge;++itg)
        {
          if(S_NULL == scl_registry::parseGraphics(arg_file, *itg, arg_parser))
          { throw(std::runtime_error("Could not register graphics with the database"));  }
          if(S_NULL!=arg_graphics_parsed)//To be returned to caller
          { arg_graphics_parsed->push_back(*itg); }
        }
      }

      //4. List and parse controllers (Note this attaches controllers to the first robot that fits!!!)
      std::vector<std::pair<std::string,std::string> > ctrl_names;//<name,type>
      flag = arg_parser->listControllersInFile(arg_file,ctrl_names);
      if(false == flag) { throw(std::runtime_error("Could not list controllers in the file"));  }

      std::vector<std::pair<std::string,std::string> >::iterator itc, itce;
      for(itc = ctrl_names.begin(), itce = ctrl_names.end();itc!=itce;++itc)
      {
        int ctrl = 0;
        if((*itc).second=="gc")
        {
          for(itr = robot_names.begin(), itre = robot_names.end();itr!=itre;++itr)
          {
            if(S_NULL != scl_registry::parseGcController(arg_file, *itr, (*itc).first, arg_parser))
            { ctrl++; break;  }
          }
          if(ctrl==0)
          { throw(std::runtime_error("Could not register a gc controller with the database")); }
        }
        else if((*itc).second=="task")
        {
          for(itr = robot_names.begin(), itre = robot_names.end();itr!=itre;++itr)
          {
            if(S_NULL != scl_registry::parseTaskController(arg_file, *itr, (*itc).first, arg_parser))
            { ctrl++; break;  }
          }
          if(ctrl==0)
          { throw(std::runtime_error("Could not register a task controller with the database"));  }
        }
      }//End of for loop over controllers

      //Set the file name in the database.
      scl::CDatabase::getData()->s_parser_.file_name_ = arg_file;
    }//end of try{}
    catch (std::exception& e)
    {
      std::cerr<<"\nscl_registry::parseEverythingInFile() Failed :"<<e.what();
      return false;
    }
    return true;
  }

  const scl::SRobotParsedData* parseRobot(const std::string &arg_file,
                const std::string &arg_robot_name,
                scl_parser::CParserBase *arg_parser)
  {
    bool flag;
    scl::SRobotParsedData * rob=NULL;
    scl::SRobotIOData * rob_io=NULL;
    try
    {
      if(NULL == scl::CDatabase::getData())
      { throw(std::runtime_error("Database not initialized.")); }

      if(1 > arg_robot_name.size())
      { throw(std::runtime_error("Robot name is too short.")); }

      //Create a ds entry
      rob = scl::CDatabase::getData()->s_parser_.robots_.create(arg_robot_name);
      if(NULL==rob)
      { throw(std::runtime_error("Could not create a robot data structure on the pile..")); }

      //Fill in the ds entry from the file
      flag = arg_parser->readRobotFromFile(arg_file,arg_robot_name,*rob);
      if(false==flag)
      { throw(std::runtime_error("Could not parse the robot from the file.")); }

      if(rob->name_ != arg_robot_name)
      { throw(std::runtime_error("Parsed robot has a different name. This should never happen.")); }

      rob_io = scl::CDatabase::getData()->s_io_.io_data_.create(arg_robot_name);
      if(NULL==rob_io)
      { throw(std::runtime_error("Could not create a robot IO data structure on the pile..")); }

      flag = rob_io->init(arg_robot_name, rob->dof_);
      if(false==flag)
      { throw(std::runtime_error("Could not initialize the robot's I/O data structure.")); }

      rob_io->setJointPositions(rob->joint_default_pos_);

      rob->has_been_init_ = true;

      std::cout<<"\nscl_registry::parseRobot() : Parsed : "<<arg_robot_name;
    }
    catch (std::exception& e)
    {
      std::cerr<<"\nscl_registry::parseRobot() : "<<e.what();

      //Deallocate the ds memory for the robot
      if(NULL!=rob)
      { scl::CDatabase::getData()->s_parser_.robots_.erase(rob); }

      if(NULL!=rob_io)
      { scl::CDatabase::getData()->s_io_.io_data_.erase(rob_io); }

      return NULL;
    }
    return rob;
  }

  const scl::SGraphicsParsedData* parseGraphics(const std::string &arg_file,
      std::string & arg_graphics_name,
      scl_parser::CParserBase *arg_parser)
  {
    bool flag;
    scl::SGraphicsParsedData * tmp_gr=NULL;
    try
    {
      if(NULL == scl::CDatabase::getData())
      {throw(std::runtime_error("Database not initialized."));  }

      //Create a ds entry
      tmp_gr = scl::CDatabase::getData()->s_parser_.graphics_worlds_.create(arg_graphics_name);
      if(NULL==tmp_gr)
      {throw (std::runtime_error("Couldn't create graphics data struct on the pile"));}

      //Fill in the ds entry from the file
      flag = arg_parser->readGraphicsFromFile(arg_file,arg_graphics_name,*tmp_gr);
      if(false==flag)
      {throw (std::runtime_error("Couldn't read graphics from file"));}

      std::cout<<"\nscl_registry::parseGraphics() : Parsed : "<<arg_graphics_name;
    }
    catch (std::exception & e)
    {
      std::cout<<"\nparseGraphics() : Failed "<<e.what();

      //Deallocate the ds memory for the robot
      if(NULL!=tmp_gr)
      { scl::CDatabase::getData()->s_parser_.graphics_worlds_.erase(tmp_gr); }

      return NULL;
    }
    return tmp_gr;
  }

  scl::SGcController * parseGcController(const std::string &arg_file,
      const std::string &arg_robot_name,
      const std::string &arg_ctrl_name,
      scl_parser::CParserBase *arg_parser)
  {
    //Will fill in information from a file into this. It will never be
    //completely initialized.
    scl::SGcController tmp_ctrl;
    //Will be initialized (with the init function) and will be returned.
    scl::SGcController* ret_ctrl= S_NULL;
    scl::SDatabase* db;
    bool flag;
    try
    {
      db = scl::CDatabase::getData();
      if(NULL == db) { throw (std::runtime_error("Database not initialized."));  }

      //Read in the generic controller information from the file
      std::string must_use_robot;
      flag = arg_parser->readGcControllerFromFile(arg_file,arg_ctrl_name,
          must_use_robot,tmp_ctrl);
      if(false == flag) { throw (std::runtime_error("Could not read the controller."));  }

      if(arg_robot_name != must_use_robot)
      {
        std::string err = "Invalid robot specified. Controller will only work for : ";
        err = err+must_use_robot;
        throw (std::runtime_error(err.c_str()));
      }

      ret_ctrl = new scl::SGcController();
      if(NULL == ret_ctrl)
      { throw (std::runtime_error("Could not allocate the data structure."));  }

      //Now initialize the robot-specific parts.
      flag = ret_ctrl->init(arg_ctrl_name,db->s_parser_.robots_.at(arg_robot_name),
          db->s_io_.io_data_.at(arg_robot_name),
          tmp_ctrl.kp_,tmp_ctrl.kv_,tmp_ctrl.ki_,
          tmp_ctrl.force_gc_max_,tmp_ctrl.force_gc_min_);
      if(false == flag)
      { throw (std::runtime_error("Could not initialize the controller's data structure."));  }

      //Now register the data structure with the database
      ret_ctrl->type_ctrl_ds_ = "SGcController";

      scl::SControllerBase** ret = db->s_controller_.controllers_.create(ret_ctrl->name_,ret_ctrl);
      if(NULL == ret)
      {
        throw (std::runtime_error(
            "Could not create controller data structure pointer on the database pile."));
      }
    }
    catch (std::exception& e)
    {
      std::cout<<"\nparseTaskController() : Error : "<<e.what();
      if(S_NULL!=ret_ctrl) { delete ret_ctrl;  }
      ret_ctrl = S_NULL;
    }
    return ret_ctrl;
  }

  /** For sorting a vector by its first index */
  struct sort_task_ctrl_pred {
      bool operator()(const std::pair<int,scl::STaskBase*> &left, const std::pair<int,scl::STaskBase*> &right) {
          return left.first < right.first;
      }
  };

  scl::STaskController * parseTaskController(const std::string &arg_file,
        const std::string &arg_robot_name,
        const std::string &arg_ctrl_name,
        scl_parser::CParserBase *arg_parser)
  {
    //Will fill in information from a file into this. It will never be
    //completely initialized.
    scl::STaskController tmp_ctrl;
    std::vector<scl::STaskBase> taskvec;
    std::vector<scl::SNonControlTaskBase> taskvec_non_ctrl;

    //Will be initialized (with the init function) and will be returned.
    //The STaskController ds contains a multi-level pile map, which manages
    //the task hierarchy
    scl::STaskController* ret_ctrl= S_NULL;

    scl::SDatabase* db;
    bool flag;
    try
    {
      db = scl::CDatabase::getData();
      if(NULL == db) { throw (std::runtime_error("Database not initialized."));  }

      //Read in the generic controller information from the file
      std::string must_use_robot, use_dynamics;
      flag = arg_parser->readTaskControllerFromFile(arg_file,arg_ctrl_name,
          must_use_robot,tmp_ctrl,taskvec, taskvec_non_ctrl);
      if(false == flag) { throw (std::runtime_error("Could not read the controller."));  }

      if(arg_robot_name != must_use_robot)
      {
        std::string err = "Invalid robot specified. Controller will only work for : ";
        err = err+must_use_robot;
        throw (std::runtime_error(err.c_str()));
      }

      //Create the controller
      ret_ctrl = new scl::STaskController();
      if(NULL == ret_ctrl)
      { throw (std::runtime_error("Could not allocate the controller data structure."));  }

      //Now initialize the robot-specific parts.
      flag = ret_ctrl->init(arg_ctrl_name,db->s_parser_.robots_.at(arg_robot_name),
          db->s_io_.io_data_.at(arg_robot_name));
      if(false == flag)
      { throw (std::runtime_error("Could not initialize the controller's data structure."));  }

      //Now add the tasks to the controller sutil::CMappedMultiLevelList
      scl::sUInt tasks_parsed=0;
      for(std::vector<scl::STaskBase>::iterator it = taskvec.begin(),
          ite = taskvec.end(); it!=ite; ++it)
      {
        scl::STaskBase& tmp_task = *it;

        //Use dynamic typing to get the correct task type.
        void *get_task_type=S_NULL;
        flag = sutil::CRegisteredDynamicTypes<std::string>::getObjectForType(std::string("S")+tmp_task.type_task_,get_task_type);
        if(false == flag)
        { throw (std::runtime_error(std::string("S")+tmp_task.type_task_+
            std::string(" -- Unrecognized task type requested.\n * Did you register the task type with the database?\n * If not, see applications-linux/scl_skeleton_code/*.cpp to find out how.")));  }

        //Convert the pointer into a STaskBase*
        scl::STaskBase* tmp_task2add = reinterpret_cast<scl::STaskBase*>(get_task_type);

        flag = tmp_task2add->init(tmp_task.name_, tmp_task.type_task_,
            tmp_task.priority_, tmp_task.dof_task_,
            db->s_parser_.robots_.at(arg_robot_name),
            ret_ctrl->io_data_, &(ret_ctrl->gc_model_),
            tmp_task.kp_,tmp_task.kv_, tmp_task.ki_,
            tmp_task.force_task_max_, tmp_task.force_task_min_,
            tmp_task.task_nonstd_params_);
        if(false == flag)
        {
          std::string serr;
          serr = "Could not initialize task : ";
          serr = serr + tmp_task.type_task_+ std::string(" : ") + tmp_task.name_;
          if(S_NULL!=tmp_task2add)
          { delete tmp_task2add;  }
          throw (std::runtime_error(serr.c_str()));
        }


        //THE JUICE : Add all the tasks into the multi level pilemap!!
        if(S_NULL!=tmp_task2add)
        {
          //NOTE : We have to store a double pointer for tasks because the pilemap
          //creates its own memory.
          scl::STaskBase** tmp = ret_ctrl->tasks_.create(tmp_task2add->name_,
              tmp_task2add,tmp_task2add->priority_);

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
          //NOTE: SOMEONE HAS TO DELETE THE TASKS LATER!!!
        }

        tasks_parsed++;
      }

      if(0==tasks_parsed)
      {  throw (std::runtime_error("Found zero tasks in a task controller."));  }

      //Now parse the non-control tasks
      //Now add the tasks to the controller sutil::CMappedList
      tasks_parsed=0;
      for(std::vector<scl::SNonControlTaskBase>::iterator it = taskvec_non_ctrl.begin(),
          ite = taskvec_non_ctrl.end(); it!=ite; ++it)
      {
        scl::SNonControlTaskBase& tmp_task = *it;

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
          //NOTE : We have to store a double pointer for tasks because the pilemap
          //creates its own memory.
          scl::SNonControlTaskBase** tmp = ret_ctrl->tasks_non_ctrl_.create(tmp_task2add->name_, tmp_task2add);

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
          //NOTE: SOMEONE HAS TO DELETE THE TASKS LATER!!!
        }

        tasks_parsed++;
      }

#ifdef DEBUG
      if(0==tasks_parsed)
      {  std::cout<<"\nparseTaskController() WARNING : Found zero non-control tasks in a task controller.";  }
#endif

      //Now register the data structure with the database
      ret_ctrl->type_ctrl_ds_ = "STaskController";

      scl::SControllerBase** ret = db->s_controller_.controllers_.create(ret_ctrl->name_,ret_ctrl);
      if(NULL == ret)
      {
        throw (std::runtime_error(
            "Could not create controller data structure pointer on the database pile."));
      }
    }
    catch (std::exception& e)
    {
      std::cout<<"\nparseTaskController() : Error : "<<e.what();
      return NULL;
    }
    return ret_ctrl;
  }


  /**
   * Registers the native dynamic types:
   *  1. CControllerBase subclasses : CGcController, CTaskController
   *  2. CTaskBase subclasses : CTaskOpPos, CGcTask, CFrameTrackTask, CContactTask
   */
  scl::sBool registerNativeDynamicTypes()
  {
    bool flag;
    try
    {
      sutil::CDynamicType<std::string,scl::CGcController> typeCGc(std::string("CGcController"));
      flag = typeCGc.registerType();
      if(false == flag) {throw(std::runtime_error("CGcController"));}

      sutil::CDynamicType<std::string,scl::CTaskController> typeCTask(std::string("CTaskController"));
      flag = typeCTask.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskController"));}

      sutil::CDynamicType<std::string,scl::CTaskOpPos> typeCTaskOpPos(std::string("CTaskOpPos"));
      flag = typeCTaskOpPos.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskOpPos"));}

      sutil::CDynamicType<std::string,scl::STaskOpPos> typeSTaskOpPos(std::string("STaskOpPos"));
      flag = typeSTaskOpPos.registerType();
      if(false == flag) {throw(std::runtime_error("STaskOpPos"));}

      sutil::CDynamicType<std::string,scl::CTaskOpPosNoGravity> typeCTaskOpPosNoGravity(std::string("CTaskOpPosNoGravity"));
      flag = typeCTaskOpPosNoGravity.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskOpPosNoGravity"));}

      sutil::CDynamicType<std::string,scl::STaskOpPosNoGravity> typeSTaskOpPosNoGravity(std::string("STaskOpPosNoGravity"));
      flag = typeSTaskOpPosNoGravity.registerType();
      if(false == flag) {throw(std::runtime_error("STaskOpPosNoGravity"));}

      sutil::CDynamicType<std::string,scl::CTaskNullSpaceDamping> typeCTaskNullSpaceDamping(std::string("CTaskNullSpaceDamping"));
      flag = typeCTaskNullSpaceDamping.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskNullSpaceDamping"));}

      sutil::CDynamicType<std::string,scl::STaskNullSpaceDamping> typeSTaskNullSpaceDamping(std::string("STaskNullSpaceDamping"));
      flag = typeSTaskNullSpaceDamping.registerType();
      if(false == flag) {throw(std::runtime_error("STaskNullSpaceDamping"));}

      sutil::CDynamicType<std::string,scl::CTaskGc> typeCTaskGc(std::string("CTaskGc"));
      flag = typeCTaskGc.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskGc"));}

      sutil::CDynamicType<std::string,scl::STaskGc> typeSTaskGc(std::string("STaskGc"));
      flag = typeSTaskGc.registerType();
      if(false == flag) {throw(std::runtime_error("STaskGc"));}

      sutil::CDynamicType<std::string,scl::CTaskGcSet> typeCTaskGcSet(std::string("CTaskGcSet"));
      flag = typeCTaskGcSet.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskGcSet"));}

      sutil::CDynamicType<std::string,scl::STaskGcSet> typeSTaskGcSet(std::string("STaskGcSet"));
      flag = typeSTaskGcSet.registerType();
      if(false == flag) {throw(std::runtime_error("STaskGcSet"));}

      sutil::CDynamicType<std::string,scl::CTaskComPos> typeCTaskComPos(std::string("CTaskComPos"));
      flag = typeCTaskComPos.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskComPos"));}

      sutil::CDynamicType<std::string,scl::STaskComPos> typeSTaskComPos(std::string("STaskComPos"));
      flag = typeSTaskComPos.registerType();
      if(false == flag) {throw(std::runtime_error("STaskComPos"));}
    }
    catch (std::exception& e)
    {
      std::cout<<"\nregisterNativeDynamicTypes() : Could not register type : "<<e.what();
      return false;
    }
    return true;
  }
}
