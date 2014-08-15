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

#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SGraphicsParsed.hpp>
#include <scl/data_structs/SRobotIO.hpp>

#include <scl/control/data_structs/SControllerBase.hpp>
#include <scl/control/gc/CControllerGc.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>

#include <scl/control/task/data_structs/STaskBase.hpp>

#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>

#include <scl/control/task/tasks/CTaskOpPosPIDA1OrderInfTime.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPosPIDA1OrderInfTime.hpp>

#include <scl/control/task/tasks/CTaskOpPosNoGravity.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPosNoGravity.hpp>

#include <scl/control/task/tasks/CTaskGc.hpp>
#include <scl/control/task/tasks/data_structs/STaskGc.hpp>

#include <scl/control/task/tasks/CTaskGcLimitCentering.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcLimitCentering.hpp>

#include <scl/control/task/tasks/CTaskGcSet.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcSet.hpp>

#include <scl/control/task/tasks/CTaskNullSpaceDamping.hpp>
#include <scl/control/task/tasks/data_structs/STaskNullSpaceDamping.hpp>

#include <scl/control/task/tasks/CTaskComPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskComPos.hpp>

#include <scl/data_structs/SActuatorSetMuscleParsed.hpp>
#include <scl/actuation/muscles/data_structs/SActuatorSetMuscle.hpp>
#include <scl/actuation/muscles/CActuatorSetMuscle.hpp>

#include <scl/util/DatabaseUtils.hpp>

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
      scl::CParserBase *arg_parser,
      std::vector<std::string>* arg_robots_parsed,
      std::vector<std::string>* arg_graphics_parsed,
      std::vector<std::string>* arg_ui_parsed)
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

      //4. List and parse user interface specs
      if(S_NULL!=arg_ui_parsed)
      {
        std::vector<std::string> ui_names;
        flag = arg_parser->listUISpecsInFile(arg_file,ui_names);
        if(false == flag) { throw(std::runtime_error("Could not list ui names from the file"));  }

        std::vector<std::string>::iterator itg, itge;
        for(itg = ui_names.begin(), itge = ui_names.end();itg!=itge;++itg)
        {
          if(S_NULL == scl_registry::parseUI(arg_file, *itg, arg_parser))
          { throw(std::runtime_error("Could not register graphics with the database"));  }
          if(S_NULL!=arg_ui_parsed)//To be returned to caller
          { arg_ui_parsed->push_back(*itg); }
        }
      }

      //5. List and parse controllers (Note this attaches controllers to the first robot that fits!!!)
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

  const scl::SRobotParsed* parseRobot(const std::string &arg_file,
                const std::string &arg_robot_name,
                scl::CParserBase *arg_parser)
  {
    bool flag;
    scl::SRobotParsed * rob=NULL;
    scl::SRobotIO * rob_io=NULL;
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

      rob_io->setGcPosition(rob->gc_pos_default_);

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

  const scl::SGraphicsParsed* parseGraphics(const std::string &arg_file,
      const std::string & arg_graphics_name,
      scl::CParserBase *arg_parser)
  {
    bool flag;
    scl::SGraphicsParsed * tmp_gr=NULL;
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

      //Create
      scl::SGraphicsChai *chai_ds = scl::CDatabase::getData()->s_gui_.chai_data_.create(arg_graphics_name);
      if(NULL==chai_ds) {throw (std::runtime_error("Couldn't create graphics data struct on the pile"));}

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

  const scl::SUIParsed* parseUI(const std::string &arg_file,
      const std::string & arg_ui_name,
      scl::CParserBase *arg_parser)
  {
    bool flag;
    scl::SUIParsed * tmp_ui=NULL;
    try
    {
      if(NULL == scl::CDatabase::getData())
      {throw(std::runtime_error("Database not initialized."));  }

      //Create a ds entry
      tmp_ui = scl::CDatabase::getData()->s_parser_.user_interface_.create(arg_ui_name);
      if(NULL==tmp_ui)
      {throw (std::runtime_error(std::string("Couldn't create user interface data struct on the pile: ")+arg_ui_name));}

      //Fill in the ds entry from the file
      flag = arg_parser->readUISpecFromFile(arg_file,arg_ui_name,*tmp_ui);
      if(false==flag)
      {throw (std::runtime_error(std::string("Couldn't read user interface from file: ")+arg_ui_name));}

      std::cout<<"\nscl_registry::parseUI() : Parsed : "<<arg_ui_name;
    }
    catch (std::exception & e)
    {
      std::cout<<"\nscl_registry::parseUI() : Failed "<<e.what();

      //Deallocate the ds memory for the robot
      if(NULL!=tmp_ui)
      { scl::CDatabase::getData()->s_parser_.user_interface_.erase(tmp_ui); }

      return NULL;
    }
    return tmp_ui;
  }

  scl::SControllerGc * parseGcController(const std::string &arg_file,
      const std::string &arg_robot_name,
      const std::string &arg_ctrl_name,
      scl::CParserBase *arg_parser)
  {
    //Will fill in information from a file into this. It will never be
    //completely initialized.
    scl::SControllerGc tmp_ctrl;
    //Will be initialized (with the init function) and will be returned.
    scl::SControllerGc* ret_ctrl= S_NULL;
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
        std::string err = "Invalid robot specified. Controller must use: ";
        err = err+must_use_robot;
        throw (std::runtime_error(err.c_str()));
      }

      ret_ctrl = new scl::SControllerGc();
      if(NULL == ret_ctrl)
      { throw (std::runtime_error("Could not allocate the data structure."));  }

      //Now initialize the robot-specific parts.
      flag = ret_ctrl->init(arg_ctrl_name,db->s_parser_.robots_.at(arg_robot_name),
          db->s_io_.io_data_.at(arg_robot_name),
          tmp_ctrl.kp_,tmp_ctrl.kv_,tmp_ctrl.ka_,tmp_ctrl.ki_,
          tmp_ctrl.force_gc_max_,tmp_ctrl.force_gc_min_);
      if(false == flag)
      { throw (std::runtime_error("Could not initialize the controller's data structure."));  }

      scl::SControllerBase** ret = db->s_controller_.controllers_.create(ret_ctrl->name_,ret_ctrl);
      if(NULL == ret)
      {
        throw (std::runtime_error(
            "Could not create controller data structure pointer on the database pile."));
      }
    }
    catch (std::exception& e)
    {
      std::cout<<"\nparseGcController("<<arg_ctrl_name<<", "<<arg_robot_name<<") : "<<e.what();
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

  scl::SControllerMultiTask * parseTaskController(const std::string &arg_file,
        const std::string &arg_robot_name,
        const std::string &arg_ctrl_name,
        scl::CParserBase *arg_parser)
  {
    //Will fill in information from a file into this. It will never be
    //completely initialized.
    std::vector<scl::STaskBase*> taskvec;
    std::vector<scl::SNonControlTaskBase*> taskvec_non_ctrl;

    //Will be initialized (with the init function) and will be returned.
    //The SControllerMultiTask ds contains a multi-level pile map, which manages
    //the task hierarchy
    scl::SControllerMultiTask* ret_ctrl= S_NULL;

    scl::SDatabase* db;
    bool flag;
    try
    {
      db = scl::CDatabase::getData();
      if(NULL == db) { throw (std::runtime_error("Database not initialized."));  }

      //********************************************************
      //Read in the generic controller information from the file
      //********************************************************
      std::string must_use_robot, use_dynamics;
      flag = arg_parser->readTaskControllerFromFile(arg_file,arg_ctrl_name,
          must_use_robot,taskvec, taskvec_non_ctrl);
      if(false == flag) { throw (std::runtime_error("Could not read the controller."));  }

      if(arg_robot_name != must_use_robot)
      { throw (std::runtime_error(std::string("Invalid robot specified. Controller must use: ")+must_use_robot)); }

      //Create the controller
      ret_ctrl = new scl::SControllerMultiTask();
      if(NULL == ret_ctrl)
      { throw (std::runtime_error("Could not allocate the controller data structure."));  }

      //Now initialize the robot-specific parts.
      flag = ret_ctrl->init(arg_ctrl_name,db->s_parser_.robots_.at(arg_robot_name),
          db->s_io_.io_data_.at(arg_robot_name));
      if(false == flag)
      { throw (std::runtime_error("Could not initialize the controller's data structure."));  }

      unsigned int tasks_parsed = scl_util::initMultiTaskCtrlDsFromParsedTasks(taskvec, taskvec_non_ctrl,*ret_ctrl);

      if(0==tasks_parsed)
      {  throw(std::runtime_error("Found no control tasks in a task controller."));  }


      //**********************************************************
      //Now that everything is parsed, add an entry to the database
      scl::SControllerBase** ret = db->s_controller_.controllers_.create(ret_ctrl->name_,ret_ctrl);
      if(NULL == ret)
      { throw (std::runtime_error("Could not create controller data structure pointer on the database pile.")); }

      sutil::CMappedMultiLevelList<std::string, scl::STaskBase*>::iterator itt,itte;
      for(itt = ret_ctrl->tasks_.begin(), itte = ret_ctrl->tasks_.end(); itt!=itte;++itt)
      {
        scl::STaskBase * tmp_task2add = *itt;
        //NOTE : We have to store a double pointer for tasks because the mapped list creates its own memory for a pointer.
        // The databse will delete this object later. Remember the object originally came from a dynamic type factory
        // (look further up in the code).
        scl::STaskBase** tmp = db->s_controller_.tasks_.create(tmp_task2add->name_, tmp_task2add);

        //Detailed error reporting.
        if(S_NULL == tmp)
        {
          std::string serr;
          serr = "Could not create database task data structure for : ";
          if(S_NULL!=tmp_task2add)
          { serr = serr + tmp_task2add->name_;  }
          else
          { serr = serr + "NULL task pointer (should not be NULL)"; }
          throw (std::runtime_error(serr.c_str()));
        }
      }

      sutil::CMappedList<std::string, scl::SNonControlTaskBase*>::iterator ittnc,ittnce;
      for(ittnc = ret_ctrl->tasks_non_ctrl_.begin(), ittnce = ret_ctrl->tasks_non_ctrl_.end(); ittnc!=ittnce;++ittnc)
      {
        scl::SNonControlTaskBase * tmp_task2add = *ittnc;
        //NOTE : We have to store a double pointer for tasks because the mapped list creates its own memory for a pointer.
        // The databse will delete this object later. Remember the object originally came from a dynamic type factory
        // (look further up in the code).
        scl::SNonControlTaskBase** tmp = db->s_controller_.tasks_non_ctrl_.create(tmp_task2add->name_, tmp_task2add);

        //Detailed error reporting.
        if(S_NULL == tmp)
        {
          std::string serr;
          serr = "Could not create database task data structure for non-ctrl task: ";
          if(S_NULL!=tmp_task2add)
          { serr = serr + tmp_task2add->name_;  }
          else
          { serr = serr + "NULL task pointer (should not be NULL)"; }
          throw (std::runtime_error(serr.c_str()));
        }
      }
    }
    catch (std::exception& e)
    {
      std::cout<<"\nparseTaskController("<<arg_ctrl_name<<", "<<arg_robot_name<<") : "<<e.what();
      return NULL;
    }
    return ret_ctrl;
  }


  /**
   * Registers the native dynamic types:
   *  1. CControllerBase subclasses : CControllerGc, CControllerMultiTask
   *  2. CTaskBase subclasses : CTaskOpPos, CGcTask, CFrameTrackTask, CContactTask
   */
  scl::sBool registerNativeDynamicTypes()
  {
    bool flag;
    try
    {
      /** *****************************************************************************************************
       *                                        Controllers
       * ***************************************************************************************************** */
      sutil::CDynamicType<std::string,scl::CControllerGc> typeCGc(std::string("CControllerGc"));
      flag = typeCGc.registerType();
      if(false == flag) {throw(std::runtime_error("CControllerGc"));}

      sutil::CDynamicType<std::string,scl::CControllerMultiTask> typeCTask(std::string("CControllerMultiTask"));
      flag = typeCTask.registerType();
      if(false == flag) {throw(std::runtime_error("CControllerMultiTask"));}

      /** *****************************************************************************************************
       *                                        Control Tasks
       * ***************************************************************************************************** */
      sutil::CDynamicType<std::string,scl::CTaskOpPos> typeCTaskOpPos(std::string("CTaskOpPos"));
      flag = typeCTaskOpPos.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskOpPos"));}

      sutil::CDynamicType<std::string,scl::STaskOpPos> typeSTaskOpPos(std::string("STaskOpPos"));
      flag = typeSTaskOpPos.registerType();
      if(false == flag) {throw(std::runtime_error("STaskOpPos"));}

      sutil::CDynamicType<std::string,scl::CTaskOpPosPIDA1OrderInfTime> typeCTaskOpPosPIDA1OrderInfTime(std::string("CTaskOpPosPIDA1OrderInfTime"));
      flag = typeCTaskOpPosPIDA1OrderInfTime.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskOpPosPIDA1OrderInfTime"));}

      sutil::CDynamicType<std::string,scl::STaskOpPosPIDA1OrderInfTime> typeSTaskOpPosPIDA1OrderInfTime(std::string("STaskOpPosPIDA1OrderInfTime"));
      flag = typeSTaskOpPosPIDA1OrderInfTime.registerType();
      if(false == flag) {throw(std::runtime_error("STaskOpPosPIDA1OrderInfTime"));}

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

      sutil::CDynamicType<std::string,scl::CTaskGcLimitCentering> typeCTaskGcLimitCentering(std::string("CTaskGcLimitCentering"));
      flag = typeCTaskGcLimitCentering.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskGcLimitCentering"));}

      sutil::CDynamicType<std::string,scl::STaskGcLimitCentering> typeSTaskGcLimitCentering(std::string("STaskGcLimitCentering"));
      flag = typeSTaskGcLimitCentering.registerType();
      if(false == flag) {throw(std::runtime_error("STaskGcLimitCentering"));}

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

      /** *****************************************************************************************************
       *                                        Actuator Sets
       * ***************************************************************************************************** */
      sutil::CDynamicType<std::string,scl::SActuatorSetMuscleParsed> typeSActuatorSetMuscleParsed(std::string("SActuatorSetMuscleParsed"));
      flag = typeSActuatorSetMuscleParsed.registerType();
      if(false == flag) {throw(std::runtime_error("SActuatorSetMuscleParsed"));}

      sutil::CDynamicType<std::string,scl::SActuatorSetMuscle> typeSActuatorSetMuscle(std::string("SActuatorSetMuscle"));
      flag = typeSActuatorSetMuscle.registerType();
      if(false == flag) {throw(std::runtime_error("SActuatorSetMuscle"));}

      sutil::CDynamicType<std::string,scl::CActuatorSetMuscle> typeCActuatorSetMuscle(std::string("CActuatorSetMuscle"));
      flag = typeCActuatorSetMuscle.registerType();
      if(false == flag) {throw(std::runtime_error("CActuatorSetMuscle"));}
    }
    catch (std::exception& e)
    {
      std::cout<<"\nregisterNativeDynamicTypes() : Could not register type : "<<e.what();
      return false;
    }
    return true;
  }
}
