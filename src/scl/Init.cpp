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
/* \file Init.cpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/Init.hpp>
#include <scl/AllHeaders.hpp>

#include <sutil/CRegisteredDynamicTypes.hpp>

#include <string>

namespace scl
{
  namespace init
  {
    /** Dynamic typing helper functions for different scl types
     *  Registers the native dynamic types */
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

        sutil::CDynamicType<std::string,scl::CTaskConstraintPlane> typeCTaskConstraintPlane(std::string("CTaskConstraintPlane"));
        flag = typeCTaskConstraintPlane.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskConstraintPlane"));}

        sutil::CDynamicType<std::string,scl::STaskConstraintPlane> typeSTaskConstraintPlane(std::string("STaskConstraintPlane"));
        flag = typeSTaskConstraintPlane.registerType();
        if(false == flag) {throw(std::runtime_error("STaskConstraintPlane"));}

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
        std::cout<<"\nscl::init::registerNativeDynamicTypes() : Could not register type : "<<e.what();
        return false;
      }
      return true;
    }

    /** An acme (swiss knife/random util/highly customized junk function) that
     * consolidates a lot of init code.
     * This just happens to be what is required for a generic control app.
     * It's not general at all. If you write your own app, I strongly recommend
     * that you copy the function's contents and insert them into your main
     * file. I just don't want to keep repeating this code all over the
     * place so it's consolidated here.
     */
    bool parseAndInitRobotAndController(
        scl::CParserScl &arg_p /** Parser */,
        scl::SCmdLineOptions_OneRobot arg_rcmd /* For parsing command line options */,
        scl::SRobotParsed &arg_rds /**Robot data structure*/,
        scl::SRobotIO &arg_rio /**I/O data structure. */,
        scl::SGcModel &arg_rgcm /**Robot data structure with dynamic quantities...*/,
        scl::CDynamicsScl &arg_dyn_scl /**Robot kinematics and dynamics computation object...*/,
        scl::SControllerMultiTask &arg_rctr_ds /**A multi-task controller data structure*/,
        scl::CControllerMultiTask &arg_rctr /**A multi-task controller*/,
        std::vector<scl::STaskBase*> &arg_rtasks /**A set of executable tasks*/,
        std::vector<scl::SNonControlTaskBase*> &arg_rtasks_nc /**A set of non-control tasks*/,
        std::vector<scl::sString2> &arg_ctrl_params /**Used to parse extra xml tags)*/
    )
    {
      bool flag;
      try
      {
        flag = parseAndInitRobotAndDynamics(arg_p, arg_rcmd, arg_rds, arg_rio, arg_rgcm, arg_dyn_scl);
        if(false == flag) { throw(std::runtime_error("Failed to parse robot and dynamics"));  }

        /******************************Set up Controller Specification************************************/
        // Read xml file info into task specifications.
        flag = arg_p.readTaskControllerFromFile(arg_rcmd.name_file_config_,arg_rcmd.name_ctrl_,arg_rtasks,arg_rtasks_nc,arg_ctrl_params);
        flag = flag && arg_rctr_ds.init(arg_rcmd.name_ctrl_,&arg_rds,&arg_rio,&arg_rgcm); //Set up the control data structure..

        // Tasks are initialized after we find their type with dynamic typing.
        flag = flag && scl::init::initMultiTaskCtrlDsFromParsedTasks(arg_rtasks,arg_rtasks_nc,arg_rctr_ds);
        flag = flag && arg_rctr.init(&arg_rctr_ds,&arg_dyn_scl);  //Set up the controller (needs parsed data and a dyn object)
        if(false == flag){ throw(std::runtime_error("Could not initialize controller")); }            //Error check.

        // Set up the tasks that will receive goal positions etc.
        if(0 >= arg_rcmd.name_tasks_.size())
        { std::cout<<"\n WARNING : Did not provide any tasks in the command line arguments: -t/-op/-ui <task name>"; }
        if(SCL_NUM_UI_POINTS < arg_rcmd.name_tasks_.size())
        {
          char tmp_message[256];
          sprintf(tmp_message,"Too many tasks provided. Can support at most : %d", SCL_NUM_UI_POINTS);
          throw(std::runtime_error( tmp_message ));
        }

        // Compute the dynamics to begin with (flushes state across all matrices).
        arg_rctr.computeDynamics();
        arg_rctr.computeControlForces(); //Directly update io data structure for now...
      }
      catch (std::exception& e)
      {
        std::cout<<"\nscl::init::parseAndInitRobotAndController() : ERROR : "<<e.what();
        return false;
      }
      return true;
    }

    bool parseAndInitRobotAndDynamics(
        scl::CParserScl &arg_p /** Parser */,
        scl::SCmdLineOptions_OneRobot arg_rcmd /* For parsing command line options */,
        scl::SRobotParsed &arg_rds /**Robot data structure*/,
        scl::SRobotIO &arg_rio /**I/O data structure. */,
        scl::SGcModel &arg_rgcm /**Robot data structure with dynamic quantities...*/,
        scl::CDynamicsScl &arg_dyn_scl /**Robot kinematics and dynamics computation object...*/
    )
    {
      try
      {
        /******************************Load Robot Specification************************************/
        bool flag = arg_p.readRobotFromFile(arg_rcmd.name_file_config_,"../../specs/",arg_rcmd.name_robot_,arg_rds);
        if(false == flag){ throw(std::runtime_error("Could not parse robot spec")); }            //Error check.

        flag = arg_rio.init(arg_rds);             //Set up the IO data structure
        if(false == flag){ throw(std::runtime_error("Could not initialize robot io")); }            //Error check.

        flag = arg_rgcm.init(arg_rds);            //Simple way to set up dynamic tree...
        if(false == flag){ throw(std::runtime_error("Could not initialize robot gc model")); }            //Error check.

        flag = arg_dyn_scl.init(arg_rds);         //Set up kinematics and dynamics object
        if(false == flag){ throw(std::runtime_error("Could not initialize robot dynamics")); }            //Error check.
      }
      catch (std::exception& e)
      {
        std::cout<<"\nscl::init::parseAndInitRobotAndDynamics() : ERROR : "<<e.what();
        return false;
      }
      return true;
    }

    /** In many places we assign a subset of tasks to 3d vector control points.
     * This is a very common operation FOR SOME APPS so we'll add some code here to avoid repetition
     *
     * Note it's not general enough to warrant inclusion in any of the core functions.
     */
    bool initUI3dPointVectorFromParsedTasksAndCmdLineArgs(
        const scl::SCmdLineOptions_OneRobot &arg_rcmd /** For parsing command line options */,
        scl::SControllerMultiTask &arg_rctr_ds /** The initialized controller multi-task data struct */,
        std::vector<scl::STaskOpPos*> &arg_rtask_ui_3d_ds /** This will contain the final op pos tasks */
    )
    {
      try
      {
        if(false == arg_rctr_ds.hasBeenInit())
        { throw(std::runtime_error("The passed controller data structure was not initialized. Try again..")); }

        if(false == arg_rcmd.hasBeenInit())
        { throw(std::runtime_error("The passed command line args were not initialized. Try again..")); }

        // Pre-alloc vector data for the different tasks..
        arg_rtask_ui_3d_ds.resize(arg_rcmd.name_tasks_.size());

        // Loop over the tasks. Find the desired 3d point tasks and return them.
        for(scl::sUInt i=0; i< arg_rcmd.name_tasks_.size(); ++i)
        {
          // Find the task data structure
          arg_rtask_ui_3d_ds[i] = dynamic_cast<scl::STaskOpPos*>(*arg_rctr_ds.tasks_.at(arg_rcmd.name_tasks_[i]));
          if(NULL == arg_rtask_ui_3d_ds[i])
          { throw(std::runtime_error(std::string("Did not find the desired command line task in the controller tasks : ") + arg_rcmd.name_tasks_.at(i))); }
        }
      }
      catch (std::exception& e)
      {
        std::cout<<"\nscl::init::parseAndInitRobotAndDynamics() : ERROR : "<<e.what();
        return false;
      }
      return true;
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
        { std::cout<<"\nscl::init::initMultiTaskCtrlDsFromParsedTasks() : WARNING : Did not find any non-control tasks";}
#endif
      }
      catch(std::exception& ee)
      {
        std::cerr<<"\nscl::init::initMultiTaskCtrlDsFromParsedTasks() : "<<ee.what();
        return 0;
      }
      return tasks_parsed;
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
            ss>>ret_ctrl.option_servo_to_model_rate_;
          }
        }
        return true;
      }
      catch (std::exception & e)
      { std::cout<<"\nscl::init::initMultiTaskCtrlDsFromParsedTasks() : Failed "<<e.what(); }
      return false;
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
        std::cerr<<"\nscl::init::initDynRobotFromParsedRobot() : "<<ee.what();
        return false;
      }
      return true;
    }

  }
}
