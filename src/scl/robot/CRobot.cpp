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
/* \file CRobot.cpp
 *
 *  Created on: Dec 27, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <stdexcept>
#include <cassert>

#include <scl/robot/CRobot.hpp>

#include <scl/control/data_structs/SControllerBase.hpp>
#include <scl/control/gc/CControllerGc.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>

#include <sutil/CSystemClock.hpp>

namespace scl
{
  // **********************************************************************
  //                    Primary Computation functions
  // **********************************************************************

  /** Computes the robot's command torques.
   * Asserts false in debug mode if something bad happens */
  void CRobot::computeServo()
  {
    bool flag=true;
    if((S_NULL != ctrl_current_) && data_.has_been_init_)
    {
      if(data_.parsed_robot_data_->flag_controller_on_)
      {
        //Computes the command torques for the simulator to the controller's computed torques
        //Also stores the output in the io_data_->actuator data structure
        flag = ctrl_current_->computeControlForces();
      }
      else
      {//Controller off
        data_.io_data_->actuators_.force_gc_commanded_.setZero(data_.io_data_->dof_);
      }
    }
#ifdef DEBUG
    assert(flag);
#endif
  }

  /** Computes the robot's dynamic model.
   * Asserts false in debug mode if something bad happens */
  void CRobot::computeDynamics()
  {
    bool flag = true;
    if((S_NULL != ctrl_current_)
        && data_.has_been_init_
        && data_.parsed_robot_data_->flag_controller_on_)
    {
      flag = flag && ctrl_current_->computeDynamics();
    }
#ifdef DEBUG
    assert(flag);
#endif
  }

  /** Computes the robot's non-control related operations
   * Asserts false in debug mode if something bad happens */
  void CRobot::computeNonControlOperations()
  {
    bool flag = true;
    if((S_NULL != ctrl_current_)
        && data_.has_been_init_
        && data_.parsed_robot_data_->flag_controller_on_)
    {
      flag = flag && ctrl_current_->computeNonControlOperations();
    }
#ifdef DEBUG
    assert(flag);
#endif
  }

  /** Integrates the robot's dynamics (physics model).
   * By default, it integrates for a time period dt specifiecd
   * in the database
   *
   * Asserts false in debug mode if something bad happens */
  void CRobot::integrateDynamics()
  {
    bool flag=true;
    if((S_NULL != integrator_)
        && data_.has_been_init_)
    {
      //Apply gc limits and collision with heavy energy loss.
      if(data_.parsed_robot_data_->flag_apply_actuator_force_limits_) //Force limits controlled by a flag
      {
        data_.io_data_->actuators_.force_gc_commanded_.array().min(data_.parsed_robot_data_->actuator_forces_max_.array());
        data_.io_data_->actuators_.force_gc_commanded_.array().max(data_.parsed_robot_data_->actuator_forces_min_.array());
      }

      flag = integrator_->integrate(*(data_.io_data_), CDatabase::getData()->sim_dt_);

      //Apply gc damping
      if(data_.parsed_robot_data_->flag_apply_gc_damping_)
      {
        data_.io_data_->sensors_.dq_.array() -=
            data_.io_data_->sensors_.dq_.array() * data_.parsed_robot_data_->damping_.array(); //1% Velocity damping.
      }

      /* Note: Most models' gc limits are not correct right now. Uncomment this after
       * fixing them: */
      //Apply gc limits and collision with heavy energy loss.
      if(data_.parsed_robot_data_->flag_apply_gc_pos_limits_)
      {
        /* It is normally easier to do this:
         * x = x.array().min(max.array());//Min of self and max
         * x = x.array().max(min.array());//Max of self and min
         *
         * However, we also have to set the acceleration and velocity
         * so it is just more straightforward to loop through the code.
         */
        for(sUInt i=0; i< data_.io_data_->dof_;++i)
        {
#ifdef DEBUG
          //Logical check.
          assert(data_.parsed_robot_data_->gc_pos_limit_max_(i) >
          data_.parsed_robot_data_->gc_pos_limit_min_(i));
#endif
          //NOTE TODO : Implement the gc limit parsing and fix q at the gc limits
          if(data_.io_data_->sensors_.q_(i) > data_.parsed_robot_data_->gc_pos_limit_max_(i))
          { data_.io_data_->sensors_.q_(i) = data_.parsed_robot_data_->gc_pos_limit_max_(i); }
          else if(data_.io_data_->sensors_.q_(i) < data_.parsed_robot_data_->gc_pos_limit_min_(i))
          { data_.io_data_->sensors_.q_(i) = data_.parsed_robot_data_->gc_pos_limit_min_(i); }
          else
          { continue; }
          //Collision
          data_.io_data_->sensors_.dq_(i) = data_.io_data_->sensors_.dq_(i) * 0.01;//99% energy loss
          data_.io_data_->sensors_.ddq_(i) = 0;
#ifdef DEBUG
          std::cout<<"\nCollided with gc limits: "<<i
              <<" : "<<data_.io_data_->sensors_.q_(i)<<". Lim : "
              <<data_.parsed_robot_data_->gc_pos_limit_min_(i)<<"-"
              <<data_.parsed_robot_data_->gc_pos_limit_max_(i);
          std::cout<<"\nFull gc state: "<<data_.io_data_->sensors_.q_.transpose();
          std::cout<<"\nSleeping for a second";
          sleep(1);
#endif
        }
      }
    }

#ifdef DEBUG
    assert(flag);
#endif
  }

  // **********************************************************************
  //                       Initialization helper functions
  // **********************************************************************

  /** Convenience function. Pulls all the data from the database and calls
   * the other init function. */
  sBool CRobot::initFromDb(std::string arg_robot_name,
      CDynamicsBase* arg_dynamics,
      CDynamicsBase* arg_integrator)
  {
    bool flag;
    try
    {
      //Just pulls data from the database
      db_ = CDatabase::getData();
      if(S_NULL == db_) { throw(std::runtime_error("Database not initialized"));  }

      SRobotParsed *robot = db_->s_parser_.robots_.at(arg_robot_name);
      if(S_NULL == robot) { throw(std::runtime_error("Robot not parsed from file"));  }
      SRobotIO* io_data = db_->s_io_.io_data_.at(arg_robot_name);
      if(S_NULL == io_data)
      {
        throw(std::runtime_error(
            "Robot's I/O data structure not initialized. Use scl_registry::parseRobot() or create manually in s_io_.io_data_."));
      }

      //Set up the controllers (creates controller objects based on the parsed data
      //that is available in the database.
      std::vector<SControllerBase*> ctrls;
      flag = db_->s_controller_.getControllersForRobot(arg_robot_name,ctrls);//Read Db
      if(false == flag)
      { throw(std::runtime_error("Could not read controller(s) for robot from the database."));  }

      //Calls the init function to do the actual work.
      flag = init(arg_robot_name, arg_dynamics, arg_integrator,
                  robot, io_data, ctrls);
      if(false == flag) { throw(std::runtime_error("Could not initialize robot.")); }
    }
    catch(std::exception & e)
    { std::cout<<"\nCRobot::initFromDb("<<arg_robot_name<<") Error: "<< e.what(); }

    return data_.has_been_init_;
  }


  /** The actual data required to initialize a robot.
   * Initializes the robot:
   * 1. Verifies that the robot's data in the database is consistent
   * 2. DELETES all its existing data! (NOTE this carefully!)
   * 3. Reads the list of available controllers from the database,
   *    finds the controllers that match this robot, and initializes them.
   *    (a) Creates a controller object for each new controller
   *    (b) Adds the controller data-structure to this robot's database data structure
   *    (c) Sets the last controller as the current controller (default, can be changed) */
  sBool CRobot::init(std::string arg_robot_name,
      CDynamicsBase* arg_dynamics,
      CDynamicsBase* arg_integrator,
      SRobotParsed *arg_robot,
      SRobotIO *arg_io_data,
      std::vector<SControllerBase*>& arg_ctrls)
  {
    try
    {
      db_ = CDatabase::getData();
      if(S_NULL == db_) { throw(std::runtime_error("Database not initialized"));  }

      //Error checks.
      if(S_NULL == arg_dynamics) { throw(std::runtime_error("Passed NULL dynamics"));  }
      if(S_NULL == arg_integrator) { throw(std::runtime_error("Passed NULL integrator"));  }
      if(S_NULL == arg_robot) { throw(std::runtime_error("Passed NULL robot"));  }
      if(S_NULL == arg_io_data) { throw(std::runtime_error("Passed NULL io_data"));  }

      if(false == arg_dynamics->hasBeenInit())
      { throw(std::runtime_error("Passed unintialized dynamics"));  }
      if(false == arg_integrator->hasBeenInit())
      { throw(std::runtime_error("Passed unintialized integrator"));  }
      if(false == arg_robot->has_been_init_)
      { throw(std::runtime_error("Passed uninitialized robot data structure"));  }
      if(false == arg_io_data->has_been_init_)
      { throw(std::runtime_error("Passed uninitialized robot I/O data structure"));  }

      //In case stuff already exists : Delete and deallocate everything.
      if(S_NULL!=dynamics_){  delete dynamics_; dynamics_ = S_NULL; }
      if(S_NULL!=integrator_) {  delete integrator_; integrator_ = S_NULL; }

      sutil::CMappedList<std::basic_string<char>, scl::CControllerBase*>::iterator it, ite;
      for(it = ctrl_.begin(), ite = ctrl_.end(); it!=ite; ++it)
      { delete *it; }
      ctrl_.clear();//Clears the pilemap and deallocates the pointers
      ctrl_current_ = S_NULL;

      //Initialization.
      dynamics_ = arg_dynamics;
      integrator_ = arg_integrator;

      data_.name_ = arg_robot_name;
      data_.parsed_robot_data_ = arg_robot;
      data_.io_data_ = arg_io_data;
      data_.controllers_.clear();

      // Initialize at default joint position
      data_.io_data_->sensors_.q_.setZero(data_.io_data_->dof_);//Set size
      sutil::CMappedTree<std::string, scl::SRigidBody>::iterator itrb,itrbe;
      for (itrb = data_.parsed_robot_data_->rb_tree_.begin(),
          itrbe = data_.parsed_robot_data_->rb_tree_.end();
          itrb!=itrbe; ++itrb)
      {//Set values
        if(itrb->link_id_<0){continue;}
        else if(itrb->link_id_>=static_cast<int>(data_.io_data_->dof_))
        { throw(std::runtime_error("Rigid body tree has a link with an id greater than the robot's dof."));  }
        data_.io_data_->sensors_.q_(itrb->link_id_) = itrb->joint_default_pos_;
      }
      data_.io_data_->sensors_.dq_.setZero(data_.io_data_->dof_);
      data_.io_data_->sensors_.ddq_.setZero(data_.io_data_->dof_);

      //Some defaults:
      //Damping = 1% energy loss per second
      data_.parsed_robot_data_->damping_.setConstant(data_.io_data_->dof_,db_->sim_dt_/100);
      //Force limits: 1000 N
      data_.parsed_robot_data_->actuator_forces_max_.setConstant(data_.io_data_->dof_,3000);
      data_.parsed_robot_data_->actuator_forces_min_.setConstant(data_.io_data_->dof_,-3000);

      std::vector<SControllerBase*>::iterator itcr, itcre;
      for(itcr = arg_ctrls.begin(),itcre=arg_ctrls.end(); itcr!=itcre; ++itcr)
      {
        bool flag;
        flag = addController(*itcr);
        if(false == flag)
        {
          std::cout<<"\nCRobot::init("<<arg_robot_name
          <<") Warning: Could not add controller : "<< (*itcr)->name_;
        }
      }

      if(ctrl_.size() <= 0)
      { std::cout<<"\nCRobot::init("<<arg_robot_name<<") WARNING : Could not parse any controllers."; }
      else
      {
        if(S_NULL==ctrl_.at(0))
        { throw(std::runtime_error("Controller list contains invalid controller data structure."));  }
        ctrl_current_ = *(ctrl_.at(0));
      } //Pick the first available controller.

      data_.has_been_init_ = true;
    }
    catch(std::exception & e)
    {
      data_.has_been_init_ = false;
      std::cout<<"\nCRobot::init("<<arg_robot_name<<") Error: "<< e.what();
    }
    return data_.has_been_init_;
  }

  // **********************************************************************
  //                       Robot helper functions
  // **********************************************************************

  /** Sets the velocity damping for each gc dof */
  sBool CRobot::setDamping(const Eigen::VectorXd& arg_d)
  {
    sBool flag;
    if(static_cast<sUInt>(arg_d.size()) == data_.io_data_->dof_)
    { data_.parsed_robot_data_->damping_ = arg_d; flag = true;  }
    else { flag = false; }
    return flag;
  }

  /** Sets the velocity damping for each gc dof
   * WARNING: This will overwrite the values read in from the config file */
  sBool CRobot::setGcPosLimits(const Eigen::VectorXd& arg_max,
      const Eigen::VectorXd& arg_min)
  {
    if((static_cast<sUInt>(arg_max.size()) == data_.io_data_->dof_) &&
        (static_cast<sUInt>(arg_min.size()) == data_.io_data_->dof_))
    {
      data_.parsed_robot_data_->gc_pos_limit_max_ = arg_max;
      data_.parsed_robot_data_->gc_pos_limit_min_ = arg_min;
      return true;
    }
    return false;
  }

  /** Sets the actuator limits for each actuator */
  sBool CRobot::setActuatorForceLimits(const Eigen::VectorXd& arg_max,
      const Eigen::VectorXd& arg_min)
  {
    if((static_cast<sUInt>(arg_max.size()) == data_.io_data_->dof_) &&
        (static_cast<sUInt>(arg_min.size()) == data_.io_data_->dof_))
    {
      data_.parsed_robot_data_->actuator_forces_max_ = arg_max;
      data_.parsed_robot_data_->actuator_forces_min_ = arg_min;
      return true;
    }
    return false;
  }

  // **********************************************************************
  //                       Controller helper functions
  // **********************************************************************

  sBool CRobot::addController(SControllerBase* arg_ctrl_ds)
  {
    bool flag;
    CControllerBase *tmp_ctrl=S_NULL;
    try
    {
      if(!arg_ctrl_ds->has_been_init_)
      { throw(std::runtime_error("Passed uninitialized controller data structure.")); }

      if(arg_ctrl_ds->robot_name_ != data_.name_)
      {//Check if it was initialized correctly.
        std::string s("Passed a controller for robot : ");
        s = arg_ctrl_ds->robot_name_+std::string(". But this robot is : ")+data_.name_;
        throw(std::runtime_error(s.c_str()));
      }

      //Allocate an appropriate controller based on the passed data structure's type. NOTE TODO : Update with dynamic typing
      if("SControllerGc" == arg_ctrl_ds->getType())
      { tmp_ctrl = new CControllerGc();  }
      else if("SControllerMultiTask" == arg_ctrl_ds->getType())
      { tmp_ctrl = new CControllerMultiTask(); }
      else
      {
        std::string s("Unrecognized controller type : ");
        s = s + arg_ctrl_ds->getType();
        throw(std::runtime_error(s.c_str()));
      }

      if(S_NULL == tmp_ctrl)
      { throw(std::runtime_error("Can't create a controller object. Out of memory?")); }

      //Initialize the controller
      flag = tmp_ctrl->init(arg_ctrl_ds,dynamics_);
      if(false == flag)
      {//Check if it was initialized correctly.
        std::string s;
        s = arg_ctrl_ds->robot_name_+std::string(". Controller : ")
        +arg_ctrl_ds->name_+std::string(" failed at initialization.");
        throw(std::runtime_error(s.c_str()));
      }
      else
      {//If all is well, add the controller to the pilemap (ready to be executed).
        CControllerBase** tmp_ctrl2 = ctrl_.create(arg_ctrl_ds->name_,tmp_ctrl);
        if(S_NULL == tmp_ctrl2)
        {
          std::string s;
          s = arg_ctrl_ds->robot_name_+std::string(". Controller : ")
              + arg_ctrl_ds->name_+std::string(
              " failed to add the controller to the object pilemap (probably duplicate name).");
          throw(std::runtime_error(s.c_str()));
        }
        else
        {//Able to successfully add the controller to the pilemap.
          ctrl_current_ = *tmp_ctrl2;//Set it to be the current controller (default behavior)
          SControllerBase** tmp_cds = data_.controllers_.create(arg_ctrl_ds->name_,
              arg_ctrl_ds);//Add the controller data structure to the robot's list.
          if(S_NULL == tmp_cds)
          {
            std::string s;
            s = arg_ctrl_ds->robot_name_+std::string(". Controller : ")
                + arg_ctrl_ds->name_+std::string(
                " failed to add the data structure pilemap (probably duplicate name).");
            throw(std::runtime_error(s.c_str()));
          }
          data_.controller_current_ = *tmp_cds;
        }
      }
      return true;
    }
    catch(std::exception & e)
    {//This only issues warnings.
      std::cout<<"\nCRobot::addController() WARNING : "<< e.what();
      if(S_NULL!=tmp_ctrl)
      { delete tmp_ctrl;  }
    }
    return false;
  }

  /** Gets access to the current controller data structure */
  CControllerBase* CRobot::getControllerCurrent()
  {
    try
    {
      if(false == data_.has_been_init_)
      { throw(std::runtime_error("Robot not initialized"));}
      return ctrl_current_;
    }
    catch(std::exception & e)
    { std::cout<<"\nCRobot::getCurrentController() Error : "<< e.what();  }
    return S_NULL;
  }

  /** Selects the passed controller if it exists and has been initialized
   * Returns false if the controller doesn't exist*/
  sBool CRobot::setControllerCurrent(std::string arg_ctrl_name)
  {
    try
    {
      if(false == data_.has_been_init_)
      { throw(std::runtime_error("Robot not initialized"));}

      CControllerBase* tmp_c = S_NULL;
      SControllerBase* tmp_ds = S_NULL;

      if(S_NULL == ctrl_.at(arg_ctrl_name))
      { throw(std::runtime_error("Could not find the passed controller name for this robot."));}

      tmp_c = *(ctrl_.at(arg_ctrl_name));
      if(S_NULL == tmp_c)
      { throw(std::runtime_error("Controller object not found."));}

      tmp_ds = *(data_.controllers_.at(arg_ctrl_name));
      if(S_NULL == tmp_ds)
      { throw(std::runtime_error("Controller data structure not found."));}

      ctrl_current_ = tmp_c;
      data_.controller_current_ = tmp_ds;

      return true;
    }
    catch(std::exception & e)
    { std::cout<<"\nCRobot::setControllerCurrent("<<arg_ctrl_name<<", "<<data_.name_<<") Error : "<< e.what();  }
    return false;
  }

  /** Gets access to the current controller data structure */
  SControllerBase* CRobot::getControllerDataStruct(const std::string& arg_ctrl_name)
  {
    try
    {
      if(false == data_.has_been_init_)
      { throw(std::runtime_error("Robot not initialized"));}

      SControllerBase* tmp_ds = S_NULL;

      tmp_ds = *(data_.controllers_.at(arg_ctrl_name));
      if(S_NULL == tmp_ds)
      { throw(std::runtime_error("Controller data structure not found."));}

      return tmp_ds;
    }
    catch(std::exception & e)
    { std::cout<<"\nCRobot::getController("<<arg_ctrl_name<<") Error : "<< e.what();  }
    return S_NULL;
  }

  /** Returns the proportional gain of a given task in a controller.
   *
   * @param[out] ret_gains The variable into which the gains will be copied
   * @param[in]  ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
   * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers.
   *
   * Note: If you modify code here, you will probably also
   * need to modify the same code in the other get gain functions. */
  sBool CRobot::getProportionalGain(
      Eigen::VectorXd& ret_gains,
      const std::string& arg_ctrl_name,
      const std::string& arg_task_name)
  {
    SControllerBase* ctrl;
    if("" == arg_ctrl_name)
    { ctrl = data_.controller_current_; }
    else
    { ctrl = *data_.controllers_.at(arg_ctrl_name); }

    if(NULL == ctrl)
    { return false; }

    //Controllers may either be gc, in which case they have
    //only one set of gains.
    if("SControllerGc" == ctrl->getType())
    {
      SControllerGc* ctrlgc = dynamic_cast<SControllerGc*>(ctrl);
      ret_gains = ctrlgc->kp_;
      return true;
    }
    //Or they may be task controllers, in which case they have
    //multiple sets of gains, one for each task.
    else if("SControllerMultiTask" == ctrl->getType())
    {
      //Task name is required for task controllers
      if(""==arg_task_name)
      { return false; }
      SControllerMultiTask* ctrltask = dynamic_cast<SControllerMultiTask*>(ctrl);
      STaskBase* task = *(ctrltask->tasks_.at(arg_task_name));
      ret_gains = task->kp_;
      return true;
    }

    //Unidentified type.
    return false;
  }

  /** Returns the derivative gain of a given task in a controller.
   *
   * @param[out] ret_gains The variable into which the gains will be copied
   * @param[in]  arg_ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
   * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers.
   *
   * Note: If you modify code here, you will probably also
   * need to modify the same code in the other get gain functions. */
  sBool CRobot::getDerivativeGain(
      Eigen::VectorXd& ret_gains,
      const std::string& arg_ctrl_name,
      const std::string& arg_task_name)
  {
    SControllerBase* ctrl;
    if("" == arg_ctrl_name)
    { ctrl = data_.controller_current_; }
    else
    { ctrl = *data_.controllers_.at(arg_ctrl_name); }

    if(NULL == ctrl)
    { return false; }

    //Controllers may either be gc, in which case they have
    //only one set of gains.
    if("SControllerGc" == ctrl->getType())
    {
      SControllerGc* ctrlgc = dynamic_cast<SControllerGc*>(ctrl);
      ret_gains = ctrlgc->kv_;
      return true;
    }
    //Or they may be task controllers, in which case they have
    //multiple sets of gains, one for each task.
    else if("SControllerMultiTask" == ctrl->getType())
    {
      //Task name is required for task controllers
      if(""==arg_task_name)
      { return false; }
      SControllerMultiTask* ctrltask = dynamic_cast<SControllerMultiTask*>(ctrl);
      STaskBase* task = *(ctrltask->tasks_.at(arg_task_name));
      ret_gains = task->kv_;
      return true;
    }

    //Unidentified type.
    return false;
  }

  /** Returns the integral gain of a given task in a controller.
   *
   * @param[out] ret_gains The variable into which the gains will be copied
   * @param[in]  arg_ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
   * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers.
   *
   * Note: If you modify code here, you will probably also
   * need to modify the same code in the other get gain functions. */
  sBool CRobot::getIntegralGain(
      Eigen::VectorXd& ret_gains,
      const std::string& arg_ctrl_name,
      const std::string& arg_task_name)
  {
    SControllerBase* ctrl;
    if("" == arg_ctrl_name)
    { ctrl = data_.controller_current_; }
    else
    { ctrl = *data_.controllers_.at(arg_ctrl_name); }

    if(NULL == ctrl)
    { return false; }

    //Controllers may either be gc, in which case they have
    //only one set of gains.
    if("SControllerGc" == ctrl->getType())
    {
      SControllerGc* ctrlgc = dynamic_cast<SControllerGc*>(ctrl);
      ret_gains = ctrlgc->ki_;
      return true;
    }
    //Or they may be task controllers, in which case they have
    //multiple sets of gains, one for each task.
    else if("SControllerMultiTask" == ctrl->getType())
    {
      //Task name is required for task controllers
      if(""==arg_task_name)
      { return false; }
      SControllerMultiTask* ctrltask = dynamic_cast<SControllerMultiTask*>(ctrl);
      STaskBase* task = *(ctrltask->tasks_.at(arg_task_name));
      ret_gains = task->ki_;
      return true;
    }

    //Unidentified type.
    return false;
  }


  /** Sets the proportional gain of a given task in a controller.
   *
   * @param[in]  arg_gains The gains to be set
   * @param[in]  arg_ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
   * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers. */
  sBool CRobot::setProportionalGain(const Eigen::VectorXd& arg_kp,
      const std::string& arg_ctrl_name,
      const std::string& arg_task_name)
  {
    SControllerBase* ctrl;
    if("" == arg_ctrl_name)
    { ctrl = data_.controller_current_; }
    else
    { ctrl = *data_.controllers_.at(arg_ctrl_name); }

    if(NULL == ctrl)
    { return false; }

    //Controllers may either be gc, in which case they have
    //only one set of gains.
    if("SControllerGc" == ctrl->getType())
    {
      SControllerGc* ctrlgc = dynamic_cast<SControllerGc*>(ctrl);
      ctrlgc->kp_ = arg_kp;
      return true;
    }
    //Or they may be task controllers, in which case they have
    //multiple sets of gains, one for each task.
    else if("SControllerMultiTask" == ctrl->getType())
    {
      //Task name is required for task controllers
      if(""==arg_task_name)
      { return false; }
      SControllerMultiTask* ctrltask = dynamic_cast<SControllerMultiTask*>(ctrl);
      STaskBase* task = *(ctrltask->tasks_.at(arg_task_name));
      task->kp_ = arg_kp;
      return true;
    }

    //Unidentified type.
    return false;
  }

  /** Sets the derivative gain of a given task in a controller.
   *
   * @param[in]  arg_kv The gains to be set
   * @param[in]  arg_ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
   * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers. */
  sBool CRobot::setDerivativeGain(const Eigen::VectorXd& arg_kv,
      const std::string& arg_ctrl_name,
      const std::string& arg_task_name)
  {
    SControllerBase* ctrl;
    if("" == arg_ctrl_name)
    { ctrl = data_.controller_current_; }
    else
    { ctrl = *data_.controllers_.at(arg_ctrl_name); }

    if(NULL == ctrl)
    { return false; }

    //Controllers may either be gc, in which case they have
    //only one set of gains.
    if("SControllerGc" == ctrl->getType())
    {
      SControllerGc* ctrlgc = dynamic_cast<SControllerGc*>(ctrl);
      ctrlgc->kv_ = arg_kv;
      return true;
    }
    //Or they may be task controllers, in which case they have
    //multiple sets of gains, one for each task.
    else if("SControllerMultiTask" == ctrl->getType())
    {
      //Task name is required for task controllers
      if(""==arg_task_name)
      { return false; }
      SControllerMultiTask* ctrltask = dynamic_cast<SControllerMultiTask*>(ctrl);
      STaskBase* task = *(ctrltask->tasks_.at(arg_task_name));
      task->kv_ = arg_kv;
      return true;
    }

    //Unidentified type.
    return false;
  }

  /** Sets the derivative gain of a given task in a controller.
   *
   * @param[in]  arg_kv The gains to be set
   * @param[in]  arg_ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
   * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers. */
  sBool CRobot::setIntegralGain(const Eigen::VectorXd& arg_ki,
      const std::string& arg_ctrl_name,
      const std::string& arg_task_name)
  {
    SControllerBase* ctrl;
    if("" == arg_ctrl_name)
    { ctrl = data_.controller_current_; }
    else
    { ctrl = *data_.controllers_.at(arg_ctrl_name); }

    if(NULL == ctrl)
    { return false; }

    //Controllers may either be gc, in which case they have
    //only one set of gains.
    if("SControllerGc" == ctrl->getType())
    {
      SControllerGc* ctrlgc = dynamic_cast<SControllerGc*>(ctrl);
      ctrlgc->ki_ = arg_ki;
      return true;
    }
    //Or they may be task controllers, in which case they have
    //multiple sets of gains, one for each task.
    else if("SControllerMultiTask" == ctrl->getType())
    {
      //Task name is required for task controllers
      if(""==arg_task_name)
      { return false; }
      SControllerMultiTask* ctrltask = dynamic_cast<SControllerMultiTask*>(ctrl);
      STaskBase* task = *(ctrltask->tasks_.at(arg_task_name));
      task->ki_ = arg_ki;
      return true;
    }

    //Unidentified type.
    return false;
  }

  // **********************************************************************
  //                       Logging functions
  // **********************************************************************
  /** Sets up logging to a file */
  sBool CRobot::setLogFile(const std::string &arg_file)
  {
    if(log_file_name_ == arg_file)
    {//If it tries to open the same file, do nothing.
      if(log_file_.is_open())
      { return true;  }
    }
    else
    {
      //Once we open a new file, the logging must be reinitalized before it
      //can be turned on.
      logging_on_ = false;
      log_file_name_ = arg_file;
      //Close the file if it is open
      if(log_file_.is_open())
      { log_file_.close();  }
    }

    //Open the file in append mode
    log_file_.open(arg_file.c_str(), std::ios::out | std::ios::app);

    //Quick error check.
    if(log_file_.is_open())
    {
      logging_on_ = true;
      return true;
    }
    return false;
  }

  /** Logs data to the file */
  sBool CRobot::logState(bool arg_log_gc,  bool arg_log_gc_matrices,
      bool arg_log_task_matrices)
  {
    if(!logging_on_)  { return false; }

    bool logged_something=false;
    //Logs the time at the very least
    log_file_<<sutil::CSystemClock::getSysTime()
    <<" "<<sutil::CSystemClock::getSimTime();
    if(arg_log_gc)
    {
      log_file_<<"\nq "<<data_.io_data_->sensors_.q_.transpose()
                <<" "<<data_.io_data_->sensors_.dq_.transpose()
                <<" "<<data_.io_data_->sensors_.ddq_.transpose()
                <<" "<<data_.io_data_->actuators_.force_gc_commanded_.transpose();
      logged_something = true;
    }
    if(arg_log_gc_matrices)
    {
      log_file_<<"\nA "<<data_.controller_current_->gc_model_->M_gc_
          <<"\nAinv "<<data_.controller_current_->gc_model_->M_gc_inv_
          <<"\n"<<data_.controller_current_->gc_model_->force_gc_cc_.transpose()
          <<" "<<data_.controller_current_->gc_model_->force_gc_grav_.transpose();
      logged_something = true;
    }
    if(arg_log_task_matrices)
    {
      //Well be careful to ask for task logging only when you use a task
      //controller! Else the dynamic cast won't work.
      SControllerMultiTask* ds = dynamic_cast<SControllerMultiTask*>(data_.controller_current_);
      sutil::CMappedMultiLevelList<std::string, STaskBase*>::const_iterator it,ite;
      for(it = ds->servo_.task_data_->begin(), ite = ds->servo_.task_data_->end();
          it != ite; ++it)
      {
        log_file_<<"\nPri "<<(*it)->priority_
            <<" "<<(*it)->force_task_.transpose()
            <<" "<<(*it)->force_gc_.transpose()
            <<"\nJ "<<(*it)->J_
            <<"\nJinv "<<(*it)->J_dyn_inv_
            <<"\n L "<<(*it)->M_task_
            <<"\n Linv "<<(*it)->M_task_inv_
            <<"\n "<<(*it)->force_task_cc_
            <<" "<<(*it)->force_task_grav_;
        logged_something = true;
      }
    }
    return logged_something;
  }

  // **********************************************************************
  //                        Constructors etc.
  // **********************************************************************

  CRobot::CRobot()
  {
    db_ = S_NULL;
    dynamics_ = S_NULL;
    integrator_ = S_NULL;
    ctrl_current_ = S_NULL;
    logging_on_ = false;
  }

  //NOTE TODO : This guy shouldn't really do anything. Fix this sometime when
  //the application API is stable.
  CRobot::~CRobot()
  {
    if(dynamics_==integrator_)
    {
      if(S_NULL!=dynamics_){  delete dynamics_; dynamics_ = S_NULL; }
    }
    else
    {
      if(S_NULL!=dynamics_){  delete dynamics_; dynamics_ = S_NULL; }
      if(S_NULL!=integrator_) {  delete integrator_; integrator_ = S_NULL; }
    }

    //Close the log file if it is open.
    if(log_file_.is_open())
    { log_file_.close();  }
  }
}
