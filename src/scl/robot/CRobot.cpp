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
#include <scl/control/gc/CGcController.hpp>
#include <scl/control/task/CTaskController.hpp>

#include <sutil/CSystemClock.hpp>

namespace scl
{

  CRobot::CRobot()
  {
    dynamics_ = S_NULL;
    integrator_ = S_NULL;
    ctrl_current_ = S_NULL;
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

      SWorldParsedData* world = &(db_->s_parser_.world_);
      if(false == world->has_been_init_)
      { throw(std::runtime_error("World properties (gravity) not initialized"));  }
      SRobotParsedData *robot = db_->s_parser_.robots_.at(arg_robot_name);
      if(S_NULL == robot) { throw(std::runtime_error("Robot not parsed from file"));  }
      SRobotIOData* io_data = db_->s_io_.io_data_.at(arg_robot_name);
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
                  world, robot, io_data, ctrls);
      if(false == flag) { throw(std::runtime_error("Could not initialize robot.")); }
    }
    catch(std::exception & e)
    { std::cout<<"\nCRobot::init("<<arg_robot_name<<") Error: "<< e.what(); }

    return data_.has_been_init_;
  }


  sBool CRobot::init(std::string arg_robot_name,
      CDynamicsBase* arg_dynamics,
      CDynamicsBase* arg_integrator,
      SWorldParsedData *arg_world,
      SRobotParsedData *arg_robot,
      SRobotIOData *arg_io_data,
      std::vector<SControllerBase*>& arg_ctrls)
  {
    try
    {
      db_ = CDatabase::getData();
      if(S_NULL == db_) { throw(std::runtime_error("Database not initialized"));  }

      //Error checks.
      if(S_NULL == arg_dynamics) { throw(std::runtime_error("Passed NULL dynamics"));  }
      if(S_NULL == arg_integrator) { throw(std::runtime_error("Passed NULL integrator"));  }
      if(S_NULL == arg_world) { throw(std::runtime_error("Passed NULL world"));  }
      if(S_NULL == arg_robot) { throw(std::runtime_error("Passed NULL robot"));  }
      if(S_NULL == arg_io_data) { throw(std::runtime_error("Passed NULL io_data"));  }

      if(false == arg_dynamics->hasBeenInit())
      { throw(std::runtime_error("Passed unintialized dynamics"));  }
      if(false == arg_integrator->hasBeenInit())
      { throw(std::runtime_error("Passed unintialized integrator"));  }
      if(false == arg_world->has_been_init_)
      { throw(std::runtime_error("World properties (gravity) not initialized"));  }
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

      data_.parsed_world_data_ = arg_world;
      data_.name_ = arg_robot_name;
      data_.parsed_robot_data_ = arg_robot;
      data_.io_data_ = arg_io_data;
      data_.controllers_.clear();

      //Some defaults:
      //Damping = 1% energy loss per second
      data_.parsed_robot_data_->damping_.setConstant(data_.io_data_->dof_,db_->sim_dt_/100);
      //Force limits: 1000 N
      data_.parsed_robot_data_->max_actuator_forces_.setConstant(data_.io_data_->dof_,3000);
      data_.parsed_robot_data_->min_actuator_forces_.setConstant(data_.io_data_->dof_,-3000);

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
      if(arg_ctrl_ds->type_ctrl_ds_ == "SGcController")
      { tmp_ctrl = new CGcController();  }
      else if(arg_ctrl_ds->type_ctrl_ds_ == "STaskController")
      { tmp_ctrl = new CTaskController(); }
      else
      {
        std::string s("Unrecognized controller type : ");
        s = s + arg_ctrl_ds->type_ctrl_ds_;
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
      return false;
    }
  }

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

        if(data_.parsed_robot_data_->flag_apply_actuator_force_limits_) //Force limits controlled by a flag
        {
          for(sUInt i=0; i<data_.io_data_->dof_; i++)
          {
            if(data_.io_data_->actuators_.force_gc_commanded_(i) < data_.parsed_robot_data_->min_actuator_forces_(i))
              data_.io_data_->actuators_.force_gc_commanded_(i) = data_.parsed_robot_data_->min_actuator_forces_(i);
            else if(data_.io_data_->actuators_.force_gc_commanded_(i) > data_.parsed_robot_data_->max_actuator_forces_(i))
              data_.io_data_->actuators_.force_gc_commanded_(i) = data_.parsed_robot_data_->max_actuator_forces_(i);
          }
        }
      }
      else
      {//Controller off
        data_.io_data_->actuators_.force_gc_commanded_.setZero(data_.io_data_->dof_);
      }
    }
#ifdef W_TESTING
    assert(flag);
#endif
  }

  void CRobot::computeDynamics()
  {
    bool flag = true;
    if((S_NULL != ctrl_current_)
        && data_.has_been_init_
        && data_.parsed_robot_data_->flag_controller_on_)
    {
      flag = flag && ctrl_current_->computeDynamics();
    }
#ifdef W_TESTING
    assert(flag);
#endif
  }

  void CRobot::integrateDynamics()
  {
    bool flag=true;
    if((S_NULL != integrator_)
        && data_.has_been_init_)
    {
      flag = integrator_->integrate(*(data_.io_data_), CDatabase::getData()->sim_dt_);

      //NOTE TODO : DESIGN DECISION : Consider moving these into the integrator instead.
      if(data_.parsed_robot_data_->flag_apply_damping_)
      {
        data_.io_data_->sensors_.dq_.array() -=
            data_.io_data_->sensors_.dq_.array() * data_.parsed_robot_data_->damping_.array(); //1% Velocity damping.
      }

      /* Note: Most models' joint limits are not correct right now. Uncomment this after
       * fixing them:
      //Apply joint limits and collision.
      if(data_.parsed_robot_data_->flag_apply_joint_limits_)
      {
        for(int i=0; i< data_.io_data_->dof_;++i)
        {
          //NOTE TODO : Implement the joint limit parsing and fix q at the joint limits
          if(data_.io_data_->sensors_.q_(i) > data_.parsed_robot_data_->joint_limit_max_(i)+0.1)
          { data_.io_data_->sensors_.q_(i) = data_.parsed_robot_data_->joint_limit_max_(i); }
          if(data_.io_data_->sensors_.q_(i) < data_.parsed_robot_data_->joint_limit_min_(i)-0.1)
          { data_.io_data_->sensors_.q_(i) = data_.parsed_robot_data_->joint_limit_min_(i); }
          else
          { continue; }
          //Collision
          data_.io_data_->sensors_.dq_(i) = -data_.io_data_->sensors_.dq_(i) / 100;//99% energy loss
          data_.io_data_->sensors_.ddq_(i) = 0;
          //std::cout<<"\nCollided with joint limits: "<<data_.io_data_->sensors_.q_.transpose();
        }
      }*/

    }

#ifdef W_TESTING
    assert(flag);
#endif
  }

  sBool CRobot::hasBeenInit()
  { return data_.has_been_init_;  }


  /** Turn velocity damping on or off. Turning it on will
   * make the robot lose some (1% default) velocity each
   * second */
  void CRobot::setFlagApplyDamping(sBool arg_flag)
  { data_.parsed_robot_data_->flag_apply_damping_ = arg_flag;  }

  /** Turn the actuator limits on or off. Simulates physical
   * force limits of the actuators */
  void CRobot::setFlagApplyActuatorForceLimits(sBool arg_flag)
  { data_.parsed_robot_data_->flag_apply_actuator_force_limits_ = arg_flag;  }

  /** Turn the controller on or off. Controller sends zero
   * command gc forces if off. */
  void CRobot::setFlagControllerOn(sBool arg_flag)
  { data_.parsed_robot_data_->flag_controller_on_ = arg_flag;  }

  /** Sets the velocity damping for each joint */
  sBool CRobot::setDamping(Eigen::VectorXd arg_d)
  {
    sBool flag;
    if(static_cast<sUInt>(arg_d.size()) == data_.io_data_->dof_)
    { data_.parsed_robot_data_->damping_ = arg_d; flag = true;  }
    else { flag = false; }
    return flag;
  }

  /** Sets the actuator limits for each joint */
  sBool CRobot::setActuatorForceLimits(Eigen::VectorXd arg_max,Eigen::VectorXd arg_min)
  {
    sBool flag;
    if((static_cast<sUInt>(arg_max.size()) == data_.io_data_->dof_) &&
        (static_cast<sUInt>(arg_min.size()) == data_.io_data_->dof_))
    {
      data_.parsed_robot_data_->max_actuator_forces_ = arg_max;
      data_.parsed_robot_data_->min_actuator_forces_ = arg_min;
      flag = true;
    }
    else { flag = false; }
    return flag;
  }

  /** Selects the passed controller if it exists and has been initialized
   * Returns false if the controller doesn't exist*/
  sBool CRobot::setController(std::string arg_ctrl_name)
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
    {
      std::cout<<"\nCRobot::selectController() Error : "<< e.what();
      return false;
    }
  }

  /** Gets access to the current controller data structure */
  CControllerBase* CRobot::getCurrentController()
  {
    try
    {
      if(false == data_.has_been_init_)
      { throw(std::runtime_error("Robot not initialized"));}
      return ctrl_current_;
    }
    catch(std::exception & e)
    {
      std::cout<<"\nCRobot::getCurrentController() Error : "<< e.what();
      return S_NULL;
    }
  }

  /** Gets access to the current controller data structure */
  SControllerBase* CRobot::getController(std::string arg_ctrl_name)
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
    {
      std::cout<<"\nCRobot::getController("<<arg_ctrl_name<<") Error : "<< e.what();
      return S_NULL;
    }
  }

  /** Returns a pointer to the robot's data structure */
  SRobot* CRobot::getData()
  { return &data_;  }

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
      log_file_name_ = arg_file;
      //Close the file if it is open
      if(log_file_.is_open())
      { log_file_.close();  }
    }

    //Open the file in append mode
    log_file_.open(arg_file.c_str(), std::ios::out | std::ios::app);

    //Quick error check.
    if(log_file_.is_open())
    { return true;  }
    else
    { return false; }
  }

  /** Logs data to the file */
  sBool CRobot::logState(bool arg_log_gc,  bool arg_log_gc_matrices,
      bool arg_log_task_matrices)
  {
    //Logs the time at the very least
    log_file_<<sutil::CSystemClock::getSysTime()
          <<" "<<sutil::CSystemClock::getSimTime();
    if(arg_log_gc)
    {
      log_file_<<" "<<data_.io_data_->sensors_.q_
          <<" "<<data_.io_data_->sensors_.dq_
          <<" "<<data_.io_data_->sensors_.ddq_
          <<" "<<data_.io_data_->actuators_.force_gc_commanded_;
    }
    if(arg_log_gc_matrices)
    {
      log_file_<<" "<<data_.controller_current_->gc_model_.A_
          <<" "<<data_.controller_current_->gc_model_.Ainv_
          <<" "<<data_.controller_current_->gc_model_.b_
          <<" "<<data_.controller_current_->gc_model_.g_;
    }
    if(arg_log_task_matrices)
    {
      //Well be careful to ask for task logging only when you use a task
      //controller! Else the dynamic cast won't work.
      STaskController* ds = dynamic_cast<STaskController*>(data_.controller_current_);
      sutil::CMappedMultiLevelList<std::string, STaskBase*>::const_iterator it,ite;
      for(it = ds->servo_.task_data_->begin(), ite = ds->servo_.task_data_->end();
          it != ite; ++it)
      {
        log_file_<<" "<<(*it)->priority_
            <<" "<<(*it)->force_task_
            <<" "<<(*it)->force_gc_
            <<" "<<(*it)->jacobian_
            <<" "<<(*it)->jacobian_dyn_inv_
            <<" "<<(*it)->lambda_
            <<" "<<(*it)->lambda_inv_
            <<" "<<(*it)->mu_
            <<" "<<(*it)->p_;
      }
    }
  }

}
