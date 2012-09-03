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
/* \file test_controller.cpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "test_controller.hpp"

#include <Eigen/Dense>

#include <sutil/CSystemClock.hpp>

#include <scl/DataTypes.hpp>

#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>

#include <scl/parser/lotusparser/CLotusParser.hpp>

#include <scl/control/data_structs/SControllerBase.hpp>
#include <scl/control/task/CServo.hpp>
#include <scl/control/task/CTaskController.hpp>
#include <scl/control/task/tasks/CTaskNULL.hpp>
#include <scl/control/task/tasks/COpPointTask.hpp>

#include <scl/util/DatabaseUtils.hpp>

//Tao Dynamics
#include <scl/dynamics/tao/CTaoDynamics.hpp>
#include <scl/dynamics/tao/CTaoRepCreator.hpp>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <cmath>
#include <stdio.h>
#include <math.h>

namespace scl_test
{
#define ROBDOF 150

  /**
   * Tests the performance of the control servo loop:
   *
   * Constructs a dummy 6dof robot with k tasks
   */
  void test_control_servo(int id)
  {
    //Dynamically allocated vars
    scl::SServo *servo_ds=S_NULL;

    scl::sUInt r_id=0;

    try
    {
      //0. Create vars
      //      long long i; //Counters
      //      long long imax; //Counter limits
      //      sClock t1,t2; //Clocks: pre and post
      //      bool flag;

      scl::CServo servo;

      //Test database
      scl::SDatabase * db = scl::CDatabase::getData();
      if(S_NULL==db) { throw(std::runtime_error("Database not initialized."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized database";  }
      std::cout<<std::flush;

      /* NOTE TODO : This test needs to be re-written
      //1. Create robot
      std::string robot_name="sclBot";
      scl::SRobotParsedData* rob_ds;
      // ************** Hard coded initialization for now.
      rob_ds = db->data_.s_parser_.robots_.create(robot_name);
      if(S_NULL==db){ throw(std::runtime_error("Could not create robot"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Created a robot on the pile";  }
      rob_ds->dof_ = ROBDOF;

      std::cout<<std::flush;

      //2. Init servo data structure
      servo_ds = new scl::SServo();
      if(S_NULL==servo_ds)
      { throw(std::runtime_error("Could not create servo data structure"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Created servo data structure";  }

      std::cout<<std::flush;

      //3. Add the servo data structure to the servo
//      bool init(const std::string &arg_robot,
//              SServo* arg_data);
      flag = servo.init(robot_name,servo_ds);
      //flag = servo.init(robot_name,servo_ds);
      if(false==flag) { throw(std::runtime_error("Failed to initialize servo."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized the servo";  }

      std::cout<<std::flush;

      //4. Create a task servo data structure and add it to the servo
      task_servo = new scl::STaskServo();
      // ********Hard coded initialization for now.
      task_servo->robot_ndof_ = ROBDOF;
      flag = servo.addTask(task_servo);
      task_servo->robot_ndof_ = 4;
      if(true==servo.addTask(task_servo))
      { throw(std::runtime_error("Servo added incorrectly initialized task"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Servo rejected bad task";  }

      task_servo->robot_ndof_ = ROBDOF;
      flag = servo.addTask(task_servo);
      if(false==flag) { throw(std::runtime_error("Servo failed to add good task"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Servo added good task";  }

      //5. Create a second task and add it to the servo
      task_servo2 = new scl::STaskServo();
      task_servo2->robot_ndof_ = ROBDOF;
      flag = servo.addTask(task_servo2);
      if(false==flag) { throw(std::runtime_error("Servo failed to add good task"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Servo added good task";  }

      //5. Set up the task matrices (Should normally come from the dynamics model)
      task_servo->range_space_.setIdentity(ROBDOF,ROBDOF); i =3;
      task_servo->range_space_(i,i) = 0; ++i;
      task_servo->range_space_(i,i) = 0; ++i;
      task_servo->range_space_(i,i) = 0;

      task_servo2->range_space_.setIdentity(ROBDOF,ROBDOF); i =0;
      task_servo2->range_space_(i,i) = 0; ++i;
      task_servo2->range_space_(i,i) = 0; ++i;
      task_servo2->range_space_(i,i) = 0;

      //6. Run the servo : Stress test
      task_servo->range_space_ = Eigen::Matrix<double,ROBDOF,ROBDOF>::Identity();
      task_servo2->range_space_ = Eigen::Matrix<double,ROBDOF,ROBDOF>::Random();

      Eigen::MatrixXd J1T, J2T; //Force dynamically allocated JacobianT (testing)
      //Randomize jacobians
      J1T = Eigen::Matrix<double,ROBDOF,6>::Random();
      J2T = Eigen::Matrix<double,ROBDOF,6>::Random();

      task_servo->force_gc_.setZero(task_servo->robot_ndof_);//Will be updated each servo loop
      task_servo2->force_gc_.setZero(task_servo2->robot_ndof_);//Will be updated each servo loop

      imax = 2000;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        task_servo->force_task_ = Eigen::Matrix<double,6,1>::Random();
        task_servo2->force_task_ = Eigen::Matrix<double,6,1>::Random();

        J1T = Eigen::Matrix<double,ROBDOF,6>::Random();
        J2T = Eigen::Matrix<double,ROBDOF,6>::Random();

        task_servo->force_gc_ = J1T*task_servo->force_task_;
        task_servo2->force_gc_ = J2T*task_servo2->force_task_;

        task_servo2->range_space_ = Eigen::Matrix<double,ROBDOF,ROBDOF>::Random();

        servo.computeControlForces();
      }
      t2 = sutil::CSystemClock::getSysTime();

      std::cout<<"\nTest Result ("<<r_id++<<")  Servo Stress. Dof="<<ROBDOF
          <<". Simulated real-world controller. "
          <<"\n\t Random {2nd task range space, Jacobians, 6D forces} at each tick."
          <<"\n\t Time for "<<imax<<" ticks: "<<t2-t1;

      //6. Run the servo : Controller simulation stress test
      //Do not set an identity matrix to identity -- It will segfault for some reason.
      //task_servo->range_space_ = Eigen::Matrix<double,ROBDOF,ROBDOF>::Identity();
      task_servo2->range_space_ = Eigen::Matrix<double,ROBDOF,ROBDOF>::Random();

      //Randomize jacobians
      J1T = Eigen::Matrix<double,ROBDOF,6>::Random();
      J2T = Eigen::Matrix<double,ROBDOF,6>::Random();

      task_servo->force_gc_.setZero(task_servo->robot_ndof_);//Will be updated each servo loop
      task_servo2->force_gc_.setZero(task_servo2->robot_ndof_);//Will be updated each servo loop

      imax = 30000;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        task_servo->force_task_ = Eigen::Matrix<double,6,1>::Random();
        task_servo2->force_task_ = Eigen::Matrix<double,6,1>::Random();

        task_servo->force_gc_ = J1T*task_servo->force_task_;
        task_servo2->force_gc_ = J2T*task_servo2->force_task_;

        servo.computeControlForces();
      }
      t2 = sutil::CSystemClock::getSysTime();

      std::cout<<"\nTest Result ("<<r_id++<<")  Servo Stress. Dof="<<ROBDOF
          <<". \n\t Random 6D forces each tick."
          <<"\n\t Time for "<<imax<<" ticks: "<<t2-t1;

      //7. Run the servo : Precision test
      //
      task_servo2->range_space_ = Eigen::Matrix<double,ROBDOF,ROBDOF>::Random();

      //Randomize jacobians
      J1T = Eigen::Matrix<double,ROBDOF,6>::Random();
      J2T = Eigen::Matrix<double,ROBDOF,6>::Random();

      task_servo->force_gc_.setZero(task_servo->robot_ndof_);//Will be updated each servo loop
      task_servo2->force_gc_.setZero(task_servo2->robot_ndof_);//Will be updated each servo loop

      imax = 30000;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        task_servo->force_task_ = Eigen::Matrix<double,6,1>::Random();
        task_servo2->force_task_ = Eigen::Matrix<double,6,1>::Random();

        task_servo->force_gc_ = J1T*task_servo->force_task_;
        task_servo2->force_gc_ = J2T*task_servo2->force_task_;

        servo.computeControlForces();
      }
      t2 = sutil::CSystemClock::getSysTime();

      std::cout<<"\nTest Result ("<<r_id++<<")  Servo Stress. Dof="<<ROBDOF
          <<". \n\t Random 6D forces each tick."
          <<"\n\t Time for "<<imax<<" ticks: "<<t2-t1;*/

      std::cout<<"\nTest #"<<id<<" : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" : Failed.";

      if(S_NULL!=servo_ds){ delete servo_ds;  }
    }
  }

  /**
   * Tests the performance of the controller's dynamics
   * engine:
   *
   * Reads in a toy robot specification and controls
   * it with full dynamics.
   */
  void test_controller_dynamics(int id, const std::string &file_name)
  {
    scl::CTaoDynamics * dynamics = S_NULL;
    scl::sUInt r_id=0;
    //    bool flag;
    try
    {
      //0. Create vars
      //      long long i; //Counters
      //      long long imax; //Counter limits
      //      sClock t1,t2; //Clocks: pre and post

      scl::CServo servo;

      //Test database
      scl::SDatabase * db = scl::CDatabase::getData();
      if(S_NULL==db)
      { throw(std::runtime_error("Database not initialized."));  }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized database"<<std::flush;  }

      /* NOTE TODO : This test needs to be re-written
      //0. Probe file for robots
      std::string tmp_infile;
      tmp_infile = scl::CDatabase::getData()->data_.cwd_+ file_name;
      std::cout<<"\nTest Result ("<<r_id++<<")  Opening file : "<<tmp_infile;

      scl_parser::CLotusParser tmp_lparser;
      std::vector<std::string> rob_names;
      flag = tmp_lparser.listRobotsInFile(tmp_infile,rob_names);
      if(false == flag)
      { throw(std::runtime_error("Could not read a list of robots from the file"));  }

      //1.a Read in a world
      flag = scl_registry::parseWorld(tmp_infile, &tmp_lparser);
      if(false == flag)
      { throw(std::runtime_error("Could not register world with the database"));  }

      //1.b Create robot from a file specification (And register it with the db)
      std::string robot_name;
      robot_name = rob_names.at(0);
      scl::SRobotParsedData* rob_ds;
      flag = scl_registry::parseRobot(tmp_infile, robot_name, &tmp_lparser);
      if(false == flag)
      { throw(std::runtime_error("Could not register robot with the database"));  }
      else
      {
        std::cout<<"\nTest Result ("<<r_id++<<")  Created a robot "
            <<robot_name<<" on the pile"<<std::flush;
      }

#ifdef DEBUG
      std::cout<<"\nPrinting parsed robot "
          <<db->data_.s_parser_.robots_.at(robot_name)->name_;
      scl_util::printRobotLinkTree(*( db->data_.s_parser_.robots_.at(robot_name)->robot_br_rep_.getRootNode()),0);
#endif

      //2. Pull out the robot's ds from the db
      rob_ds = db->data_.s_parser_.robots_.at(robot_name);
      if(S_NULL == rob_ds)
      { throw(std::runtime_error("Could not find robot in the database"));  }
      else
      {
        std::cout<<"\nTest Result ("<<r_id++<<")  Found robot "
            <<robot_name<<" on the pile"<<std::flush;
      }

      //Ok. Now we have a robot's dynamic specification sitting in
      //the database. And we have a handle to it. Lets move on.
      std::cout<<std::flush;

      //3. Initialize the controller's data struct
      std::string controller_name = "sclBotController";
      scl::SControllerBase* tmp_ctrl_ds;
      tmp_ctrl_ds = scl_registry::registerController(robot_name, controller_name);
      scl::STaskController* ctrl_ds=S_NULL;
      ctrl_ds = dynamic_cast<scl::STaskController*>(tmp_ctrl_ds);
      if(S_NULL == ctrl_ds)
      { throw(std::runtime_error("Could not create the controller's data struct on the pile"));  }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Created controller's data struct" <<std::flush;  }

      //Initialize the dynamics computational object
      dynamics = new scl::CTaoDynamics();
      if (S_NULL==dynamics)
      { throw(std::runtime_error("Failed to allocate memory for tao dynamics."));  }

      std::cout<<"\n"; //CTaoDynamics uses \n at the end convention.
      flag = dynamics->init(robot_name);
      if (false==flag) { throw(std::runtime_error("Failed to initialize tao dynamics."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized tao dynamics for the robot.";  }


      //4. Initialize the controller (initializes the tao tree within the init function).
      scl::CTaskController ctrl;
      flag = ctrl.init(ctrl_ds, dynamics);
      if(false == flag) { throw(std::runtime_error("Could not initialize the controller"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized the controller" <<std::flush;  }

      //5. Create a task and add it to the controller
      std::string task_name = "sclbot_task0";
      scl::CPileMap<std::string,scl::CTaskNULL> task_pile; //Mem mgt for null tasks
      scl::CTaskNULL *task;
      task = task_pile.create(task_name);
      if(0 > ctrl.addTask(task_name,(scl::CTaskBase*) task,0)) //Failed to add a NULL task if task-level < 0 (returned)
      { throw(std::runtime_error("Could not add a null task to the controller"));  }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Added a null task to the controller (does nothing)";  }

      imax = 200;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        ctrl.computeModel();
        ctrl.computeTaskTorques();
        ctrl.computeTorques();
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<") Executed "<<imax <<" iterations of a null task."
          <<"\n\tServo took time: "<<t2-t1<<std::flush;

      //6. Re-Initialize the controller (initializes the tao tree within the init function).
      ctrl.reset();
      flag = ctrl.init(ctrl_ds, dynamics);
      if(false == flag) { throw(std::runtime_error("Could not reset and reinitialize the controller"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Reset and reinitialized the controller" <<std::flush;  }

      //Get the Dynamics ds
      scl::STaskController* c_ds = db->data_.s_controller_.task_controllers_.at(controller_name);
      if(S_NULL == c_ds)
      { throw(std::runtime_error("Couldn't access the controller's data structure in the database")); }

      //7. Create a task and add it to the controller
      std::string task_name2 = "sclbot_optask";
      std::string task_link = "end-effector";
      Eigen::Vector3d pos_in_parent;
      Eigen::VectorXd max_acc, max_vel;
      pos_in_parent = Eigen::Vector3d::Zero();
      max_vel.setZero(6);max_vel<<0.1,0.1,0.1,0.1,0.1,0.1;
      max_acc.setZero(6);max_acc<<0.1,0.1,0.1,0.1,0.1,0.1;
      scl::CPileMap<std::string,scl::COpPointTask> task_pile2; //Mem mgt for null tasks

      scl::COpPointTask *task2;
      task2 = task_pile2.create(task_name);
      if(S_NULL == task2) { throw(std::runtime_error("Couldn't create a task data structure on the pile")); }

      //Initialize the task
      scl::STaskBase* task_data = c_ds->tasks_.create(task_name2);
      flag = task2->init(task_data,dynamics,
          task_link,pos_in_parent,max_vel,max_acc);
      if(false == flag) { throw(std::runtime_error("Couldn't initialize task object"));  }

      flag = task2->initServo(max_vel,max_vel,max_vel);
      if(false == flag) { throw(std::runtime_error("Couldn't initialize task servo"));  }

      flag = task2->initModel(&(c_ds->gc_model_));
      if(false == flag) { throw(std::runtime_error("Couldn't initialize task model"));  }

      //Finally, add the task to the controller
      if(0>ctrl.addTask(task_name2,(scl::CTaskBase*) task2,0))
      { throw(std::runtime_error("Couldn't add task to controller"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Added an operational point task to the controller" <<std::flush;  }

      Eigen::VectorXd goal;
      goal.setZero(6);
      goal<<0.1,0.1,0.1,0.1,0.1,0.1;

      imax = 200;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        c_ds->io_data_->sensors_.q_.cwise() += 0.1;
        task2->setGoal(goal);

        ctrl.computeModel();
        if((0==i)||(i==imax))
        {
          std::cout<<"\nA : \n" <<c_ds->gc_model_.A_;
          std::cout<<"\nAinv : \n" <<c_ds->gc_model_.Ainv_;
          std::cout<<"\nb : \n" <<c_ds->gc_model_.b_.transpose();
          std::cout<<"\ng : \n" <<c_ds->gc_model_.g_.transpose();
        }

        ctrl.computeTaskModels(0);
        if((0==i)||(i==imax))
        {
          std::cout<<"\nJ : \n" <<task_data->model_.jacobian_;
          std::cout<<"\nLambda : \n" <<task_data->model_.lambda_;
          std::cout<<"\nJdyninv : \n" <<task_data->model_.jacobian_dyn_inv_;
          std::cout<<"\nMu : \n" <<task_data->model_.mu_.transpose();
          std::cout<<"\np : \n" <<task_data->model_.p_.transpose();
          std::cout<<"\nN : \n" <<task_data->model_.null_space_;
        }

        ctrl.computeTaskTorques();
        if((0==i)||(i==imax))
        {
          std::cout<<"\nTask Forces : \n" <<task_data->servo_.force_task_.transpose();
          std::cout<<"\nTask Tau : \n" <<task_data->servo_.force_gc_.transpose();
          std::cout<<"\nTask R : \n" <<task_data->servo_.range_space_;
        }

        ctrl.computeTorques();
        if((0==i)||(i==imax))
        {
          std::cout<<"\nServo Torques:"<<c_ds->servo_.force_gc_.transpose();
        }
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<") Executed "<<imax<<" iterations of an operational space task"
          <<"\n\tServo+Model time: "<<t2-t1<<std::flush;

      //Now stress test the setup -- Only in release mode.
#ifndef DEBUG
      imax = 2000;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        c_ds->io_data_->sensors_.q_.cwise() += 0.1;
        task2->setGoal(goal);
        ctrl.computeModel();
        ctrl.computeTaskModels(0);
        ctrl.computeTaskTorques();
        ctrl.computeTorques();
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<") Stress Test : Executed "<<imax<<" iterations of an operational space task"
          <<"\n\tServo+Model time: "<<t2-t1<<std::flush;
#endif

      //Delete stuff
      if(S_NULL!= dynamics)  {  delete dynamics; dynamics = S_NULL; }*/

      std::cout<<"\nTest #"<<id<<" : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" : Failed.";

      if(S_NULL!= dynamics)  {  delete dynamics; dynamics = S_NULL; }
    }
  }

}
