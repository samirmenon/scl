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
/* \file muscle_gc_main.cpp
 *
 *  Created on: Jul 22, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
//scl lib
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/graphics/chai/CChaiGraphics.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/gc/CControllerGc.hpp>
#include <scl/parser/sclparser/CSclParser.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <scl/dynamics/tao/tao/dynamics/taoDNode.h>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/actuation/muscles/CActuatorSetMuscle.hpp>

#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cassert>

#include <omp.h>
#include <GL/freeglut.h>

const double SVD_THESHOLD = 0.0001;

/**
 * A sample application to demonstrate scl running one or more robots.
 *
 * Use it as a template to write your own (more detailed/beautiful/functional)
 * application.
 *
 * A simulation requires running 3 things:
 * 1. A dynamics/physics engine                  :  Tao
 * 2. A controller                               :  Scl
 * 3. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
 */
int main(int argc, char** argv)
{
  bool flag;
  if((argc < 2)&&(argc > 4))
  {
    std::cout<<"\nscl-robot demo application demonstrates how scl controls joint angles of single robots."
        <<"\nThe command line input is: ./<executable> <file_name.xml> <optional: robot name> <optional: controller name>\n";
    return 0;
  }
  else
  {
    try
    {
      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

      scl::SDatabase* db = scl::CDatabase::getData(); //Sanity Check
      if(S_NULL==db) { throw(std::runtime_error("Database not initialized"));  }

      //For parsing controllers
      flag = scl_registry::registerNativeDynamicTypes();
      if(false ==flag)  { throw(std::runtime_error("Could not register native dynamic types"));  }

      db->dir_specs_ = db->cwd_ + std::string("../../specs/"); //Set the specs dir so scl knows where the graphics are.

      //Get going..
      std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

      std::string tmp_infile(argv[1]);
      std::cout<<"Running scl benchmarks for input file: "<<tmp_infile;

      /******************************File Parsing************************************/
      scl_parser::CSclParser tmp_lparser;//Use the scl tinyxml parser

      std::string robot_name;
      if(argc<3)
      {//Find the robot specs in the file if one isn't specified by the user.
        std::vector<std::string> robot_names;
        flag = tmp_lparser.listRobotsInFile(tmp_infile,robot_names);
        if(false == flag) { throw(std::runtime_error("Could not read robot names from the file"));  }
        robot_name = robot_names[0];//Use the first available robot.
        std::cout<<"\nNo robot name passed. Using first robot by default: "<<robot_name;
      }
      else { robot_name = argv[2];}//If robot name was passed, use it.

      if(S_NULL == scl_registry::parseRobot(tmp_infile, robot_name, &tmp_lparser))
      { throw(std::runtime_error("Could not register robot with the database"));  }

      std::vector<std::string> graphics_names;
      flag = tmp_lparser.listGraphicsInFile(tmp_infile,graphics_names);
      if(false == flag) { throw(std::runtime_error("Could not list graphics names from the file"));  }

      if(S_NULL == scl_registry::parseGraphics(tmp_infile, graphics_names[0], &tmp_lparser))
      { throw(std::runtime_error("Could not register graphics with the database"));  }

      std::string ctrl_name; //Parse all the gc controllers for this robot!
      if(argc<4)
      {//Find the robot controller in the file if one isn't specified by the user.
        ctrl_name = "";

        std::vector<std::pair<std::string,std::string> > ctrl_names;//<name,type>
        flag = tmp_lparser.listControllersInFile(tmp_infile,ctrl_names);
        if(false == flag) { throw(std::runtime_error("Could not list controllers in the file"));  }

        //Find only the gc controller
        std::vector<std::pair<std::string,std::string> >::iterator itc, itce;
        for(itc = ctrl_names.begin(), itce = ctrl_names.end();itc!=itce;++itc)
        { if((*itc).second=="gc"){ ctrl_name = (*itc).first; break; } }

        if("" == ctrl_name)
        { throw(std::runtime_error("Could not find any gc controllers in the file"));  }

        std::cout<<"\nNo controller name passed. Using first controller by default: "<<ctrl_name;
      }
      else { ctrl_name = argv[3];}//If robot name was passed, use it.

      if(S_NULL == scl_registry::parseGcController(tmp_infile, robot_name, ctrl_name, &tmp_lparser))
      { throw(std::runtime_error("Could not register controller with the database"));  }

      scl::SRobotParsed *rob_ds = scl::CDatabase::getData()->s_parser_.robots_.at(robot_name);
      if(NULL == rob_ds)
      { throw(std::runtime_error("Could not find robot in database after parsing"));  }

#ifdef DEBUG
      std::cout<<"\nPrinting parsed robot "<<robot_name;
      scl_util::printRobotLinkTree(*( rob_ds->rb_tree_.getRootNode()),0);
#endif

      /**************************Initialize Tao Dynamics for Integrator********************************/
      scl::CTaoDynamics dyn_tao_int;
      flag = dyn_tao_int.init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
      if(false == flag) { throw(std::runtime_error("Could not initialize physics simulator"));  }

      /***************************Initialize Scl Dynamics for Algorithms********************/
      scl::CDynamicsScl dyn_scl; //Use for model updates.
      flag = dyn_scl.init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name)); //Reads stuff from the database.
      if(false == flag) { throw(std::runtime_error("Could not initialize dynamics object"));  }

      /******************************Shared I/O Data Structure************************************/
      scl::SRobotIO* rob_io_ds;
      rob_io_ds = db->s_io_.io_data_.at(robot_name);
      if(S_NULL == rob_io_ds)
      { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

      /**************************Initialize Robot Controller*******************/
      scl::SControllerGc * gc_ctrl_ds;
      gc_ctrl_ds = dynamic_cast<scl::SControllerGc*>(*(db->s_controller_.controllers_.at(ctrl_name)));
      if(S_NULL == gc_ctrl_ds) { throw(std::runtime_error("Could not find the controller in the database"));  }

      scl::CControllerGc robot_gc_ctrl;
      flag = robot_gc_ctrl.init(gc_ctrl_ds,&dyn_scl);
      if(false == flag) { throw(std::runtime_error("Could not initialize the controller object"));  }

      /**********************Initialize Muscle Actuator Model & Dynamics*******************/
      scl::CActuatorSetMuscle rob_mset;
      flag = rob_mset.init(rob_ds->muscle_system_.name_, /** parsed */ rob_ds, &(rob_ds->muscle_system_),
          /** rbd tree */ gc_ctrl_ds->gc_model_.rbdyn_tree_, /** dynamics */ &dyn_scl);
      if(false == flag) { throw(std::runtime_error("Could not initialize muscle actuator set"));  }

      // Create an actuator set in the database
      scl::SActuatorSetBase **pact = rob_io_ds->actuators_.actuator_sets_.create(rob_ds->muscle_system_.name_);
      *pact = rob_mset.getData();
      scl::SActuatorSetBase *act = *pact;
      act->force_actuator_.setZero(rob_mset.getNumberOfMuscles());

      // Run the compute Jacobian function once (resizes the matrix etc.).
      Eigen::MatrixXd rob_muscle_J, rob_muscle_Jpinv;
      flag = rob_mset.computeJacobian(rob_io_ds->sensors_.q_, rob_muscle_J);
      if(false == flag) { throw(std::runtime_error("Could not use muscle actuator set to compute a Jacobian"));  }

      // Set up an SVD to compute the inv to get muscle activation for gc control
      Eigen::JacobiSVD<Eigen::MatrixXd > rob_svd;
      // Singular value matrix for J'
      Eigen::MatrixXd rob_sing_val;
      rob_sing_val.setZero(rob_ds->dof_, rob_mset.getNumberOfMuscles()); //NOTE : Rectangular matrix

      // Compute svd to set up matrix sizes etc.
      rob_svd.compute(rob_muscle_J.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV | Eigen::ColPivHouseholderQRPreconditioner);
      for(unsigned int i=0;i<rob_ds->dof_;++i)
      {
        if(rob_svd.singularValues()(i)>SVD_THESHOLD)
        { rob_sing_val(i,i) = 1/rob_svd.singularValues()(i);  }
        else
        { rob_sing_val(i,i) = 0;  }
      }

      rob_muscle_Jpinv.setZero(rob_muscle_J.rows(), rob_muscle_J.cols());
      rob_muscle_Jpinv = rob_svd.matrixV() * rob_sing_val.transpose() * rob_svd.matrixU().transpose();

      // The muscle force vector
      act->force_actuator_.setZero(rob_ds->muscle_system_.muscles_.size());

      /******************************ChaiGlut Graphics************************************/
      glutInit(&argc, argv);

      scl::CChaiGraphics chai_gr;
      flag = chai_gr.initGraphics(graphics_names[0]);
      if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

      flag = chai_gr.addRobotToRender(robot_name);
      if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

      if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_names[0], &chai_gr))
      { throw(std::runtime_error("Glut initialization error")); }

      /******************************Main Loop************************************/
      std::cout<<std::flush;

      scl::sLongLong ctrl_ctr=0;//Controller computation counter
      scl::sLongLong gr_ctr=0;//Controller computation counter

      scl::sFloat t_start, t_end;

      //Simulation loop.
      std::cout<<"\nStarting simulation. Timestep : "<<db->sim_dt_<<std::flush;

#ifndef NOPARALLEL
      omp_set_num_threads(2);
      int thread_id;
      t_start = sutil::CSystemClock::getSysTime();
#pragma omp parallel private(thread_id)
      {
        thread_id = omp_get_thread_num();
        if(thread_id==1)
        {
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            sutil::CSystemClock::tick(db->sim_dt_);

            //1. Simulation Dynamics
            flag = dyn_tao_int.integrate((*rob_io_ds), scl::CDatabase::getData()->sim_dt_);
            //rob_io_ds->sensors_.dq_ -= rob_io_ds->sensors_.dq_ * (db->sim_dt_/100); //1% Velocity damping.

            //2. Update the controller
            for(unsigned int i=0; i< gc_ctrl_ds->robot_->dof_;i++)
            { gc_ctrl_ds->des_q_(i) = 0.2*sin(sutil::CSystemClock::getSysTime());  }

            //Slower dynamics update.
            if(ctrl_ctr%500 == 0)
            {
              robot_gc_ctrl.computeKinematics();
              robot_gc_ctrl.computeDynamics();
              rob_mset.computeJacobian(rob_io_ds->sensors_.q_, rob_muscle_J);

              // Compute svd to set up matrix sizes etc.
              rob_svd.compute(rob_muscle_J.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV | Eigen::ColPivHouseholderQRPreconditioner);
              for(unsigned int i=0;i<rob_ds->dof_;++i)
              {
                if(rob_svd.singularValues()(i)>SVD_THESHOLD)
                { rob_sing_val(i,i) = 1/rob_svd.singularValues()(i);  }
                else
                { rob_sing_val(i,i) = 0;  }
              }
              rob_muscle_Jpinv = rob_svd.matrixV() * rob_sing_val.transpose() * rob_svd.matrixU().transpose();

              act->force_actuator_ = rob_muscle_Jpinv*rob_io_ds->actuators_.force_gc_commanded_;

              if(ctrl_ctr%5000 == 0)
              {
                std::cout<<"\nJ':\n"<<rob_muscle_J.transpose();
                std::cout<<"\nFgc':"<<rob_io_ds->actuators_.force_gc_commanded_.transpose();
                std::cout<<"\nFm {";
                for (int j=0; j<rob_mset.getNumberOfMuscles(); j++)
                { std::cout<<rob_ds->muscle_system_.muscle_id_to_name_[j]<<", "; }
                std::cout<<"} : "<<act->force_actuator_.transpose();
              }
            }
            robot_gc_ctrl.computeControlForces();

            //Set the command torques for the simulator to the controller's computed torques
            rob_io_ds->actuators_.force_gc_commanded_ = gc_ctrl_ds->des_force_gc_;
            ctrl_ctr++;//Increment the counter for dynamics computed.
          }
        }
        else
        {
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            glutMainLoopEvent();
            gr_ctr++;
            const timespec ts = {0, 15000000};//Sleep for 15ms
            nanosleep(&ts,NULL);
          }
        }
      }
#else
      t_start = sutil::CSystemClock::getSysTime();
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        sutil::CSystemClock::tick(db->sim_dt_);

        assert(rob_io_ds == gc_ctrl_ds->io_data_);

        //1. Simulation Dynamics
        flag = dyn_tao_int.integrate((*rob_io_ds), scl::CDatabase::getData()->sim_dt_);
//        rob_io_ds->sensors_.dq_ -= rob_io_ds->sensors_.dq_ * (db->sim_dt_/100); //1% Velocity damping.

        //2. Update the controller
        for(unsigned int i=0; i< gc_ctrl_ds->robot_->dof_;i++)
        { gc_ctrl_ds->des_q_(i) = 0.5*sin(sutil::CSystemClock::getSysTime());  }

        //Slower dynamics update.
        if(ctrl_ctr%50 == 0)
        {
          robot_gc_ctrl.computeKinematics();
          robot_gc_ctrl.computeDynamics();
          rob_mset.computeJacobian(rob_io_ds->sensors_.q_, rob_muscle_J);

          // Compute svd to set up matrix sizes etc.
          rob_svd.compute(rob_muscle_J.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV | Eigen::ColPivHouseholderQRPreconditioner);
          for(unsigned int i=0;i<rob_ds->dof_;++i)
          {
            if(rob_svd.singularValues()(i)>SVD_THESHOLD)
            { rob_sing_val(i,i) = 1/rob_svd.singularValues()(i);  }
            else
            { rob_sing_val(i,i) = 0;  }
          }
          rob_muscle_Jpinv = rob_svd.matrixV() * rob_sing_val.transpose() * rob_svd.matrixU().transpose();

          act->force_actuator_ = rob_muscle_Jpinv * rob_io_ds->actuators_.force_gc_commanded_;

          if(ctrl_ctr%5000 == 0)
          {
#ifdef DEBUG
            rob_io_ds->printInfo();
#endif
            std::cout<<"\nJ':\n"<<rob_muscle_J.transpose();
            std::cout<<"\nFm {";
            for (int j=0; j<rob_mset.getNumberOfMuscles(); j++)
            { std::cout<<rob_ds->muscle_system_.muscle_id_to_name_[j]<<", "; }
            std::cout<<"} : "<<act->force_actuator_.transpose();
          }
        }
        robot_gc_ctrl.computeControlForces();

        //Set the command torques for the simulator to the controller's computed torques
        rob_io_ds->actuators_.force_gc_commanded_ = gc_ctrl_ds->des_force_gc_;
        ctrl_ctr++;//Increment the counter for dynamics computed.

        if(ctrl_ctr%50 == 0)
        {
          glutMainLoopEvent();
          gr_ctr++;
        }
      }
#endif

      t_end = sutil::CSystemClock::getSysTime();

      /****************************Print Collected Statistics*****************************/
      std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
      std::cout<<"\nSimulation Took Time : "<<t_end-t_start <<" sec";
      std::cout<<"\nReal World End Time  : "<<sutil::CSystemClock::getSysTime() <<" sec \n";
      std::cout<<"\nTotal Control Model and Servo Updates : "<<ctrl_ctr;
      std::cout<<"\nTotal Graphics Updates                : "<<gr_ctr;

      /****************************Deallocate Memory And Exit*****************************/
      flag = chai_gr.destroyGraphics();
      if(false == flag) { throw(std::runtime_error("Error deallocating graphics pointers")); } //Sanity check.

      std::cout<<"\nSCL : Executed Successfully";
      std::cout<<"\n*************************\n"<<std::flush;
      return 0;
    }
    catch(std::exception & e)
    {
      std::cout<<"\nEnd Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

      std::cout<<"\nSCL Failed: "<< e.what();
      std::cout<<"\n*************************\n";
      return 1;
    }
  }
}
