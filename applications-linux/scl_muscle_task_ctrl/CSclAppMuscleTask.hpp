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
/* \file CSclAppMuscleTask.hpp
 *
 *  Created on: Oct 30, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
/*
 * Setup steps for a standard 1-robot simulation.
 */

#ifndef CSCL_APP_MUSCLETASK_HPP_
#define CSCL_APP_MUSCLETASK_HPP_

//Standard includes
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/Init.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/actuation/muscles/CActuatorSetMuscle.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/robot/CRobot.hpp>
#include <sutil/CSystemClock.hpp>
#include <scl/util/HelperFunctions.hpp>

#include <omp.h>
#include <GL/freeglut.h>

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <string>

//User modified includes to suit your application
#include <scl/control/task/tasks/CTaskOpPos.hpp>

#define SCL_TASK_APP_MAX_MARKERS_TO_ADD 500
const double SVD_THESHOLD = 0.001;

namespace scl_app
{
  /* Generic code required to run a scl simulation
   * A simulation requires running 3 things. This example uses:
   * 1. A dynamics/physics engine                  :  Tao
   * 2. A controller                               :  Scl
   * 3. A graphic rendering+interaction interface  :  Chai3d + FreeGlut
   */
  class CSclAppMuscleTask
  {
  public:
    /************************************************************************/
    /***********NOTE : You can modify this to suit your application *********/
    /************************************************************************/
    scl::CControllerMultiTask* ctrl;           //Use a task controller

    std::string op_link_name,op_link2_name;
    scl::CTaskOpPos* tsk, *tsk2;
    scl::STaskOpPos* tsk_ds, *tsk2_ds;
    scl::sBool op_link_set, op_link2_set;

    /** Implement this function. Else you will get a linker error. */
    inline void stepMySimulation();

    /** Implement this function. Else you will get a linker error. */
    scl::sBool initMyController(int argc, char** argv);

    /** Default constructor. Sets stuff to zero. */
    CSclAppMuscleTask()
    {
      db_ = S_NULL;
      rob_io_ds_ = S_NULL;
      rob_ds_ = S_NULL;
      gc_model_ = S_NULL;
      dyn_tao_ = S_NULL;
      dyn_scl_ = S_NULL;

      ctrl = S_NULL;           //Use a task controller
      op_link_name = "not_init";
      op_link2_name = "not_init";
      tsk = S_NULL; tsk2 = S_NULL;;
      tsk_ds = S_NULL; tsk2_ds = S_NULL;
      op_link_set = false; op_link2_set = false;
      act_ = S_NULL;

      ctrl_ctr_ = 0;
      gr_ctr_ = 0;
      t_start_ = 0.0;
      t_end_ = 0.0;
      traj_markers_added_so_far_ = 0;
    }

    /** Destructor: Cleans up */
    virtual ~CSclAppMuscleTask() { }

    /************************************************************************/
    /****************NOTE : You should NOT need to modify this **************/
    /****************       But feel free to use the objects   **************/
    /************************************************************************/
  public:
    /** Initialize the basic global variables, database etc.*/
    scl::sBool init(int argc, char** argv);

    /** Terminates the simulation and prints some statistics */
    void terminate();

    /** Runs a simulation using two threads:
     * Thread 1: Computes the robot dynamics
     * Thread 2: Renders the graphics and handles gui interaction */
    void runMainLoopThreaded();

    /** Runs a simulation using one thread.
     * 1: Computes the robot dynamics
     * 2: Renders the graphics and handles gui interaction */
    void runMainLoop();

    //Data types. Feel free to use them.
    scl::SDatabase* db_;                 //Generic database (for sharing data)

    std::vector<std::string> robots_parsed_;   //Parsed robots
    std::vector<std::string> graphics_parsed_; //Parsed graphics views

    std::string robot_name_;             //Currently selected robot
    std::string ctrl_name_;              //Currently selected controller

    scl::CRobot robot_;                  //Generic robot
    scl::SRobotIO* rob_io_ds_;           //Access the robot's sensors and actuators
    scl::SRobotParsed *rob_ds_;           //The robot's parsed data structure
    scl::SGcModel *gc_model_;            //The controller data struct

    scl::CDynamicsTao* dyn_tao_;          //Generic tao dynamics
    scl::CDynamicsScl* dyn_scl_;          //Generic tao dynamics
    scl::CGraphicsChai chai_gr_;         //Generic chai graphics

    scl::CActuatorSetMuscle rob_mset_;   //Muscle actuator set
    Eigen::VectorXd fmuscle_in_range_;   //The muscular force in the Jm range space.
    Eigen::VectorXd fmuscle_in_null_;    //The muscular force in the Jm range space.
    Eigen::MatrixXd rob_muscle_J_, rob_muscle_Jpinv_; //Pseudoinv
    Eigen::MatrixXd rob_mnull_matrix_;   //For Muscle null space svd
    Eigen::JacobiSVD<Eigen::MatrixXd > rob_svd_, rob_mnull_svd_;     //SVD
    Eigen::MatrixXd rob_sing_val_;         // Singular value matrix for J'
    scl::SActuatorSetMuscle *act_;         //The muscle set

    scl::sLongLong ctrl_ctr_;            //Controller computation counter
    scl::sLongLong gr_ctr_;              //Controller computation counter
    scl::sFloat t_start_, t_end_;        //Start and end times

    chai3d::cGenericObject* traj_markers_[SCL_TASK_APP_MAX_MARKERS_TO_ADD];
    int traj_markers_added_so_far_;
  };


  /************************************************************************/
  /****************NOTE : You should NOT need to modify this **************/
  /****************       All the function implementations   **************/
  /************************************************************************/

  scl::sBool CSclAppMuscleTask::init(int argc, char** argv)
  {
    bool flag;
    if((argc != 5)&&(argc != 6))
    {
      std::cout<<"\nDemo application demonstrates operational space task control."
          <<"\nThe command line input is: ./<executable> <file_name.xml> <robot_name> <controller_name> <operational task point> <optional : operational task point2>";
      return false;
    }
    else
    {
      try
      {
        /******************************Initialization************************************/
        //1. Initialize the database and clock.
        if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

        db_ = scl::CDatabase::getData(); //Sanity Check
        if(S_NULL==db_) { throw(std::runtime_error("Database not initialized"));  }

        db_->dir_specs_ = db_->cwd_ + std::string("../../specs/"); //Set the specs dir so scl knows where the graphics are.

        //For parsing controllers
        flag = scl::init::registerNativeDynamicTypes();
        if(false ==flag)  { throw(std::runtime_error("Could not register native dynamic types"));  }

        //Get going..
        std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

        std::string tmp_infile(argv[1]);
        std::cout<<"Running scl task controller for input file: "<<tmp_infile;

        /******************************File Parsing************************************/
        scl::CParserScl tmp_lparser;//Use the scl tinyxml parser
        flag = scl_registry::parseEverythingInFile(tmp_infile,
            &tmp_lparser,&robots_parsed_,&graphics_parsed_);
        if((false == flag) || (robots_parsed_.size()<=0)
            || (graphics_parsed_.size()<=0) )
        { throw(std::runtime_error("Could not parse the file"));  }

        robot_name_ = argv[2];
        if(!scl_util::isStringInVector(robot_name_,robots_parsed_))
        { throw(std::runtime_error("Could not find passed robot name in file"));  }

        rob_ds_ = scl::CDatabase::getData()->s_parser_.robots_.at(robot_name_);
        if(NULL == rob_ds_)
        { throw(std::runtime_error("Could not find robot in database after parsing"));  }

        /******************************TaoDynamics************************************/
        dyn_tao_ = new scl::CDynamicsTao();
        flag = dyn_tao_->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name_));
        if(false == flag) { throw(std::runtime_error("Could not initialize physics simulator"));  }

        /******************************Scl Dynamics************************************/
        dyn_scl_ = new scl::CDynamicsScl();
        flag = dyn_scl_->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name_));
        if(false == flag) { throw(std::runtime_error("Could not initialize dynamics algorithms"));  }

        /******************************Shared I/O Data Structure************************************/
        rob_io_ds_ = db_->s_io_.io_data_.at(robot_name_);
        if(S_NULL == rob_io_ds_)
        { throw(std::runtime_error("Robot I/O data structure does not exist in the database"));  }

        /**********************Initialize Robot Dynamics and Controller*******************/
        flag = robot_.initFromDb(robot_name_,dyn_scl_,dyn_tao_);//Note: The robot deletes these pointers.
        if(false == flag) { throw(std::runtime_error("Could not initialize robot"));  }

        ctrl_name_ = argv[3];
        flag = robot_.setControllerCurrent(ctrl_name_);
        if(false == flag) { throw(std::runtime_error("Could not initialize robot's controller"));  }

        /**********************Initialize Muscle Actuator Model & Dynamics*******************/
        gc_model_ = robot_.getControllerDataStruct(ctrl_name_)->gc_model_;
        flag = rob_mset_.init(rob_ds_->muscle_system_.name_, /** parsed */ rob_ds_, &(rob_ds_->muscle_system_),
            /** rbd tree */ gc_model_->rbdyn_tree_, /** dynamics */ dyn_scl_);
        if(false == flag) { throw(std::runtime_error("Could not initialize muscle actuator set"));  }

        // Create an actuator set in the database
        scl::SActuatorSetBase **pact = rob_io_ds_->actuators_.actuator_sets_.create(rob_ds_->muscle_system_.name_);
        *pact = rob_mset_.getData();
        act_ = rob_mset_.getData();
        act_->force_actuator_.setZero(rob_mset_.getNumberOfMuscles());
        // Set up muscle range space vector size
        fmuscle_in_range_.setZero(rob_mset_.getNumberOfMuscles());
        fmuscle_in_null_.setZero(rob_mset_.getNumberOfMuscles());

        // Run the compute Jacobian function once (resizes the matrix etc.).
        flag = rob_mset_.computeJacobian(rob_io_ds_->sensors_.q_, rob_muscle_J_);
        if(false == flag) { throw(std::runtime_error("Could not use muscle actuator set to compute a Jacobian"));  }

        // Set up an SVD to compute the inv to get muscle activation for gc control
        rob_sing_val_.setZero(rob_ds_->dof_, rob_mset_.getNumberOfMuscles()); //NOTE : Rectangular matrix

        // Compute svd to set up matrix sizes etc.
        rob_svd_.compute(rob_muscle_J_.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV | Eigen::ColPivHouseholderQRPreconditioner);
        for(unsigned int i=0;i<rob_ds_->dof_;++i)
        {
          if(rob_svd_.singularValues()(i)>SVD_THESHOLD)
          { rob_sing_val_(i,i) = 1/rob_svd_.singularValues()(i);  }
          else
          { rob_sing_val_(i,i) = 0;  }
        }

        rob_mnull_matrix_.setIdentity(rob_mset_.getNumberOfMuscles(),rob_mset_.getNumberOfMuscles());
        // Just set the svd inner data structure sizes
        rob_mnull_svd_.compute(rob_mnull_matrix_, Eigen::ComputeThinU | Eigen::ComputeThinV);

        rob_muscle_Jpinv_.setZero(rob_muscle_J_.rows(), rob_muscle_J_.cols());
        rob_muscle_Jpinv_ = rob_svd_.matrixV() * rob_sing_val_.transpose() * rob_svd_.matrixU().transpose();

        // The muscle force vector
        act_->force_actuator_.setZero(rob_ds_->muscle_system_.muscles_.size());

        /******************************ChaiGlut Graphics************************************/
        if(!db_->s_gui_.glut_initialized_)
        {
          glutInit(&argc, argv);
          db_->s_gui_.glut_initialized_ = true;
        }

        scl::SGraphicsParsed *gr_parsed = db_->s_parser_.graphics_worlds_.at(graphics_parsed_[0]);
        scl::SGraphicsChai *chai_ds = db_->s_gui_.chai_data_.at(graphics_parsed_[0]);
        flag = chai_gr_.initGraphics(gr_parsed,chai_ds);
        if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

        flag = chai_gr_.addRobotToRender(rob_ds_,rob_io_ds_);
        if(false==flag) { throw(std::runtime_error("Couldn't add robot to the chai rendering object")); }

        if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_parsed_[0], &chai_gr_))
        { throw(std::runtime_error("Glut initialization error")); }

        /**********************Initialize Single Control Task *******************/
        // NOTE : This MUST come after chai is initialized
        flag = initMyController(argc,argv);
        if(false == flag)
        { throw(std::runtime_error("Could not initialize user's custom controller"));  }

        /**************************** Reset counters ***************************/
        ctrl_ctr_=0;//Controller computation counter
        gr_ctr_=0;//Controller computation counter
        traj_markers_added_so_far_=0;

        //Simulation loop.
        std::cout<<"\nStarting simulation. Integration timestep: "<<db_->sim_dt_<<std::flush;

        return true;
      }
      catch(std::exception & e)
      {
        std::cout<<"\nCSclAppMuscleTask::setup() Failed : "<< e.what();
        terminate();
        return false;
      }
    }
  }

  void CSclAppMuscleTask::terminate()
  {
    /****************************Print Collected Statistics*****************************/
    std::cout<<"\nTotal Simulated Time : "<<sutil::CSystemClock::getSimTime() <<" sec";
    std::cout<<"\nTotal Control Model and Servo Updates : "<<ctrl_ctr_;
    std::cout<<"\nTotal Graphics Updates                : "<<gr_ctr_;

    /******************************Termination************************************/
    bool flag = chai_gr_.destroyGraphics();
    if(false == flag) { std::cout<<"\nError deallocating graphics pointers"; } //Sanity check.

    std::cout<<"\nEnd Time:"<<sutil::CSystemClock::getSysTime();
    std::cout<<"\n********************************************\n"<<std::flush;
  }

  void CSclAppMuscleTask::runMainLoopThreaded()
  {
    omp_set_num_threads(2);
    int thread_id;

#pragma omp parallel private(thread_id)
    {//Start threaded region
      thread_id = omp_get_thread_num();
      if(thread_id==1)
      {
        //Thread 1 : Run the simulation
        while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
        {
          if(!scl::CDatabase::getData()->pause_ctrl_dyn_)
          { stepMySimulation(); }
          //If paused, but step required, step it and set step flag to false.
          else if(scl::CDatabase::getData()->step_ctrl_dyn_)
          {
            scl::CDatabase::getData()->step_ctrl_dyn_ = false;
            stepMySimulation();
          }
          else
          {//Paused and no step required. Sleep for a bit.
            const timespec ts = {0, 15000000};//Sleep for 15ms
            nanosleep(&ts,NULL);
          }
        }
      }
      else
      {
        //Thread 2 : Run the graphics and gui
        while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
        {
          const timespec ts = {0, 15000000};//Sleep for 15ms
          nanosleep(&ts,NULL);
          glutMainLoopEvent(); //Update the graphics
          gr_ctr_++;
        }
      }
    }//End of threaded region
  }

  void CSclAppMuscleTask::runMainLoop()
  {
    while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
    {
      //If not paused, step the simulation.
      if(!scl::CDatabase::getData()->pause_ctrl_dyn_)
      { stepMySimulation(); }
      //If paused, but step required, step it and set step flag to false.
      else if(scl::CDatabase::getData()->step_ctrl_dyn_)
      {
        scl::CDatabase::getData()->step_ctrl_dyn_ = false;
        stepMySimulation();
      }

      static int gr_skip_ctr=0;
      if(gr_skip_ctr<=500)
      { gr_skip_ctr++; continue; }
      gr_skip_ctr = 0;
      glutMainLoopEvent(); //Update the graphics
      gr_ctr_++;
    }
  }

}
#endif /* CSCL_APP_TASK_HPP_ */
