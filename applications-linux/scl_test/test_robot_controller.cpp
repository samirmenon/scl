/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* \file test_robot_controller.cpp
 *
 *  Created on: Jul 25, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "test_robot_controller.hpp"


#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/control/data_structs/SControllerBase.hpp>
#include <scl/control/task/CServo.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/CTaskNULL.hpp>
#include <scl/control/task/tasks/CTaskOpPos.hpp>

#include "CSclAppTestTask.hpp"

#include <sutil/CSystemClock.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <cmath>
#include <stdio.h>
#include <math.h>

namespace scl_test
{
  /**
   * Tests the performance of the task controller
   * on the given robot specification:
   */
  void test_task_controller(int id, int argc, char** argv,
      const std::string & arg_file,
      const std::string & arg_robot_name,
      const std::string & arg_controller_name,
      const std::string & arg_op_link_name1,
      const std::string & arg_op_link_name2,
      const double arg_sim_dt,
      const bool arg_damping)
  {
    scl::sUInt r_id=0;
    bool flag;

    //Create the database and set the integration time-step
    scl::CDatabase::getData()->sim_dt_ = arg_sim_dt;

    try
    {
      //Test database
      scl::SDatabase * db = scl::CDatabase::getData();
      if(S_NULL==db)
      { throw(std::runtime_error("Database not initialized."));  }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized database"<<std::flush;  }

      //Initialize the controller application
      scl_test::CSclAppTestTask app;
      flag = app.init(argc,argv, arg_file, arg_robot_name, arg_controller_name,
          arg_op_link_name1, arg_op_link_name2);
      if(false==flag)
      { throw(std::runtime_error("Could not initialize the controller for the robot."));  }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized controller"<<std::flush;  }

      //NOTE TODO : Add code to test the controller's "accuracy" at following the trajectory.

      //Turn on the damping
      scl::SRobotParsed* tmp_rob = scl::CDatabase::getData()->s_parser_.robots_.at(arg_robot_name);
      tmp_rob->flag_apply_gc_damping_ = true;
      tmp_rob->damping_gc_*=0;
      tmp_rob->damping_gc_.array()+=0.05; //5% velocity loss per second

      //Test the controller application's performance
      double t1,t2;
      double max_sim_time = 10.1;//seconds
      double delta_sim_time = 1.0;//seconds
      double start_sim_time = 0.0;//seconds
      //1. Threaded + Simulation + Graphics
      for(double sim_time = start_sim_time; sim_time < max_sim_time; sim_time+=delta_sim_time)
      {
        t1 = sutil::CSystemClock::getSysTime();
        app.runMainLoopThreaded(sim_time);
        t2 = sutil::CSystemClock::getSysTime();
        std::cout<<"\nTest Result ("<<r_id++<<") Theaded+Sim+Gr. Sim time: "<<sim_time<<". Sys time: "<<t2-t1<<std::flush;
        sleep(1);
      }

      //2. Simulation + Graphics
      for(double sim_time = start_sim_time; sim_time < max_sim_time; sim_time+=delta_sim_time)
      {
        t1 = sutil::CSystemClock::getSysTime();
        app.runMainLoop(sim_time);
        t2 = sutil::CSystemClock::getSysTime();
        std::cout<<"\nTest Result ("<<r_id++<<") Sim+Gr.         Sim time: "<<sim_time<<". Sys time: "<<t2-t1<<std::flush;
        sleep(1);
      }

      //3. Simulation Only
      for(double sim_time = start_sim_time; sim_time < max_sim_time; sim_time+=delta_sim_time)
      {
        t1 = sutil::CSystemClock::getSysTime();
        app.runSimulation(sim_time);
        t2 = sutil::CSystemClock::getSysTime();
        std::cout<<"\nTest Result ("<<r_id++<<") Sim only.       Sim time: "<<sim_time<<". Sys time: "<<t2-t1<<std::flush;
        sleep(1);
      }

      //4. Controller Only
      for(double sim_time = start_sim_time; sim_time < max_sim_time; sim_time+=delta_sim_time)
      {
        t1 = app.runSimulationClockCtrl(sim_time);
        std::cout<<"\nTest Result ("<<r_id++<<") Ctrl only.      Sim time: "<<sim_time<<". Sys time: "<<t1<<std::flush;
        sleep(1);
      }

      app.terminate();
      std::cout<<"\nTest #"<<id<<" (Task Controller: "<<arg_robot_name<<", "<<arg_controller_name<<") : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" (Task Controller: "<<arg_robot_name<<", "<<arg_controller_name<<") : Failed.";
    }
  }

}
