/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* \file main.cpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//Random system queries
#include "test_random_stuff.hpp"
//Robot parser base tests
#include "test_scl_parser.hpp"
//Serialization tests
#include "test_serialization_json.hpp"
//Matrix and lin-alg test
#include "test_math.hpp"
//Controller tests
#include "test_controller.hpp"
#include "test_controller2.hpp"
#include "test_robot_controller.hpp"
//Test Scl spatial dynamics engine
#include "test_dynamics.hpp"
//Test scl dynamics engine
#include "test_dynamics_scl.hpp"
//Test scl spatial dynamics engine math helper functions
#include "test_dynamics_sclspatial_math.hpp"
//Test scl spatial dynamics engine
#include "test_dynamics_sclspatial.hpp"
//Test chai graphic rendering
#include "test_graphics.hpp"

#include <scl/Singletons.hpp>

#include <sutil/CRegisteredDynamicTypes.hpp>
#include <sutil/CSystemClock.hpp>

#include <stdexcept>
#include <iostream>

using namespace std;
using namespace scl_test;

int main(int argc, char** argv)
{
  srand(time(0));
  int tid, id = 1;
  if(argc != 2)
  {
    cout<<"\nThe command line input is: ./<executable> <test_id>"
        <<"\n0 : Run all tests";
  }
  else
  {
    tid = atoi(argv[1]);
    cout<<"\nRunning scl tests for case: "<<tid;
    if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }
    cout<<"\nStarting tests. Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

    if((tid==0)||(tid==id))
    {//Test Random stuff
      std::cout<<"\n\nTest #"<<id<<". Random system tests [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      scl_test::test_random_stuff(id);
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Scl XML Parser
      std::cout<<"\n\nTest #"<<id<<". Scl XML Parser [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      scl_test::test_scl_parser(id);
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Scl JSON Serialization/deserialization
      std::cout<<"\n\nTest #"<<id<<". Scl JSON Serialization [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      scl_test::test_serialization_json(id);
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Matrix speeds
      std::cout<<"\n\nTest #"<<id<<". Matrix Speeds [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      scl_test::test_matrix_libs(id);
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Control Servo
      std::cout<<"\n\nTest #"<<id<<". Controller [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      scl_test::test_control_servo(id);
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Analytic vs. Scl spatial Dynamics
      std::cout<<"\n\nTest #"<<id<<". Analytic vs. Scl spatial Dynamics [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      scl_test::test_dynamics_scl_sp_vs_analytic_rpp(id);
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Analytic vs. Scl spatial Dynamics
      std::cout<<"\n\nTest #"<<id<<". Analytic vs. Scl Dynamics [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      scl_test::test_dynamics_scl_vs_analytic_rpp(id);
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Scl Spatial Dynamics Math helper functions
      std::cout<<"\n\nTest #"<<id<<". Scl Spatial Dynamics Math [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      scl_test::test_dynamics_sclspatial_math(id);
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Scl Spatial Dynamics functions
      std::cout<<"\n\nTest #"<<id<<". Scl Spatial Dynamics [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      scl_test::test_dynamics_sclspatial(id);
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Controller for robots : Controller V2
      std::cout<<"\n\nTest #"<<id<<". Controller2 [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime() <<" "
          <<sutil::CSystemClock::getSimTime() <<"]";
      scl_test::test_controller2(id);
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Controller for robots : Puma
      std::cout<<"\n\nTest #"<<id<<". Controller [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime() <<" "
          <<sutil::CSystemClock::getSimTime() <<"]";
      scl_test::test_task_controller(id,argc,argv,"../../specs/Puma/PumaCfg.xml",
          "PumaBot","opc","hand","null");
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Controller for robots : Pr2
      std::cout<<"\n\nTest #"<<id<<". Controller [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime() <<" "
          <<sutil::CSystemClock::getSimTime() <<"]";
      scl_test::test_task_controller(id,argc,argv,"../../specs/Pr2/Pr2Cfg.xml",
          "Pr2Bot","opc","hand","hand2");
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Controller for robots : Pr2 (faster integration timestep+damping)
      std::cout<<"\n\nTest #"<<id<<". Controller [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime() <<" "
          <<sutil::CSystemClock::getSimTime() <<"]";
      scl_test::test_task_controller(id,argc,argv,"../../specs/Pr2/Pr2Cfg.xml",
          "Pr2Bot","opc","hand","hand2",0.001,true);
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;

    if((tid==0)||(tid==id))
    {//Test Graphics
      std::cout<<"\n\nTest #"<<id<<". Chai graphics [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()<<" "
          <<sutil::CSystemClock::getSimTime()<<"]";
      std::string file = "../../specs/Pr2/Pr2Cfg.xml";
      scl_test::test_graphics(id,file,argc, argv);
      scl::CDatabase::resetData(); sutil::CRegisteredDynamicTypes<std::string>::resetDynamicTypes();
    }
    ++id;


    /**** Under development
    if((tid==0)||(tid==99))
    {//Test Haptics
      std::cout<<"\n\nTest #"<<id<<". Chai haptics [Sys time, Sim time :"
          <<sutil::CSystemClock::getSysTime()
          <<" "
          <<sutil::CSystemClock::getSimTime()
          <<"]";
      std::string file = "../../specs/Puma/PumaCfg.xml";
      scl_test::test_haptics(id,file,argc, argv);
    }
    ++id;*/
  }

  printf("\n\nEnding SCL Tests\n\n");
  return 0;
}
