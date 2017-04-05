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
/* \file test_controller2.cpp
 *
 *  Created on: Apr 02, 2017
 *
 *  Copyright (C) 2017
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "test_robot_controller.hpp"

// Just to make sure we don't have conflicts..
#include <scl/scl.hpp>
// We're testing the second version of the controller
#include <scl/control/tasks/AllHeaders.hpp>

namespace scl_test
{
  /**
   * Tests the performance of the task controller
   * on the given robot specification:
   */
  void test_controller2(int id)
  {
    scl::sUInt r_id=0;
    bool flag=true;

    try
    {
      //We'll just go with the Puma for now..
      scl::CParserScl p;
      scl::SRobotParsed rds;
      scl::SGcModel rgcm;

      // ********************** ROBOT PARSER TESTING **********************
      flag = flag && p.readRobotFromFile("../../specs/Puma/PumaCfg.xml", "../../specs/", "PumaBot", rds);
      flag = flag && rgcm.init(rds);
      //Test code.
      if(false==flag) { throw(std::runtime_error("Could not parse robot from file."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Parsed robot from file."<<std::flush;  }

      // ********************** CONTROL TASK DS TESTING **********************
      // Let's test a control task..
      scl::tasks::STaskOpPos t_op_ds;
      sutil::CMappedList<std::string, std::string> params;

      // As a sanity check, we'll just test the first key.
      std::string *tt;
      tt = params.create(std::string("name"),std::string("hand"));
      if(NULL == tt)
      { throw(std::runtime_error("Could not create name:hand key in mapped list."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Created key value pair name:"<<*tt<<". Testing recall from list for val: "<<*params.at_const("name")<<std::flush;  }

      params.create(std::string("type"),std::string("TaskOpPos"));
      params.create(std::string("priority"),std::string("0"));
      params.create(std::string("task_dof"),std::string("3"));
      params.create(std::string("kp"),std::string("10"));
      params.create(std::string("kv"),std::string("3"));
      params.create(std::string("ka"),std::string("0"));
      params.create(std::string("ki"),std::string("0"));
      params.create(std::string("ftask_max"),std::string("10"));
      params.create(std::string("ftask_min"),std::string("-10"));
      params.create(std::string("parent_link"),std::string("end-effector"));
      params.create(std::string("pos_in_parent"),std::string("0.01 0.00 0.00"));
      params.create(std::string("flag_compute_op_gravity"),std::string("true"));
      params.create(std::string("flag_compute_op_cc_forces"),std::string("false"));
      params.create(std::string("flag_compute_op_inertia"),std::string("true"));

      flag = t_op_ds.init(params,&rds);
      //Test code.
      if(false==flag) { throw(std::runtime_error("Could not init op task data from the parsed params."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Init op task data from the parsed params."<<std::flush;  }

      // ********************** CONTROL TASK TESTING **********************
      scl::tasks::CTaskOpPos t_op;

      flag = t_op.init(params);
      if(false==flag) { throw(std::runtime_error("Could not init op task object from the parsed params."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Init op task object from the parsed params."<<std::flush;  }

      std::cout<<"\nTest #"<<id<<" (Task Controller2) : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" (Task Controller2) : Failed.";
    }
  }

}
