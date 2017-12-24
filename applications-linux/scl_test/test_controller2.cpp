/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
#include <scl/control/ControlFunctions.hpp>

#include <sutil/CRegisteredDynamicTypes.hpp>

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
      // ********************** INIT AND DYNAMIC TYPE SETUP TESTING **********************
      sutil::CDynamicType<std::string,scl::tasks::CTaskOpPos> typeCTaskOpPos(std::string("CTaskOpPos"));
      flag = typeCTaskOpPos.registerType();
      if(false == flag) {throw(std::runtime_error("CTaskOpPos"));}

      sutil::CDynamicType<std::string,scl::tasks::STaskOpPos> typeSTaskOpPos(std::string("STaskOpPos"));
      flag = typeSTaskOpPos.registerType();
      if(false == flag) {throw(std::runtime_error("STaskOpPos"));}

      //We'll just go with the Puma for now..
      scl::CParserScl p;
      scl::SRobotParsed rds;
      scl::SRobotIO rio;
      scl::SGcModel rgcm;
      scl::CDynamicsScl dyn_scl;

      // ********************** ROBOT PARSER TESTING **********************
      flag = flag && p.readRobotFromFile("../../specs/Puma/PumaCfg.xml", "../../specs/", "PumaBot", rds);
      flag = flag && rgcm.init(rds);
      flag = flag && rio.init(rds);
      flag = flag && dyn_scl.init(rds);
      //Test code.
      if(false==flag) { throw(std::runtime_error("Could not parse robot from file and init data/dynamics."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Parsed robot from file and init data/dynamics."<<std::flush;  }

      // ********************** CONTROL TASK DS TESTING **********************
      // Let's test a control task..
      scl::tasks::STaskOpPos t_op_ds;
      sutil::CMappedList<std::string, std::string> params;

      // Initialize the task parameters
      std::string *tt;
      tt = params.create(std::string("name"),std::string("hand"));
      // As a sanity check, we'll just test the first key. We'll assume the others work..
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

      // Initialize the task
      flag = t_op_ds.init(params,&rds);
      //Test code.
      if(false==flag) { throw(std::runtime_error("Could not init op task data from the parsed params."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Init op task data from the parsed params."<<std::flush;  }

      // Copy the task
      scl::tasks::STaskOpPos *t_op_ds2 = new scl::tasks::STaskOpPos(t_op_ds);
      if( t_op_ds2->J_.rows() != t_op_ds.J_.rows() || t_op_ds2->J_.cols() != t_op_ds.J_.cols())
      { throw(std::runtime_error("Could not init op task data from the parsed params."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Copy constructor worked for the op pos task."<<std::flush;  }
      delete t_op_ds2;

//      // NOTE TODO : FIX THIS LATER WHEN JSON STUFF WORKS!!
//      // Serialize / deserialize the task
//      std::string str_json;
//      flag = scl::serializeToJSONString(t_op_ds,str_json,true);
//      if(false==flag) { throw(std::runtime_error("Could not serialize op task data into json."));  }
//      else { std::cout<<"\nTest Result ("<<r_id++<<")  Serialized op task data into json."<<std::flush;  }

      // ********************** CONTROL TASK TESTING **********************
      scl::tasks::CTaskOpPos t_op;

      flag = t_op.init(params, rds);
      if(false==flag) { throw(std::runtime_error("Could not init op task object from the parsed params."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Init op task object using the parsed params."<<std::flush;  }

      flag = t_op.init(t_op_ds);
      t_op_ds2 = dynamic_cast<scl::tasks::STaskOpPos *>(t_op.getData());
      if( false == flag || NULL == t_op_ds2 )
      { throw(std::runtime_error("Could not init op task object using an existing task data struct."));  }

      if(t_op_ds2->J_.rows() != t_op_ds.J_.rows() || t_op_ds2->J_.cols() != t_op_ds.J_.cols())
      { throw(std::runtime_error("Could not init op task object using an existing task data struct."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Init op task object using a pre-initialized data struct."<<std::flush;  }

      flag = t_op.computeModel(rio.sensors_,rgcm, dyn_scl);
      if(false == flag)
      { throw(std::runtime_error("Could not compute task model."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Computed task model."<<std::flush;  }

      flag = t_op.computeControl(rio.sensors_,rgcm, dyn_scl);
      if(false == flag)
      { throw(std::runtime_error("Could not compute task control."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Computed task control."<<std::flush;  }

      // ********************** CONTROL Multi-level Mapped List TESTING **********************
      scl::tasks::STaskOpPos t_op_ds_b1(t_op_ds),t_op_ds_b2(t_op_ds);
      t_op_ds_b1.name_ = "bobo"; t_op_ds_b1.priority_ = 1;
      t_op_ds_b2.name_ = "bobo2"; t_op_ds_b2.priority_ = 3;//Let's see what it does with a missing priority level.

      // Load the tasks into a vector.
      std::vector<scl::tasks::STaskBase*> taskvec;
      taskvec.push_back(dynamic_cast<scl::tasks::STaskBase*>(&t_op_ds));
      taskvec.push_back(dynamic_cast<scl::tasks::STaskBase*>(&t_op_ds_b1));
      taskvec.push_back(dynamic_cast<scl::tasks::STaskBase*>(&t_op_ds_b2));

      sutil::CMappedMultiLevelList<std::string, scl::tasks::CTaskBase*> taskmllist;
      int ntasks = scl::init::initMappedMultiLevelListFromFromTasks(rds, taskvec, taskmllist);
      if(3!=ntasks)
      { throw(std::runtime_error("Could not arrange tasks into a multi level mapped list.."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Arranged tasks into a multi level mapped list.."<<std::flush;  }

      // Let's look at the allocation of the tasks in the mllist
      std::vector<scl::tasks::CTaskBase**> *plevel = taskmllist.getSinglePriorityLevel(0);
      scl::tasks::CTaskBase * tbase = *(plevel->at(0));
      if( tbase->name_ != "hand" || plevel->size() != 1)
      { throw(std::runtime_error( std::string("Priority level 0 mismatch. Task:") + tbase->name_ +std::string(", ntasks:") +std::to_string(plevel->size())));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Priority level 0 set up."<<std::flush;  }

      plevel = taskmllist.getSinglePriorityLevel(1);
      tbase = *(plevel->at(0));
      if(tbase->name_ != "bobo" || plevel->size() != 1)
      { throw(std::runtime_error("Priority level 1 doesn't contain added task"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Priority level 1 set up."<<std::flush;  }

      plevel = taskmllist.getSinglePriorityLevel(3);
      tbase = *(plevel->at(0));
      if(tbase->name_ != "bobo2" || plevel->size() != 1)
      { throw(std::runtime_error("Priority level 3 doesn't contain added task"));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Priority level 3 set up."<<std::flush;  }

      // ********************** CONTROL MODEL AND CONTROL TESTING **********************
      rio.sensors_.q_(0) = 1;

      flag = dyn_scl.computeGCModel(&rio.sensors_,&rgcm);
      if(false == flag)
      { throw(std::runtime_error("Could not update {T, J, dynamics} matrices in the gc model..."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Updating {T, J, dynamics} matrices in the gc model."<<std::flush;  }

      flag = scl::control::computeTaskModel(taskmllist,rgcm,rio,dyn_scl);
      if(false == flag)
      { throw(std::runtime_error("Could not compute task model..."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Ran the task model computations."<<std::flush;  }

      flag = scl::control::computeTaskControl(taskmllist,rgcm,rio,dyn_scl);
      if(false == flag)
      { throw(std::runtime_error("Could not compute task control..."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<") Ran the task control computations."<<std::flush;  }


      std::cout<<"\nTest #"<<id<<" (Task Controller2) : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") : ERROR : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" (Task Controller2) : Failed.";
    }
  }

}
