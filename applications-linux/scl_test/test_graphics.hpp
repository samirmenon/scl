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
/* \file test_graphics.hpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef TEST_GRAPHICS_HPP_
#define TEST_GRAPHICS_HPP_


#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <math.h>

#include <Eigen/Dense>

#include <sutil/CSystemClock.hpp>

#include <scl/DataTypes.hpp>

#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>

#include <scl/parser/lotusparser/CLotusParser.hpp>

#include <scl/graphics/chai/CChaiGraphics.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>

#include <scl/util/DatabaseUtils.hpp>


namespace scl_test
{

  using namespace scl;
  using namespace std;

  /**
   * Tests the (chai) graphics subsystem
   *
   * Reads in a toy robot specification and renders it.
   */
  void test_graphics(int id, const std::string &file_name,
      int argc, char **argv)
  {
    scl::sUInt r_id=0;
    try
    {
      //0. Create vars
      long long i; //Counters
      long long imax; //Counter limits
      sClock t1,t2; //Clocks: pre and post
      bool flag;

      // Test database
      scl::SDatabase* db = scl::CDatabase::getData();
      if(S_NULL==db)
      { throw(std::runtime_error("Database not initialized"));  }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized database"<<std::flush;  }

      /******************************Parsing************************************/

      //1. Create robot from a file specification (And register it with the db)
      std::vector<std::string> robot_names;
      scl::SRobotParsedData* rob_ds;
      scl_parser::CLotusParser tmp_lparser;
      flag = tmp_lparser.listRobotsInFile(file_name,robot_names);
      if(false == flag)
      { throw(std::runtime_error("Could not read robot names from the file"));  }
      else
      {std::cout<<"\nTest Result ("<<r_id++<<")  Creating a robot specification for "
        <<robot_names[0]<<" on the pile"<<std::flush;}


      flag = scl_registry::parseWorld(file_name, &tmp_lparser);
      if(false == flag)
      { throw(std::runtime_error("Could not register world with the database"));  }

      flag = scl_registry::parseRobot(file_name,
          robot_names[0], &tmp_lparser);

      if(false == flag)
      { throw(std::runtime_error("Could not register robot with the database"));  }
      else
      {
        std::cout<<"\nTest Result ("<<r_id++<<")  Created a robot "
            <<robot_names[0]<<" on the pile"<<std::flush;
      }

#ifdef W_TESTING
      std::cout<<"\nPrinting parsed robot "
          <<db->s_parser_.robots_.at(robot_names[0])->name_;
      scl_util::printRobotLinkTree(*( db->s_parser_.robots_.at(robot_names[0])->robot_br_rep_.getRootNode()),0);
#endif

      //1.b. Pull out the robot's ds from the db
      rob_ds = db->s_parser_.robots_.at(robot_names[0]);
      if(S_NULL == rob_ds)
      { throw(std::runtime_error("Could not find robot in the database"));  }
      else
      {
        std::cout<<"\nTest Result ("<<r_id++<<")  Found robot "
            <<robot_names[0]<<" on the pile"<<std::flush;
      }

      //Ok. Now we have a robot's specification specification parsed into
      //the database. Lets move on.

      //2. Read in the graphics spec from a file...
      std::vector<std::string> graphics_names;
      flag = tmp_lparser.listGraphicsInFile(file_name,graphics_names);
      if(false == flag)
      { throw(std::runtime_error("Could not read graphics names from the file"));  }
      else
      {std::cout<<"\nTest Result ("<<r_id++<<")  Creating a graphics specification for "
        <<graphics_names[0]<<" on the pile"<<std::flush;}

      flag = scl_registry::parseGraphics(file_name,
          graphics_names[0], &tmp_lparser);

      if(false == flag)
      { throw(std::runtime_error("Could not register graphics with the database"));  }
      else
      {
        std::cout<<"\nTest Result ("<<r_id++<<")  Created a graphics "
            <<graphics_names[0]<<" on the pile"<<std::flush;
      }

      //2.b. Pull out the graphics's ds from the db
      SGraphicsParsedData * gr_ds = db->s_parser_.graphics_worlds_.at(graphics_names[0]);
      if(S_NULL==gr_ds)
      {
        throw(std::runtime_error(
            "Could not find any graphics specification in the database. (Did you parse it from a file yet?)"));
      }
      else
      {
        std::cout<<"\nTest Result ("<<r_id++<<")  Found graphics specification "
            <<gr_ds->name_<<" on the pile"<<std::flush;
      }

      /******************************Chai Initialization************************************/
      //3. Initialize a chai graphics object
      CChaiGraphics chai_gr;
      flag = chai_gr.initGraphics(gr_ds->name_);
      if(false==flag)
      { throw(std::runtime_error("Couldn't initialize chai graphics")); }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized chai graphics"<<std::flush;  }

      //5. Add a robot to render
      flag = chai_gr.addRobotToRender(robot_names[0]);
      if(false==flag)
      { throw(std::runtime_error("Couldn't find robot to render")); }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized robot "<<robot_names[0]<<" to render"<<std::flush;  }

      //-----------------------------------------------------------------------
      // OPEN GL - WINDOW DISPLAY
      //-----------------------------------------------------------------------
      // initialize GLUT
      if(!db->s_gui_.glut_initialized_)
      {
        glutInit(&argc, argv);
        db->s_gui_.glut_initialized_ = true;
      }

      if(false == scl_chai_glut_interface::initializeGlutForChai(gr_ds->name_, &chai_gr))
      { throw(std::runtime_error("Glut initialization error")); }

      //6. Test graphics rendering speed.
      imax = 100;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        chai_gr.updateGraphics();
      }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<") "<<imax<<" graphics updates."
          <<" Time: "<<t2-t1<<std::flush;

      //-----------------------------------------------------------------------
      // START SIMULATION
      //-----------------------------------------------------------------------
      // Start the main graphics rendering loop
      t1 = sutil::CSystemClock::getSysTime();
      for(int glIter=0;glIter<imax;glIter++)
      { glutMainLoopEvent();  }
      t2 = sutil::CSystemClock::getSysTime();
      std::cout<<"\nTest Result ("<<r_id++<<") "<<imax<<" graphics+glut updates."
          <<" Time: "<<t2-t1<<std::flush;

      std::cout<<"\nTest #"<<id<<" Succeeded.";
    }
    catch(std::exception & e)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") "<< e.what();
      std::cout<<"\nTest #"<<id<<" Failed.";
    }
  }
}

#endif /* TEST_GRAPHICS_HPP_ */
