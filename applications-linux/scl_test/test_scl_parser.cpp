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
/* \file test_scl_parser.cpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "test_scl_parser.hpp"

#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include <stdexcept>

#include <scl/DataTypes.hpp>
#include <scl/parser/CParserBase.hpp>

#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>

#include <scl/parser/sclparser/CSclParser.hpp>

#include <scl/Singletons.hpp>

namespace scl_test
{

/**
 * Tests the robot parser with the sample Scl format
 *
 *   File  = "specs/SclBot/SclBot.xml"
 */
void test_scl_parser(int id)
{
  bool flag=true;
  int r_id=0;
  try
  {
    //0. Create a parser and objects to be filled in from a file.
    scl_parser::CSclParser tmp_parser;

    scl::SRobotParsedData tmp_robot;
    scl::SGraphicsParsedData tmp_graphics;

    //1. Read in a file
    std::string tmp_infile;
    tmp_infile = scl::CDatabase::getData()->cwd_+ "../../specs/Puma/PumaCfg.xml";
    scl::CDatabase::getData()->dir_specs_ = scl::CDatabase::getData()->cwd_+ "../../specs/";

    std::cout<<"\nTest Result ("<<r_id++<<") Test file is : "<<tmp_infile;

    std::vector<std::string> robot_names;
    flag = tmp_parser.listRobotsInFile(tmp_infile,robot_names);
    if( (false == flag) || (1 > robot_names.size()) )
    { throw(std::runtime_error("Could not read robot names from the file"));  }
    else  { std::cout<<"\nTest Result ("<<r_id++<<") Read robot names from file"; }

    flag = tmp_parser.readRobotFromFile(tmp_infile, robot_names[0],tmp_robot);
    if(false==flag) {
      std::string err; err = "Read robot ("+robot_names[0]+") from file : Failed";
      throw(std::runtime_error(err.c_str()));  }
    else  { std::cout<<"\nTest Result ("<<r_id++<<") Read robot ("+robot_names[0]+") from file.";  }

    flag = tmp_robot.robot_tree_.linkNodes();
    if(false==flag)
    { throw(std::runtime_error("Connect links into a \'branching representation\' tree: Failed"));  }
    else
    {
      std::cout<<"\nTest Result ("<<r_id++
          <<") Connected links into a \'branching structure\' tree.";
    }

    std::cout<<"\nTest Result ("<<r_id++<<") Printing links:";
    sutil::CMappedTree<std::basic_string<char>, scl::SRigidBody>::iterator itbr,itbre;
    for(itbr = tmp_robot.robot_tree_.begin(), itbre = tmp_robot.robot_tree_.end();
        itbr!=itbre; ++itbr)
    {
      scl::SRigidBody& tmp_link = *itbr;

      std::cout<<"\n\tNode: "<<tmp_link.name_ <<". Children:";

      std::vector<scl::SRigidBody*>::const_iterator it, ite;
      ite = tmp_link.child_addrs_.end();
      for(it = tmp_link.child_addrs_.begin();it!=ite;++it)
      {
        if(S_NULL==(*it))
        { throw(std::runtime_error("Link has a NULL child-link : Failed"));  }
        std::cout<<(*it)->name_;
        if((*it)->parent_addr_ != &tmp_link)
        { throw(std::runtime_error("Child-link has incorrect parent-link address : Failed"));  }
      }
    }

    std::cout<<"\nTest Result ("<<r_id++
        <<") Verified links and their children in the branching representation tree.";

    //7. Test Map (Idx and name of pointed object should match)
    const std::string test_link_name("ground");
    scl::SRigidBody* tmp_link = tmp_robot.robot_tree_.at(test_link_name);
    if(S_NULL == tmp_link)
    {
      throw(std::runtime_error(
          "Could not find a \'ground\' link in the robot's branching representation"));
    }
    else  { std::cout<<"\nTest Result ("<<r_id++<<") Found the \'ground\' link"; }

    if(tmp_link->name_ != test_link_name)
    {
      throw(std::runtime_error(
          "\'Name-LinkPointer\' mapping of the \'ground\' link is incorrect"));
    }
    else
    {
      std::cout<<"\nTest Result ("<<r_id++
          <<") \'Name-LinkPointer\' mapping of the \'ground\' link verified";
    }

    //8. Test the full parser and scl_registry API
    flag = scl_registry::registerNativeDynamicTypes();
    if(false == flag)
    { throw(std::runtime_error("Could not register the native dynamic types (required for parsing)")); }
    else
    { std::cout<<"\nTest Result ("<<r_id++<<") Registered native dynamic types for config file"; }

    flag = scl_registry::parseEverythingInFile("../../specs/Puma/PumaCfg.xml",&tmp_parser);
    if(false == flag)
    { throw(std::runtime_error("Could not parse the Puma spec")); }
    else
    { std::cout<<"\nTest Result ("<<r_id++<<") Parsed full PumaCfg.xml file"; }

    std::cout<<"\nTest #"<<id<<" : Succeeded.";
  }
  catch (std::exception& ee)
  {
    std::cout<<"\nTest Result ("<<r_id++<<") : "<<ee.what();
    std::cout<<"\nTest #"<<id<<" : Failed.";
  }
}


/***
 * Tests the tao representation creator:
 *
 * Basically converts a parsed robot representation
 * into a tao dynamics engine tree.
 *
 */
void test_tao_rep_creator(int id)
{
  try
  {

  }
  catch(int i)
  {
    printf("\nTest %d : Failed",id);
    printf("\n*************************");
    return;
  }
  printf("\nTest %d : Succeeded",id);
  printf("\n*************************");
}

}
