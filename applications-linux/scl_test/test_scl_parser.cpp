/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/* \file test_scl_parser.cpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "test_scl_parser.hpp"


#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>

#include <scl/Singletons.hpp>
#include <scl/DataTypes.hpp>
#include <scl/Init.hpp>

#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdexcept>

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
    scl::CParserScl tmp_parser;

    scl::SRobotParsed tmp_robot;
    scl::SGraphicsParsed tmp_graphics;

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

    flag = tmp_parser.readRobotFromFile(tmp_infile, scl::CDatabase::getData()->dir_specs_, robot_names[0],tmp_robot);
    if(false==flag) {
      std::string err; err = "Read robot ("+robot_names[0]+") from file : Failed";
      throw(std::runtime_error(err.c_str()));  }
    else  { std::cout<<"\nTest Result ("<<r_id++<<") Read robot ("+robot_names[0]+") from file.";  }

    flag = tmp_robot.rb_tree_.linkNodes();
    if(false==flag)
    { throw(std::runtime_error("Connect links into a \'branching representation\' tree: Failed"));  }
    else
    {
      std::cout<<"\nTest Result ("<<r_id++
          <<") Connected links into a \'branching structure\' tree.";
    }

    std::cout<<"\nTest Result ("<<r_id++<<") Printing links:";
    sutil::CMappedTree<std::basic_string<char>, scl::SRigidBody>::iterator itbr,itbre;
    for(itbr = tmp_robot.rb_tree_.begin(), itbre = tmp_robot.rb_tree_.end();
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
    scl::SRigidBody* tmp_link = tmp_robot.rb_tree_.at(test_link_name);
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
    flag = scl::init::registerNativeDynamicTypes();
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
}
