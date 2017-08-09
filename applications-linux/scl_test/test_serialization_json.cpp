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
/* \file test_serialization_json.cpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "test_serialization_json.hpp"

#include <scl/serialization/SerializationJSON.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/data_structs/SRigidBody.hpp>

#include <string>
#include <iostream>
#include <stdexcept>

namespace scl_test
{
  /**
   * Tests the json serialization and deserialization module
   */
  void test_serialization_json(int id)
  {
    bool flag;
    scl::sUInt r_id=0;
    try
    {
      //0. Create vars
      scl::SRigidBody rb;
      rb.init();
      rb.inertia_ <<1.1,2.22443,3,4,5,6,7,8,9;
      std::cout<<"\nTest Result ("<<r_id++<<")  : Set up data structure"<<std::endl;

      sutil::CMappedList<std::string, scl::SRigidBody> rb_list;
      rb.name_ = "node0";
      rb_list.create("node0",rb,true);
      rb.name_ = "node1";
      rb_list.create("node1",rb,true);
      rb.name_ = "node2";
      rb_list.create("node2",rb,true); //Insert in wrong order (sorting should fix this)..

      std::vector<std::string> sort_order;
      sort_order.push_back("node0");
      sort_order.push_back("node1");
      sort_order.push_back("node2");
      rb_list.sort(sort_order);

      rb.name_ = "floating_node";

      //1. Parse object to JSON value
      Json::Value json_val;
      flag = scl::serializeToJSON(rb,json_val);
      if(!flag) { throw(std::runtime_error("Could not serialize SRigidBody to JSON value")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SRigidBody object to JSON value : "<<std::endl;
      std::cout<<json_val<<std::endl;

      //2. Parse object to JSON string (compact)
      std::string str;
      flag = scl::serializeToJSONString(rb,str,true);
      if(!flag) { throw(std::runtime_error("Could not serialize SRigidBody to JSON string")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SRigidBody object to JSON string : "<<std::endl;
      std::cout<<str<<std::endl;

      //3. Parse object to JSON string (readable)
      flag = scl::serializeToJSONString(rb,str,false);
      if(!flag) { throw(std::runtime_error("Could not serialize SRigidBody to human-readable JSON string")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SRigidBody object to human-readable JSON string : "<<std::endl;
      std::cout<<str<<std::endl;

      //4. Parse mapped list of objects to JSON string (readable)
      flag = scl::serializeToJSONString(rb_list,str,false);
      if(!flag) { throw(std::runtime_error("Could not serialize CMappedList<string,SRigidBody> to human-readable JSON string")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized CMappedList<string,SRigidBody> object to human-readable JSON string : "<<std::endl;
      std::cout<<str<<std::endl;

      //5. Deserialize object from JSON value (we will re-use the json_val from above)
      json_val["name_"] = "Bobo";
      flag = scl::deserializeFromJSON(rb,json_val);
      if(!flag)
      { throw(std::runtime_error("Could not deserialize SRigidBody from JSON value")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Deserialized SRigidBody object from JSON value : "<<std::endl;
      std::cout<<"  My name should be bobo. Name : "<<rb.name_<<std::endl;

      //6. Deserialize object from JSON string (we will re-use the str from above)
      rb.name_ = "Bobo Nomo";
      scl::serializeToJSONString(rb,str); //This just worked. So won't check again.
      Json::Reader json_reader;
      flag = json_reader.parse(str,json_val);
      if(!flag) { throw(std::runtime_error("Could not use the Json reader to parse a recently generated Json string")); }
      flag = scl::deserializeFromJSON(rb,json_val);
      if(!flag) { throw(std::runtime_error("Could not deserialize SRigidBody from JSON value")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Deserialized SRigidBody object from JSON value obtained from a JSON string : "<<std::endl;
      std::cout<<"  I am bobo no mo. Name : "<<rb.name_<<std::endl;

      //7. Serialize a complex object that includes mapped lists etc. to a JSON string
      scl::SRobotParsed rparsed;
      rparsed.reset();//Initializes the vectors.
      for(auto&& elem : rb_list)
      {
        rparsed.rb_tree_.create(elem.name_, elem, elem.is_root_);
        rparsed.robot_tree_numeric_id_to_name_.push_back(elem.name_);
      }
      // Initializes stuff, since things are pre-sorted anyway...
      rparsed.rb_tree_.sort(rparsed.robot_tree_numeric_id_to_name_);

      flag = scl::serializeToJSONString(rparsed,str,false);
      if(!flag) { throw(std::runtime_error("Could not serialize SRobotParsed to human-readable JSON string")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SRobotParsed object to human-readable JSON string : "<<std::endl;
      std::cout<<"  SRobotParsed : "<<str<<std::endl;

      //8. Serialize a Puma robot's parsed spec..
      scl::CParserScl parser;
      std::vector<std::string> robot_list;
      flag = parser.listRobotsInFile("../../specs/Puma/PumaCfg.xml",robot_list);
      if(!flag) { throw(std::runtime_error("Could not check Puma xml")); }
      rparsed.reset();//Reset everything..
      flag = parser.readRobotFromFile("../../specs/Puma/PumaCfg.xml","../../specs/",robot_list[0],rparsed);
      if(!flag) { throw(std::runtime_error("Could not load Puma robot xml")); }

      flag = scl::serializeToJSONString(rparsed,str,false);
      if(!flag) { throw(std::runtime_error("Could not serialize SRobotParsed : PUMA to human-readable JSON string")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SRobotParsed : PUMA object to human-readable JSON string : "<<std::endl;
      std::cout<<"  SRobotParsed : Puma \n"<<str<<std::endl;

      flag = scl::serializeToJSONString(rparsed,str,true);
      if(!flag) { throw(std::runtime_error("Could not serialize SRobotParsed : PUMA to compressed JSON string")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SRobotParsed : PUMA object to compressed JSON string : "<<std::endl;
      std::cout<<"  SRobotParsed : Puma \n"<<str<<std::endl;

      //9. Serialize a different complex object that includes mapped lists etc. to a JSON string
      scl::SMusclePointParsed muscle_pt;
      muscle_pt.parent_link_ = "bobo";
      muscle_pt.pos_in_parent_<<1, 1, 1;
      muscle_pt.position_on_muscle_ = 0;

      scl::SMuscleParsed muscle;
      muscle.points_.push_back(muscle_pt);
      muscle_pt.position_on_muscle_ = 1;
      muscle.points_.push_back(muscle_pt);
      muscle_pt.position_on_muscle_ = 2;
      muscle.points_.push_back(muscle_pt);


      scl::SActuatorSetMuscleParsed actsetm;
      muscle.name_ = "m0";
      actsetm.muscles_.create(muscle.name_,muscle);
      muscle.name_ = "m1";
      actsetm.muscles_.create(muscle.name_,muscle);
      muscle.name_ = "m2";
      actsetm.muscles_.create(muscle.name_,muscle);

      actsetm.muscle_id_to_name_.push_back("m0");
      actsetm.muscle_id_to_name_.push_back("m1");
      actsetm.muscle_id_to_name_.push_back("m2");

      actsetm.muscle_name_to_id_.create("m0",0,false);
      actsetm.muscle_name_to_id_.create("m1",1,false);
      actsetm.muscle_name_to_id_.create("m2",2,false);

      flag = scl::serializeToJSONString(actsetm,str,false);
      if(!flag) { throw(std::runtime_error("Could not serialize SActuatorSetMuscleParsed to human-readable JSON string")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SActuatorSetMuscleParsed object to human-readable JSON string : "<<std::endl;
      std::cout<<str<<std::endl;

      flag = scl::serializeToJSONString(actsetm,str,true);
      if(!flag) { throw(std::runtime_error("Could not serialize SActuatorSetMuscleParsed to compact JSON string")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SActuatorSetMuscleParsed object to compact JSON string : "<<std::endl;
      std::cout<<str<<std::endl;

      std::cout<<"\nTest #"<<id<<" : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<")  : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" : Failed.";
    }
  }

}
