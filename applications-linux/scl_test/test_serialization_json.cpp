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

      //1. Parse to JSON value
      Json::Value json_val;
      flag = scl::serializeToJSON(rb,json_val);
      if(!flag)
      { throw(std::runtime_error("Could not serialize SRigidBody to JSON value")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SRigidBody object to JSON value : "<<std::endl;
      std::cout<<json_val<<std::endl;

      //2. Parse to JSON string
      std::string str;
      flag = scl::serializeToJSONString(rb,str,true);
      if(!flag)
      { throw(std::runtime_error("Could not serialize SRigidBody to JSON string")); }
      std::cout<<"\nTest Result ("<<r_id++<<")  : Serialized SRigidBody object to JSON string : "<<std::endl;
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
