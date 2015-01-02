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
/* \file SerializationJSON.cpp
 *
 *  Created on: Dec 31, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
#include <scl/serialization/SerializationJSON.hpp>
#include <scl/util/EigenExtensions.hpp>
#include <cmath>

namespace scl
{
  /** *****************************************************************************
   *                  Terrible macros to greatly simplify life
   * **************************************************************************** */
  //Yes this is terrible. But seriously. Who the heck wants to write tons of useless code.
  //This is where a metacompiler would be useful. moc moc moc.. Sigh...
  // NOTE TODO : Remove this when you write a script to autogen all this stuff...
  // NOTE TODO : C++1X might have features that does this sort of stuff.. Look into Document number: N3951.

#define MACRO_SER_ARGOBJ_RETJSONVAL(AAA) ret_json_val[#AAA] = arg_obj.AAA;

#define MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(AAA) \
  scl_util::eigentoStringArrayJSON(arg_obj.AAA, str); \
  json_reader.parse(str, ret_json_val[#AAA]);


#define MACRO_DESER_RETOBJ_ARGJSONVAL(AAA,TTT) \
    if(!arg_json_val.isMember(#AAA)) { \
      std::cout<<"\n deserializeFromJSON() Error : Could not find : "<<#AAA<<std::flush; \
      return false; \
    } \
    ret_obj.AAA = arg_json_val[#AAA].TTT();

  /** *****************************************************************************
   *                          Serialize data to a string
   * **************************************************************************** */
  template<> bool serializeToJSON<SActuatorSetMuscleParsed>(const SActuatorSetMuscleParsed &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SActuatorSetParsed>(const SActuatorSetParsed &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SDatabase>(const SDatabase &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SForce>(const SForce &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SGcModel>(const SGcModel &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SGraphicsParsed>(const SGraphicsParsed &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SObject>(const SObject &arg_obj, Json::Value &ret_json_val)
  {
    MACRO_SER_ARGOBJ_RETJSONVAL(has_been_init_)
    MACRO_SER_ARGOBJ_RETJSONVAL(name_)
    ret_json_val["type_"] = arg_obj.getType();
    return true;
  }

  template<> bool serializeToJSON<SRigidBody>(const SRigidBody &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    Json::Reader json_reader;
    std::string str;

    //Read in the standard types (supported by json)
    MACRO_SER_ARGOBJ_RETJSONVAL(collision_type_)
    MACRO_SER_ARGOBJ_RETJSONVAL(force_gc_lim_lower_)
    MACRO_SER_ARGOBJ_RETJSONVAL(force_gc_lim_upper_)
    MACRO_SER_ARGOBJ_RETJSONVAL(friction_gc_kv_)
    MACRO_SER_ARGOBJ_RETJSONVAL(inertia_gc_)
    MACRO_SER_ARGOBJ_RETJSONVAL(is_root_)
    MACRO_SER_ARGOBJ_RETJSONVAL(joint_default_pos_)
    MACRO_SER_ARGOBJ_RETJSONVAL(joint_limit_lower_)
    MACRO_SER_ARGOBJ_RETJSONVAL(joint_limit_upper_)
    MACRO_SER_ARGOBJ_RETJSONVAL(joint_name_)
    MACRO_SER_ARGOBJ_RETJSONVAL(joint_type_)
    MACRO_SER_ARGOBJ_RETJSONVAL(link_id_)
    MACRO_SER_ARGOBJ_RETJSONVAL(link_is_fixed_)
    MACRO_SER_ARGOBJ_RETJSONVAL(mass_)
    MACRO_SER_ARGOBJ_RETJSONVAL(parent_name_)
    MACRO_SER_ARGOBJ_RETJSONVAL(render_type_)
    MACRO_SER_ARGOBJ_RETJSONVAL(robot_name_)
    MACRO_SER_ARGOBJ_RETJSONVAL(stiction_gc_force_lower_)
    MACRO_SER_ARGOBJ_RETJSONVAL(stiction_gc_force_upper_)
    MACRO_SER_ARGOBJ_RETJSONVAL(stiction_gc_vel_lower_)
    MACRO_SER_ARGOBJ_RETJSONVAL(stiction_gc_vel_upper_)

    //Read in the Eigen matrix types..
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(com_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(inertia_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(ori_parent_quat_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(pos_in_parent_)

    return flag;
  }

  template<> bool serializeToJSON<SRigidBodyDyn>(const SRigidBodyDyn &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SRobotIO>(const SRobotIO &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SRobotParsed>(const SRobotParsed &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SUIParsed>(const SUIParsed &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  /** *****************************************************************************
   *                        Deserialize data to an object
   * **************************************************************************** */
  template<> bool deserializeFromJSON<SActuatorSetMuscleParsed>(SActuatorSetMuscleParsed &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SActuatorSetParsed>(SActuatorSetParsed &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SDatabase>(SDatabase &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SForce>(SForce &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SGcModel>(SGcModel &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SGraphicsParsed>(SGraphicsParsed &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SObject>(SObject &ret_obj, const Json::Value &arg_json_val)
  {
    MACRO_DESER_RETOBJ_ARGJSONVAL(has_been_init_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(name_,asString)
    return true;
  }

#define MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(AAA) \
    if(!arg_json_val.isMember(#AAA)) { \
      std::cout<<"\n deserializeFromJSON() Error : Could not find : "<<#AAA<<std::flush; \
      return false; \
    } \
    flag = scl_util::eigenFromJSON(ret_obj.AAA, arg_json_val[#AAA]); \
    if(!flag) { \
      std::cout<<"\n deserializeFromJSON() Error : Could not deserialize : "<<#AAA<<std::flush; \
      return false; \
    }

  template<> bool deserializeFromJSON<SRigidBody>(SRigidBody &ret_obj, const Json::Value &arg_json_val)
  {
    bool flag = deserializeFromJSON(*dynamic_cast<const SObject*>(&ret_obj), arg_json_val);
    if(!flag){ return false; }

    //Read in the standard types (supported by json)
    MACRO_DESER_RETOBJ_ARGJSONVAL(collision_type_,asInt)
    MACRO_DESER_RETOBJ_ARGJSONVAL(force_gc_lim_lower_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(force_gc_lim_upper_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(friction_gc_kv_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(inertia_gc_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(is_root_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(joint_default_pos_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(joint_limit_lower_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(joint_limit_upper_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(joint_name_,asString)
    MACRO_DESER_RETOBJ_ARGJSONVAL(link_id_,asInt)
    MACRO_DESER_RETOBJ_ARGJSONVAL(link_is_fixed_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(mass_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(parent_name_,asString)
    MACRO_DESER_RETOBJ_ARGJSONVAL(robot_name_,asString)
    MACRO_DESER_RETOBJ_ARGJSONVAL(stiction_gc_force_lower_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(stiction_gc_force_upper_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(stiction_gc_vel_lower_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL(stiction_gc_vel_upper_,asDouble)

    //Special cases. Too much work to make a macro..
    if(!arg_json_val.isMember("joint_type_")) return false;
    ret_obj.joint_type_ = static_cast<EJointType> (arg_json_val["joint_type"].asInt());

    if(!arg_json_val.isMember("render_type_")) return false;
    ret_obj.render_type_ = static_cast<ERenderType> (arg_json_val["render_type"].asInt());

    //Read in the Eigen matrix types..
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(com_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(inertia_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(ori_parent_quat_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(pos_in_parent_)

    return true;
  }

  template<> bool deserializeFromJSON<SRigidBodyDyn>(SRigidBodyDyn &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SRobotIO>(SRobotIO &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SRobotParsed>(SRobotParsed &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SUIParsed>(SUIParsed &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }
}
