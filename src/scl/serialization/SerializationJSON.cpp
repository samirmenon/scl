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

namespace scl
{
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
    ret_json_val["has_been_init"] = arg_obj.hasBeenInit();
    ret_json_val["name"] = arg_obj.getName();
    ret_json_val["type"] = arg_obj.getType();
    return true;
  }

  template<> bool serializeToJSON<SRigidBody>(const SRigidBody &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    Json::Reader json_reader;
    std::string str;

    ret_json_val["collision_type"] = arg_obj.collision_type_;
    scl_util::eigentoStringArrayJSON(arg_obj.com_, str);
    json_reader.parse(str, ret_json_val["com"]);
    ret_json_val["force_gc_lim_lower"] = arg_obj.force_gc_lim_lower_;
    ret_json_val["force_gc_lim_upper"] = arg_obj.force_gc_lim_upper_;
    ret_json_val["friction_gc_kv"] = arg_obj.friction_gc_kv_;
    scl_util::eigentoStringArrayJSON(arg_obj.inertia_, str);
    json_reader.parse(str, ret_json_val["inertia"]);
    ret_json_val["inertia_gc"] = arg_obj.inertia_gc_;
    ret_json_val["is_root"] = arg_obj.is_root_;
    ret_json_val["joint_default_pos"] = arg_obj.joint_default_pos_;
    ret_json_val["joint_limit_lower"] = arg_obj.joint_limit_lower_;
    ret_json_val["joint_limit_upper"] = arg_obj.joint_limit_upper_;
    ret_json_val["joint_name"] = arg_obj.joint_name_;
    ret_json_val["joint_type"] = arg_obj.joint_type_;
    ret_json_val["link_id"] = arg_obj.link_id_;
    ret_json_val["link_is_fixed"] = arg_obj.link_is_fixed_;
    ret_json_val["mass"] = arg_obj.mass_;
    scl_util::eigentoStringArrayJSON(arg_obj.ori_parent_quat_, str);
    json_reader.parse(str, ret_json_val["ori_parent_quat"]);
    ret_json_val["parent_name"] = arg_obj.parent_name_;
    scl_util::eigentoStringArrayJSON(arg_obj.pos_in_parent_, str);
    json_reader.parse(str, ret_json_val["pos_in_parent"]);
    ret_json_val["render_type"] = arg_obj.render_type_;
    ret_json_val["robot_name"] = arg_obj.robot_name_;
    ret_json_val["stiction_gc_force_lower"] = arg_obj.stiction_gc_force_lower_;
    ret_json_val["stiction_gc_force_upper"] = arg_obj.stiction_gc_force_upper_;
    ret_json_val["stiction_gc_vel_lower"] = arg_obj.stiction_gc_vel_lower_;
    ret_json_val["stiction_gc_vel_upper"] = arg_obj.stiction_gc_vel_upper_;

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
}
