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

// NOTE : We need this here to make sure the extern symbols are
//        ignored _FOR THIS CPP FILE ONLY_
#define SCL_LIBRARY_COMPILE_FLAG 1

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

#define MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(AAA) \
    if(!serializeToJSON(arg_obj.AAA, ret_json_val[#AAA])) { \
      std::cout<<"\n serializeToJSON() Error : Could not serialize : "<<#AAA<<std::flush; \
      return false; \
    }

  //If you figure out a way to get the multi-template thing working then just add a template
  //specialization and remove this specific macro..
#define MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(AAA) \
  scl_util::eigentoStringArrayJSON(arg_obj.AAA, str); \
  json_reader.parse(str, ret_json_val[#AAA]);


#define MACRO_DESER_RETOBJ_ARGJSONVAL(AAA,TTT) \
    if(!arg_json_val.isMember(#AAA)) { \
      std::cout<<"\n deserializeFromJSON() Error : Could not find : "<<#AAA<<std::flush; \
      return false; \
    } \
    ret_obj.AAA = arg_json_val[#AAA].TTT();

#define MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(AAA) \
    if(!arg_json_val.isMember(#AAA)) { \
      std::cout<<"\n deserializeFromJSON() Error : Could not find : "<<#AAA<<std::flush; \
      return false; \
    } \
    if(false == deserializeFromJSON(ret_obj.AAA,arg_json_val[#AAA])){ \
      std::cout<<"\n deserializeFromJSON() Error : Could not deserialize object : "<<#AAA<<std::flush; \
      return false; \
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

  /** *****************************************************************************
   *                          Serialize data to a string
   * **************************************************************************** */
  template<> bool serializeToJSON<sUInt>(const sUInt &arg_obj, Json::Value &ret_json_val)
  { ret_json_val = arg_obj; return true; }

  template<> bool serializeToJSON(const sSpatialXForm &arg_obj, Json::Value &ret_json_val)
  {
    std::string str;
    Json::Reader json_reader;
    scl_util::eigentoStringArrayJSON(arg_obj, str); //Gets a json string..
    json_reader.parse(str, ret_json_val);
    return true;
  }

  template<> bool serializeToJSON<SObject>(const SObject &arg_obj, Json::Value &ret_json_val)
  {
    MACRO_SER_ARGOBJ_RETJSONVAL(has_been_init_)
    MACRO_SER_ARGOBJ_RETJSONVAL(name_)
    ret_json_val["type_"] = arg_obj.getType();
    return true;
  }

  template<> bool serializeToJSON<SMusclePointParsed>(const SMusclePointParsed &arg_obj, Json::Value &ret_json_val)
  {
    MACRO_SER_ARGOBJ_RETJSONVAL(parent_link_);
    MACRO_SER_ARGOBJ_RETJSONVAL(position_on_muscle_);

    std::string str;
    Json::Reader json_reader;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(pos_in_parent_);

    return true;
  }

  template<> bool serializeToJSON<SMuscleParsed>(const SMuscleParsed &arg_obj, Json::Value &ret_json_val)
  {
    //Typical data
    MACRO_SER_ARGOBJ_RETJSONVAL(optimal_fiber_length_)
    MACRO_SER_ARGOBJ_RETJSONVAL(tendon_slack_length_)
    MACRO_SER_ARGOBJ_RETJSONVAL(pennation_angle_)
    MACRO_SER_ARGOBJ_RETJSONVAL(activation_time_constt_)
    MACRO_SER_ARGOBJ_RETJSONVAL(max_contraction_vel_)
    MACRO_SER_ARGOBJ_RETJSONVAL(deactivation_time_constt_)
    MACRO_SER_ARGOBJ_RETJSONVAL(max_contraction_vel_low_)
    MACRO_SER_ARGOBJ_RETJSONVAL(max_contraction_vel_high_ )
    MACRO_SER_ARGOBJ_RETJSONVAL(max_tendon_strain_)
    MACRO_SER_ARGOBJ_RETJSONVAL(max_muscle_strain_)
    MACRO_SER_ARGOBJ_RETJSONVAL(stiffness_)
    MACRO_SER_ARGOBJ_RETJSONVAL(damping_)
    MACRO_SER_ARGOBJ_RETJSONVAL(stiffness_tendon_)
    MACRO_SER_ARGOBJ_RETJSONVAL(muscle_type_)
    MACRO_SER_ARGOBJ_RETJSONVAL(name_)

    // Std vector of strings (iterable)
    ret_json_val["points_"] = Json::Value(Json::arrayValue);
    // C++11 auto iterator (compact!)
    for (auto&& element: arg_obj.points_) {
      Json::Value tmp;
      serializeToJSON(element,tmp);
      ret_json_val["points_"].append(tmp);
    }

    return true;
  }

  template<> bool serializeToJSON<SActuatorSetMuscleParsed>(const SActuatorSetMuscleParsed &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    //Typical data
    MACRO_SER_ARGOBJ_RETJSONVAL(render_muscle_thickness_)
    MACRO_SER_ARGOBJ_RETJSONVAL(render_muscle_via_pt_sz_)
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(muscles_)
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(muscle_name_to_id_)

    // Std vector of strings (iterable)
    ret_json_val["muscle_id_to_name_"] = Json::Value(Json::arrayValue);
    // C++11 auto iterator (compact!)
    for (auto&& element: arg_obj.muscle_id_to_name_) {
      ret_json_val["muscle_id_to_name_"].append(element);
    }

    return true;
  }

  template<> bool serializeToJSON<SActuatorSetParsed>(const SActuatorSetParsed &arg_obj, Json::Value &ret_json_val)
  {
    // Need if-else here... Ugh! Direct data access so can't use virtual functions
    if(arg_obj.getType() == "SActuatorSetMuscleParsed")
    {
      const SActuatorSetMuscleParsed * obj = dynamic_cast<const SActuatorSetMuscleParsed *>(&arg_obj);
      if(obj) { return serializeToJSON(*obj, ret_json_val); }
    }
    return true;
  }

  template<> bool serializeToJSON<SParserData>(const SParserData &arg_obj, Json::Value &ret_json_val)
  {
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(graphics_worlds_)
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(robots_)
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(muscle_sets_)
    return true;
  }

  template<> bool serializeToJSON<SDatabase>(const SDatabase &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SForce>(const SForce &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    ret_json_val["robot_name_"] = arg_obj.robot_->name_;
    MACRO_SER_ARGOBJ_RETJSONVAL(link_name_)

    std::string str;
    Json::Reader json_reader;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(pos_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(direction_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(J_)
    return true;
  }

  template<> bool serializeToJSON<SGraphicsParsed>(const SGraphicsParsed &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    std::string str;
    Json::Reader json_reader;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(cam_pos_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(cam_lookat_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(cam_up_)

    ret_json_val["cam_clipping_dist_"] = Json::Value(Json::arrayValue);
    ret_json_val["cam_clipping_dist_"].append(arg_obj.cam_clipping_dist_[0]);
    ret_json_val["cam_clipping_dist_"].append(arg_obj.cam_clipping_dist_[1]);

    ret_json_val["file_background_"] = arg_obj.file_background_;
    ret_json_val["file_foreground_"] = arg_obj.file_foreground_;

    ret_json_val["background_color_"] = Json::Value(Json::arrayValue);
    for(auto i : {0,1,2}){ret_json_val["background_color_"] = arg_obj.background_color_[i]; }//RGB.

    ret_json_val["directional_lights_"] = Json::Value(Json::arrayValue);
    for(auto&& elem : arg_obj.lights_)
    {
      Json::Value val;
      val["lookat_"] = Json::Value(Json::arrayValue);
      for(auto i : {0,1,2}){val["lookat_"] = elem.lookat_[i]; }
      val["pos_"] = Json::Value(Json::arrayValue);
      for(auto i : {0,1,2}){val["pos_"] = elem.lookat_[i]; }
      ret_json_val["directional_lights_"].append(val);
    }
    return true;
  }

  template<> bool serializeToJSON<SRigidBodyGraphics>(const SRigidBodyGraphics &arg_obj, Json::Value &ret_json_val)
  {
    MACRO_SER_ARGOBJ_RETJSONVAL(collision_type_)
    MACRO_SER_ARGOBJ_RETJSONVAL(file_name_)
    MACRO_SER_ARGOBJ_RETJSONVAL(class_)

    std::string str;
    Json::Reader json_reader;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(pos_in_parent_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(ori_parent_quat_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(scaling_)

    /** Color (rgb) */
    ret_json_val["color_"] = Json::Value(Json::arrayValue);
    for(auto i:{0,1,2}){ ret_json_val["color_"].append(arg_obj.color_[i]); }

    return true;
  }

  template<> bool serializeToJSON<SRigidBody>(const SRigidBody &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

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

    ret_json_val["graphics_obj_vec_"] = Json::Value(Json::arrayValue);
    for(auto&& elem : arg_obj.graphics_obj_vec_)
    {
      Json::Value val;
      if(!serializeToJSON(elem,val)){ return false; }
      ret_json_val["graphics_obj_vec_"].append(val);
    }

    //Read in the Eigen matrix types..
    std::string str;
    Json::Reader json_reader;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(com_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(inertia_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(ori_parent_quat_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(pos_in_parent_)

    return flag;
  }

  template<> bool serializeToJSON<SRigidBodyDyn>(const SRigidBodyDyn &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    // Special case.. Pointer deref..
    ret_json_val["name_"] = arg_obj.link_ds_->name_;

    MACRO_SER_ARGOBJ_RETJSONVAL(q_T_)
    MACRO_SER_ARGOBJ_RETJSONVAL(parent_name_)

    //Read in the Eigen matrix types..
    std::string str;
    Json::Reader json_reader;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(J_com_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(T_o_lnk_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(T_lnk_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(sp_q_T_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(sp_dq_T_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(sp_inertia_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(spatial_acceleration_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(spatial_force_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(sp_X_within_link_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(sp_X_o_lnk_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(sp_S_joint_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(sp_Sorth_joint_)

    // Special case.. Demands an additional template in the code. Ugh..
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(sp_X_joint_)

    return true;
  }

  template<> bool serializeToJSON<SGcModel>(const SGcModel &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    MACRO_SER_ARGOBJ_RETJSONVAL(name_robot_)

    std::string str;
    Json::Reader json_reader;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(M_gc_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(M_gc_inv_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_gc_cc_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_gc_grav_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(q_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(dq_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(pos_com_)

    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(rbdyn_tree_)
    MACRO_SER_ARGOBJ_RETJSONVAL(mass_)
    MACRO_SER_ARGOBJ_RETJSONVAL(computed_spatial_transformation_and_inertia_)

    ret_json_val["processing_order_"] = Json::Value(Json::arrayValue);
    for(auto&&element : arg_obj.processing_order_)
    { ret_json_val["processing_order_"].append(element);  }

    // This is a normal c style array. So can't use fancy methods to parse it.
    ret_json_val["vec_scratch_"] = Json::Value(Json::arrayValue);
    for(auto i : {0,1,2,3,4})
    {
      Json::Value val;
      scl_util::eigentoStringArrayJSON(arg_obj.vec_scratch_[i], str);
      json_reader.parse(str, val);
      ret_json_val["vec_scratch_"].append(val);
    }

    return true;
  }

  template<> bool serializeToJSON<SRobotSensors>(const SRobotSensors &arg_obj, Json::Value &ret_json_val)
  {
    //Read in the Eigen matrix types..
    std::string str;
    Json::Reader json_reader;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(q_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(dq_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(ddq_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_gc_measured_)

    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(forces_external_)
    return true;
  }

  template<> bool serializeToJSON<SRobotActuators>(const SRobotActuators &arg_obj, Json::Value &ret_json_val)
  {
    //Read in the Eigen matrix types..
    std::string str;
    Json::Reader json_reader;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_gc_commanded_)

    /** NOTE TODO :
    sutil::CMappedPointerList<std::string, SActuatorSetBase, false> actuator_sets_;*/
    return true;
  }

  template<> bool serializeToJSON<SRobotIO>(const SRobotIO &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    MACRO_SER_ARGOBJ_RETJSONVAL(name_robot_)
    MACRO_SER_ARGOBJ_RETJSONVAL(dof_)
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(sensors_)
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(actuators_)
    return true;
  }

  template<> bool serializeToJSON<SRobotParsed>(const SRobotParsed &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    MACRO_SER_ARGOBJ_RETJSONVAL(dof_)
    MACRO_SER_ARGOBJ_RETJSONVAL(log_file_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_apply_gc_damping_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_apply_gc_pos_limits_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_apply_actuator_force_limits_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_apply_actuator_pos_limits_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_apply_actuator_vel_limits_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_apply_actuator_acc_limits_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_controller_on_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_logging_on_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_wireframe_on_)
    MACRO_SER_ARGOBJ_RETJSONVAL(option_axis_frame_size_)
    MACRO_SER_ARGOBJ_RETJSONVAL(option_muscle_via_pt_sz_)

    //Required for the Eigen serialization macro to work
    Json::Reader json_reader;
    std::string str;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(gc_pos_limit_max_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(gc_pos_limit_min_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(gc_pos_default_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(damping_gc_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(actuator_forces_max_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(actuator_forces_min_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(gravity_)

    //Special cases : Mapped tree as a list..
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(rb_tree_)

    ret_json_val["robot_tree_numeric_id_to_name_"] = Json::Value(Json::arrayValue);
    for (auto&& element: arg_obj.robot_tree_numeric_id_to_name_)
    { ret_json_val["robot_tree_numeric_id_to_name_"].append(element); }

    //NOTE TODO : Handle this later...
    // arg_obj.actuator_sets_

    return true;
  }

  template<> bool serializeToJSON<SUIParsed>(const SUIParsed &arg_obj, Json::Value &ret_json_val)
  {  return false;  }

  template<> bool serializeToJSON<SCmdLineOptions_OneRobot>(const SCmdLineOptions_OneRobot &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    MACRO_SER_ARGOBJ_RETJSONVAL(name_file_config_)
    MACRO_SER_ARGOBJ_RETJSONVAL(name_robot_)
    MACRO_SER_ARGOBJ_RETJSONVAL(name_ctrl_)
    MACRO_SER_ARGOBJ_RETJSONVAL(name_graphics_)
    MACRO_SER_ARGOBJ_RETJSONVAL(name_actuator_set_)
    MACRO_SER_ARGOBJ_RETJSONVAL(id_time_created_str_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_pause_at_start_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_pause_at_start_)
    MACRO_SER_ARGOBJ_RETJSONVAL(flag_is_redis_master_)

    // Std vector of strings (iterable)
    ret_json_val["name_tasks_"] = Json::Value(Json::arrayValue);
    // C++11 auto iterator (compact!)
    for (auto&& element: arg_obj.name_tasks_) {
      Json::Value tmp;
      serializeToJSON(element,tmp);
      ret_json_val["name_tasks_"].append(tmp);
    }

    return true;
  }


  template<>  bool serializeToJSON<STaskBase>(const STaskBase &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    MACRO_SER_ARGOBJ_RETJSONVAL(type_task_)
    MACRO_SER_ARGOBJ_RETJSONVAL(has_been_activated_)
    MACRO_SER_ARGOBJ_RETJSONVAL(has_control_null_space_)
    MACRO_SER_ARGOBJ_RETJSONVAL(priority_)
    MACRO_SER_ARGOBJ_RETJSONVAL(dof_task_)

    //Required for the Eigen serialization macro to work
    Json::Reader json_reader;
    std::string str;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(J_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(J_6_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(J_dyn_inv_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(null_space_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(M_task_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(M_task_inv_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_task_cc_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_task_grav_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_task_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_task_max_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_task_min_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(force_gc_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(range_space_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(kp_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(kv_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(ka_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(ki_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(shared_data_)

    // Std vector of strings (iterable)
    ret_json_val["task_nonstd_params_"] = Json::Value(Json::arrayValue);
    // C++11 auto iterator (compact!)
    for (auto&& element: arg_obj.task_nonstd_params_) {
      Json::Value tmp;
      serializeToJSON(element.data_[0]+std::string(" : ") +element.data_[1],tmp);
      ret_json_val["task_nonstd_params_"].append(tmp);
    }

    // Also save the pointers
    // NOTE TODO : Think about whether this is necessary.. Reconsider doing this if it's not a good idea.
    std::stringstream ss;
    ss << static_cast<const void*>(arg_obj.parent_controller_);
    ret_json_val["parent_controller_"] = ss.str(); ss.clear();
    ss << static_cast<const void*>(arg_obj.robot_);
    ret_json_val["robot_"] = ss.str(); ss.clear();
    ss << static_cast<const void*>(arg_obj.gc_model_);
    ret_json_val["gc_model_"] = ss.str(); ss.clear();

    return true;
  }

  template<>  bool serializeToJSON<SNonControlTaskBase>(const SNonControlTaskBase &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    MACRO_SER_ARGOBJ_RETJSONVAL(type_task_)
    MACRO_SER_ARGOBJ_RETJSONVAL(has_been_activated_)

    // Std vector of strings (iterable)
    ret_json_val["task_nonstd_params_"] = Json::Value(Json::arrayValue);
    // C++11 auto iterator (compact!)
    for (auto&& element: arg_obj.task_nonstd_params_) {
      Json::Value tmp;
      serializeToJSON(element.data_[0]+std::string(" : ") +element.data_[1],tmp);
      ret_json_val["task_nonstd_params_"].append(tmp);
    }

    // Also save the pointers
    // NOTE TODO : Think about whether this is necessary.. Reconsider doing this if it's not a good idea.
    std::stringstream ss;
    ss << static_cast<const void*>(arg_obj.parent_controller_);
    ret_json_val["parent_controller_"] = ss.str(); ss.clear();

    return true;
  }

  template<>  bool serializeToJSON<SServo>(const SServo &arg_obj, Json::Value &ret_json_val)
  { return false; }

  template<>  bool serializeToJSON<STaskOpPosPIDA1OrderInfTime>(const STaskOpPosPIDA1OrderInfTime &arg_obj, Json::Value &ret_json_val)
  { return false; }

  template<>  bool serializeToJSON<STaskGc>(const STaskGc &arg_obj, Json::Value &ret_json_val)
  { return false; }

  template<>  bool serializeToJSON<STaskGcLimitCentering>(const STaskGcLimitCentering &arg_obj, Json::Value &ret_json_val)
  { return false; }

  template<>  bool serializeToJSON<STaskOpPosNoGravity>(const STaskOpPosNoGravity &arg_obj, Json::Value &ret_json_val)
  { return false; }

  template<>  bool serializeToJSON<STaskComPos>(const STaskComPos &arg_obj, Json::Value &ret_json_val)
  { return false; }

  template<>  bool serializeToJSON<STaskGcSet>(const STaskGcSet &arg_obj, Json::Value &ret_json_val)
  { return false; }

  template<>  bool serializeToJSON<STaskOpPos>(const STaskOpPos &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON<STaskBase>(*dynamic_cast<const STaskBase *>(&arg_obj), ret_json_val);

    // NOTE : Adding the task-specific fields..
    MACRO_SER_ARGOBJ_RETJSONVAL(parent_controller_->name_)

    // To use the eigen json macro.
    Json::Reader json_reader;
    std::string str;
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(x_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(dx_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(ddx_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(x_goal_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(dx_goal_)
    MACRO_SER_ARGOBJ_RETJSONVAL_Eigen(ddx_goal_)

    return flag;
  }

  template<>  bool serializeToJSON<STaskNullSpaceDamping>(const STaskNullSpaceDamping &arg_obj, Json::Value &ret_json_val)
  { return false; }


  template<>  bool serializeToJSON<SControllerBase>(const SControllerBase &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SObject*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    /** Name of the robot */
    MACRO_SER_ARGOBJ_RETJSONVAL(robot_name_)

    // NOTE TODO : Think about whether this should be added (or somehow linked)
    // const SRobotParsed* robot_;
    // SRobotIO* io_data_;
    // SGcModel* gc_model_;

    return true;
  }

  template<>  bool serializeToJSON<SControllerGc>(const SControllerGc &arg_obj, Json::Value &ret_json_val)
  { return false; }

  /** Note this isn't as easy as the more elementary data structures. This is because it contains
   * complex data types that don't fit into the macro style.. */
  template<>  bool serializeToJSON<SControllerMultiTask>(const SControllerMultiTask &arg_obj, Json::Value &ret_json_val)
  {
    bool flag = serializeToJSON(*dynamic_cast<const SControllerBase*>(&arg_obj), ret_json_val);
    if(!flag) { return false; }

    // The servo data structure..
    MACRO_SER_ARGOBJ_RETJSONVAL_MemberObj(servo_)

    // Note, since we specialized the mapped list with a pointer member, we can directly use the
    // associated template function.
    const sutil::CMappedList<std::string, STaskBase*> *tmp =
        dynamic_cast<const sutil::CMappedList<std::string, STaskBase*> *>(& (arg_obj.tasks_));
    flag = serializeToJSON(*tmp, ret_json_val);
    if(!flag) { return false; }

    // Note, since we specialized the mapped list with a pointer member, we can directly use the
    // associated template function.
    const sutil::CMappedList<std::string, SNonControlTaskBase*> *tmp2 =
        dynamic_cast<const sutil::CMappedList<std::string, SNonControlTaskBase*> *>(& (arg_obj.tasks_non_ctrl_));
    flag = serializeToJSON(*tmp2, ret_json_val);
    if(!flag) { return false; }

    MACRO_SER_ARGOBJ_RETJSONVAL(option_servo_to_model_rate_)

    return true;
  }

  /** *****************************************************************************
   *                        Deserialize data to an object
   * **************************************************************************** */
  template<> bool deserializeFromJSON<sUInt>(sUInt &ret_obj, const Json::Value &arg_json_val)
  { ret_obj = arg_json_val.asInt(); return true; }

  template<> bool deserializeFromJSON(sSpatialXForm &ret_obj, const Json::Value &arg_json_val)
  {
    bool flag = scl_util::eigenFromJSON(ret_obj, arg_json_val);
    return flag;
  }

  template<> bool deserializeFromJSON<SObject>(SObject &ret_obj, const Json::Value &arg_json_val)
  {
    // NOTE TODO : Why is this commented out?
    // MACRO_DESER_RETOBJ_ARGJSONVAL(has_been_init_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(name_,asString)
    return true;
  }

  template<> bool deserializeFromJSON<SMusclePointParsed>(SMusclePointParsed &ret_obj, const Json::Value &arg_json_val)
  {
    MACRO_DESER_RETOBJ_ARGJSONVAL(parent_link_,asString)
    MACRO_DESER_RETOBJ_ARGJSONVAL(position_on_muscle_,asInt)

    bool flag;
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(pos_in_parent_)
    return flag;
  }

  template<> bool deserializeFromJSON<SMuscleParsed>(SMuscleParsed &ret_obj, const Json::Value &arg_json_val)
  {
    //Typical data
    MACRO_DESER_RETOBJ_ARGJSONVAL(optimal_fiber_length_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(tendon_slack_length_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(pennation_angle_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(activation_time_constt_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(max_contraction_vel_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(deactivation_time_constt_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(max_contraction_vel_low_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(max_contraction_vel_high_ ,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(max_tendon_strain_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(max_muscle_strain_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(stiffness_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(damping_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(stiffness_tendon_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(muscle_type_,asString)
    MACRO_DESER_RETOBJ_ARGJSONVAL(name_,asString)
    // Std vector of strings (iterable)
    for (auto& json_element : arg_json_val["points_"]) {
      SMusclePointParsed point;
      deserializeFromJSON(point, json_element);
      ret_obj.points_.push_back(point);
    }

    return true;
  }

  template<> bool deserializeFromJSON<SActuatorSetMuscleParsed>(SActuatorSetMuscleParsed &ret_obj, const Json::Value &arg_json_val)
  {
    bool flag = deserializeFromJSON(*dynamic_cast<SObject *>(&ret_obj), arg_json_val);
    if (!flag) { return false; }

    // Typical data
    MACRO_DESER_RETOBJ_ARGJSONVAL(render_muscle_thickness_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(render_muscle_via_pt_sz_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(muscles_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(muscle_name_to_id_)

    return true;
  }

  template<> bool deserializeFromJSON<SActuatorSetParsed>(SActuatorSetParsed &ret_obj, const Json::Value &arg_json_val)
  {
    if (ret_obj.getType() == "SActuatorSetMuscleParsed")
    {
      SActuatorSetMuscleParsed *obj = dynamic_cast<SActuatorSetMuscleParsed *>(&ret_obj);
      if (obj) { return deserializeFromJSON(*obj, arg_json_val); }
    }
    return true;
  }

  template<> bool deserializeFromJSON<SParserData>(SParserData &ret_obj, const Json::Value &arg_json_val)
  {
    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(graphics_worlds_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(robots_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(muscle_sets_)
    return true;
  }


  template<> bool deserializeFromJSON<SDatabase>(SDatabase &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SForce>(SForce &ret_obj, const Json::Value &arg_json_val)
  {
    // Get the SObject
    scl::SObject *tmp_obj = dynamic_cast<SObject *>(&ret_obj);
    bool flag = deserializeFromJSON(*tmp_obj, arg_json_val);
    if(!flag){ return false; }

    // Now get the force information
    // ret_obj.robot_->name_ = arg_json_val["robot_name_"].asString();
    MACRO_DESER_RETOBJ_ARGJSONVAL(link_name_,asString)

    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(pos_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(direction_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(J_)
    return true;
  }

  template<> bool deserializeFromJSON<SGraphicsParsed::SLight>(SGraphicsParsed::SLight &ret_obj, const Json::Value &arg_json_val)
  {
    if(!arg_json_val.isMember("lookat_")) {
      std::cout<<"\n deserializeFromJSON() Error : Could not find : "<<"lookat_"<<std::flush;
      return false;
    }

    if(!arg_json_val.isMember("pos_")) {
      std::cout<<"\n deserializeFromJSON() Error : Could not find : "<<"pos_"<<std::flush;
      return false;
    }

    // Get the vectors.
    for (int i = 0; i < 3; i++) {
      ret_obj.lookat_[i] = arg_json_val["link_name_"][i].asFloat();
      ret_obj.pos_[i] = arg_json_val["pos_"][i].asFloat();
    }

    return true;
  }

  template<> bool deserializeFromJSON<SGraphicsParsed>(SGraphicsParsed &ret_obj, const Json::Value &arg_json_val)
  {
    bool flag = deserializeFromJSON(*dynamic_cast<SObject *>(&ret_obj), arg_json_val);
    if (!flag) { return false; }

    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(cam_pos_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(cam_lookat_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(cam_up_)

    if(!arg_json_val.isMember("cam_clipping_dist_")) {
      std::cout<<"\n deserializeFromJSON() Error : Could not find : "<<"cam_clipping_dist_"<<std::flush;
      return false;
    }
    ret_obj.cam_clipping_dist_[0] = arg_json_val["cam_clipping_dist_"][0].asFloat();
    ret_obj.cam_clipping_dist_[1] = arg_json_val["cam_clipping_dist_"][1].asFloat();

    MACRO_DESER_RETOBJ_ARGJSONVAL(file_background_,asString)
    MACRO_DESER_RETOBJ_ARGJSONVAL(file_foreground_,asString)

    if(!arg_json_val.isMember("background_color_")) {
      std::cout<<"\n deserializeFromJSON() WARNING : Could not find (optional) : background_color_. Setting to (1,1,1)"<<std::flush;
      for (int i = 0; i < 3; i++)
      { ret_obj.background_color_[i] = 1.0; } // white by default
    }
    else{
      for (int i = 0; i < 3; i++) {
        ret_obj.background_color_[i] = arg_json_val["background_color_"][i].asFloat();
      }
    }

    if(!arg_json_val.isMember("lights_")) {
      std::cout<<"\n deserializeFromJSON() Error : Could not find : "<<"lights_"<<std::flush;
      return false;
    }
    for (auto& json_light : arg_json_val["lights_"]) {
      SGraphicsParsed::SLight light;
      flag = deserializeFromJSON(light, json_light);
      if(false == flag) { return false; }
      ret_obj.lights_.push_back(light);
    }

    return true;
}

  template<> bool deserializeFromJSON<SRigidBodyGraphics>(SRigidBodyGraphics &ret_obj, const Json::Value &arg_json_val)
  {
    MACRO_DESER_RETOBJ_ARGJSONVAL(collision_type_,asInt)
    MACRO_DESER_RETOBJ_ARGJSONVAL(file_name_,asString)
    ret_obj.class_ = static_cast<SRigidBodyGraphics::EGraphicObjectType>(arg_json_val["class_"].asInt());

    bool flag;
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(pos_in_parent_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(ori_parent_quat_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(scaling_)

    if(!arg_json_val.isMember("color_")) {
      std::cout<<"\n deserializeFromJSON() WARNING : Could not find : color_. Setting to gray (.5,.5,.5)"<<std::flush;
      for (int i = 0; i < 3; i++) { ret_obj.color_[i] = 0.5; }
    }
    else{
      for (int i = 0; i < 3; i++)
      { ret_obj.color_[i] = arg_json_val["color_"].asFloat(); }
    }

    return true;
  }

  template<> bool deserializeFromJSON<SRigidBody>(SRigidBody &ret_obj, const Json::Value &arg_json_val)
  {
    scl::SObject *tmp_obj = dynamic_cast<SObject*>(&ret_obj);
    bool flag = deserializeFromJSON(*tmp_obj, arg_json_val);
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
  {
    bool flag = deserializeFromJSON(*dynamic_cast<SObject *>(&ret_obj), arg_json_val);
    if (!flag) { return false; }

    // ret_obj.link_ds_->name_ = arg_json_val["name_"].asString();

    MACRO_DESER_RETOBJ_ARGJSONVAL(q_T_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(parent_name_,asString)

    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(T_o_lnk_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(T_lnk_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(sp_q_T_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(sp_dq_T_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(sp_inertia_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(spatial_acceleration_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(spatial_force_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(sp_X_within_link_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(sp_X_o_lnk_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(sp_S_joint_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(sp_Sorth_joint_)

    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(sp_X_joint_)

    return true;
}

  template<> bool deserializeFromJSON<SGcModel>(SGcModel &ret_obj, const Json::Value &arg_json_val)
  {
    bool flag = deserializeFromJSON(*dynamic_cast<SObject *>(&ret_obj), arg_json_val);
    if (!flag) { return false; }

    MACRO_DESER_RETOBJ_ARGJSONVAL(name_robot_,asString)

    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(M_gc_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(M_gc_inv_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_gc_cc_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_gc_grav_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(q_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(dq_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(pos_com_)

    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(rbdyn_tree_)
    MACRO_DESER_RETOBJ_ARGJSONVAL(mass_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(computed_spatial_transformation_and_inertia_,asBool)

    for (auto &element : arg_json_val) {
      ret_obj.processing_order_.push_back(element.asString());
    }

    // This is a normal c style array. So can't use fancy methods to parse it.
    int i = 0;
    for (auto &element : arg_json_val) {
      scl_util::eigenFromJSON(ret_obj.vec_scratch_[i], element);
      i++;
    }

    return true;
  }


  template<> bool deserializeFromJSON<SRobotSensors>(SRobotSensors &ret_obj, const Json::Value &arg_json_val)
  {
    //Read in the Eigen matrix types..
    bool flag;
    std::string str;
    Json::Reader json_reader;

    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(q_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(dq_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(ddq_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_gc_measured_)

    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(forces_external_)
    return true;
  }

  template<> bool deserializeFromJSON<SRobotActuators>(SRobotActuators &ret_obj, const Json::Value &arg_json_val)
  {
    bool flag;
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_gc_commanded_)

    /** NOTE TODO :
    sutil::CMappedPointerList<std::string, SActuatorSetBase, false> actuator_sets_;*/
    return flag;
  }

  template<> bool deserializeFromJSON<SRobotIO>(SRobotIO &ret_obj, const Json::Value &arg_json_val)
  {
    scl::SObject *tmp_obj = dynamic_cast<SObject*>(&ret_obj);
    bool flag = deserializeFromJSON(*tmp_obj, arg_json_val);
    if(!flag){ return false; }

    MACRO_DESER_RETOBJ_ARGJSONVAL(name_robot_,asString)
    MACRO_DESER_RETOBJ_ARGJSONVAL(dof_,asDouble)
    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(sensors_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(actuators_)
    return true;
  }

  template<> bool deserializeFromJSON<SRobotParsed>(SRobotParsed &ret_obj, const Json::Value &arg_json_val)
  {
    bool flag = deserializeFromJSON(*dynamic_cast<SObject *>(&ret_obj), arg_json_val);
    if (!flag) { return false; }

    MACRO_DESER_RETOBJ_ARGJSONVAL(dof_,asInt)
    MACRO_DESER_RETOBJ_ARGJSONVAL(log_file_,asString)
    MACRO_DESER_RETOBJ_ARGJSONVAL(flag_apply_gc_damping_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(flag_apply_gc_pos_limits_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(flag_apply_actuator_force_limits_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(flag_apply_actuator_pos_limits_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(flag_apply_actuator_vel_limits_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(flag_apply_actuator_acc_limits_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(flag_controller_on_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(flag_logging_on_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(flag_wireframe_on_,asBool)
    MACRO_DESER_RETOBJ_ARGJSONVAL(option_axis_frame_size_,asFloat)
    MACRO_DESER_RETOBJ_ARGJSONVAL(option_muscle_via_pt_sz_,asFloat)

    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(gc_pos_limit_max_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(gc_pos_limit_min_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(gc_pos_default_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(damping_gc_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(actuator_forces_max_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(actuator_forces_min_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(gravity_)

    MACRO_DESER_RETOBJ_ARGJSONVAL_MemberObj(rb_tree_)

    for (auto& element : arg_json_val["robot_tree_numeric_id_to_name_"]) {
        ret_obj.robot_tree_numeric_id_to_name_.push_back(element.asString());
    }

    //NOTE TODO : Handle this later...
    // arg_obj.actuator_sets_

    return true;
}

  template<> bool deserializeFromJSON<SUIParsed>(SUIParsed &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SCmdLineOptions_OneRobot>(SCmdLineOptions_OneRobot &ret_obj, const Json::Value &arg_json_val)
  { return false; }

  template<> bool deserializeFromJSON<STaskBase>(STaskBase &ret_obj, const Json::Value &arg_json_val)
  {
    bool flag = deserializeFromJSON(*dynamic_cast<SObject *>(&ret_obj), arg_json_val);
    if (!flag) { return false; }

    MACRO_DESER_RETOBJ_ARGJSONVAL(type_task_,asString)
    MACRO_DESER_RETOBJ_ARGJSONVAL(priority_,asUInt)
    MACRO_DESER_RETOBJ_ARGJSONVAL(dof_task_,asUInt)

    // I don't think we should deserialize these (since they are dynamic quantities).
    // It's okay to serialize them (for debugging/visualization).
    // But I'll leave them anyway.
    //MACRO_DESER_RETOBJ_ARGJSONVAL(has_been_activated_,asBool)
    //MACRO_DESER_RETOBJ_ARGJSONVAL(has_control_null_space_,asBool)

    std::string str;
    Json::Reader json_reader;

    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(kp_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(kv_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(ka_)
    MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(ki_)

    // I don't think we should deserialize these (since they are dynamic quantities).
    // It's okay to serialize them (for debugging/visualization).
    // But I'll leave them anyway.
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(J_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(J_6_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(J_dyn_inv_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(null_space_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(M_task_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(M_task_inv_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_task_cc_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_task_grav_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_task_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_task_max_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_task_min_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(force_gc_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(range_space_)
    //MACRO_DESER_RETOBJ_ARGJSONVAL_Eigen(shared_data_)

    // Std vector of strings (iterable)
    /* This is how these were encoded... "str:str"
      // Std vector of strings (iterable)
      ret_json_val["task_nonstd_params_"] = Json::Value(Json::arrayValue);
      // C++11 auto iterator (compact!)
      for (auto&& element: arg_obj.task_nonstd_params_) {
        Json::Value tmp;
        serializeToJSON(element.data_[0]+std::string(" : ") +element.data_[1],tmp);
        ret_json_val["task_nonstd_params_"].append(tmp);
      }
     */
    for (auto itr : arg_json_val["task_nonstd_params_"]) {
      std::vector<std::string> tmp_vec;
      flag = scl_util::splitString(itr.asString(), ':', tmp_vec);
      if(false == flag || tmp_vec.size() != 2)
      { std::cout<<"\n ERROR : deserializeFromJSON<STaskBase>() failed at task_nonstd_params; should have contained two strings separated by a : ('str:str')";  return false; }

      scl::sString2 str2;
      str2.data_[0] = tmp_vec[0];
      str2.data_[1] = tmp_vec[1];
      ret_obj.task_nonstd_params_.push_back(str2);
    }

    //Extra function to init subclass
    ret_obj.initTaskParams();

    return true;
  }

  template<> bool deserializeFromJSON<SNonControlTaskBase>(SNonControlTaskBase &ret_obj, const Json::Value &arg_json_val)
  {
    bool flag = deserializeFromJSON(*dynamic_cast<SObject *>(&ret_obj), arg_json_val);
    if (!flag) { return false; }

    MACRO_DESER_RETOBJ_ARGJSONVAL(type_task_,asString)

    // Std vector of strings (iterable)
    /* This is how these were encoded... "str:str"
      // Std vector of strings (iterable)
      ret_json_val["task_nonstd_params_"] = Json::Value(Json::arrayValue);
      // C++11 auto iterator (compact!)
      for (auto&& element: arg_obj.task_nonstd_params_) {
        Json::Value tmp;
        serializeToJSON(element.data_[0]+std::string(" : ") +element.data_[1],tmp);
        ret_json_val["task_nonstd_params_"].append(tmp);
      }
     */
    for (auto itr : arg_json_val["task_nonstd_params_"]) {
      std::vector<std::string> tmp_vec;
      flag = scl_util::splitString(itr.asString(), ':', tmp_vec);
      if(false == flag || tmp_vec.size() != 2)
      { std::cout<<"\n ERROR : deserializeFromJSON<STaskBase>() failed at task_nonstd_params; should have contained two strings separated by a : ('str:str')";  return false; }

      scl::sString2 str2;
      str2.data_[0] = tmp_vec[0];
      str2.data_[1] = tmp_vec[1];
      ret_obj.task_nonstd_params_.push_back(str2);
    }

    //Extra function to init subclass
    ret_obj.initTaskParams();

    return true;
  }

  template<> bool deserializeFromJSON<SServo>(SServo &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<STaskOpPosPIDA1OrderInfTime>(STaskOpPosPIDA1OrderInfTime &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<STaskGc>(STaskGc &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<STaskGcLimitCentering>(STaskGcLimitCentering &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<STaskOpPosNoGravity>(STaskOpPosNoGravity &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<STaskComPos>(STaskComPos &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<STaskGcSet>(STaskGcSet &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<STaskOpPos>(STaskOpPos &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<STaskNullSpaceDamping>(STaskNullSpaceDamping &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }


  template<> bool deserializeFromJSON<SControllerBase>(SControllerBase &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SControllerGc>(SControllerGc &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

  template<> bool deserializeFromJSON<SControllerMultiTask>(SControllerMultiTask &ret_obj, const Json::Value &arg_json_val)
  {  return false;  }

}
