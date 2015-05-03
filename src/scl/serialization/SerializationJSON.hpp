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
/* \file SerializationJSON.hpp
 *
 *  Created on: Dec 31, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SERIALIZATIONJSON_HPP_
#define SERIALIZATIONJSON_HPP_

#include <scl/data_structs/SDatabase.hpp>

#include <scl/control/data_structs/SControllerBase.hpp>
#include <scl/control/gc/data_structs/SControllerGc.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPosPIDA1OrderInfTime.hpp>
#include <scl/control/task/tasks/data_structs/STaskGc.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcLimitCentering.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPosNoGravity.hpp>
#include <scl/control/task/tasks/data_structs/STaskComPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcSet.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskNullSpaceDamping.hpp>
#include <scl/control/task/data_structs/SControllerMultiTask.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>
#include <scl/control/task/data_structs/SNonControlTaskBase.hpp>
#include <scl/control/task/data_structs/SServo.hpp>

#include <jsoncpp/json/json.h>
#include <string>

namespace scl
{
  /** *****************************************************************************
   *                          Serialize data to a string
   * **************************************************************************** */
  /** Serialize object into JSON value.
   * Note that this function has to be specialized for each template arg (done
   * in the cpp file).
   *
   * Points to note:
   * * Public data in objects is serialized as is with identical variable names
   *   and class structure
   * * Meta data in objects is serialized using a "__var" notation. For instance,
   *   The sort ordering in a mapped list.
   *
   * To avoid having to re-compile this function over and over again, we'll
   * use C++11's new extern template feature.
   * http://www.stroustrup.com/C++11FAQ.html#extern-templates
   *
   * The SCL library will compile the actual specializations for different native
   * data types. Clients are free to add specilizations for their custom data types.
   * The scl lib sets the following directive to achive this:
   *
   *   #define SCL_LIBRARY_COMPILE_FLAG
   *
   * Clients should ideally not set this flag during the compile process.
   *
   * Client code should not worry about this and go about using the template
   * with any SCL data type. Unrecognized data type entries will simply return
   * false.*/
  template <typename T>
  bool serializeToJSON(const T &arg_obj, Json::Value &ret_json_val)
  { return false; }

  /** Overload the function to handle mapped lists.
   *
   * Note: This isn't a nice (read straightforward) process. Ideally this would
   * involve a partial specialization of the template function above. However,
   * C++ doesn't allow partial specialization for functions. So we have to instead
   * overload the function to get the desired results.
   * Notably, some people caution against this sort of stuff (Herb Sutter).
   * NOTE TODO : Find a better solution to this if possible.
   *
   * (Implemented in the same file further below)..
   */
  template <typename T>
  bool serializeToJSON(const sutil::CMappedList<std::string,T> &arg_mlist, Json::Value &ret_json_val);
  /** We will parse lists of pointers by dereferencing the pointers (we get full objects) */
  template <typename T>
  bool serializeToJSON(const sutil::CMappedList<std::string,T*> &arg_mlist, Json::Value &ret_json_val);
  /** We will parse trees as lists (the tree information is in the data structure anyway) */
  template <typename T>
  bool serializeToJSON(const sutil::CMappedTree<std::string,T> &arg_mlist, Json::Value &ret_json_val);

  /** Serialize object into JSON string. Note strings are also YAML compatible.
   *
   * This is merely a convenience wrapper around the actual JSON value object.
   *
   * Can generate a string in a nice human-readable format if you set style_fast to
   * false.
   *
   * If efficiency is your thing (i.e., you are calling this object on a regular
   * basis) then use the actual JSON value. */
  template <typename T>
  bool serializeToJSONString(const T &arg_obj, std::string &ret_str, bool style_fast=true);

  /** *****************************************************************************
   *                        Deserialize data to an object
   * **************************************************************************** */
  /** Deserialize object from JSON value.
   *  Also see comment for the serialize function to understand design decisions.. */
  template <typename T>
  bool deserializeFromJSON(T &ret_obj, const Json::Value &arg_json_val)
  { return false; }

  /** Overload the function to handle mapped lists.
   *  Also see comment for the serialize function to understand design decisions..
   *  (Implemented in the same file further below).. */
  template <typename T>
  bool deserializeFromJSON(sutil::CMappedList<std::string,T> &ret_mlist, const Json::Value &arg_json_val);

  /** Deserialize object from JSON string. Note strings are also YAML compatible.
   * This is merely a convenience wrapper around the actual JSON value object.
   * If efficiency is your thing (i.e., you are calling this object on a regular
   * basis) then use the actual JSON value. */
  template <typename T>
  bool deserializeFromJSONString(T &ret_obj, const std::string &arg_str);


  /** *****************************************************************************
   * *****************************************************************************
   *                          *******************************
   *                           Random implementation details:
   *                                     Serializer
   *                          *******************************
   * *****************************************************************************
   * ***************************************************************************** */
  // This looks complicated, but helps speed up compilation. Not setting this flag marks the template
  // specialization functions as extern for client apps, which means that they won't be compiled.
  // Only the scl_lib will set this flag, and will compile all the specialized functions (from the cpp file).
#ifndef SCL_LIBRARY_COMPILE_FLAG
  extern template bool serializeToJSON<sUInt>(const sUInt &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON(const sSpatialXForm &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SMusclePointParsed>(const SMusclePointParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SMuscleParsed>(const SMuscleParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SActuatorSetMuscleParsed>(const SActuatorSetMuscleParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SActuatorSetParsed>(const SActuatorSetParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SParserData>(const SParserData &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SDatabase>(const SDatabase &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SForce>(const SForce &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SGraphicsParsed>(const SGraphicsParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SObject>(const SObject &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRigidBodyGraphics>(const SRigidBodyGraphics &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRigidBody>(const SRigidBody &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRigidBodyDyn>(const SRigidBodyDyn &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SGcModel>(const SGcModel &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRigidBody>(const SRigidBody &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRobotSensors>(const SRobotSensors &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRobotActuators>(const SRobotActuators &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRobotIO>(const SRobotIO &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRobotParsed>(const SRobotParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SUIParsed>(const SUIParsed &arg_obj, Json::Value &ret_json_val);

  // The controller data structures
  extern template bool serializeToJSON<STaskBase>(const STaskBase &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SNonControlTaskBase>(const SNonControlTaskBase &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SServo>(const SServo &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<STaskOpPosPIDA1OrderInfTime>(const STaskOpPosPIDA1OrderInfTime &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<STaskGc>(const STaskGc &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<STaskGcLimitCentering>(const STaskGcLimitCentering &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<STaskOpPosNoGravity>(const STaskOpPosNoGravity &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<STaskComPos>(const STaskComPos &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<STaskGcSet>(const STaskGcSet &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<STaskOpPos>(const STaskOpPos &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<STaskNullSpaceDamping>(const STaskNullSpaceDamping &arg_obj, Json::Value &ret_json_val);

  extern template bool serializeToJSON<SControllerBase>(const SControllerBase &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SControllerGc>(const SControllerGc &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SControllerMultiTask>(const SControllerMultiTask &arg_obj, Json::Value &ret_json_val);
#endif

  template <typename T> bool serializeToJSON(const sutil::CMappedList<std::string,T> &arg_mlist, Json::Value &ret_json_val)
  {
    //Add the index and object values for each entry
    auto it = arg_mlist.begin(), ite = arg_mlist.end();
    for(;it!=ite;++it)
    {
      // Temps for code clarity
      const T& data = *it;
      const std::string& index = !it;
      Json::Value &val = ret_json_val[index.c_str()];

      if(false == serializeToJSON(*it, val))
      { ret_json_val = Json::Value(Json::arrayValue); return false; }
    }

    //Add sorting information if present.
    if(arg_mlist.isSorted())
    {
      ret_json_val["__is_sorted"] = arg_mlist.isSorted();
      std::vector<std::string> sort_order;
      if(false == arg_mlist.sort_get_order(sort_order))
      { return false; }

      ret_json_val["__sort_order"] = Json::Value(Json::arrayValue);
      // C++11 auto iterator (compact!)
      for (auto&& element: sort_order) {
        ret_json_val["__sort_order"].append(element);
      }
    }

    return true;
  }

  /** Get the pointer deref */
  template <typename T> bool serializeToJSON(const sutil::CMappedList<std::string,T*> &arg_mlist, Json::Value &ret_json_val)
  {
    //Add the index and object values for each entry
    auto it = arg_mlist.begin(), ite = arg_mlist.end();
    for(;it!=ite;++it)
    {
      // Temps for code clarity
      const T& data = **it;
      const std::string& index = !it;
      Json::Value &val = ret_json_val[index.c_str()];

      if(false == serializeToJSON(**it, val))
      { ret_json_val = Json::Value(Json::arrayValue); return false; }
    }

    //Add sorting information if present.
    if(arg_mlist.isSorted())
    {
      ret_json_val["__is_sorted"] = arg_mlist.isSorted();
      std::vector<std::string> sort_order;
      if(false == arg_mlist.sort_get_order(sort_order))
      { return false; }

      ret_json_val["__sort_order"] = Json::Value(Json::arrayValue);
      // C++11 auto iterator (compact!)
      for (auto&& element: sort_order) {
        ret_json_val["__sort_order"].append(element);
      }
    }

    return true;
  }

  template <typename T> bool serializeToJSON(const sutil::CMappedTree<std::string,T> &arg_mlist, Json::Value &ret_json_val)
  {
    // Parse it like a mapped list..
    const sutil::CMappedList<std::string,T> &val = arg_mlist;
    return serializeToJSON(val,ret_json_val);
  }

  template <typename T>
  bool serializeToJSONString(const T &arg_obj, std::string &ret_str, bool style_fast)
  {
    Json::Value json_val;
    if( serializeToJSON(arg_obj, json_val) )
    {
      if(style_fast)
      {
        Json::FastWriter json_writer;
        ret_str = json_writer.write(json_val);
      }
      else
      {
        Json::StyledWriter json_writer;
        ret_str = json_writer.write(json_val);
      }
      return true;
    }
    ret_str = "";
    return false;
  }

  /** *****************************************************************************
   *                          *******************************
   *                           Random implementation details:
   *                                     Serializer
   *                          *******************************
   * **************************************************************************** */
  // This looks complicated, but helps speed up compilation. Not setting this flag marks the template
  // specialization functions as extern for client apps, which means that they won't be compiled.
  // Only the scl_lib will set this flag, and will compile all the specialized functions (from the cpp file).
#ifndef SCL_LIBRARY_COMPILE_FLAG
  extern template bool deserializeFromJSON<SMusclePointParsed>(SMusclePointParsed &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SMuscleParsed>(SMuscleParsed &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SActuatorSetMuscleParsed>(SActuatorSetMuscleParsed &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SActuatorSetParsed>(SActuatorSetParsed &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SParserData>(SParserData &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SDatabase>(SDatabase &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SForce>(SForce &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SGraphicsParsed::SLight>(SGraphicsParsed::SLight &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SGraphicsParsed>(SGraphicsParsed &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SObject>(SObject &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SRigidBodyGraphics>(SRigidBodyGraphics &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SRigidBody>(SRigidBody &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SRigidBodyDyn>(SRigidBodyDyn &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SGcModel>(SGcModel &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SRobotSensors>(SRobotSensors &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SRobotActuators>(SRobotActuators &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SRobotIO>(SRobotIO &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SRobotParsed>(SRobotParsed &ret_obj, const Json::Value &arg_json_val);
  extern template bool deserializeFromJSON<SUIParsed>(SUIParsed &ret_obj, const Json::Value &arg_json_val);

  // The controller data structures
  extern template bool deserializeFromJSON<STaskBase>(STaskBase &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<SNonControlTaskBase>(SNonControlTaskBase &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<SServo>(SServo &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<STaskOpPosPIDA1OrderInfTime>(STaskOpPosPIDA1OrderInfTime &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<STaskGc>(STaskGc &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<STaskGcLimitCentering>(STaskGcLimitCentering &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<STaskOpPosNoGravity>(STaskOpPosNoGravity &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<STaskComPos>(STaskComPos &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<STaskGcSet>(STaskGcSet &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<STaskOpPos>(STaskOpPos &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<STaskNullSpaceDamping>(STaskNullSpaceDamping &arg_obj, const Json::Value &ret_json_val);

  extern template bool deserializeFromJSON<SControllerBase>(SControllerBase &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<SControllerGc>(SControllerGc &arg_obj, const Json::Value &ret_json_val);
  extern template bool deserializeFromJSON<SControllerMultiTask>(SControllerMultiTask &arg_obj, const Json::Value &ret_json_val);
#endif

  template <typename T> bool deserializeFromJSON(sutil::CMappedList<std::string,T> &ret_mlist, const Json::Value &arg_json_val)
  { return false; }

  template <typename T>
  bool deserializeFromJSONString(T &ret_obj, const std::string &arg_str)
  {
    Json::Value json_val;
    Json::Reader json_reader;

    if(!json_reader.parse(arg_str,json_val))
    { return false; }

    bool flag = deserializeFromJSON(ret_obj, json_val);
    return flag;
  }

}


#endif /* SERIALIZATIONJSON_HPP_ */
