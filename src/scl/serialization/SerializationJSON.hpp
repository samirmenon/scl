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
  bool serializeToJSONString(const T &arg_obj, std::string &ret_str, bool style_fast=true)
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

  // This looks complicated, but helps speed up compilation. Not setting this flag marks the template
  // specialization functions as extern for client apps, which means that they won't be compiled.
  // Only the scl_lib will set this flag, and will compile all the specialized functions (from the cpp file).
#ifndef SCL_LIBRARY_COMPILE_FLAG
  extern template bool serializeToJSON<SActuatorSetMuscleParsed>(const SActuatorSetMuscleParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SActuatorSetParsed>(const SActuatorSetParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SDatabase>(const SDatabase &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SForce>(const SForce &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SGcModel>(const SGcModel &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SGraphicsParsed>(const SGraphicsParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SObject>(const SObject &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRigidBody>(const SRigidBody &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRigidBodyDyn>(const SRigidBodyDyn &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRigidBody>(const SRigidBody &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRobotIO>(const SRobotIO &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SRobotParsed>(const SRobotParsed &arg_obj, Json::Value &ret_json_val);
  extern template bool serializeToJSON<SUIParsed>(const SUIParsed &arg_obj, Json::Value &ret_json_val);
  //extern template bool serializeToJSON<SVirtualLinkage>(const SVirtualLinkage &arg_obj, Json::Value &ret_json_val);
#endif

  template <typename T> bool serializeToJSON(const sutil::CMappedList<std::string,T> &arg_mlist, Json::Value &ret_json_val)
  {
    //Set the returned value to an empty array
    ret_json_val["data"] = Json::Value(Json::arrayValue);
    auto it = arg_mlist.begin(), ite = arg_mlist.end();
    for(int i=0;it!=ite;++it,++i)
    {
      // Temps for code clarity
      const T& data = *it;
      const std::string& index = !it;
      Json::Value &val = ret_json_val["data"][i];

      if(false == serializeToJSON(*it, val["obj"]))
      { ret_json_val = Json::Value(Json::arrayValue); return false; }

      val["idx"] = index;
    }

    //Add sorting information if present.
    if(arg_mlist.isSorted())
    {
      ret_json_val["is_sorted"] = arg_mlist.isSorted();
      std::vector<std::string> sort_order;
      if(false == arg_mlist.sort_get_order(sort_order))
      { return false; }

      ret_json_val["sort_order"] = Json::Value(Json::arrayValue);
      // C++11 auto iterator (compact!)
      for (auto&& element: sort_order) {
        ret_json_val["sort_order"].append(element);
      }
    }

    return true;
  }

}


#endif /* SERIALIZATIONJSON_HPP_ */
