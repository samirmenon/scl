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
/* \file SCmdLineOptions.hpp
 *
 *  Created on: Apr 1, 2017
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SRC_SCL_DATA_STRUCTS_SCMDLINEOPTIONS_HPP_
#define SRC_SCL_DATA_STRUCTS_SCMDLINEOPTIONS_HPP_

#include <scl/data_structs/SObject.hpp>

#include <string>
#include <vector>
#include <time.h>

namespace scl
{
  /** Reads the command line arguments into a cmd arg structure.
   * There are many common arguments that we will populate into a
   * data structure.
   */
  class SCmdLineOptions_OneRobot : public SObject
  {
  public:
    std::string name_file_config_="";

    std::string name_robot_="";
    std::string name_ctrl_="";
    std::vector<std::string> name_tasks_;

    std::string name_graphics_="";

    std::string name_actuator_set_="";

    /** A unique id for a program that may be specified
     * at the start in the command line args. */
    std::string id_time_created_str_="";

    bool flag_pause_at_start_=false;
    bool flag_muscles_=false;
    /** Setting this to true will enable the program
     * to control redis keys related to itself
     * A few roles for program types:
     *  gui : Controls camera pos, ui points, etc.
     *  py-cam script : Controls camera (disable gui from controlling camera) */
    bool flag_is_redis_master_ = false;

    SCmdLineOptions_OneRobot() : SObject("SCmdLineOptions_OneRobot")
    {
      time_t curtime;
      time(&curtime);
      id_time_created_str_ = ctime(&curtime);
    }
  };
}

#endif /* SRC_SCL_DATA_STRUCTS_SCMDLINEOPTIONS_HPP_ */
