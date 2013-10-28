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
/*
 * SControllerBase.hpp
 *
 *  Created on: Dec 29, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCONTROLLERBASE_HPP_
#define SCONTROLLERBASE_HPP_

#include <string>

#include <scl/data_structs/SRobotParsed.hpp>

#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SRobotIOData.hpp>

#include <scl/data_structs/SGcModel.hpp>

namespace scl
{

  class SControllerBase : public SObject
  {
  public:
    /** Name of the robot */
    std::string robot_name_;

    /** Controller type */
    std::string type_ctrl_ds_;

    /** Pointer to the static robot data (parsed from a file) */
    const SRobotParsed* robot_;

    /** Robot sensor and actuator data */
    SRobotIOData* io_data_;

    /** Pointer to the JSpace model DS. */
    SGcModel gc_model_;

    /** The constructor sets the initialization state to false */
    SControllerBase();

    /** The destructor does nothing */
    virtual ~SControllerBase();

    /** Initializes the data structure */
    virtual sBool init(const std::string & arg_controller_name,
        const SRobotParsed* arg_robot_ds,
        SRobotIOData* arg_io_data);
  };

}

#endif /* SCONTROLLERBASE_HPP_ */
