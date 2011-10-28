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
/* \file DbRegisterFunctions.cpp
 *
 *  Created on: Jul 2, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

/* These are a set of functions to register various data objects
 * with the database.
 *
 * Simplify dealing with external data sources and with
 * data shared across different components.
 */

/** \file DbRegisterFunctions.hpp */

#ifndef DBREGISTERFUNCTIONS_HPP_
#define DBREGISTERFUNCTIONS_HPP_

#include <string>

#include <scl/data_structs/SDatabase.hpp>

#include <scl/parser/CParserBase.hpp>

#include <scl/dynamics/tao/CTaoDynamics.hpp>

namespace scl_registry
{
  /** Loads everything in a file and registers
   * it with the database:
   * 1. Multiple robots
   * 2. Multiple graphics specifications
   * 3. Multiple controllers (initializes controllers to first possible robot)
   * 4. One world
   *
   * @param file_ : The file containing the robot
   *                and world definition
   *
   * @param name_ : A parser implementation (subclass of CParserBase)
   *
   * @return :
   *    success : true
   *    failure : false
   *
   * NOTE : This is a wrapper function that calls other functions.
   *
   * Either iterate over the database to find stuff or user the parser's
   * helper functions to see what was loaded into the database.
   *
   * NOTE TODO : Should return a list of robots, graphics etc. that
   *  were just parsed. Or implement using passable vectors, which will
   *  be filled with the data if they aren't null. */
  bool parseEverythingInFile(const std::string &arg_file,
      scl_parser::CParserBase *arg_parser,
      std::vector<std::string>* arg_robots_parsed=S_NULL,
      std::vector<std::string>* arg_graphics_parsed=S_NULL);

  /** Loads a robot from a file and registers
   * it with the database.
   *
   * Also creates an I/O data structure in the database.
   *
   * Requires the parser implementation to include:
   * bool readRobotFromFile(const std::string &arg_file,
   *                        const std::string& arg_robot_name,
   *                        scl::SRobotParsedData * arg_robot)
   *
   * @param file_ : The file containing the robot definition
   *
   * @param name_ : A parser implementation
   *
   * @return :
   *    success : A const pointer to the database entry
   *    failure : NULL */
  const scl::SRobotParsedData*
  parseRobot(const std::string &arg_file,
                const std::string &arg_robot_name,
                scl_parser::CParserBase *arg_parser);

  /** Loads a world from a file and registers
   * it with the database.
   *
   * Requires the parser implementation to include:
   * virtual bool readWorldFromFile(const std::string &arg_file,
   *                                scl::SRobotParsedData * arg_robot)
   *
   * @param file_ : The file containing the world definition
   *
   * @param name_ : A parser implementation
   *
   * @return :
   *    success : A const pointer to the database entry
   *    failure : NULL */
  const scl::SWorldParsedData*
  parseWorld(const std::string &arg_file,
                scl_parser::CParserBase *arg_parser);


  /** Loads a graphics specification from a file and registers
   * it with the database.
   *
   * Requires the parser implementation to include:
   * virtual bool readWorldFromFile(const std::string &arg_file,
   *                                scl::SRobotParsedData * arg_robot)
   *
   * @param file_ : The file containing the world definition
   *
   * @param name_ : A parser implementation
   *
   * @return :
   *    success : A const pointer to the database entry
   *    failure : NULL */
  const scl::SGraphicsParsedData*
  parseGraphics(const std::string &arg_file,
                   std::string & arg_graphics_name,
                   scl_parser::CParserBase *arg_parser);

  enum controllerType{SCL_CONTROLLER_TYPE_GC=0, SCL_CONTROLLER_TYPE_TASK=1};
  /** Registers a generalized coordinate controller:
   *
   * 1. Creates a controller with the given name on
   *    the pile if a similarly named controller doesn't
   *    already exist.
   *
   * 2. Initializes the static information in the data
   *    structure. */
  scl::SGcController * parseGcController(const std::string &arg_file,
        const std::string &arg_robot_name,
        const std::string &arg_ctrl_name,
        scl_parser::CParserBase *arg_parser);

  /** Registers a task controller:
   *
   * 1. Creates a controller with the given name on
   *    the pile if a similarly named controller doesn't
   *    already exist.
   *
   * 2. Initializes the static information in the data
   *    structure. */
  scl::STaskController * parseTaskController(const std::string &arg_file,
          const std::string &arg_robot_name,
          const std::string &arg_ctrl_name,
          scl_parser::CParserBase *arg_parser);

  /** Registers the native dynamic types */
  scl::sBool registerNativeDynamicTypes();
}

#endif /* DBREGISTERFUNCTIONS_HPP_ */
