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
/* \file CLotusParser.hpp
 *
 *  Created on: May, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CLOTUSPARSER_HPP_
#define CLOTUSPARSER_HPP_

#include <scl/parser/CParserBase.hpp>

#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/data_structs/SGraphicsParsedData.hpp>

namespace scl_parser {

/**
 * This class implements the entire CParserBase API.
 *
 * It represents the file format of choice for scl.
 *
 * Please see the CParserBase API for more details.
 */
class CLotusParser: public CParserBase {
public:
  CLotusParser(){}
  virtual ~CLotusParser(){}

  virtual bool listRobotsInFile(const std::string& arg_file,
      std::vector<std::string>& arg_robot_names);

  virtual bool readRobotFromFile(const std::string& arg_file,
      const std::string& arg_robot_name,
      scl::SRobotParsedData& arg_robot_object);

  virtual bool saveRobotToFile(scl::SRobotParsedData& arg_robot,
      const std::string &arg_file);

  virtual bool listGraphicsInFile(const std::string& arg_file,
            std::vector<std::string>& arg_graphics_names);

  virtual bool readGraphicsFromFile(const std::string &arg_file,
      const std::string &arg_graphics_name,
      scl::SGraphicsParsedData& arg_graphics);

  virtual bool listControllersInFile(const std::string &arg_file,
      std::vector<std::pair<std::string,std::string> > &arg_ctrl_name_and_type);

  virtual bool readGcControllerFromFile(const std::string &arg_file,
      const std::string &arg_ctrl_name,
      std::string &arg_must_use_robot,
      scl::SGcController& arg_ctrl);

  virtual bool readTaskControllerFromFile(const std::string &arg_file,
      const std::string &arg_ctrl_name,
      std::string &arg_must_use_robot,
      scl::STaskController& arg_ctrl,
      /** Returns a vector of tasks that this task controller can execute.
       * Typically, the task controller will execute all of them simultaneously
       * with a priority order.*/
      std::vector<scl::STaskBase> &arg_taskvec);

private:
  bool readRobotSpecFromFile(const std::string& arg_spec_file,
      const std::string& arg_robot_spec_name,
      scl::SRobotParsedData& arg_robot);
};

}

#endif /*CLOTUSPARSER_HPP_*/
