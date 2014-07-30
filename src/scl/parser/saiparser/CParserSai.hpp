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
/* \file CParserSai.hpp
 *
 *  Created on: Jan, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CPARSERSAI_HPP_
#define CPARSERSAI_HPP_

#include <scl/parser/CParserBase.hpp>

//The tinyxml parser implementation for sai xml files
#include <scl_tinyxml/scl_tinyxml.h>

namespace scl {

/**
 * This class implements a limited subset of the CParserBase API
 * for the legacy "SAI" file format.
 *
 * Uses tinyXml to parse files.
 */
class CParserSai: public CParserBase {
public:
  CParserSai():root_link_name_("ground"){}
  virtual ~CParserSai(){}

  virtual bool listRobotsInFile(const std::string& arg_file,
      std::vector<std::string>& arg_robot_names)
  { return false; }

  /**
   * Only support reading from SAI files. Use the file converter
   * to convert them into Scl files. (applications/scl_file_converter)
   *
   * Since the SAI format only has one robot in a file, the "arg_robot_name"
   * argument is not used.
   *
   * Tags not supported:
   * 1. Damping -> scl's damping is flat
   * 2. OpID -> don't know what it does
   * 3. collisionType -> scl doesn't presently have collision support
   *
   * Notes:
   * 1. Throws errors when required tags are missing
   * 2. Prints warnings when non-required but important (dynamics info) tags are missing
   * 3. Does nothing when default values can usually replace missing tags. Eg. joint limits
   * 4. Sai seems to support w,x,y,z quaternions in its orientation tag. Eigen supports x,y,z,w.
   * 5. Sai often has missing orientation tags (for free joints). This info is essential
   *    but we'll only issue warnings for now.
   */
  virtual bool readRobotFromFile(const std::string& arg_file,
      /** This argument is useless for SAI files. Only reads the first robot!!! */
      const std::string& arg_robot_name,
      scl::SRobotParsed& arg_robot_object);

  virtual bool saveRobotToFile(scl::SRobotParsed& arg_robot,
      const std::string &arg_file)
  { return false; }

  virtual bool listGraphicsInFile(const std::string& arg_file,
      std::vector<std::string>& arg_graphics_names)
  { return false; }

  virtual bool readGraphicsFromFile(const std::string &arg_file,
      const std::string &arg_graphics_name, scl::SGraphicsParsed& arg_graphics)
  { return false; }

private:
  /** Recursive SAI link specification reader */
  bool readLink(const scl_tinyxml::TiXmlHandle& arg_tiHndl_link, const bool arg_is_root,
      const std::string& arg_parent_lnk_name, scl::SRobotParsed& arg_robot);

  const std::string root_link_name_;
};

}

#endif /*CPARSERSAI_HPP_*/
