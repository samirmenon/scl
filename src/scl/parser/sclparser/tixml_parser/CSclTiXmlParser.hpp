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
/* \file CSclTiXmlParser.hpp
 *
 *  Created on: May, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CLOTUSTIXMLPARSER_H_
#define CLOTUSTIXMLPARSER_H_

#include <iostream>
#include <string>
#include <scl/parser/scl_tinyxml/scl_tinyxml.h>

//Include the data structures
#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/data_structs/SGraphicsParsedData.hpp>
#include <scl/data_structs/SRobotLink.hpp>

#include <scl/parser/CParserBase.hpp>

namespace scl_parser {

/**
 * Provides a set of helper functions to parse a scl
 * xml file using tinyXML.
 *
 * Stores the parsed information in passed data structures
 */
class CSclTiXmlParser
{
private:

public:
  CSclTiXmlParser() {}
  ~CSclTiXmlParser() {}

  /**
   * Reads single links
   */
  static bool readLink(const scl_tinyxml::TiXmlHandle& arg_link_txml,
      scl::SRobotLink& arg_link_ds, bool arg_is_root);
  
  /**
   * Reads in graphics data
   */
  static bool readGraphics(
      const scl_tinyxml::TiXmlHandle &arg_graphics_data_txml,
      scl::SGraphicsParsedData& arg_graphics_ds);
};

}
#endif /* CLOTUSTIXMLPARSER_H_ */
