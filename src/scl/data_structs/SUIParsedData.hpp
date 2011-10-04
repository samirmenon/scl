/* This file is part of Busylizzy, a control and simulation library
for robots and biomechanical models.

Busylizzy is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

Busylizzy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
Busylizzy. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file SUIParsedData.hpp
 *
 *  Created on: Sep 15, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 

#ifndef SUIPARSEDDATA_HPP_
#define SUIPARSEDDATA_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SDatabase.hpp>
#include <scl/data_structs/SObject.hpp>

#include <string>
#include <vector>

namespace scl
{

/** This structure contains all the user interaction settings
 * Eg. Specifying custom user interaction handlers etc. */
struct SUIParsedData : public SObject
{
public:
	//***********************
  std::vector<std::string>

  SWorldParsedData() : SObject("SWorldParsedData"){}
};
}


#endif //SUIPARSEDDATA_HPP_
