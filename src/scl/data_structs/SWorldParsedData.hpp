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
/* \file SWorldParsedData.hpp
 *
 *  Created on: Jul 22, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 

#ifndef SWORLDPARSEDDATA_HPP_
#define SWORLDPARSEDDATA_HPP_

#include <string>
#include <vector>

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>

namespace scl
{

/**This structure contains all the non-robot specification
 * information required to construct a robotic world.
 * Individual robot definitions are required in addition to this
 * in order to create a robotic simulation environment with controllable
 * robots.
 */
struct SWorldParsedData : public SObject
{
public:
	//***********************
  //Global data:
  //Only one physics simulation. So all physical constants
  //are conserved in SWorldParsedData
  sFloat gravity_[3];

  SWorldParsedData() : SObject("SWorldParsedData"){}
};
}


#endif //SWORLDPARSEDDATA_HPP_
