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
/* \file AllHeaders.hpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SRC_SCL_DATA_STRUCTS_ALLHEADERS_HPP_
#define SRC_SCL_DATA_STRUCTS_ALLHEADERS_HPP_

// Helper fuctions to extract data from the remaining data structs
// These help reduce code duplication.
#include <scl/data_structs/DataStructQueryFunctions.hpp>

// The base data structs that are used widely across scl.
#include <scl/data_structs/SActuatorSetMuscleParsed.hpp>
#include <scl/data_structs/SForce.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/data_structs/SGraphicsParsed.hpp>
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/data_structs/SRigidBodyDyn.hpp>
#include <scl/data_structs/SRobotIO.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SUIParsed.hpp>

// An all-in-one data struct used by some programs. Contains all of
// the above in a structured manner.
#include <scl/data_structs/SDatabase.hpp>

#endif /* SRC_SCL_DATA_STRUCTS_ALLHEADERS_HPP_ */
