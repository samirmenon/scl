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
/* \file scl.hpp
 *
 *  Created on: Nov 14, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCL_HPP_
#define SCL_HPP_

#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>

// All the header files except for those from the robot/ directory
#include <scl/AllHeaders.hpp>

// The robot directory isn't part of the standard suite of scl stuff
// It basically is a wrapper that ties together a set of stuff to
// realize simple simulations. (Though the simplest simulations
// use a coding style similar to the tutorials).
#include <scl/robot/CRobotApp.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/robot/GenericCallbacks.hpp>
#include <scl/robot/data_structs/SRobot.hpp>
#include <scl/robot/CRobot.hpp>
#include <scl/robot/GenericPrintables.hpp>

#endif /* SCL_HPP_ */
