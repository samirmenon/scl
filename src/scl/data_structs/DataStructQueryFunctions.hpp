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
/* \file DataStructHelperFunctions.hpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef DATASTRUCTHELPERFUNCTIONS_HPP_
#define DATASTRUCTHELPERFUNCTIONS_HPP_

#include <scl/data_structs/SActuatorSetMuscleParsed.hpp>
#include <scl/data_structs/SRobotParsed.hpp>

#include <iostream>

namespace scl
{
  namespace dsquery
  {
    /** Does what the name says. Else returns NULL */
    scl::SActuatorSetMuscleParsed* getFromRobotParsedFirstMuscleSet(scl::SRobotParsed& arg_rds);
  }
}
#endif /* DATASTRUCTHELPERFUNCTIONS_HPP_ */
