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
/* \file STaskOpPosNoGravity.hpp
 *
 *  Created on: Jan 1, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef STASKOPPOSNOGRAVITY_HPP_
#define STASKOPPOSNOGRAVITY_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>

#include <Eigen/Dense>

namespace scl
{
  /** This is identical to the STaskOpPos for now */
  class STaskOpPosNoGravity : public scl::STaskOpPos
  {
  public:
    /** Default constructor sets stuff to S_NULL */
    STaskOpPosNoGravity(){}

    /** Default destructor does nothing */
    virtual ~STaskOpPosNoGravity() {}
  };

}

#endif /* STASKOPPOSNOGRAVITY_HPP_ */
