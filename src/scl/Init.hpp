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
/* \file Init.hpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCL_INIT_HPP_
#define SCL_INIT_HPP_

#include <scl/DataTypes.hpp>

namespace scl
{
  namespace init
  {

    /** ***********************************************************
                        DYNAMIC TYPES
     *********************************************************** **/
    /** Dynamic typing helper functions for different scl types
     *  Registers the native dynamic types */
    scl::sBool registerNativeDynamicTypes();

    ///////////////////////////////////////////////////////////////
    ///////////////////////////THE END/////////////////////////////
  }
}

#endif /* SCL_INIT_HPP_ */
