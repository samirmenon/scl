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
/* \file EigenExtensions.hpp
 *
 *  Created on: Aug 19, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon
 */

#include "EigenExtensions.hpp"


namespace scl_util
{

  /** Convert an Eigen Transform into a JSON array or array of arrays
   *  Output (Row-major) : [[1,2,3,p0],[4,5,6,p1],[7,8,9,p2],[0,0,0,1]]
   * Useful for serialization and deserialization. */
  void eigentoStringArrayJSON(const Eigen::Affine3d& x, std::string& arg_str)
  {
    eigentoStringArrayJSON<>(x.matrix(),arg_str);
  }

}
