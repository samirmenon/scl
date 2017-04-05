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

#include <ctype.h>

namespace scl_util
{

  /** Convert an Eigen Transform into a JSON array or array of arrays
   *  Output (Row-major) : [[1,2,3,p0],[4,5,6,p1],[7,8,9,p2],[0,0,0,1]]
   * Useful for serialization and deserialization. */
  void eigentoStringArrayJSON(const Eigen::Affine3d& x, std::string& arg_str)
  {
    eigentoStringArrayJSON<>(x.matrix(),arg_str);
  }

  /** Initialize an Eigen Matrix from a JSON array or array of arrays
   *
   * Input (JSON) : [[1,2,3,1],[4,5,6,0],[7,8,9,0],[0,0,0,1]]
   * Eigen matrix (Affine 3d):
   *     1 2 3 1
   *     4 5 6 0
   *     7 8 9 0
   *     0 0 0 1
   */
  bool eigenFromJSON(Eigen::Affine3d& x, const Json::Value &jval)
  {
    if(!jval.isArray()) return false; //Must be an array..
    unsigned int nrows = jval.size();
    if(nrows != 4) return false; //Must be 4x4.

    bool is_matrix = jval[0].isArray();
    if(!is_matrix)
    {
      return false;
    }
    else
    {
      unsigned int ncols = jval[0].size();
      if(ncols != 4) return false; //Must be 4x4.
      for(unsigned int i=0;i<nrows-1;++i){
        if(ncols != jval[i].size()) return false; //inconsistent cols
        for(unsigned int j=0;j<ncols-1;++j)
          x.matrix()(i,j) = jval[i][j].asDouble();
        x.matrix()(0,3) = jval[0][3].asDouble();
        x.matrix()(1,3) = jval[1][3].asDouble();
        x.matrix()(2,3) = jval[2][3].asDouble();
      }
    }
    return true;
  }
}
