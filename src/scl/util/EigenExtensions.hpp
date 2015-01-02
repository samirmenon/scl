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
 *  Created on: Jan 1, 2015
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon
 */

#ifndef EIGENEXTENSIONS_HPP_
#define EIGENEXTENSIONS_HPP_

namespace scl_util
{
  /** Convert an Eigen Matrix into a matlab style string.
   *
   * Useful for serialization and deserialization. */
  template<typename Derived>
  void eigentoStringMatlab(const Eigen::MatrixBase<Derived>& x, std::string& arg_str)
  {
    std::stringstream ss;
    arg_str = "[";
    for(int i=0;i<x.rows();++i){
      if(i>0) arg_str.append("; ");
      for(int j=0;j<x.cols();++j){
        ss<<x(i,j);
        if(j>0) arg_str.append(", ");
        arg_str.append(ss.str());
        ss.str(std::string());
      }
    }
    arg_str.append("]");
  }

  /** Convert an Eigen Matrix into a linear array of numbers.
   *
   * Useful for serialization and deserialization.
   *
   * NOTE : This leads to a loss of information regarding the size of the matrix!!!
   *        Use with care.. */
  template<typename Derived>
  void eigentoStringArray(const Eigen::MatrixBase<Derived>& x, std::string& arg_str, bool row_major=true)
  {
    std::stringstream ss;
    bool flag = false;
    arg_str = "";
    if(row_major)
    {
      for(int i=0;i<x.rows();++i){
        for(int j=0;j<x.cols();++j){
          ss<<x(i,j);
          if(!flag){ flag=true; }
          else{ arg_str.append(" "); }
          arg_str.append(ss.str());
          ss.str(std::string());
        }
      }
    }
    else
    {
      for(int j=0;j<x.cols();++j){
        for(int i=0;i<x.rows();++i){
          ss<<x(i,j);
          if(!flag){ flag=true; }
          else{ arg_str.append(" "); }
          arg_str.append(ss.str());
          ss.str(std::string());
        }
      }
    }
  }

}

#endif /* EIGENEXTENSIONS_HPP_ */
