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

#include <Eigen/Core>
#include <jsoncpp/json/json.h>

#include <string>
#include <sstream>

namespace scl_util
{
  /** Convert an Eigen Matrix into a matlab style string.
   * Uses row-major, with commas to separate row-values and
   * semi-colons to separate col values.
   *
   * Format [ row ; second-row; third-row ]
   *
   * Input matrix:
   *     1 2 3
   *     4 5 6
   *     7 8 9
   *
   * Output (always row-major) : [1, 2, 3; 4, 5, 6; 7, 8, 9]
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
   * Input matrix:
   *     1 2 3
   *     4 5 6
   *     7 8 9
   *
   * Output Row-major : 1 2 3 4 5 6 7 8 9
   * Output Col-major : 1 4 7 2 5 8 3 6 9
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
    {// [1 2 3; 4 5 6] == 1 2 3 4 5 6
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
    {// [1 2 3; 4 5 6] == 1 4 2 5 3 6
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

  /** Convert an Eigen Matrix into a JSON array or array of arrays
   *
   * While this format is more expressive than Matlab, we will
   * assume the same convention of always using row-major.
   *
   * As an exception, since Eigen stores Vectors as Matrices, we
   * will always store vectors in an array. By default a vector is
   * always assumed to be in a column; matches Eigen's formatting.
   *
   * Input matrix:
   *     1 2 3
   *     4 5 6
   *     7 8 9
   *
   * Output (Row-major) : [[1,2,3],[4,5,6],[7,8,9]]
   *
   * Input matrix: vector.transpose()
   *     1 2 3
   *
   * Output (Row-major) : [1,2,3]
   *
   * Input matrix: vector
   *     1
   *     2
   *     3
   *
   * Output (Col-major) : [1,2,3]
   *
   * Useful for serialization and deserialization. */
  template<typename Derived>
  void eigentoStringArrayJSON(const Eigen::MatrixBase<Derived>& x, std::string& arg_str)
  {
    std::stringstream ss;
    bool row_major = true;
    if(x.cols() == 1) row_major = false; //This is a Vector!
    arg_str = "[";
    if(row_major)
    {// [1 2 3; 4 5 6] == [ [1, 2, 3], [4, 5, 6] ]
      for(int i=0;i<x.rows();++i){
        if(x.rows() > 1){
          // If it is only one row, don't need the second one
          if(i>0) arg_str.append(",[");
          else arg_str.append("[");
        }
        else if(i>0) arg_str.append(",");
        for(int j=0;j<x.cols();++j){
          ss<<x(i,j);
          if(j>0) arg_str.append(",");
          arg_str.append(ss.str());
          ss.str(std::string());
        }
        if(x.rows() > 1){
          // If it is only one row, don't need the second one
          arg_str.append("]");
        }
      }
      arg_str.append("]");
    }
    else
    {// [1 2 3; 4 5 6] == 1 4 2 5 3 6
      for(int j=0;j<x.cols();++j){
        if(x.cols() > 1){
          // If it is only one row, don't need the second one
          if(j>0) arg_str.append(",[");
          else arg_str.append("[");
        }
        else if(j>0) arg_str.append(",");
        for(int i=0;i<x.rows();++i){
          ss<<x(i,j);
          if(i>0) arg_str.append(",");
          arg_str.append(ss.str());
          ss.str(std::string());
        }
        if(x.cols() > 1){
          // If it is only one row, don't need the second one
          arg_str.append("]");
        }
      }
      arg_str.append("]");
    }
  }

  /** Convert an Eigen Quaternion into a JSON array
   *
   * Input matrix: quaternion
   *
   * Output (Row-major) : [x,y,z,w]
   *
   * Useful for serialization and deserialization. */
  template<typename Derived>
  void eigentoStringArrayJSON(const Eigen::QuaternionBase<Derived>& q, std::string& arg_str)
  {
    std::stringstream ss;
    arg_str = "[";

    ss<<q.x();
    arg_str.append(ss.str());
    ss.str(std::string());

    ss<<q.y();
    arg_str.append(",");
    arg_str.append(ss.str());
    ss.str(std::string());

    ss<<q.z();
    arg_str.append(",");
    arg_str.append(ss.str());
    ss.str(std::string());

    ss<<q.w();
    arg_str.append(",");
    arg_str.append(ss.str());
    ss.str(std::string());

    arg_str.append("]");
  }

  /** Initialize an Eigen Matrix from a JSON array or array of arrays
   *
   * Input (JSON) : [[1,2,3],[4,5,6],[7,8,9]]
   * Eigen matrix:
   *     1 2 3
   *     4 5 6
   *     7 8 9
   *
   * Input (JSON) : [1,2,3]
   *
   * Eigen vector:
   *     1
   *     2
   *     3
   */
  template<typename Derived>
  bool eigenFromJSON(Eigen::MatrixBase<Derived>& x, const Json::Value &jval)
  {
    if(!jval.isArray()) return false; //Must be an array..
    unsigned int nrows = jval.size();
    if(nrows < 1) return false; //Must have elements.

    bool is_matrix = jval[0].isArray();
    if(!is_matrix)
    {
      x.setIdentity(nrows,1);//Convert it into a vector.
      for(int i=0;i<nrows;++i) x(i,0) = jval[i].asDouble();
    }
    else
    {
      unsigned int ncols = jval[0].size();
      if(ncols < 1) return false; //Must have elements.
      for(int i=0;i<nrows;++i){
        if(ncols != jval[i].size()) return false;
        for(int j=0;j<ncols;++j)
          x(i,j) = jval[i][j].asDouble();
      }
    }
    return true;
  }

  /** Extract an Eigen Quaternion from a JSON value
   *
   * Input JSON: [x,y,z,w]
   *
   * Output : Eigen quaternion */
  template<typename Derived>
  bool eigenFromJSON(Eigen::QuaternionBase<Derived>& q, const Json::Value &jval)
  {
    if(!jval.isArray()) return false; //Must be an array..
    if(4 != jval.size()) return false; //Must have 4 elements.

    for(int i : {0,1,2,3})
      if(!jval[i].isNumeric()) return false; //Elements must be scalars

    q = Eigen::Quaternion<scl::sFloat>(jval[0].asDouble(),jval[1].asDouble(),jval[2].asDouble(),jval[3].asDouble());

    return true;
  }

}

#endif /* EIGENEXTENSIONS_HPP_ */
