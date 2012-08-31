/*
 * Stanford Whole-Body Control Framework http://stanford-scl.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file jspace/wrap_eigen.hpp
   \author Roland Philippsen
 */

#ifndef JSPACE_WRAP_EIGEN_HPP
#define JSPACE_WRAP_EIGEN_HPP

#include <Eigen/Geometry>
#include <vector>

#ifdef W_QNX
extern "C" {
#include <stddef.h> // for qnx
}
#endif

//NOTE TODO : This file needs to be deleted. There is no need to have wrappers.

namespace jspace
{
  // ...an idea that needs more thought...
  // typedef Eigen::Map<Eigen::VectorXd> VectorMap;
  // inline VectorMap map(std::vector<double> & from) { return Vector::Map(&from[0], from.size()); }
  // inline VectorMap const map(std::vector<double> const & from) { return Vector::Map(&from[0], from.size()); }

  void convert(Eigen::VectorXd const & from, std::vector<double> & to);
  void convert(std::vector<double> const & from, Eigen::VectorXd & to);
  void convert(double const * from, size_t length, Eigen::VectorXd & to);

  // should also work as-is for jspace::Vector
  bool compare(Eigen::MatrixXd const & lhs, Eigen::MatrixXd const & rhs, double precision);
  bool compare(Eigen::Quaternion<double> const & lhs, Eigen::Quaternion<double> const & rhs, double precision);

  std::string pretty_string(double vv);
  std::string pretty_string(Eigen::VectorXd const & vv);
  std::string pretty_string(Eigen::Quaternion<double> const & qq);
  std::string pretty_string(Eigen::MatrixXd const & mm, std::string const & prefix);

  void pretty_print(Eigen::VectorXd const & vv, std::ostream & os,
      std::string const & title, std::string const & prefix, bool nonl = false);

  inline void pretty_print(Eigen::Vector3d const & vv, std::ostream & os,
      std::string const & title, std::string const & prefix, bool nonl = false)
  {
    pretty_print(static_cast<Eigen::VectorXd const &>(vv), os, title, prefix, nonl);
  }

  void pretty_print(Eigen::Quaternion<double> const & qq, std::ostream & os,
      std::string const & title, std::string const & prefix, bool nonl = false);

  void pretty_print(Eigen::MatrixXd const & mm, std::ostream & os,
      std::string const & title, std::string const & prefix,
      bool vecmode = false, bool nonl = false);

}

#endif // JSPACE_WRAP_EIGEN_HPP
