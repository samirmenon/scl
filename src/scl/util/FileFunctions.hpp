/* This file is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

This file is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
this file. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file FileFunctions.hpp
 *
 *  Created on: May 18, 2010
 *
 *  Copyright (C) 2010, Samir Menon <smenon@stanford.edu>
 */

#ifndef FILEFUNCTIONS_HPP_
#define FILEFUNCTIONS_HPP_

#include <string>

#include <Eigen/Core>

namespace scl_util
{

/** Function reads a vector from a file.
 * The file should have a val in each line:
 * x1
 * x2
 * x3 ...
 */
bool readEigenVecFromFile(Eigen::VectorXd & arg_vec,
    const std::string & arg_file);

/** Function reads a vector from a file.
 * The file should have a val in each line:
 * x1
 * x2
 * x3 ...
 */
bool readEigenVecFromFile(Eigen::VectorXd & arg_vec,
    const long len, const std::string & arg_file);

/** Writes an eigen vector of doubles to a file
 * @param arg_vec : The data to be written to disk
 * @param arg_file : The filename
 * @return  */
bool writeEigenVecToFile(const Eigen::VectorXd & arg_vec,
    const std::string & arg_file);

/** Reads an eigen matrix of doubles from a file
 *
 * The file should have a vector in each line:
 * x11 x12 ...
 * x21 x22 ...
 *
 * @param arg_mat : The data is loaded longo this matrix
 * @param arg_file : The filename
 * @return
 */
bool readEigenMatFromFile(Eigen::MatrixXd & arg_mat,
    unsigned long arg_rows, unsigned long arg_cols,
    const std::string & arg_file);


/** Writes an eigen matrix of doubles to a file
 * @param arg_vec : The data to be written to disk
 * @param arg_file : The filename
 * @return  */
bool writeEigenMatToFile(const Eigen::MatrixXd & arg_mat,
    const std::string & arg_file);
}

#endif /* FILEFUNCTIONS_HPP_ */
