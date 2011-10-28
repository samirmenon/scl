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
/*
 * \file SGcModel.hpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SGCMODEL_HPP_
#define SGCMODEL_HPP_

#include <scl/DataTypes.hpp>

#include <scl/data_structs/SRobotParsedData.hpp>

#include <Eigen/Dense>

namespace scl
{
  /** A data structure to store the joint space model.
   * This model serves as the foundation for all tasks
   * to compute their own models (mass, coriolis/centrifugal
   * and gravity).
   *
   * Each controller data structure contains an object of this type.   */
  class SGcModel
  {
  public:
    /** A: Mass matrix */
    Eigen::MatrixXd A_;

    /** Ainv : Mass matrix inverse */
    Eigen::MatrixXd Ainv_;

    /** b : Coriolis+centrifugal torque vector */
    Eigen::VectorXd b_;

    /** g : Gravity torque vector */
    Eigen::VectorXd g_;

    /** Constructor does nothing */
    SGcModel();

    /** Initialization function sets up the matrix sizes */
    sBool init(const sUInt arg_robot_dof_);
  };
}

#endif /* SGCMODEL_HPP_ */
