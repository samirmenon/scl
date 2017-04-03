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
/* \file STaskComPos.hpp
 *
 *  Created on: Sep 2, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SRC_SCL_CONTROL_TASKS_DATA_STRUCTS_STASKCOMPOS_HPP_
#define SRC_SCL_CONTROL_TASKS_DATA_STRUCTS_STASKCOMPOS_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>

#include <Eigen/Dense>

namespace scl
{
  /** This is a task to control the position, velocity and acceleration
   * of the center of mass of an articulated body system. The controller
   * operates in Euclidean space, and in the global (origin) frame.
   *
   * Should the center of mass Jacobian become singular (loses rank k), it
   * will use an svd to compute the n-k'th rank approximation of the
   * control equations.
   */
  class STaskComPos : public scl::STaskBase
  {
  public:
    //Computed attributes (last measured, in x dimensional task-space)
    Eigen::Vector3d x_;             //Position in the global frame
    Eigen::Vector3d dx_;            //Velocity in the global frame
    Eigen::Vector3d ddx_;           //Acceleration in the global frame

    Eigen::Vector3d x_goal_;        //Goal Position in the global frame
    Eigen::Vector3d dx_goal_;       //Goal Velocity in the global frame
    Eigen::Vector3d ddx_goal_;      //Goal Acceleration in the global frame

    Eigen::Vector3d pos_in_parent_; //Position in the origin frame (x,y,z)

    sFloat spatial_resolution_;     //Meters

    /** Default constructor sets stuff to S_NULL */
    STaskComPos();

    /** Default destructor does nothing */
    virtual ~STaskComPos();

    /** Initializes the task specific data members. */
    virtual bool initTaskParams();
  };

}

#endif /* SCOMPOSTASRC_SCL_CONTROL_TASKS_DATA_STRUCTS_STASKCOMPOS_HPP_SK_HPP_ */
