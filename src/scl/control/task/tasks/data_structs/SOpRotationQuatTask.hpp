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
/* \file SOpRotationQuatTask.hpp
 *
 *  Created on: Sep 14, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SOPROTATIONQUATTASK_HPP_
#define SOPROTATIONQUATTASK_HPP_

#include <scl/control/task/data_structs/STaskBase.hpp>
#include <scl/DataTypes.hpp>

#include <Eigen/Dense>

namespace scl
{
  /** This data structure contains variables used while
   * computing a rotation control task   */
  class SOpRotationQuatTask : public scl::STaskBase
  {
  public:
    //Computed attributes (last measured, in x dimensional task-space)
    /** The current orientation in quaternions */
    Eigen::Quaterniond ori_quat_;

    /** The desired orientation in euler angles 3D XYZ */
    Eigen::Vector3d ori_eulerang_goal_;

    /** The current orientation in quaternions */
    Eigen::Quaterniond ori_quat_goal_;

    /** The rotational Jacobian between the generalized coords
     * and rotational coords */
    Eigen::MatrixXd J_;

    /** The position in the parent's local frame (xyz) at
     * which the axis of rotation is centered */
    Eigen::Vector3d pos_in_parent_;

    /** The parent link */
    std::string link_name_;

    /** The parent link's parsed data structure */
    const SRobotLink *link_ds_;

    /** In rad. Task stops once the error reaches this bound. */
    sFloat spatial_resolution_;

    /** This is a custom data type used to quickly identify the
     * parent link. It is the address of a data struct "in the
     * dynamics engine". */
    const void *link_dynamic_id_;

    /** Default constructor sets stuff to S_NULL */
    SOpRotationQuatTask();

    /** Default destructor does nothing */
    virtual ~SOpRotationQuatTask();

    /** 1. Initializes the task specific data members.
     *
     * 2. Parses non standard task parameters,
     * which are stored in STaskBase::task_nonstd_params_.
     * Namely:
     *  (a) parent link name
     *  (b) pos in parent.*/
    virtual bool initTaskParams();
  };

}

#endif /* SOPROTATIONQUATTASK_HPP_ */
