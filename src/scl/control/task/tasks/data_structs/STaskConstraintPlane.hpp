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
/* \file STaskConstraintPlane.hpp
 *
 *  Created on: Nov 02, 2015
 *
 *  Copyright (C) 2015
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef STASKCONSTRAINTPLANE_HPP_
#define STASKCONSTRAINTPLANE_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>

#include <Eigen/Dense>

namespace scl
{

  class STaskConstraintPlane : public scl::STaskBase
  {
  public:
    //Computed attributes (last measured, in x dimensional task-space)
    Eigen::VectorXd p0_,p1_,p2_;    //The three points that define the plane.
    double stiffness;

    Eigen::VectorXd x_;             //Position of the operational point in the global frame

    Eigen::Vector3d pos_in_parent_; //Position in the parent link's local frame (x,y,z)
    std::string link_name_;         //The parent link
    const SRigidBody *link_ds_;     //The parent link's parsed data structure

    const SRigidBodyDyn *rbd_;   //For quickly obtaining a task Jacobian

    /** Default constructor sets stuff to S_NULL */
    STaskConstraintPlane();

    /** Default destructor does nothing */
    virtual ~STaskConstraintPlane();

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

#endif /* SOPPOINTTASK_HPP_ */
