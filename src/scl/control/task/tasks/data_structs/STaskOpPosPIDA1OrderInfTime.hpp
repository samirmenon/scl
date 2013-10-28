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
/* \file STaskOpPosPIDA1OrderInfTime.hpp
 *
 *  Created on: Aug 9, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SOPPOINTTASKPIDA1ORDERINFTIME_HPP_
#define SOPPOINTTASKPIDA1ORDERINFTIME_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>

#include <Eigen/Dense>

namespace scl
{

  class STaskOpPosPIDA1OrderInfTime : public scl::STaskBase
  {
  public:
    //Computed attributes (last measured, in x dimensional task-space)
    Eigen::VectorXd x_;             //Position in the global frame
    Eigen::VectorXd dx_;            //Velocity in the global frame
    Eigen::VectorXd ddx_;           //Acceleration in the global frame

    Eigen::VectorXd x_goal_;        //Goal Position in the global frame
    Eigen::VectorXd dx_goal_;       //Goal Velocity in the global frame
    Eigen::VectorXd ddx_goal_;      //Goal Acceleration in the global frame

    /** The integral force term. Required because integral control is not
     * time invariant. */
    Eigen::VectorXd integral_force_;

    Eigen::Vector3d pos_in_parent_; //Position in the parent link's local frame (x,y,z)
    std::string link_name_;         //The parent link
    const SRigidBody *link_ds_;     //The parent link's parsed data structure

    sFloat spatial_resolution_;     //Meters

    const void *link_dynamic_id_;   //For quickly obtaining a task Jacobian

    /** Typical use in a PID controller:
     *  Force = Force + ki * pos_err * (t_curr - t_pre);
     * */
    sFloat integral_gain_time_pre_, integral_gain_time_curr_;

    /** Default constructor sets stuff to S_NULL */
    STaskOpPosPIDA1OrderInfTime();

    /** Default destructor does nothing */
    virtual ~STaskOpPosPIDA1OrderInfTime();

    /** 1. Initializes the task specific data members.
     *
     * 2. Parses non standard task parameters,
     * which are stored in STaskBase::task_nonstd_params_.
     * Namely:
     *  (a) parent link name
     *  (b) pos in parent
     *  (c) integral gain time constant
     *  (d) integral gain time max */
    virtual bool initTaskParams();
  };

}

#endif /* SOPPOINTTASKPIDA1ORDERINFTIME_HPP_ */
