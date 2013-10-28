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
 * \file SControllerGc.hpp
 *
 *  Created on: Dec 29, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCONTROLLERGC_HPP_
#define SCONTROLLERGC_HPP_

#include <vector>
#include <list>
#include <string>

#include <scl/control/data_structs/SControllerBase.hpp>


namespace scl
{

  /** Generalized coordinate force controller (joint-space controller)
   * data structure.
   *
   * Works with an arbitrary actuated articulated body, with direct
   * force control at the generalized coordinates.
   *
   * Supports dynamically decoupled control, which is equivalent to controlling
   * a unit mass in the generalized coordinates.
   */
  class SControllerGc : public SControllerBase
  {
  public:
    /** Desired generalized coordinate forces (usually joint torques) */
    Eigen::VectorXd des_force_gc_;

    /** Desired generalized positions */
    Eigen::VectorXd des_q_;
    /** Desired generalized velocities */
    Eigen::VectorXd des_dq_;
    /** Desired generalized accelerations */
    Eigen::VectorXd des_ddq_;

    /** Upper and lower applied force limits
     * (Why separate max and min? Muscles can only pull, not push.) */
    Eigen::VectorXd force_gc_max_,force_gc_min_;

    /** Gains (scalar if same for different dimensions;
     * vector if different for different dimensions)
     *
     * Computed by : None. These are constants.
     * Used by     : The GcController to calculate gc forces */
    Eigen::VectorXd kp_, kv_, ka_, ki_;

    /** Set the integral gain decay rate. Ie. Degrade last timestep's
     * influence by x%
     *
     * Typical use in a PID controller
     *
     * if( (t_curr - t_pre) > integral_gain_time_max_ )
     * { Force = 0; }
     * else
     * { Force = Force + ki * pos_err * (t_curr - t_pre) / integral_gain_time_constt_; }
     * */
    sFloat integral_gain_time_pre_, integral_gain_time_curr_,
           integral_gain_time_constt_, integral_gain_time_max_;

    /** The integral force term. Required because integral control is not
     * time invariant. */
    Eigen::VectorXd integral_force_;

    /* ********************************************************
     *              Initialization Functions
     * ******************************************************** */
    /** Constructor. Does nothing */
    SControllerGc();

    /* Initialization function. Sets all the control parameters */
    sBool init(const std::string & arg_controller_name,
        SRobotParsed* arg_robot_ds,
        SRobotIO* arg_robot_io_ds,
        /* The remaining variables initialize the gc controller */
        const Eigen::VectorXd & arg_kp,
        const Eigen::VectorXd & arg_kv,
        const Eigen::VectorXd & arg_ka,
        const Eigen::VectorXd & arg_ki,
        const Eigen::VectorXd & arg_fgc_max,
        const Eigen::VectorXd & arg_fgc_min,
        /** Default:0.1s time constant for the integral gain */
        const sFloat arg_integral_gain_time_constt=0.1,
        /** If the integral gain is not recomputed for this time,
         * the force term is reset. */
        const sFloat arg_integral_gain_time_max=0.05);

    /** Inherits:
      std::string name_;
      sBool has_been_init_;
      std::string robot_name_;
      const SRobotParsed* robot_;
      SRobotIO* io_data_;
      SGcModel gc_model_;
    */
  };
}

#endif /* SGCCONTROLLER_HPP_ */
