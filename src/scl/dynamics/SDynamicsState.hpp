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
/* \file SDynamicsState.hpp
 *
 *  Created on: Aug 31, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CDYNAMICSSTATE_HPP_
#define CDYNAMICSSTATE_HPP_

#include <scl/DataTypes.hpp>
#include <Eigen/Dense>
#include <string>

namespace scl
{

  /** The state for a dynamics engine to operate on.
   *
   * All dynamics implementations must try to be stateless.
   *
   * At the very least, they should be able to support storing
   * information in a state object similar to this one.
   * The object can be passed to the computation class every time,
   * removing the need for the computational class to contain
   * robot-specific member data.
   *
   * NOTE : This is only the "minimal dynamic" state. Static
   * properties may be obtained from the database. Un-necessary
   * state must be computed and stored where it is required
   */
  class SDynamicsState : public SObject
  {
  public:
    // ==================================================
    //            Generalized Coordinate State
    // 2nd order system can be completely described by q
    // and dq
    // ==================================================
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;

    // ==================================================
    //            Kinematic and Inertial State
    // ==================================================
    /** The center of mass Jacobians */
    std::vector<Eigen::MatrixXd> J_com_;

    // ==================================================
    //    Various flags to control dynamics    Defaults
    // ==================================================
    sBool flag_apply_damping_dq_;              //false
    sBool flag_apply_limits_q_;                //true
    sBool flag_apply_limits_dq_;               //false
    sBool flag_apply_limits_ddq_;              //false
    sBool flag_apply_errors_integrator_;       //false
    sBool flag_apply_errors_matrices_;         //false

    // ==================================================
    //          Generalized Coordinate Properties
    // ==================================================
    Eigen::VectorXd damping_dq_;
    Eigen::VectorXd limits_q_;
    Eigen::VectorXd limits_dq_;
    Eigen::VectorXd limits_ddq_;

    // ==================================================
    //           Initialization functions
    // ==================================================
    /** Default constructor sets the initialization state to false */
    SDynamicsState():
      flag_apply_damping_dq_(false),
      flag_apply_limits_q_(true),
      flag_apply_limits_dq_(false),
      flag_apply_limits_ddq_(false),
      flag_apply_errors_integrator_(false),
      flag_apply_errors_matrices_(false){}

    /** Default destructor does nothing */
    virtual ~SDynamicsState(){}
  };

}

#endif /* CDYNAMICSSTATE_HPP_ */
