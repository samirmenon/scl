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
 * \file CActuatorSetBase.hpp
 *
 *  Created on: Aug 24, 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CACTUATORSETBASE_HPP_
#define CACTUATORSETBASE_HPP_

#include <scl/actuation/CActuatorBase.hpp>
#include <scl/data_structs/SObject.hpp>
#include <sutil/CMappedList.hpp>
#include <string>

namespace scl
{
  /** An actuator set combines a variety of potentially different
   * actuators into a common interface that a controller can interact
   * with. The number of actuators is not necessarily related to the
   * degrees of freedom of the system (could be more, equal or less).
   *
   * Subclasses of this class will simply organize actuators into a
   * coherent interface, abstracting all physical details and exposing
   * a mathematical interface for the controller.
   *
   * Models:
   * (a) Actuator Kinematics : Actuator inputs might require some functional
   *     transform to map on to the generalized coordinates (uses Jacobian).
   * (b) Actuator Dynamics : Actuator forces might be a function of inputs,
   *     and present state.
   *
   * NOTE for "Real Robots": While working with a real robot, subclass
   *  this base class and implement your driver in it.
   */
  class CActuatorSetBase : public SObject
  {
  public:
    /* *****************************************************************
     *                        Actuator Kinematics
     * ***************************************************************** */
    /** Some actuator sets don't directly actuate the generalized coordinates
     * and require a Jacobian to compute their contribution to the generalized
     * forces.
     *
     * Each actuator instance must implement this. */
    virtual sBool computeJacobian(
        const Eigen::VectorXd arg_q,
        const Eigen::VectorXd arg_dq,
        Eigen::MatrixXd &ret_J)=0;

    /* *****************************************************************
     *                        Actuator Dynamics
     * ***************************************************************** */
    /** Set the actuator commands. These will be translated into individual
     * actuator commands.
     * Gets the output of the actuator. Returned in the passed variable.
     *
     * arg_input : The actuator input (possibly arbitrary coordinates)
     * arg_state : Possibly actuator coordinates, velocities etc. Rows in matrix.
     * ret_output_force : The actuator forces such that:
     *                       Fgc = J' * ret_output_force;
     */
    virtual sBool computeDynamics(const Eigen::VectorXd& arg_input,
        const Eigen::MatrixXd& arg_state,
        Eigen::VectorXd& ret_output_force)=0;

    /* *****************************************************************
     *                        Initialization
     * ***************************************************************** */
    /** Default constructor. Sets stuff to NULL */
    CActuatorSetBase(const std::string& arg_subclass_type_name) :
      SObject(arg_subclass_type_name) {}

    /** Default destructor. Does nothing. */
    virtual ~CActuatorSetBase() {}

  private:
    CActuatorSetBase();
  };

} /* namespace sutil */
#endif /* CACTUATORSETBASE_HPP_ */
