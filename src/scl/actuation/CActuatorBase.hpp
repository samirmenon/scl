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
 * \file CActuatorBase.hpp
 *
 *  Created on: Aug 24, 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CACTUATORBASE_HPP_
#define CACTUATORBASE_HPP_

#include <scl/data_structs/SObject.hpp>

#include <Eigen/Eigen>

namespace scl
{
  /** Some defaults that allow controlling an actuator model's behavior.
   * All actuator models should support these.
   *
   * NOTE : Please don't be confused. This is an actuator "model", and
   *        as these flags make sense for a simulated actuator. It is
   *        "NOT" meant as an interface to a real actuator. */
  class SActuatorSettings
  {
  public:
    sBool flag_apply_limits_force_;   //true
    sBool flag_apply_limits_pos_;     //true
    sBool flag_apply_limits_vel_;     //true
    sBool flag_apply_limits_acc_;     //true
    sBool flag_apply_noise_force_;   //true
    sBool flag_apply_noise_pos_;     //true
    sBool flag_apply_noise_vel_;     //true
    sBool flag_apply_noise_acc_;     //true
    sBool flag_apply_delay_forces_;   //false

    Eigen::VectorXd limits_force_;
    Eigen::VectorXd limits_pos_;
    Eigen::VectorXd limits_vel_;
    Eigen::VectorXd limits_acc_;

    /** Constructor : Sets defaults */
    SActuatorSettings() :
      flag_apply_limits_force_(true),
      flag_apply_limits_pos_(true),
      flag_apply_limits_vel_(true),
      flag_apply_limits_acc_(true),
      flag_apply_noise_force_(true),
      flag_apply_noise_pos_(true),
      flag_apply_noise_vel_(true),
      flag_apply_noise_acc_(true),
      flag_apply_delay_forces_(false)
    {}
  };

  /** This will serve as the base class for a variety
   * of actuator models.
   *
   * The typical pipeline for an scl simulation is
   * { Controller } -> { { Actuator } -> { Dynamics Engine } }
   *
   * The logical breakdown for picking such a system is that while a
   * controller might not be tied down to particular hardware, an
   * actuator set typically is. Next, all hardware are constrained to
   * obey the laws of physics.
   *
   * Why pick this breakdown?
   * Ans: One alternative formulation would have been to include
   * the actuator model in the dynamics engine. Doing so, however,
   * makes the dynamics engine very inflexible and less modular
   * than it is otherwise.
   *
   * The second alternative formulation would have been to include
   * the actuator model into the controller. However, this would
   * prevent something as general as an op-point controller from
   * generalizing across a musculoskeletal system and a Puma robot.
   *
   * Logical breakdown:
   * { Robot-aware/agnostic Math } -> { Robot-specific Physical Properties } -> { Robot-agnostic Physics Laws }
   *
   * NOTE : Please don't be confused. This is an actuator "model", and
   *        as these functions make sense for a simulated actuator. It is
   *        "NOT" meant as an interface to a real actuator.
   *
   * __DO NOT__ subclass this base and start writing a driver in it.
   */
  class CActuatorBase
  {
  public:
    /* *****************************************************************
     *                        Actuation Model
     * ***************************************************************** */
    /** Convenience function that combines an actuator's functions
     *
     * A typical implementation might split this function into three stages:
     * {
     *  bool flag=true;
     *  flag = flag && setActuatorCommand(arg_input);
     *  flag = flag && applyActuatorModel();
     *  flag = flag && getActuatorOutput(ret_output_gc);
     *  return flag;
     * }
     * */
    virtual sBool actuate(const std::vector<sFloat>& arg_input,
        Eigen::VectorXd& ret_output_gc_force) = 0;

    /** Some actuator sets don't directly actuate the generalized coordinates
     * and require a Jacobian to compute their contribution to the generalized
     * forces.
     *
     * Each actuator instance must implement this. */
    virtual sBool computeJacobian()=0;

    /* *****************************************************************
     *                        Actuator Properties
     * ***************************************************************** */
    /** Does this actuator actuate the generalized coordinates?
     * If so, the computeJacobian function is not used. */
    virtual sBool getActuatesGC()=0;

    /* *****************************************************************
     *                        Initialization
     * ***************************************************************** */
    /** All actutors must be initialized */
    virtual sBool init(SActuatorSettings* arg_state)=0;

    /** Has this actuator been initialized */
    virtual sBool hasBeenInit()=0;

    /** Default constructor. Sets stuff to NULL */
    CActuatorBase() {}

    /** Default destructor. Frees memory */
    virtual ~CActuatorBase() {}

  protected:
    /** The subclass must set this during the initialization call. */
    SActuatorSettings settings_;
  };

} /* namespace scl */
#endif /* CACTUATORBASE_HPP_ */
