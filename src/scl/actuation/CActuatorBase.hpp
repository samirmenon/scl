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
  class SActuatorBase : public SObject
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
    SActuatorBase() :
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
   *        as these flags make sense for a simulated actuator. It is
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
    /** Applies the actuator model to the current input commands */
    virtual sBool applyActuatorModel()=0;

    /** Sets the actuator's command */
    virtual sBool setActuatorCommand(const Eigen::VectorXd& arg_input)=0;

    /** Gets the output of the actuator. Returned in the passed variable. */
    virtual sBool getActuatorOutput(Eigen::VectorXd& ret_output)=0;

    /** Convenience function that combines an actuator's functions */
    virtual sBool actuate(const Eigen::VectorXd& arg_input,
        Eigen::VectorXd& ret_output)
    {
      bool flag=true;
      flag = flag && setActuatorCommand(arg_input);
      flag = flag && applyActuatorModel();
      flag = flag && getActuatorOutput(ret_output);
      return flag;
    }

    /** Has this actuator been initialized */
    virtual sBool hasBeenInit()
    {
      if(S_NULL == state_) {  return false; }
      return state_->has_been_init_;
    }

    /* *****************************************************************
     *                        Initialization
     * ***************************************************************** */
    /** All actutors must be initialized */
    virtual sBool init(SActuatorBase* arg_state)=0;

    /** Default constructor. Sets stuff to NULL */
    CActuatorBase() : state_(S_NULL) {}

    /** Default destructor. Frees memory */
    virtual ~CActuatorBase() {}

  protected:
    /** The subclass must set this during the initialization call. */
    SActuatorBase* state_;
  };

} /* namespace scl */
#endif /* CACTUATORBASE_HPP_ */
