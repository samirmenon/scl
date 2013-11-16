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
 * \file CSensorBase.hpp
 *
 *  Created on: Aug 24, 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CSENSORBASE_HPP_
#define CSENSORBASE_HPP_

#include <scl/data_structs/SObject.hpp>

#include <Eigen/Eigen>

namespace scl
{
  /** Some defaults that allow controlling a sensor model's behavior.
   * All sensor models should support these.
   *
   * NOTE : Please don't be confused. This is an actuator "model", and
   *        as these flags make sense for a simulated actuator. It is
   *        "NOT" meant as an interface to a real actuator. */
  class SSensorBase : public SObject
  {
  public:
    /** This applies any temporal sampling limits to the sensor.
     * In simulation, an encoder or a camera might have potentially
     * infinite resolution. Setting this flag will force sampling
     * the sensor at its actual temporal precision.
     * Eg. Sample encoder at 1ms
     *
     * DEFAULT : true
     */
    sBool flag_apply_temporal_limits_;  //true

    /** This applies any real-time jitter limits to the sensor.
     * In simulation, an encoder or a camera can instantaneously
     * sample the physical world. Setting this flag will force
     * some temporal jitter in the sensor. Ie. The sampled state
     * could be stale.
     * Eg. Sample encoder at 1ms
     *
     * DEFAULT : true
     */
    sBool flag_apply_temporal_jitter_;  //true

    /** This applies any real-time constant lag at the sensor.
     * In simulation, an encoder or a camera can instantaneously
     * sample the physical world. Setting this flag will force
     * some constant temporal lag in the sensor. Ie. The sampled
     * state will be stale.
     * Eg. Line delay from an encoder to the computer
     *
     * DEFAULT : true
     */
    sBool flag_apply_temporal_delay_;  //true

    /** This applies any spatial sampling limits to the sensor.
     * In simulation, an encoder or a camera might have potentially
     * infinite resolution. Setting this flag will force sampling
     * the sensor at its actual spatial precision.
     * Eg. Discretize encoder values,
     *     Fix video resolution to 640x480 etc.
     *
     * DEFAULT : true
     */
    sBool flag_apply_spatial_limits_;   //true

    /** A sensor may have a noise model to simulate the real world.
     * Setting this flag will enable it.
     *
     * DEFAULT : true
     */
    sBool flag_apply_noise_;            //true

    SSensorBase() : SObject("SSensorBase"),
      flag_apply_temporal_limits_(true),
      flag_apply_temporal_jitter_(true),
      flag_apply_temporal_delay_(true),
      flag_apply_spatial_limits_(true),
      flag_apply_noise_(true)
    {}
  };

  /** This class models a generic sensor such as an encoder or a camera.
   * It presents a high level interface to the simulated sensor.
   *
   * NOTE : Please don't be confused. This is a sensor "model", and
   *        as these functions make sense for a simulated sensor. It is
   *        "NOT" meant as an interface to a real sensor.
   *
   * __DO NOT__ subclass this base and start writing a driver in it.
   */
  class CSensorBase
  {
  public:
    /* *****************************************************************
     *                        Sensing Model
     * ***************************************************************** */
    /** Sets the sensor's values. This is done by the simulation.
     * Typically, the sensors are given a perfect value and the sensor
     * model processes the value in whatever way required. */
    virtual sBool setSensors(const Eigen::VectorXd& arg_sensory_input)=0;

    /** Applies the sensor model to the current input commands */
    virtual sBool applySensorModel()=0;

    /** Gets the processed output of the sensor.
     * Returned in the passed variable. */
    virtual sBool getSensorOutput(Eigen::VectorXd& ret_sensor_output)=0;

    /** Convenience function that combines a sensor's functions */
    virtual sBool sense(const Eigen::VectorXd& arg_sensory_input,
        Eigen::VectorXd& ret_sensor_output)
    {
      bool flag=true;
      flag = flag && setSensors(arg_sensory_input);
      flag = flag && applySensorModel();
      flag = flag && getSensorOutput(ret_sensor_output);
      return flag;
    }

    /* *****************************************************************
     *                        Initialization
     * ***************************************************************** */
    /** All actutors must be initialized */
    virtual sBool init(SSensorBase* arg_state)=0;

    /** Has this sensor been initialized */
    virtual sBool hasBeenInit()
    {
      if(S_NULL == state_) {  return false; }
      return state_->has_been_init_;
    }

    /** Default constructor. Sets stuff to NULL */
    CSensorBase() : state_(S_NULL) {}

    /** Default destructor. Frees memory */
    virtual ~CSensorBase() {}

  protected:
    /** The subclass must set this during the initialization call. */
    SSensorBase* state_;
  };

} /* namespace scl */
#endif /* CSENSORBASE_HPP_ */
