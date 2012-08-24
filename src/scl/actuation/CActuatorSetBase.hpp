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
   * NOTE for "Real Robots": While working with a real robot, subclass
   *  this base class and implement your driver in it.
   */
  class CActuatorSetBase
  {
  public:
    /* *****************************************************************
     *                        Actuator Modeling
     * ***************************************************************** */
    /** Set the actuator commands. These will be translated into individual
     * actuator commands */
    virtual sBool actuate(const Eigen::VectorXd& arg_input)=0;

    /** Gets the output of the actuator. Returned in the passed variable.
     * Typically used for logging, validation etc.
     * Should "NOT" be used by a controller. */
    virtual sBool getActuatorOutputs(Eigen::VectorXd& ret_output)=0;

    /* *****************************************************************
     *                        Initialization
     * ***************************************************************** */
    /** Initialize the actuator set */
    virtual sBool init()=0;

    /** Has this actuator been initialized */
    virtual sBool hasBeenInit()
    { return has_been_init_; }

    /** Default constructor. Sets stuff to NULL */
    CActuatorSetBase(): has_been_init_(false) {}

    /** Default destructor. Does nothing. */
    virtual ~CActuatorSetBase() {}

  protected:
    /** A collection of actuators that a controller can send commands to */
    sutil::CMappedPointerList<std::string, CActuatorBase, true> actuators_;

    /** Initialization state */
    sBool has_been_init_;
  };

} /* namespace sutil */
#endif /* CACTUATORSETBASE_HPP_ */
