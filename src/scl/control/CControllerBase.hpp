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
 * \file CControllerBase.hpp
 *
 *  Created on: Jul 21, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CCONTROLLERBASE_HPP_
#define CCONTROLLERBASE_HPP_

#include <scl/DataTypes.hpp>

#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/data_structs/SRobotIOData.hpp>

#include <scl/control/data_structs/SControllerBase.hpp>

#include <scl/dynamics/CDynamicsBase.hpp>

#include <Eigen/Dense>

namespace scl
{
  /**
   * Basic force controller.
   *
   * All force controllers should inherit from this.
   *
   * Should be associated with a robot at initialization
   * time. Else should be stateless. Ie. A CRobot object
   * should be able to use this controller with any control
   * data structure.
   */
  class CControllerBase
  {
  public:
    /** The constructor sets the initialization state to false */
    CControllerBase() : has_been_init_(false), dynamics_(S_NULL){}

    /** The destructor does nothing */
    virtual ~CControllerBase(){}

    /** Equal to task forces or generalized coordinate forces
     * depending on the type of controller that implements this API */
    virtual sBool computeControlForces()=0;

    /** Computes the dynamic model : Mass, MassInv, centrifugal/coriolis, gravity, null spaces etc. */
    virtual sBool computeDynamics()=0;

    /** Whether the controller has been initialized to a particular robot
     *
     * The controllers must implement their choice of "public" init function, which
     * sets has_been_setup_ to true.
     *
     * The public init must accept a subclass of SControllerBase as its (pre-initialized)
     * data structure.
     *
     * See CGcController's implementation for more details.
     *
     * (has_been_setup_ == true) MUST BE a pre-requisite to running any controller.
     */
    virtual sBool init(SControllerBase* arg_data,
        scl::CDynamicsBase* arg_dynamics)=0;

    /** Reset the controller (set it to execute a different behavior perhaps)
     * NOTE : This does not reset the data structure or deallocate its memory
     *        because the data structure really should sit on a pile in the database
     * */
    virtual sBool reset()=0;

    /** Whether the controller has been attached to a robot or not */
    virtual sBool hasBeenInit() { return has_been_init_;  }

  protected:
    sBool has_been_init_;
    CDynamicsBase* dynamics_;
  };

}

#endif /* CCONTROLLERBASE_HPP_ */
