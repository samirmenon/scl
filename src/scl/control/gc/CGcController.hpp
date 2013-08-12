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
 * \file CGcController.hpp
 *
 *  Created on: Dec 29, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CGCCONTROLLER_HPP_
#define CGCCONTROLLER_HPP_

#include <scl/DataTypes.hpp>

#include <scl/control/CControllerBase.hpp>

#include <scl/control/gc/data_structs/SGcController.hpp>

namespace scl
{
  /** A generic generalized coordinate space force controller.
   *
   * Requires set points for position and velocity. Optionally,
   * can also use acceleration set points and can introduce integral
   * gain.
   *
   * This controller's advantage is its simplicity. It's disadvantage
   * is that it is usually hard trajectories in the generalized coordinates.
   * Moreover, for redundant robots, such trajectories are energy sub-optimal
   * and require controlling motor task null spaces, which may be superfluous.
   */
  class CGcController : public scl::CControllerBase
  {
  public:
    /* ******************************************************************
     *                        CControllerBase API
     * ****************************************************************** */
    /** The constructor does nothing */
    CGcController();

    /** The destructor does nothing */
    virtual ~CGcController();

    /** Computes generalized coordinate forces. Uses position,
     * velocity and acceleration set points and gains.
     *
     * Does not use integral gain by default. To get an integral
     * gain controller look at computeControlForcesPIDA() */
    virtual sBool computeControlForces();

    /** Computes the kinematic model : Jacobian
     *
     * This function does nothing in GcControllers.
     * For the curious : The jacobian for a generalized coordinate
     * force controllers is the identity matrix
     * */
    virtual sBool computeKinematics(){  return true;  }

    /** Computes the dynamic model : Mass, MassInv, centrifugal/coriolis, gravity
     *
     * Following Khatib's convention : A and Ainv matrices + b and g vectors
     */
    virtual sBool computeDynamics();

    /** Support for non control computations in the controller.
     * This is really basic controller and does nothing here. Subclasses might
     * consider adding stuff to this function.. */
    virtual sBool computeNonControlOperations()
    { return true;  }

    /** Returns the current control forces */
    virtual const Eigen::VectorXd* getControlForces();

    /** Whether the controller has been initialized to a particular robot */
    virtual sBool init(SControllerBase* arg_data,
        scl::CDynamicsBase* arg_dynamics);

    /** Reset the controller (set it to execute a different behavior perhaps) */
    virtual sBool reset();

    /** Whether the controller has been attached to a robot or not */
    virtual sBool hasBeenInit() { return has_been_init_;  }

    /* ******************************************************************
     *                   Additional control functions
     * ****************************************************************** */
    /** Generic PD controller
     * Dynamic decoupling + PD controller on decoupled closed loop system.
     *
     * Does not include acceleration set points or integral gain. */
    sBool computePDControlForces();

    /** Controller that only compensates for gravity:
     * Gravity compensation + damping */
    sBool computeFloatForces();

  protected:

    SGcController* data_;
  };

}

#endif /* CGCCONTROLLER_HPP_ */
