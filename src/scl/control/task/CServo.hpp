/* This file is part of Busylizzy, a control and simulation library
for robots and biomechanical models.

Busylizzy is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

Busylizzy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
Busylizzy. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file CServo.hpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CSERVO_HPP_
#define CSERVO_HPP_

#include <scl/DataTypes.hpp>

#include <scl/data_structs/SObject.hpp>

#include <scl/control/task/data_structs/SServo.hpp>

namespace scl {

  /**
   *  Computes the main servo loop's data:
   *
   *  Main servo loop:
   *  1. Accesses the current torques from each task
   *  2. Projects the torques through the range spaces
   *  3. Sums them up to obtain the combined robot torque
   *
   *  NOTE: The servo DOES NOT update each individual task's torques.
   *  The controller (above) does that.
   */
  class CServo : public SObject
  {
  public:
    /** Default constructor. Does nothing. */
    CServo(): SObject("CServo") {}
    /** Default destructor. Does nothing. */
    virtual ~CServo(){}

    /** Initializes a servo.
     * NOTE : The arg_data pointer should point to a parent
     * controller's STaskController object */
    bool init(const std::string &arg_ctrl_name, SServo* arg_data);

    /** Resets the servo to its default state (so that
     * an new controller may be attached to it). */
    void reset();

    /** Computes the command forces (in generalized coordinate;
     * usually joint torques) by accepting the latest
     * forces from the task servos and filtering them
     * through their range spaces.
     *
     * Assumes that the range spaces are computed by a
     * controller implementation (orthogonal task-levels
     * or any other style) */
    bool computeControlForces();

  private:
      /** All the data a servo loop should ever need
       * see doc/ControllerDesign.eps for more details.
       *
       * This a pointer because the actual data structure
       * is stored in the STaskController (so that the
       * graphics, for instance, can access control torques).
       */
      SServo* data_;
  };

}

#endif /* CSERVO_HPP_ */
