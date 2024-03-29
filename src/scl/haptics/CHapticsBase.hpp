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
/* \file CHapticsBase.hpp
 *
 *  Created on: Oct 23, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CHAPTICSBASE_HPP_
#define CHAPTICSBASE_HPP_

//For interfacing with chai
#include <scl/data_structs/SObject.hpp>
#include <scl/DataTypes.hpp>
#include <Eigen/Core>

#include <vector>
#include <string>

namespace scl
{
  /** This class is a simple API to access a haptics subsystem.
   * It is designed to abstract the actual device initialization
   * and operation. Provides a high level interface to get device
   * positions for multiple haptic devices and specify their control
   * torques.
   */
  class CHapticsBase : public SObject
  {
  public:
    /** Default constructor. Does nothing. */
    CHapticsBase() : SObject("CHapticsBase") {}

    /** Default destructor. Closes haptic connections. */
    virtual ~CHapticsBase(){}

    /* ********************************************************
     *                    HAPTIC FUNCTIONS
     * ******************************************************** */
    /** Connects to the haptic devices.
     * Returns:
     *    success : The number of devices it found.
     *    failure : -1
     */
    virtual scl::sInt connectToDevices()=0;

    /** Get the present state of a single haptic devices. This is typically
     * the position, but can also include the orientation and/or a push
     * button. */
    virtual scl::sBool getHapticDevicePosition(const sUInt arg_id, Eigen::VectorXd& ret_pos_vec) const =0;

    /** Get the present state of all the haptic devices. This is typically
     * the position, but can also include the orientation and/or a push
     * button. */
    virtual scl::sBool getAllHapticDevicePositions(std::vector<Eigen::VectorXd>& ret_pos_vec) const =0;

    /** Sets the actuators on the haptic devices. */
    virtual scl::sBool setHapticDeviceActuator(const sUInt arg_id, const Eigen::VectorXd&  arg_cmd_vec)=0;

    /** Sets the actuators on the haptic devices. */
    virtual scl::sBool setAllHapticDeviceActuators(const std::vector<Eigen::VectorXd>& arg_cmd_vec)=0;

    /* Close the connections to the haptic devices */
    virtual scl::sBool closeConnectionToDevices()=0;

    /** Returns the number of currently connected devices */
    virtual scl::sUInt getNumDevicesConnected()=0;
  };
}

#endif /* CHAPTICSBASE_HPP_ */

