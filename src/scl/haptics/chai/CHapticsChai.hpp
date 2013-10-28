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
/* \file CHapticsChai.hpp
 *
 *  Created on: Sep 3, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CCHAIHAPTICS_HPP_
#define CCHAIHAPTICS_HPP_

//For interfacing with chai
#include <scl/haptics/CHapticsBase.hpp>

#include <Eigen/Core>

#include <vector>
#include <string>

// Forward declarations for chai classes so we don't have
// to include the chai header here. Instead, we can just
//include it in the .cpp file
namespace chai3d
{
class cHapticDeviceHandler;
class cGenericHapticDevice;
}

namespace scl
{
  class CHapticsChai : public CHapticsBase
  {
  public:
    /* ********************************************************
     *                    HAPTIC FUNCTIONS
     * ******************************************************** */
    /** Connects to the haptic devices.
     * Returns:
     *    success : The number of devices it found.
     *    failure : -1
     */
    virtual scl::sInt connectToDevices();

    /** Get the present state of a single haptic devices. This is typically
     * the position, but can also include the orientation and/or a push
     * button. */
    virtual scl::sBool getHapticDevicePosition(const sUInt arg_id, Eigen::VectorXd& ret_pos_vec) const ;

    /** Get the present state of the haptic devices. This is typically
     * the position, but can also include the orientation and/or a push
     * button. */
    virtual scl::sBool getAllHapticDevicePositions(std::vector<Eigen::VectorXd>& ret_pos_vec) const ;

    /** Sets the actuators on the haptic devices. */
    virtual scl::sBool setHapticDeviceActuator(const sUInt arg_id, const Eigen::VectorXd&  arg_cmd_vec)
    { /** NOTE TODO : Implement this */ return false; }

    /** Sets the actuators on the haptic devices. */
    virtual scl::sBool setAllHapticDeviceActuators(const std::vector<Eigen::VectorXd>& arg_cmd_vec)
    { /** NOTE TODO : Implement this */ return false; }

    /* Close the connections to the haptic devices */
    virtual bool closeConnectionToDevices();

    /** Returns the number of currently connected devices */
    virtual scl::sUInt getNumDevicesConnected()
    { return static_cast<scl::sUInt>(haptic_devices_.size());  }

    /* ********************************************************
     *                 INITIALIZTION FUNCTIONS
     * ******************************************************** */
    /** Default constructor. Does nothing. */
    CHapticsChai() : CHapticsBase(), haptics_handler_(NULL) {}

    /** Default destructor. Closes haptic connections. */
    ~CHapticsChai();

    /* ********************************************************
     *                    DATA STRUCTURES
     * ******************************************************** */
  private:
    /** A vector to contain the state of all presently connected haptic devices */
    std::vector<chai3d::cGenericHapticDevice*> haptic_devices_;

    /** Chai communicates with a haptic device through a handler. One handler keeps
     * track of all connected devices. */
    chai3d::cHapticDeviceHandler* haptics_handler_;
  };
}

#endif /* CCHAIHAPTICS_HPP_ */

