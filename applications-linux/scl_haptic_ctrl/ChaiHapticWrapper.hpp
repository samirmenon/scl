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
/* \file ChaiHapticWrapper.hpp
 *
 *  Created on: Sep 3, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CHAIHAPTICWRAPPER_HPP_
#define CHAIHAPTICWRAPPER_HPP_

//For interfacing with chai
#include <scl/DataTypes.hpp>
#include <sutil/CSingleton.hpp>
#include <Eigen/Core>

#include <vector>
#include <string>

// Forward declarations for chai classes so we don't have
// to include the chai header here. Instead, we can just
//include it in the .cpp file
class cHapticDeviceHandler;
class cGenericHapticDevice;

namespace scl_app
{
  class ChaiHapticWrapper
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
    scl::sInt connectToDevices();

    /** Get the present state of the haptic devices. This is typically
     * the position, but can also include the orientation and/or a push
     * button. */
    bool getHapticDevicePositions(std::vector<Eigen::Vector3d>& ret_pos_vec);

    /* Close the connections to the haptic devices */
    bool closeConnectionToDevices();

    /** Returns the number of currently connected devices */
    scl::sInt getNumDevicesConnected()
    { return haptic_devices_.size();  }

    /* ********************************************************
     *                 INITIALIZTION FUNCTIONS
     * ******************************************************** */
    /** Default constructor. Does nothing. */
    ChaiHapticWrapper() : haptics_handler_(NULL), has_been_init_(false) {}

    /** Default destructor. Closes haptic connections. */
    ~ChaiHapticWrapper();

    /* ********************************************************
     *                    DATA STRUCTURES
     * ******************************************************** */
  private:
    /** A vector to contain the state of all presently connected haptic devices */
    std::vector<cGenericHapticDevice*> haptic_devices_;

    /** Chai communicates with a haptic device through a handler. One handler keeps
     * track of all connected devices. */
    cHapticDeviceHandler* haptics_handler_;

    /** Whether any devices are connected */
    scl::sBool has_been_init_;
  };
}

#endif /* CHAIHAPTICWRAPPER_HPP_ */

