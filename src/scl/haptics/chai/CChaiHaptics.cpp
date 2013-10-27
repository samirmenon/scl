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
/* \file CChaiHaptics.cpp
 *
 *  Created on: Sep 3, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/haptics/chai/CChaiHaptics.hpp>
#include <chai3d.h>

#include <stdio.h>
#include <sstream>
#include <stdexcept>
#include <iostream>

using namespace chai3d;

namespace scl
{
  CChaiHaptics::~CChaiHaptics()
  {
    closeConnectionToDevices();
    if(NULL!=haptics_handler_)
    { delete haptics_handler_;  }
  }


  scl::sInt CChaiHaptics::connectToDevices()
  {
    bool flag;
    try
    {
      // Create a haptic device handler :
      // NOTE : Chai apparently connects to the new devices within the
      //        constructor (Could lead to weird error states!).
      haptics_handler_ = new cHapticDeviceHandler();

      if (NULL == haptics_handler_)
      { throw(std::runtime_error("Could not create haptic handler (out of memory?)")); }
      else
      {
        std::cout<<"\nCChaiHaptics::connectToDevices() : Searched and connected to ["
            <<haptics_handler_->getNumDevices() <<"] devices";
      }

      // ***********************************************************
      //                     CONNECT TO DEVICES
      // ***********************************************************
      for(unsigned int i=0;i<haptics_handler_->getNumDevices();++i)
      {// Now connect to all the haptic devices.
        cGenericHapticDevice* tmp_haptic_device = NULL;

        // 1. Get a handle to the haptic device
        int tmp = haptics_handler_->getDevice(tmp_haptic_device, i);
        if (0 != tmp)
        {
          std::stringstream ss; ss<<"Could not register haptic device: "<<i;
          throw(std::runtime_error(ss.str()));
        }
        else
        { std::cout << "\nRegistered haptic device: "<< i << std::flush; }

        // 2. Open a connection to the haptic device.
        flag = tmp_haptic_device->open();
        if (false == flag)
        {
          std::stringstream ss; ss<<"Could not connect to haptic device: "<<i;
          throw(std::runtime_error(ss.str()));
        }
        else
        { std::cout << "\nConnected to haptic device: "<< i << std::flush; }

        //Add the haptic device pointer to the vector
        haptic_devices_.push_back(tmp_haptic_device);
      }//End of loop over haptic devices
    }
    catch (std::exception& e)
    {
      std::cerr << "\nCChaiHaptics::connectToDevices() :" << e.what();
      return 0;
    }
    return haptic_devices_.size();
  }

  /** Get the present state of a single haptic devices. This is typically
   * the position, but can also include the orientation and/or a push
   * button. */
  scl::sBool CChaiHaptics::getHapticDevicePosition(
      const sUInt arg_id, Eigen::VectorXd& ret_pos_vec) const
  {
    if(arg_id >= haptics_handler_->getNumDevices())
    {
      std::cout<<"\nCChaiHaptics::getHapticDevicePosition() : Error:"
          <<"\n\tThe requested haptic ID is greater than the number of connected devices";
      return false;
    }

    // ***********************************************************
    //                     READ DEVICE STATE
    // ***********************************************************
    cVector3d tmpv;
    int tmp = haptic_devices_[arg_id]->getPosition(tmpv);
    if (0 != tmp)
    { std::cout<<"\nCChaiHaptics::getHapticDevicePosition() : WARNING : \n\tCould not read position of haptic device: "<<arg_id; }

    Eigen::Vector3d tmp_vec;
    tmp_vec<<tmpv(0),tmpv(1),tmpv(2);
    ret_pos_vec = tmp_vec;

    return true;
  }

  bool CChaiHaptics::getAllHapticDevicePositions (
      std::vector<Eigen::VectorXd>& ret_pos_vec) const
  {
    if(ret_pos_vec.size() > haptics_handler_->getNumDevices())
    {
      std::cout<<"\nCChaiHaptics::getAllHapticDevicePositions () : Error:"
          <<"\n\tThe passed vector has more elements than the number of connected devices";
      return false;
    }

    // ***********************************************************
    //                     READ DEVICE STATE
    // ***********************************************************
    std::vector<cGenericHapticDevice*>::const_iterator it,ite;
    std::vector<Eigen::VectorXd>::iterator itv, itve;
    int i=0;
    for(it = haptic_devices_.begin(), ite = haptic_devices_.end(),
        itv = ret_pos_vec.begin(), itve = ret_pos_vec.end();
        it!=ite && itv!=itve; ++it, ++itv)
    {// Now read the state from the devices.
      cVector3d tmpv;
      bool device_ready =(*it)->getPosition(tmpv); //Read position into tmpv
      Eigen::Vector3d tmp_vec;
      tmp_vec<<tmpv(0),tmpv(1),tmpv(2);
      *itv = tmp_vec;                       //Set the positions in the vector
      if (false == device_ready)
      { std::cout<<"\nWARNING : Could not read position of haptic device: "<<i; }
      i++;
    }
    return true;
  }

  bool CChaiHaptics::closeConnectionToDevices()
  {
    bool flag = true;
    // ***********************************************************
    //                        CLOSE DEVICES
    // ***********************************************************
    int i=0;
    std::vector<cGenericHapticDevice*>::iterator it,ite;
    for(it = haptic_devices_.begin(), ite = haptic_devices_.end();
        it!=ite; ++it)
    {// Now read the state from the devices.
      int tmp = (*it)->close();
      if (0 != tmp)
      {
        std::cout<<"\nWARNING: Could not close connection to haptic device: "<<i;
        flag = false;
      }
      else
      { std::cout<<"\nClosed connection to haptic device: "<<i; }
      i++;
    }
    return flag;
  }

} /* namespace scl */
