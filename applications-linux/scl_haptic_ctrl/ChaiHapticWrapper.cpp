/*
 * ChaiHapticWrapper.cpp
 *
 *  Created on: Sep 3, 2012
 *      Author: samir
 */

#include "ChaiHapticWrapper.hpp"
#include <chai3d.h>
#include <stdio.h>
#include <sstream>
#include <stdexcept>
#include <iostream>

namespace scl_app
{
  ChaiHapticWrapper::~ChaiHapticWrapper()
  {
    closeConnectionToDevices();
    if(NULL!=haptics_handler_)
    { delete haptics_handler_;  }
  }


  scl::sInt ChaiHapticWrapper::connectToDevices()
  {

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
        std::cout<<"\nChaiHapticWrapper::connectToDevices() : Searched and connected to ["
            <<haptics_handler_->getNumDevices() <<"] devices";
      }

      // ***********************************************************
      //                     CONNECT TO DEVICES
      // ***********************************************************
      for(int i=0;i<haptics_handler_->getNumDevices();++i)
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
        tmp = tmp_haptic_device->open();
        if (0 != tmp)
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
      std::cerr << "\nChaiHapticWrapper::connectToDevices() :" << e.what();
      return false;
    }
    return true;
  }

  bool ChaiHapticWrapper::getHapticDevicePositions(
      std::vector<Eigen::Vector3d>& ret_pos_vec)
  {
    if(ret_pos_vec.size() != haptics_handler_->getNumDevices())
    {
      std::cout<<"\n The passed vector doesn't have the same elements as the number of connected devices";
      return false;
    }

    // ***********************************************************
    //                     READ DEVICE STATE
    // ***********************************************************
    std::vector<cGenericHapticDevice*>::iterator it,ite;
    std::vector<Eigen::Vector3d>::iterator itv, itve;
    int i=0;
    for(it = haptic_devices_.begin(), ite = haptic_devices_.end(),
        itv = ret_pos_vec.begin(), itve = ret_pos_vec.end();
        it!=ite && itv!=itve; ++it, ++itv)
    {// Now read the state from the devices.
      cVector3d tmpv;
      int tmp =(*it)->getPosition(tmpv);
      *itv = tmpv;
      if (0 != tmp)
      { std::cout<<"WARNING : Could not read position of haptic device: "<<i; }
      i++;
    }
    return true;
  }

  bool ChaiHapticWrapper::closeConnectionToDevices()
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

} /* namespace scl_app */
