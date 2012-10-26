//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 800 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CHapticDeviceHandlerH
#define CHapticDeviceHandlerH
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CHapticDeviceHandler.h

    \brief
    <b> Devices </b> \n 
    Universal Haptic Device Handler.
*/
//===========================================================================

#include "devices/CGenericHapticDevice.h"

#if defined(C_ENABLE_VIRTUAL_DEVICE_SUPPORT)
#include "devices/CVirtualDevice.h"
#endif

#if defined(C_ENABLE_DELTA_DEVICE_SUPPORT)
#include "devices/CDeltaDevices.h"
#endif

#if defined(C_ENABLE_FALCON_DEVICE_SUPPORT)
#include "devices/CFalconDevice.h"
#endif

#if defined(C_ENABLE_PHANTOM_DEVICE_SUPPORT)
#include "devices/CPhantomDevices.h"
#endif

#if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)
#include "devices/CMyCustomDevice.h"
#endif

#if defined(C_ENABLE_BFR_DEVICE_SUPPORT)
#include "devices/CBFR1Device.h"
#endif
//---------------------------------------------------------------------------
//! Maximum number of devices that can be connected at the same time.
const unsigned int C_MAX_HAPTIC_DEVICES = 16;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \class      cHapticDeviceHandler
    \ingroup    devices  

    \brief
    This class implements a manager which lists the different devices
    available on your computer and provides handles to them.
*/
//===========================================================================
class cHapticDeviceHandler
{
  public:
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cHapticDeviceHandler.
    cHapticDeviceHandler();

    //! Destructor of cHapticDeviceHandler.
    virtual ~cHapticDeviceHandler();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Returns the number of devices connected to your computer.
    unsigned int getNumDevices() { return (m_numDevices); }

    //! Updates information regarding the devices that are connected to your computer.
    void update();

    //! Returns the specifications of the ith device.
    int getDeviceSpecifications(cHapticDeviceInfo& a_deviceSpecifications, 
								unsigned int a_index = 0);

    //! Returns a handle to the ith device if available.
    int getDevice(cGenericHapticDevice*& a_hapticDevice, 
				  unsigned int a_index = 0);


  private:

    //! Number of devices.
    unsigned int m_numDevices;

    //! Array of available haptic devices.
    cGenericHapticDevice* m_devices[C_MAX_HAPTIC_DEVICES];

    //! A default device with no functionalities.
    cGenericHapticDevice* m_nullHapticDevice;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
