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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 799 $
 */
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CFalconDeviceH
#define CFalconDeviceH
//---------------------------------------------------------------------------
#if defined(C_ENABLE_FALCON_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------
#define FALCON_NUM_DEVICES_MAX 8
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CFalconDevice.h

    \brief
    <b> Devices </b> \n 
    Falcon Haptic Device.
 */
//===========================================================================

//===========================================================================
/*!
    \class      cFalconDevice
    \ingroup    devices

    \brief  
    cFalconDevice describes an interface to the Falcon haptic device
    from Novint Technlogies.
 */
//===========================================================================
class cFalconDevice : public cGenericHapticDevice
{
public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cFalconDevice.
    cFalconDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cFalconDevice.
    virtual ~cFalconDevice();

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Open connection to haptic device (0 indicates success).
    int open();

    //! Close connection to haptic device (0 indicates success).
    int close();

    //! Calibrate haptic device (0 indicates success).
    int calibrate();

    //! Returns the number of devices available from this class of device.
    unsigned int getNumDevices();

    //! Read the position of the device. Units are meters [m].
    int getPosition(cVector3d& a_position);

    //! Send a force [N] to the haptic device.
    int setForce(const cVector3d& a_force);

    //! read the status of the user switch [\b true = \b ON / \b false = \b OFF].
    int getUserSwitch(int a_switchIndex, bool& a_status);


private:

    //! counter and device handles.
    static int m_opencount;
    static int m_handles[FALCON_NUM_DEVICES_MAX];

    //! Device ID number.
    int m_deviceID;

    //! Are the Falcon drivers installed and available.
    bool m_driverInstalled;

    //! handle to the Falcon device and its callback mechanism
    int    m_servoHandle;
    HANDLE m_servoEvent;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif //C_ENABLE_FALCON_DEVICE_SUPPORT
//---------------------------------------------------------------------------
