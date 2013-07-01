//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2013, CHAI3D.
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
    \author    Federico Barbagli
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1057 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CPhantomDevicesH
#define CPhantomDevicesH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_PHANTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CPhantomDevices.h

    \brief
    <b> Devices </b> \n 
    Phantom Haptic Device.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cPhantomDevice
    \ingroup    devices  

    \brief
    Interface to Sensable/Geomagic Phantom haptic devices.

    \details
    cPhantomDevice implements an interface for all Sensable/Geomagic haptic devices.
*/
//==============================================================================
class cPhantomDevice : public cGenericHapticDevice
{
  public:
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cPhantomDevice.
    cPhantomDevice(unsigned int a_deviceNumber);

    //! Destructor of cPhantomDevice.
    ~cPhantomDevice();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

    //! Open connection to haptic device (0 indicates success).
    virtual bool open();

    //! Close connection to haptic device (0 indicates success).
    virtual bool close();

    //! Calibrate haptic device (0 indicates success).
    virtual bool calibrate(bool a_forceCalibration = false);

    //! Read the position of the device. Units are meters [m].
    virtual bool getPosition(cVector3d& a_position);

    //! Read orientation frame (3x3 matrix) of the haptic device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! Send force [N] to haptic device.
    virtual bool setForce(const cVector3d& a_force);

    //! Send force [N] and torque [N*m] to haptic device.
    virtual bool setForceAndTorque(const cVector3d& a_force, const cVector3d& a_torque);

    //! Read status of user switch [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitch(int a_switchIndex, bool& a_status);


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public: 

    //! Get number of haptic devices available for this class of devices.
    static unsigned int getNumDevices();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - DEVICE LIBRARY INITIALIZATION:
    //--------------------------------------------------------------------------

protected:

    //! Open libraries for this class of devices.
    static bool openLibraries();

    //! Close libraries for this class of devices.
    static bool closeLibraries();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - DEVICE LIBRARIES:
    //--------------------------------------------------------------------------

protected:

    //! Allocation table for devices of this class. __true__ means that the device has been allocated, __false__ means free.
    static bool s_allocationTable[C_MAX_DEVICES];

    //! Number of instances for this class of devices currently using the libraries.
    static unsigned int s_libraryCounter;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS
    //--------------------------------------------------------------------------

protected:

    //! Device ID number among the Phantom devices connected to the computer.
    int m_deviceID;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif // C_ENABLE_PHANTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
