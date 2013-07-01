//==============================================================================
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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 994 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CVirtualDeviceH
#define CVirtualDeviceH
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_VIRTUAL_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include <mmsystem.h>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CVirtualDevice.h

    \brief
    <b> Devices </b> \n 
    Virtual Haptic Device.
*/
//==============================================================================
#ifndef DOXYGEN_SHOULD_SKIP_THIS
struct cVirtualDeviceData
{
    double       ForceX;   // Force component X.
    double       ForceY;   // Force component Y.
    double       ForceZ;   // Force component Z.
    double       TorqueA;  // Torque alpha.
    double       TorqueB;  // Torque beta.
    double       TorqueG;  // Torque gamma.
    double       PosX;     // Position X.
    double       PosY;     // Position Y.
    double       PosZ;     // Position Z.
    double       AngleA;   // Angle alpha.
    double       AngleB;   // Angle beta.
    double       AngleG;   // Angle gamma.
    bool         Button0;  // Button 0 status.
    bool         AckMsg;   // Acknowledge Message
    bool         CmdReset; // Command Reset
};


//------------------------------------------------------------------------------

void CALLBACK internal_timer_callback(UINT uTimerID, 
                                      UINT uMsg, 
                                      DWORD_PTR dwUser, 
                                      DWORD_PTR dw1, 
                                      DWORD_PTR dw2);

//------------------------------------------------------------------------------

#endif  // DOXYGEN_SHOULD_SKIP_THIS 

//==============================================================================
/*!
    \class      cVirtualDevice
    \ingroup    devices  

    \brief      
    Class which interfaces with the virtual device
*/
//==============================================================================
class cVirtualDevice : public cGenericHapticDevice
{
  public:
    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cVirtualDevice.
    cVirtualDevice();

    //! Destructor of cVirtualDevice.
    virtual ~cVirtualDevice();


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------
    //! Open connection to virtual device.
    bool open();

    //! Close connection to virtual device.
    bool close();

    //! Calibrate virtual device.
    bool calibrate(bool a_forceCalibration = false);

    //! Returns the number of devices available from this class of device.
    unsigned int getNumDevices();

    //! Read the position of the device. Units are meters [m].
    bool getPosition(cVector3d& a_position);

    //! Read the orientation frame of the device end-effector.
    bool getRotation(cMatrix3d& a_rotation);

    //! Read the status of the user switch [__true__ = \e ON / __false__ = \e OFF].
    bool getUserSwitch(int a_switchIndex, bool& a_status);

    //! Send a force [N] to the haptic device.
    bool setForce(const cVector3d& a_force);

    //! Return the last force [N] sent to the device.
    bool getForce(cVector3d& a_force);

  private:
    //! Shared memory connection to virtual haptic device.
    HANDLE m_hMapFile;

    //! Pointer to shared memory.
    LPVOID m_lpMapAddress;

    //! Pointer to shared memory data structure.
    cVirtualDeviceData* m_pDevice;

    //! 1-KHz timer
    MMRESULT m_timer;

    //! time interval
    long m_interval;

    //! Timer callback
    friend void CALLBACK internal_timer_callback(UINT uTimerID, 
                                                 UINT uMsg, 
                                                 DWORD_PTR dwUser, 
                                                 DWORD_PTR dw1, 
                                                 DWORD_PTR dw2);

    //! Synchronization
    HANDLE m_sync;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
#endif // C_ENABLE_VIRTUAL_DEVICE_SUPPORT
//------------------------------------------------------------------------------

