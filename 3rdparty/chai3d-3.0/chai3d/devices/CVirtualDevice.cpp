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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 993 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CVirtualDevice.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_VIRTUAL_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------


void CALLBACK internal_timer_callback(UINT uTimerID, 
                                      UINT uMsg, 
                                      DWORD_PTR dwUser, 
                                      DWORD_PTR dw1, 
                                      DWORD_PTR dw2)
{
    cVirtualDevice* device = (cVirtualDevice*)(dwUser);
    SetEvent(device->m_sync);
}

//==============================================================================
/*!
    Constructor of cVirtualDevice.
*/
//==============================================================================
cVirtualDevice::cVirtualDevice()
{
    // settings:
    m_specifications.m_model                         = C_HAPTIC_DEVICE_VIRTUAL;
    m_specifications.m_manufacturerName              = "CHAI3D";
    m_specifications.m_modelName                     = "virtual";
    m_specifications.m_maxLinearForce                = 10.0;    // [N]
    m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]
    m_specifications.m_maxGripperForce               = 0.0;     // [N]
    m_specifications.m_maxLinearStiffness            = 1000.0;  // [N/m]
    m_specifications.m_maxAngularStiffness           = 0.0;     // [N*m/Rad]
    m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N/m]
    m_specifications.m_maxLinearDamping              = 100;     // [N/(m/s)]
    m_specifications.m_maxAngularDamping             = 0.0;     // [N*m/(Rad/s)]
    m_specifications.m_maxGripperAngularDamping	     = 0.0;     // [N*m/(Rad/s)]
    m_specifications.m_workspaceRadius               = 0.15;    // [m]
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = false;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = true;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;

    m_deviceAvailable = false;
    m_deviceReady = false;

    // search for virtual device
    m_hMapFile = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,
        FALSE,
        "dhdVirtual");

    // no virtual device available
    if (m_hMapFile == NULL)
    {
        m_deviceReady = false;
        m_deviceAvailable = false;
        return;
    }

    // open connection to virtual device
    m_lpMapAddress = MapViewOfFile(
      m_hMapFile,
      FILE_MAP_ALL_ACCESS,
      0,
      0,
      0);

    // check whether connection succeeded
    if (m_lpMapAddress == NULL)
    {
        m_deviceReady = false;
        m_deviceAvailable = false;
        return;
    }

    // map memory
    m_pDevice = (cVirtualDeviceData*)m_lpMapAddress;

    // synchronization
    m_sync = CreateEvent(NULL, false, false, NULL);

    // setup 1-KHz timer
    TIMECAPS tc;
    if (timeGetDevCaps(&tc, sizeof(TIMECAPS)) != TIMERR_NOERROR)
    {
        return;
    }

    long min_interval = tc.wPeriodMin;
    m_interval = cMax(min_interval, (long)1);

    MMRESULT result = timeBeginPeriod(m_interval);

    // Start the timer
    m_timer = timeSetEvent(m_interval,
                           0,
                           internal_timer_callback,
                           (unsigned long)(this),
                           TIME_PERIODIC | TIME_CALLBACK_FUNCTION
                           );

    if (m_timer == 0)
    {
        return;
    }

    // virtual device is available
    m_deviceAvailable = true;
}


//==============================================================================
/*!
    Destructor of cVirtualDevice.
*/
//==============================================================================
cVirtualDevice::~cVirtualDevice()
{
    if (m_deviceAvailable)
    {
        // Stop the timer
        timeKillEvent(m_timer);
        timeEndPeriod(m_interval);

        // close shared memory
        CloseHandle(m_hMapFile);
    }
}


//==============================================================================
/*!
    Open connection to virtual device.

    \return Return 0 is operation succeeds, C_ERROR if an error occurs.
*/
//==============================================================================
bool cVirtualDevice::open()
{
    if (m_deviceAvailable)
    {
        m_deviceReady = true;
    }
    return (C_SUCCESS);
}


//==============================================================================
/*!
    Close connection to virtual device

    \return Return 0 is operation succeeds, C_ERROR if an error occurs.
*/
//==============================================================================
bool cVirtualDevice::close()
{
    m_deviceReady = false;

    return (C_SUCCESS);
}


//==============================================================================
/*!
    Calibrate virtual device.  

    \return Return 0 is operation succeeds, C_ERROR if an error occurs.
*/
//==============================================================================
bool cVirtualDevice::calibrate()
{
    if (m_deviceReady)
    {
        return (C_SUCCESS);
    }
    else
    {
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    Returns the number of devices available from this class of device.

    \return  Returns the result
*/
//==============================================================================
unsigned int cVirtualDevice::getNumDevices()
{
    // only one device can be enabled
    int result;
    if (m_deviceAvailable)
    {
        result = 1;
    }
    else
    {
        result = 0;
    }
    return (result);
}


//==============================================================================
/*!
    Read the position of the device. Units are meters [m].

    \param  a_position  Return value.
*/
//==============================================================================
bool cVirtualDevice::getPosition(cVector3d& a_position)
{
    if (!m_deviceReady)
    {
        a_position.set(0, 0, 0);
        return (C_ERROR);
    }

    double x,y,z;
    x = (double)(*m_pDevice).PosX;
    y = (double)(*m_pDevice).PosY;
    z = (double)(*m_pDevice).PosZ;
    a_position.set(x, y, z);

    return (C_SUCCESS);
}


//==============================================================================
/*!
    Read the orientation frame of the device end-effector.

    \param  a_rotation  Return value.
*/
//==============================================================================
bool cVirtualDevice::getRotation(cMatrix3d& a_rotation)
{
    if (!m_deviceReady)
    {
        a_rotation.identity();
        return (C_ERROR);
    }

    a_rotation.identity();

    return (C_SUCCESS);
}


//==============================================================================
/*!
    Send a force [N] to the haptic device.

    \param  a_force  Force command to be applied to device.
*/
//==============================================================================
bool cVirtualDevice::setForce(const cVector3d& a_force)
{
    if (!m_deviceReady) return (C_ERROR);

	// wait for synchronization event
	WaitForSingleObject(m_sync, 1000);

    ((*m_pDevice).ForceX) = a_force(0) ;
    ((*m_pDevice).ForceY) = a_force(1) ;
    ((*m_pDevice).ForceZ) = a_force(2) ;

    return (C_SUCCESS);
}


//==============================================================================
/*!
    Return the last force sent to the device.

    \param  a_force  Return value.
*/
//==============================================================================
bool cVirtualDevice::getForce(cVector3d& a_force)
{
    if (!m_deviceReady)
    {
        a_force.set(0,0,0);
        return (C_ERROR);
    }

    a_force(0)  = ((*m_pDevice).ForceX);
    a_force(1)  = ((*m_pDevice).ForceY);
    a_force(2)  = ((*m_pDevice).ForceZ);

    return (C_SUCCESS);
}

//==============================================================================
/*!
    Read the status of the user switch [__true__ = \e ON / __false__ = \e OFF].

    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.
*/
//==============================================================================
bool cVirtualDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    if (!m_deviceReady)
    {
        a_status = false;
        return (C_ERROR);
    }

    a_status = ((bool)(*m_pDevice).Button0);

    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif  // C_ENABLE_VIRTUAL_DEVICE_SUPPORT
//------------------------------------------------------------------------------


