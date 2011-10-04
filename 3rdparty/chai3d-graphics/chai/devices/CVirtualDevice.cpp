//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   2.0.0 $Rev: 244 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "devices/CVirtualDevice.h"
//---------------------------------------------------------------------------
#if defined(_ENABLE_VIRTUAL_DEVICE_SUPPORT)
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cVirtualDevice.

    \fn     cVirtualDevice::cVirtualDevice()
*/
//===========================================================================
cVirtualDevice::cVirtualDevice()
{
    // settings:
    m_specifications.m_manufacturerName              = "CHAI 3D";
    m_specifications.m_modelName                     = "virtual";
    m_specifications.m_maxForce                      = 10.0;    // [N]
    m_specifications.m_maxForceStiffness             = 2000.0;  // [N/m]
    m_specifications.m_maxTorque                     = 0.0;     // [N*m]
    m_specifications.m_maxTorqueStiffness            = 0.0;     // [N*m/Rad]
    m_specifications.m_maxGripperTorque              = 0.0;     // [N]
    m_specifications.m_maxGripperTorqueStiffness     = 0.0;     // [N/m]
    m_specifications.m_workspaceRadius               = 0.15;    // [m]
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = false;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = true;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;

    m_systemAvailable = false;
    m_systemReady = false;

    // search for virtual device
    m_hMapFile = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,
        FALSE,
        "dhdVirtual");

    // no virtual device available
    if (m_hMapFile == NULL)
    {
        m_systemReady = false;
        m_systemAvailable = false;
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
        m_systemReady = false;
        m_systemAvailable = false;
        return;
    }

    // map memory
    m_pDevice = (cVirtualDeviceData*)m_lpMapAddress;

    // virtual device is available
    m_systemAvailable = true;
}


//===========================================================================
/*!
    Destructor of cVirtualDevice.

    \fn         cVirtualDevice::~cVirtualDevice()
*/
//===========================================================================
cVirtualDevice::~cVirtualDevice()
{
    if (m_systemAvailable)
    {
        CloseHandle(m_hMapFile);
    }
}


//===========================================================================
/*!
    Open connection to virtual device.

    \fn     int cVirtualDevice::open()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cVirtualDevice::open()
{
    if (m_systemAvailable)
    {
        m_systemReady = true;
    }
    return (0);
}


//===========================================================================
/*!
    Close connection to virtual device

    \fn     int cVirtualDevice::close()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cVirtualDevice::close()
{
    m_systemReady = false;

    return (0);
}


//===========================================================================
/*!
    Initialize virtual device.  a_resetEncoders is ignored

    \fn     void cVirtualDevice::initialize(const bool a_resetEncoders=false)
    \param  a_resetEncoders ignored
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cVirtualDevice::initialize(const bool a_resetEncoders)
{
    if (m_systemReady)
    {
        return (0);
    }
    else
    {
        return (-1);
    }
}


//===========================================================================
/*!
    Returns the number of devices available from this class of device.

    \fn      unsigned int cVirtualDevice::getNumDevices()
    \return  Returns the result
*/
//===========================================================================
unsigned int cVirtualDevice::getNumDevices()
{
    // only one device can be enabled
    int result;
    if (m_systemAvailable)
    {
        result = 1;
    }
    else
    {
        result = 0;
    }
    return (result);
}


//===========================================================================
/*!
    Read the position of the device. Units are meters [m].

    \fn     int cVirtualDevice::getPosition(cVector3d& a_position)
    \param  a_position  Return value.
*/
//===========================================================================
int cVirtualDevice::getPosition(cVector3d& a_position)
{
    if (!m_systemReady)
    {
        a_position.set(0, 0, 0);
        return (-1);
    }

    double x,y,z;
    x = (double)(*m_pDevice).PosX;
    y = (double)(*m_pDevice).PosY;
    z = (double)(*m_pDevice).PosZ;
    a_position.set(x, y, z);

    return (0);
}


//===========================================================================
/*!
    Read the orientation frame of the device end-effector.

    \fn     int cVirtualDevice::getRotation(cMatrix3d& a_rotation)
    \param  a_rotation  Return value.
*/
//===========================================================================
int cVirtualDevice::getRotation(cMatrix3d& a_rotation)
{
    if (!m_systemReady)
    {
        a_rotation.identity();
        return (-1);
    }

    a_rotation.identity();

    return (0);
}


//===========================================================================
/*!
    Send a force [N] to the haptic device.

    \fn     int cVirtualDevice::setForce(cVector3d& a_force)
    \param  a_force  Force command to be applied to device.
*/
//===========================================================================
int cVirtualDevice::setForce(cVector3d& a_force)
{
    if (!m_systemReady) return (-1);

    ((*m_pDevice).ForceX) = a_force.x;
    ((*m_pDevice).ForceY) = a_force.y;
    ((*m_pDevice).ForceZ) = a_force.z;

    return (0);
}


//===========================================================================
/*!
    Return the last force sent to the device.

    \fn     int cVirtualDevice::getForce(cVector3d& a_force)
    \param  a_force  Return value.
*/
//===========================================================================
int cVirtualDevice::getForce(cVector3d& a_force)
{
    if (!m_systemReady)
    {
        a_force.set(0,0,0);
        return (-1);
    }

    a_force.x = ((*m_pDevice).ForceX);
    a_force.y = ((*m_pDevice).ForceY);
    a_force.z = ((*m_pDevice).ForceZ);

    return (0);
}

//===========================================================================
/*!
    Read the status of the user switch [\b true = \e ON / \b false = \e OFF].

    \fn     int cVirtualDevice::getUserSwitch(int a_switchIndex, bool& a_status)
    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.
*/
//===========================================================================
int cVirtualDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    if (!m_systemReady)
    {
        a_status = false;
        return (-1);
    }

    a_status = ((bool)(*m_pDevice).Button0);

    return (0);
}


//---------------------------------------------------------------------------
#endif  // _ENABLE_VIRTUAL_DEVICE_SUPPORT
//---------------------------------------------------------------------------


