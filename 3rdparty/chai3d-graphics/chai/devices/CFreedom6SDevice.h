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
    \author    Stephen Sinclair
    \author    http://www.mpb-technologies.ca/
    \version   2.0.0 $Rev: 256 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CFreedom6SDeviceH
#define CFreedom6SDeviceH
//---------------------------------------------------------------------------
#if defined(_ENABLE_MPB_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CFreedom6SDevice.h

    \brief
    <b> Devices </b> \n 
    Freedom 6S Haptic Device.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cFreedom6SDevice
    \ingroup    devices  

    \brief  
    cFreedom6SDevice describes an interface to the Freedom6S haptic
    device from MPB Technologies Inc.
*/
//===========================================================================
class cFreedom6SDevice : public cGenericHapticDevice
{
  public:
 
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cFreedom6SDevice.
    cFreedom6SDevice();

    //! Destructor of cFreedom6SDevice.
    virtual ~cFreedom6SDevice();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Open connection to MPB device.
    int open();

    //! Close connection to MPB device.
    int close();

    //! Initialize MPB device.
    int initialize(const bool a_resetEncoders=false);

    //! Returns the number of devices available from this class of device.
    unsigned int getNumDevices();

    //! Read the position of the device. Units are meters [m].
    int getPosition(cVector3d& a_position);

    //! Read the linear velocity of the device. Units are in meters per second [m/s].
    int getLinearVelocity(cVector3d& a_linearVelocity);

    //! Read the orientation frame of the device end-effector.
    int getRotation(cMatrix3d& a_rotation);

    //! Read the angular velocity of the device. Units are in radians per second [m/s].
    int getAngularVelocity(cVector3d& a_angularVelocity);

    //! Read the gripper angle in radian.
    int getGripperAngleRad(double& a_angle);

    //! Send a force [N] to the haptic device.
    int setForce(cVector3d& a_force);

    //! Send a torque [N*m] to the haptic device.
    int setTorque(cVector3d& a_torque);

    //! Send a torque [N*m] to the gripper.
    int setGripperTorque(double a_gripperTorque);

    //! Send a force [N] and a torque [N*m] and gripper torque [N*m] to the haptic device.
    int setForceAndTorqueAndGripper(cVector3d& a_force, cVector3d& a_torque, double a_gripperTorque);

    //! Read the status of the user switch [\b true = \b ON / \b false = \b OFF].
    int getUserSwitch(int a_switchIndex, bool& a_status);


  protected:

    //! Reference count used to control access to the DLL.
    static int m_activeFreedom6SDevices;

    //! Handle to device.
    void* m_hf6s;
};

//---------------------------------------------------------------------------
#endif  // _ENABLE_MPB_DEVICE_SUPPORT
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

