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
    \author    Stephen Sinclair (
    \author    http://www.mpb-technologies.ca/
    \version   2.0.0 $Rev: 244 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "devices/CFreedom6SDevice.h"
//---------------------------------------------------------------------------
#if defined(_ENABLE_MPB_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------
HINSTANCE hf6sDLL = NULL;
//---------------------------------------------------------------------------
// Copied from f6s.h in the Freedom6S API
//---------------------------------------------------------------------------

typedef enum
{
  F6SRC_NOERROR         =  0,
  F6SRC_ALREADYEXIST    = -1,   // A Freedom6S device is already open in the system
  F6SRC_BADVALUE        = -2,   // Value out of range
  F6SRC_BADPOINTER      = -3,   // Bad pointer passed to function
  F6SRC_MEMORY          = -4,   // Out of memory
  F6SRC_REGISTRY        = -5,   // Error reading registry values (will user defaults)
  F6SRC_INIFILE_READ    = -6,   // Error reading ini file (settings)
  F6SRC_INIFILE_WRITE   = -7,   // Error writing ini file (settings)
  F6SRC_NOTINITIALIZED  = -8,   // Attempt to call a function before f6s_Initialize()
  F6SRC_BADHANDLE       = -9,   // A function received a bad HF6S value
  F6SRC_BADMOTORTEMP    = -10,  // Motor temperatures were out of range or not read correctly (warning only)
  F6SRC_JOINTVELINIT    = -11,  // Attempt to read velocity without joint velocity computation enabled
  F6SRC_CALIBRATION     = -12,  // Unable to calibrate, require mechanical calibration
  F6SRC_ROLLANGLE       = -13,  // Unable to calculate roll angle, sensors 4 & 5 require mechanical re-calibration
  F6SRC_DRIVERINIT      = -14,  // Unable to initialize the drivers for ADC or DAC hardware
  F6SRC_IOERROR         = -15,  // Error returned from ADC or DAC drivers
  F6SRC_DAQCONFIG       = -16,  // Unknown DAQ configuration
  F6SRC_HOTMOTOR        = -17,  // One or more motors have been flagged hot, causing the max current to decrease
  F6SRC_FAILURE         = -18   // Operation failed
} F6SRC;

typedef void* HF6S;

F6SRC (*f6s_Initialize)( HF6S* phf6s );
F6SRC (*f6s_ComputeJointVel)( HF6S hf6s, float ftimeStep, int inewBufferSize );
F6SRC (*f6s_Cleanup)( HF6S hf6s );
F6SRC (*f6s_SetHoldDist)( HF6S hf6s, float fdist );
F6SRC (*f6s_SetForceTorque)( HF6S hf6s, const double force[3], const double torque[3] );
F6SRC (*f6s_GetPositionMatrixGL)( HF6S hf6s, double kineMat[16] );
F6SRC (*f6s_UpdateKinematics)( HF6S hf6s );
F6SRC (*f6s_GetVelocityGL)( HF6S hf6s, double linearVel[3],  double angularVel[3] );

// Initialize dhd dll reference count
int cFreedom6SDevice::m_activeFreedom6SDevices = 0;

//---------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cFreedom6SDevice.
    Loads interface DLL.

    \fn     cFreedom6SDevice::cFreedom6SDevice()
*/
//===========================================================================
cFreedom6SDevice::cFreedom6SDevice() : cGenericHapticDevice()
{
    m_systemReady = false;
    m_systemAvailable = false;
    m_hf6s = 0;

    m_activeFreedom6SDevices++;

    if (hf6sDLL==NULL)
    {
        hf6sDLL = LoadLibrary("freedom6s.dll");

        if (hf6sDLL==NULL)
            return;

        f6s_Initialize = (F6SRC (*)( HF6S* )) GetProcAddress(hf6sDLL, "f6s_Initialize");
        f6s_ComputeJointVel = (F6SRC (*)( HF6S , float , int  )) GetProcAddress(hf6sDLL, "f6s_ComputeJointVel");
        f6s_Cleanup = (F6SRC (*)( HF6S  )) GetProcAddress(hf6sDLL, "f6s_Cleanup");
        f6s_SetHoldDist = (F6SRC (*)( HF6S , float  )) GetProcAddress(hf6sDLL, "f6s_SetHoldDist");
        f6s_SetForceTorque = (F6SRC (*)( HF6S , const double [3], const double [3] )) GetProcAddress(hf6sDLL, "f6s_SetForceTorque");
        f6s_GetPositionMatrixGL = (F6SRC (*)( HF6S , double [16] )) GetProcAddress(hf6sDLL, "f6s_GetPositionMatrixGL");
        f6s_UpdateKinematics = (F6SRC (*)( HF6S  )) GetProcAddress(hf6sDLL, "f6s_UpdateKinematics");
        f6s_GetVelocityGL = (F6SRC (*)( HF6S hf6s, double [3],  double [3] )) GetProcAddress(hf6sDLL, "f6s_GetVelocityGL");

        if (  !f6s_Initialize
          ||  !f6s_ComputeJointVel
          ||  !f6s_Cleanup
          ||  !f6s_SetHoldDist
          ||  !f6s_SetForceTorque
          ||  !f6s_GetPositionMatrixGL
          ||  !f6s_UpdateKinematics
          ||  !f6s_GetVelocityGL)
        {
            FreeLibrary(hf6sDLL);
            hf6sDLL = NULL;
        }
    }

    // setup information about device
    m_specifications.m_manufacturerName              = "MPB Technologies";
    m_specifications.m_modelName                     = "freedom 6S";
    m_specifications.m_maxForce                      = 2.500;   // [N]
    m_specifications.m_maxForceStiffness             = 2000.0;  // [N/m]
    m_specifications.m_maxTorque                     = 0.250;   // [N*m]
    m_specifications.m_maxTorqueStiffness            = 3.000;   // [N*m/Rad]
    m_specifications.m_maxGripperTorque              = 0.000;   // [N*m]
    m_specifications.m_maxGripperTorqueStiffness     = 0.000;   // [N/m]
    m_specifications.m_maxLinearDamping              = 3.5;     // [N/(m/s)]
    m_specifications.m_workspaceRadius               = 0.100;   // [m]
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = true;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = true;
    m_specifications.m_actuatedRotation              = true;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;

    m_systemAvailable = true;
}

//===========================================================================
/*!
    Destructor of cFreedom6SDevice.

    \fn     cFreedom6SDevice::~cFreedom6SDevice()
*/
//===========================================================================
cFreedom6SDevice::~cFreedom6SDevice()
{
    if (m_hf6s != 0)
       f6s_Cleanup(m_hf6s);
    m_hf6s = 0;

    m_activeFreedom6SDevices--;

    if (m_activeFreedom6SDevices == 0 && hf6sDLL)
    {
        FreeLibrary(hf6sDLL);
        hf6sDLL = NULL;
    }
}

//===========================================================================
/*!
    Open connection to Freedom6S device.

    \fn     int cFreedom6SDevice::open()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cFreedom6SDevice::open()
{
   return 0;
}

//===========================================================================
/*!
    Close connection to Freedom6S device.

    \fn     int cFreedom6SDevice::close()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cFreedom6SDevice::close()
{
   return 0;
}

//===========================================================================
/*!
    Calibrate Freedom6S device. Initializes the driver, loading appropriate
    settings according to current Freedom6S configuration.

    \fn     int cFreedom6SDevice::initialize(const bool a_resetEncoders=false)
    \param  a_resetEncoders Ignored; exists for forward compatibility.
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cFreedom6SDevice::initialize(const bool a_resetEncoders)
{
    if (m_hf6s != 0)
        return -1;

    F6SRC rc = f6s_Initialize(&m_hf6s);
    if (m_hf6s && rc == F6SRC_NOERROR)
    {
        // Joint velocity computation:
        //   timestep = 1ms
        //   sample buffer size = 15
        f6s_ComputeJointVel(m_hf6s, 0.001f, 15);
        return 0;
    }

    m_hf6s = 0;
    return -1;
}


//===========================================================================
/*!
    Returns the number of devices available from this class of device.

    \fn      unsigned int cFreedom6SDevice::getNumDevices()
    \return  Returns the result
*/
//===========================================================================
unsigned int cFreedom6SDevice::getNumDevices()
{
    int result = 0;

    if (m_systemAvailable == true)
    {
        result = 1;
    }

    return (result);
}


//===========================================================================
/*!
    Read the position of the device. Units are meters [m].

    \fn     int cFreedom6SDevice::getPosition(cVector3d& a_position)
    \param  a_position  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::getPosition(cVector3d& a_position)
{
    // temp variables
    double kinemat[16];

    // read position information from device
    f6s_UpdateKinematics(m_hf6s);
    f6s_GetPositionMatrixGL(m_hf6s, kinemat);

    // kinemat is a row-major 4x4 rotation/translation matrix
    cVector3d result;
    result.x = kinemat[14];
    result.y = kinemat[12];
    result.z = kinemat[13];

    // return result
    a_position = result;

    // success
    return (0);
}


//===========================================================================
/*!
    Read the linear velocity of the device. Units are in meters per second [m/s].

    \fn     int cFreedom6SDevice::getLinearVelocity(cVector3d& a_linearVelocity)
    \param  a_linearVelocity  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::getLinearVelocity(cVector3d& a_linearVelocity)
{
    // temp variables
    double velLinear[3], velAngular[3];

    // read velocities from device
    f6s_GetVelocityGL(m_hf6s, velLinear, velAngular);

    m_linearVelocity.x = velLinear[2];
    m_linearVelocity.y = velLinear[0];
    m_linearVelocity.z = velLinear[1];

    m_angularVelocity.x = velAngular[2];
    m_angularVelocity.y = velAngular[0];
    m_angularVelocity.z = velAngular[1];

    // return result
    a_linearVelocity = m_linearVelocity;

    // success
    return (0);
}


//===========================================================================
/*!
    Read the orientation frame of the device end-effector

    \fn     int cFreedom6SDevice::getRotation(cMatrix3d& a_rotation)
    \param  a_rotation  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::getRotation(cMatrix3d& a_rotation)
{
    // success
    return (0);
}


//===========================================================================
/*!
    Read the angular velocity of the device. Units are in radians per
    second [m/s].

    \fn     int cFreedom6SDevice::getAngularVelocity(cVector3d& a_angularVelocity)
    \param  a_angularVelocity  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::getAngularVelocity(cVector3d& a_angularVelocity)
{
    // temp variables
    double velLinear[3], velAngular[3];

    // read velocities from device
    f6s_GetVelocityGL(m_hf6s, velLinear, velAngular);

    m_linearVelocity.x = velLinear[2];
    m_linearVelocity.y = velLinear[0];
    m_linearVelocity.z = velLinear[1];

    m_angularVelocity.x = velAngular[2];
    m_angularVelocity.y = velAngular[0];
    m_angularVelocity.z = velAngular[1];

    // return result
    a_angularVelocity = m_angularVelocity;

    // success
    return (0);
}


//===========================================================================
/*!
    Read the gripper angle in radian.

    \fn     int cFreedom6SDevice::getGripperAngleRad(double& a_angle)
    \param  a_angle  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::getGripperAngleRad(double& a_angle)
{
    // success
    return (0);
}


//===========================================================================
/*!
    Send a force [N] to the haptic device

    \fn     int cFreedom6SDevice::setForce(cVector3d& a_force)
    \param  a_force  Force command to be applied to device.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::setForce(cVector3d& a_force)
{
    // temp variables
    double force[3];
    double torque[3];

    // store the new force
    m_prevForce = a_force;

    // convert force and torque in local coordinates of device
    force[0] = m_prevForce.y;
    force[1] = -m_prevForce.x;
    force[2] = m_prevForce.z;

    torque[0] = m_prevTorque.y;
    torque[1] = -m_prevTorque.x;
    torque[2] = m_prevTorque.z;

    // write values to device
    f6s_SetForceTorque(m_hf6s, force, torque);

    // success
    return (0);
}


//===========================================================================
/*!
    Send a torque [N*m] to the haptic device

    \fn     int cFreedom6SDevice::setTorque(cVector3d& a_torque)
    \param  a_torque Force command to be applied to device.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::setTorque(cVector3d& a_torque)
{
    // temp variables
    double force[3];
    double torque[3];

    // store the new torque
    m_prevTorque = a_torque;

    // convert force and torque in local coordinates of device
    force[0] = m_prevForce.y;
    force[1] = -m_prevForce.x;
    force[2] = m_prevForce.z;

    torque[0] = m_prevTorque.y;
    torque[1] = -m_prevTorque.x;
    torque[2] = m_prevTorque.z;

    // write values to device
    f6s_SetForceTorque(m_hf6s, force, torque);

    // success
    return (0);
}


//===========================================================================
/*!
    Send a torque [N*m] to the gripper

    \fn     int cFreedom6SDevice::setGripperTorque(double a_gripperTorque)
    \param  a_gripperTorque  Torque command to be sent to gripper.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::setGripperTorque(double a_gripperTorque)
{
    // success
    return (0);
}


//===========================================================================
/*!
    Send a force [N] and a torque [N*m] and gripper torque [N*m] to the haptic device.

    \fn     int cFreedom6SDevice::setForceAndTorqueAndGripper(cVector3d& a_force,
            cVector3d& a_torque, double a_gripperTorque)
    \param  a_force  Force command.
    \param  a_torque  Torque command.
    \param  a_gripperTorque  Gripper torque command.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::setForceAndTorqueAndGripper(cVector3d& a_force, cVector3d& a_torque, double a_gripperTorque)
{
    // temp variables
    double force[3];
    double torque[3];

    // store the new force, torque and gripper torque
    m_prevTorque = a_torque;
    m_prevForce = a_force;
    m_prevGripperTorque = a_gripperTorque;

    // convert force and torque in local coordinates of device
    force[0] = m_prevForce.y;
    force[1] = -m_prevForce.x;
    force[2] = m_prevForce.z;

    torque[0] = m_prevTorque.y;
    torque[1] = -m_prevTorque.x;
    torque[2] = m_prevTorque.z;

    // write values to device
    f6s_SetForceTorque(m_hf6s, force, torque);

    // success
    return (0);
}


//===========================================================================
/*!
    Read the status of the user switch [1 = ON / 0 = OFF].

    \fn     int cFreedom6SDevice::getUserSwitch(int a_switchIndex, bool& a_status)
    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFreedom6SDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
	// no switch implemented
	a_status = false;

    // success
    return (0);
}

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
