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
    \author    Force Dimension - www.forcedimension.com
    \version   2.0.0 $Rev: 248 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "devices/CDeltaDevices.h"
//---------------------------------------------------------------------------
#if defined(_ENABLE_DELTA_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

// DHD-API function availability
bool cDeltaDevice::sdhdGetDeviceCount              = true;
bool cDeltaDevice::sdhdGetDeviceID                 = true;
bool cDeltaDevice::sdhdGetSystemType               = true;
bool cDeltaDevice::sdhdOpenID                      = true;
bool cDeltaDevice::sdhdClose                       = true;
bool cDeltaDevice::sdhdGetButton                   = true;
bool cDeltaDevice::sdhdReset                       = true;
bool cDeltaDevice::sdhdGetPosition                 = true;
bool cDeltaDevice::sdhdGetLinearVelocity           = true;
bool cDeltaDevice::sdhdSetForce                    = true;
bool cDeltaDevice::sdhdGetOrientationRad           = true;
bool cDeltaDevice::sdhdSetTorque                   = true;
bool cDeltaDevice::sdhdGetOrientationFrame         = true;
bool cDeltaDevice::sdhdSetForceAndGripperForce     = true;
bool cDeltaDevice::sdhdGetGripperThumbPos          = true;
bool cDeltaDevice::sdhdGetGripperFingerPos         = true;
bool cDeltaDevice::sdhdEnableExpertMode            = true;
bool cDeltaDevice::sdhdDisableExpertMode           = true;
bool cDeltaDevice::sdhdEnableForce                 = true;
bool cDeltaDevice::sdhdIsLeftHanded                = true;

#if defined(_WIN32)
HINSTANCE dhdDLL = NULL;

int (__stdcall *dhdGetDeviceCount)             (void);
int (__stdcall *dhdGetDeviceID)                (void);
int (__stdcall *dhdGetSystemType)              (char ID);
int (__stdcall *dhdOpenID)                     (char ID);
int (__stdcall *dhdClose)                      (char ID);
int (__stdcall *dhdGetButton)                  (int index, char ID);
int (__stdcall *dhdReset)                      (char ID);
int (__stdcall *dhdGetPosition)                (double *px, double *py, double *pz, char ID);
int (__stdcall *dhdGetLinearVelocity)          (double *vx, double *vy, double *vz, char ID);
int (__stdcall *dhdSetForce)                   (double  fx, double  fy, double  fz, char ID);
int (__stdcall *dhdGetOrientationRad)          (double *oa, double *ob, double *og, char ID);
int (__stdcall *dhdSetTorque)                  (double  ta, double  tb, double  tg, char ID);
int (__stdcall *dhdGetOrientationFrame)        (double matrix[3][3], char ID);
int (__stdcall *dhdSetForceAndGripperForce)    (double fx, double fy, double fz, double f, char ID);
int (__stdcall *dhdGetGripperThumbPos)         (double *tx, double *ty, double *tz,  char ID);
int (__stdcall *dhdGetGripperFingerPos)        (double *fx, double *fy, double *fz,  char ID);
int (__stdcall *dhdEnableExpertMode)           (void);
int (__stdcall *dhdDisableExpertMode)          (void);
int (__stdcall *dhdEnableForce)                (unsigned char val, char ID);
bool (__stdcall *dhdIsLeftHanded)              (char ID);

#else
#include "dhdc.h"
#endif
//---------------------------------------------------------------------------

// Initialize dhd dll reference count
int cDeltaDevice::m_activeDeltaDevices = 0;

//---------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cDeltaDevice.

    \fn     cDeltaDevice::cDeltaDevice(unsigned int a_deviceNumber)
*/
//===========================================================================
cDeltaDevice::cDeltaDevice(unsigned int a_deviceNumber)
{
    // init variables
    statusEnableForcesFirstTime = true;

    // name of the device manufacturer
    m_specifications.m_manufacturerName = "Force Dimension";

    // device is not yet available or ready
    m_systemAvailable = false;
    m_systemReady = false;
    m_deviceType = -1;
    m_activeDeltaDevices++;

#if defined(_WIN32)
    // load dhd.dll library
    if (dhdDLL==NULL)
    {
        dhdDLL = LoadLibrary("dhd.dll");
    }

    // check if DLL loaded correctly
    if (dhdDLL == NULL)
    {
        return;
    }

    // load different callbacks
    dhdGetDeviceCount           = (int (__stdcall*)(void))GetProcAddress(dhdDLL, "dhdGetDeviceCount");
    if (dhdGetDeviceCount == NULL) { sdhdGetDeviceCount = false; }

    dhdGetDeviceID              = (int (__stdcall*)(void))GetProcAddress(dhdDLL, "dhdGetDeviceID");
    if (dhdGetDeviceID == NULL) { sdhdGetDeviceID = false; }

    dhdGetSystemType            = (int (__stdcall*)(char))GetProcAddress(dhdDLL, "dhdGetSystemType");
    if (dhdGetSystemType == NULL) { sdhdGetSystemType = false; }

    dhdOpenID                   = (int (__stdcall*)(char))GetProcAddress(dhdDLL, "dhdOpenID");
    if (dhdOpenID == NULL) { sdhdOpenID = false; }

    dhdClose                    = (int (__stdcall*)(char))GetProcAddress(dhdDLL, "dhdClose");
    if (dhdClose == NULL) { sdhdClose = false; }

    dhdGetButton                = (int (__stdcall*)(int, char))GetProcAddress(dhdDLL, "dhdGetButton");
    if (dhdGetButton == NULL) { sdhdGetButton = false; }

    dhdReset                    = (int (__stdcall*)(char))GetProcAddress(dhdDLL, "dhdReset");
    if (dhdReset == NULL) { sdhdReset = false; }

    dhdGetPosition              = (int (__stdcall*)(double*, double*, double*, char))GetProcAddress(dhdDLL, "dhdGetPosition");
    if (dhdGetPosition == NULL) { sdhdGetPosition = false; }

    dhdSetForce                 = (int (__stdcall*)(double, double, double, char))GetProcAddress(dhdDLL, "dhdSetForce");
    if (dhdSetForce == NULL) { sdhdSetForce = false; }

    dhdGetOrientationRad        = (int( __stdcall*)(double*, double*, double*, char))GetProcAddress(dhdDLL, "dhdGetOrientationRad");
    if (dhdGetOrientationRad == NULL) { sdhdGetOrientationRad = false; }

    dhdSetTorque                = (int (__stdcall*)(double, double, double, char))GetProcAddress(dhdDLL, "dhdSetTorque");
    if (dhdSetTorque == NULL) { sdhdSetTorque = false; }

    dhdGetOrientationFrame      = (int (__stdcall*)(double[3][3], char ID))GetProcAddress(dhdDLL, "dhdGetRotatorMatrix");
    if (dhdGetOrientationFrame == NULL)
    {
        dhdGetOrientationFrame  = (int (__stdcall*)(double[3][3], char ID))GetProcAddress(dhdDLL, "dhdGetOrientationFrame");
    }
    if (dhdGetOrientationFrame == NULL) { sdhdGetOrientationFrame = false; }

    dhdGetLinearVelocity        = (int (__stdcall*)(double *vx, double *vy, double *vz, char ID))GetProcAddress(dhdDLL, "dhdGetLinearVelocity");
    if (dhdGetLinearVelocity == NULL) { sdhdGetLinearVelocity = false; }

    dhdSetForceAndGripperForce  = (int (__stdcall*)(double fx, double fy, double fz, double f, char ID))GetProcAddress(dhdDLL, "dhdSetForceAndGripperForce");
    if (dhdSetForceAndGripperForce == NULL) { sdhdSetForceAndGripperForce = false; }

    dhdGetGripperThumbPos       = (int (__stdcall*)(double *tx, double *ty, double *tz,  char ID))GetProcAddress(dhdDLL, "dhdGetGripperThumbPos");
    if (dhdGetGripperThumbPos == NULL) { sdhdGetGripperThumbPos = false; }

    dhdGetGripperFingerPos      = (int (__stdcall*)(double *fx, double *fy, double *fz,  char ID))GetProcAddress(dhdDLL, "dhdGetGripperFingerPos");
    if (dhdGetGripperFingerPos == NULL) { sdhdGetGripperFingerPos = false; }

    dhdEnableExpertMode         = (int (__stdcall*)(void))GetProcAddress(dhdDLL, "dhdEnableExpertMode");
    if (dhdEnableExpertMode == NULL) { sdhdEnableExpertMode = false; }

    dhdDisableExpertMode        = (int (__stdcall*)(void))GetProcAddress(dhdDLL, "dhdDisableExpertMode");
    if (dhdDisableExpertMode == NULL) { sdhdDisableExpertMode = false; }

    dhdEnableForce              = (int (__stdcall*)(unsigned char val, char ID))GetProcAddress(dhdDLL, "dhdEnableForce");
    if (dhdEnableForce == NULL) { sdhdEnableForce = false; }

    dhdIsLeftHanded             = (bool(__stdcall*)(char ID))GetProcAddress(dhdDLL, "dhdIsLeftHanded");
    if (dhdIsLeftHanded == NULL) { sdhdIsLeftHanded = false; }
#endif

    // display a message if some function calls are missing
    if (   (!sdhdGetDeviceCount)
        || (!sdhdGetDeviceID)
        || (!sdhdGetSystemType)
        || (!sdhdOpenID)
        || (!sdhdClose)
        || (!sdhdGetButton)
        || (!sdhdReset)
        || (!sdhdGetPosition)
        || (!sdhdGetLinearVelocity)
        || (!sdhdSetForce)
        || (!sdhdGetOrientationRad)
        || (!sdhdSetTorque)
        || (!sdhdGetOrientationFrame)
        || (!sdhdSetForceAndGripperForce)
        || (!sdhdGetGripperThumbPos)
        || (!sdhdGetGripperFingerPos)
        || (!sdhdEnableExpertMode)
        || (!sdhdDisableExpertMode)
        || (!sdhdEnableForce)
        || (!sdhdIsLeftHanded))
    {
      printf("Notice - For optimal performances, please update the drivers of your Force Dimension haptic device.");
    }

    // get the number ID of the device we wish to communicate with
    m_deviceID = a_deviceNumber;

    // get the number of Force Dimension devices connected to this computer
    int numDevices = dhdGetDeviceCount();

    // check if such device is available
    if ((a_deviceNumber + 1) > (unsigned int)numDevices)
    {
        // no, such ID does not lead to an existing device
        m_systemAvailable = false;
    }
    else
    {
        // yes, this ID leads to an existing device
        m_systemAvailable = true;

        // open the device to read all the technical specifications about it
        open();

        // close the device
        close();
    }

    // init code to handle buttons
    int i = 0;
    for (i=0; i<8; i++)
    {
		m_userSwitchCount[i] = 0;
        m_userSwitchStatus[i] = 0;
        m_userSwitchClock[i].reset();
        m_userSwitchClock[i].start();
    }
}


//===========================================================================
/*!
    Destructor of cDeltaDevice.

    \fn         cDeltaDevice::~cDeltaDevice()
*/
//===========================================================================
cDeltaDevice::~cDeltaDevice()
{
    // close connection to device
    if (m_systemReady)
    {
        close();
    }

    m_activeDeltaDevices--;

#if defined(_WIN32)
    if (m_activeDeltaDevices == 0 && dhdDLL)
    {
      FreeLibrary(dhdDLL);
      dhdDLL = 0;
    }
#endif
}


//===========================================================================
/*!
    Open connection to the omega.x or delta.x device.

    \fn     int cDeltaDevice::open()
*/
//===========================================================================
int cDeltaDevice::open()
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    // if system is already opened then return
    if (m_systemReady) return (0);

    // check if DHD-API call is available
    if (!sdhdOpenID) return (-1);

    // try to open the device
    int result = dhdOpenID(m_deviceID);

    // update device status
    if (result >= 0)
    {
        m_systemReady = true;
    }
    else
    {
        m_systemReady = false;
        return (-1);
    }

    // init force status
    statusEnableForcesFirstTime = true;

    // read the device type
    m_deviceType = DHD_DEVICE_OMEGA;
    if (sdhdGetSystemType)
    {
        m_deviceType = dhdGetSystemType(m_deviceID);
    }

    // left/right hand
    bool leftHandOnly = false;
    if (sdhdIsLeftHanded)
    {
        leftHandOnly = dhdIsLeftHanded(m_deviceID);
    }

    // setup information regarding device
    switch (m_deviceType)
    {
        //------------------------------------------------------------------
        // delta devices
        //------------------------------------------------------------------
        case (DHD_DEVICE_3DOF):
        {
            m_specifications.m_modelName                     = "delta.3";
            m_specifications.m_maxForce                      = 20.0;    // [N]
            m_specifications.m_maxForceStiffness             = 5000.0;  // [N/m]
            m_specifications.m_maxTorque                     = 0.0;     // [N*m]
            m_specifications.m_maxTorqueStiffness            = 0.0;     // [N*m/Rad]
            m_specifications.m_maxGripperTorque              = 0.0;     // [N]
            m_specifications.m_maxGripperTorqueStiffness     = 0.0;     // [N*m/m]
            m_specifications.m_maxLinearDamping              = 40.0;    // [N/(m/s)]
            m_specifications.m_workspaceRadius               = 0.15;    // [m]
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = true;
        }
        break;

        case (DHD_DEVICE_6DOF):
        case (DHD_DEVICE_6DOF_500):
        {
            m_specifications.m_modelName                     = "delta.6";
            m_specifications.m_maxForce                      = 20.0;    // [N]
            m_specifications.m_maxForceStiffness             = 4000.0;  // [N/m]
            m_specifications.m_maxTorque                     = 0.2;     // [N*m]
            m_specifications.m_maxTorqueStiffness            = 1.0;     // [N*m/Rad]
            m_specifications.m_maxGripperTorque              = 0.0;     // [N]
            m_specifications.m_maxGripperTorqueStiffness     = 0.0;     // [N*m/m]
            m_specifications.m_maxLinearDamping              = 40.0;    // [N/(m/s)]
            m_specifications.m_workspaceRadius               = 0.15;    // [m]
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = true;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = true;
        }
        break;

        //------------------------------------------------------------------
        // omega devices
        //------------------------------------------------------------------
        case (DHD_DEVICE_OMEGA):
        case (DHD_DEVICE_OMEGA3):
        {
            m_specifications.m_modelName                     = "omega.3";
            m_specifications.m_maxForce                      = 12.0;    // [N]
            m_specifications.m_maxForceStiffness             = 5000.0;  // [N/m]
            m_specifications.m_maxTorque                     = 0.0;     // [N*m]
            m_specifications.m_maxTorqueStiffness            = 0.0;     // [N*m/Rad]
            m_specifications.m_maxGripperTorque              = 0.0;     // [N]
            m_specifications.m_maxGripperTorqueStiffness     = 0.0;     // [N*m/m]
            m_specifications.m_maxLinearDamping              = 40.0;    // [N/(m/s)]
            m_specifications.m_workspaceRadius               = 0.075;   // [m]
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = false;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = true;
            m_specifications.m_rightHand                     = true;
        }
        break;

        case (DHD_DEVICE_OMEGA33):
        case (DHD_DEVICE_OMEGA33_LEFT):
        {
            m_specifications.m_modelName                     = "omega.6";
            m_specifications.m_maxForce                      = 12.0;    // [N]
            m_specifications.m_maxForceStiffness             = 4000.0;  // [N/m]
            m_specifications.m_maxTorque                     = 0.0;     // [N*m]
            m_specifications.m_maxTorqueStiffness            = 0.0;     // [N*m/Rad]
            m_specifications.m_maxGripperTorque              = 0.0;     // [N]
            m_specifications.m_maxGripperTorqueStiffness     = 0.0;     // [N*m/m]
            m_specifications.m_maxLinearDamping              = 40.0;    // [N/(m/s)]
            m_specifications.m_workspaceRadius               = 0.075;   // [m]
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = false;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = false;
            m_specifications.m_leftHand                      = leftHandOnly;
            m_specifications.m_rightHand                     = !leftHandOnly;
        }
        break;

        case (DHD_DEVICE_OMEGA331):
        case (DHD_DEVICE_OMEGA331_LEFT):
        {
            m_specifications.m_modelName                     = "omega.7";
            m_specifications.m_maxForce                      = 12.0;    // [N]
            m_specifications.m_maxForceStiffness             = 4000.0;  // [N/m]
            m_specifications.m_maxTorque                     = 0.0;     // [N*m]
            m_specifications.m_maxTorqueStiffness            = 0.0;     // [N*m/Rad]
            m_specifications.m_maxGripperTorque              = 8.0;     // [N]
            m_specifications.m_maxGripperTorqueStiffness     = 10.0;    // [N*m/m]
            m_specifications.m_maxLinearDamping              = 40.0;    // [N/(m/s)]
            m_specifications.m_workspaceRadius               = 0.075;   // [m]
            m_specifications.m_sensedPosition                = true;
            m_specifications.m_sensedRotation                = true;
            m_specifications.m_sensedGripper                 = true;
            m_specifications.m_actuatedPosition              = true;
            m_specifications.m_actuatedRotation              = false;
            m_specifications.m_actuatedGripper               = true;
            m_specifications.m_leftHand                      = leftHandOnly;
            m_specifications.m_rightHand                     = !leftHandOnly;
        }
        break;
    }

    return (result);
}


//===========================================================================
/*!
    Close connection to the omega.x or delta.x device.

    \fn     int cDeltaDevice::close()
*/
//===========================================================================
int cDeltaDevice::close()
{
    // check if the system has been opened previously
    if (!m_systemReady) return (-1);

    // check if DHD-API call is available
    if (!sdhdClose) return (-1);

    // yes, the device is open so let's close it
    int result = dhdClose(m_deviceID);

    // update status
    m_systemReady = false;

    // reset force status
    statusEnableForcesFirstTime = true;

    return (result);
}


//===========================================================================
/*!
    Calibrate the omega.x or delta.x device.

    This function does nothing right now; the a_resetEncoders parameter is ignored.

    \fn     int cDeltaDevice::initialize(const bool a_resetEncoders = false)
    \param  a_resetEncoders Ignored; exists for forward compatibility.
    \return Always 0
*/
//===========================================================================
int cDeltaDevice::initialize(const bool a_resetEncoders)
{
    return (0);
}


//===========================================================================
/*!
    Returns the number of devices available from this class of device.

    \fn      unsigned int cDeltaDevice::getNumDevices();
    \return  Returns the result
*/
//===========================================================================
unsigned int cDeltaDevice::getNumDevices()
{
    // check if the system is available
    if (!m_systemAvailable) return (0);

    // check if DHD-API call is available
    if (!sdhdGetDeviceCount) return (1);

    int result = dhdGetDeviceCount();
    return (result);
}


//===========================================================================
/*!
    Read the position of the device. Units are meters [m].

    \fn     int cDeltaDevice::getPosition(cVector3d& a_position)
    \param  a_position  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::getPosition(cVector3d& a_position)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    // check if DHD-API call is available
    if (!sdhdGetPosition) return (-1);

    int error = -1;
    double x,y,z;
    error = dhdGetPosition(&x, &y, &z, m_deviceID);
    a_position.set(x, y, z);
#if !defined(_MACOSX) & !defined(_LINUX)
    estimateLinearVelocity(a_position);
#endif
    return (error);
}


//===========================================================================
/*!
    Read the linear velocity of the device. Units are in [m/s].

    \fn     int cDeltaDevice::getLinearVelocity(cVector3d& a_linearVelocity)
    \param  a_linearVelocity  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::getLinearVelocity(cVector3d& a_linearVelocity)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    int error = 0;

#if defined(_MACOSX) | defined(_LINUX)
    double vx,vy,vz;
    error = dhdGetLinearVelocity(&vx, &vy, &vz, m_deviceID);
    m_linearVelocity.set(vx, vy, vz);
#endif

    a_linearVelocity = m_linearVelocity;

    return (error);
}


//===========================================================================
/*!
    Read the orientation frame of the device end-effector

    \fn     int cDeltaDevice::getRotation(cMatrix3d& a_rotation)
    \param  a_rotation  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    int error = 0;
    cMatrix3d frame;
    frame.identity();

    switch (m_deviceType)
    {
        // delta devices
        case (DHD_DEVICE_3DOF):
        case (DHD_DEVICE_6DOF):
        case (DHD_DEVICE_6DOF_500):
        {
            // read angles
            cVector3d angles;
            angles.set(0,0,0);

            // check if DHD-API call is available
            if (sdhdGetOrientationRad)
            {
                error = dhdGetOrientationRad(&angles.x, &angles.y, &angles.z, m_deviceID);
            }

            // compute rotation matrix
            angles.mul(1.5);
            frame.rotate(cVector3d(1,0,0), angles.x);
            frame.rotate(cVector3d(0,1,0), angles.y);
            frame.rotate(cVector3d(0,0,1), angles.z);
        }
        break;

        // omega devices
        case (DHD_DEVICE_OMEGA):
        case (DHD_DEVICE_OMEGA3):
        case (DHD_DEVICE_OMEGA33):
        case (DHD_DEVICE_OMEGA331):
        {
            // read rotation matrix
            double rot[3][3];
            rot[0][0] = 1.0; rot[0][1] = 0.0; rot[0][2] = 0.0;
            rot[1][0] = 0.0; rot[1][1] = 1.0; rot[1][2] = 0.0;
            rot[2][0] = 0.0; rot[2][1] = 0.0; rot[2][2] = 1.0;

            if (sdhdGetOrientationFrame)
            {
                error = dhdGetOrientationFrame(rot, m_deviceID);
            }

            cMatrix3d result;
            frame.m[0][0] = rot[0][0];
            frame.m[0][1] = rot[0][1];
            frame.m[0][2] = rot[0][2];
            frame.m[1][0] = rot[1][0];
            frame.m[1][1] = rot[1][1];
            frame.m[1][2] = rot[1][2];
            frame.m[2][0] = rot[2][0];
            frame.m[2][1] = rot[2][1];
            frame.m[2][2] = rot[2][2];
        }
        break;
    }

    // return result
    a_rotation = frame;
    estimateAngularVelocity(a_rotation);
    return (error);
}


//===========================================================================
/*!
    Read the gripper angle in radian.

    \fn     int cDeltaDevice::getGripperAngleRad(double& a_angle)
    \param  a_angle  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::getGripperAngleRad(double& a_angle)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    int error = 0;
    a_angle = 0;
    estimateGripperVelocity(a_angle);
    return (error);
}


//===========================================================================
/*!
    Send a force [N] to the haptic device

    \fn     int cDeltaDevice::setForce(cVector3d& a_force)
    \param  a_force  Force command to be applied to device.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::setForce(cVector3d& a_force)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    // check if forces need to be enable (happens only once)
    if (statusEnableForcesFirstTime) { enableForces(true); }

    // check if DHD-API call is available
    if (!sdhdSetForce) return (-1);

    int error = dhdSetForce(a_force.x, a_force.y, a_force.z, m_deviceID);
    m_prevForce = a_force;
    return (error);
}


//===========================================================================
/*!
    Send a torque [N*m] to the haptic device

    \fn     int cDeltaDevice::setTorque(cVector3d& a_torque)
    \param  a_torque Force command to be applied to device.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::setTorque(cVector3d& a_torque)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    // check if forces need to be enable (happens only once)
    if (statusEnableForcesFirstTime) { enableForces(true); }

    // check if DHD-API call is available
    if (!sdhdSetTorque) return (-1);

    int error = dhdSetTorque(a_torque.x, a_torque.y, a_torque.z, m_deviceID);
    m_prevTorque = a_torque;
    return (error);
}


//===========================================================================
/*!
    Send a torque [N*m] to the gripper

    \fn     int cDeltaDevice::setGripperTorque(double a_gripperTorque)
    \param  a_gripperTorque  Torque command to be sent to gripper.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::setGripperTorque(double a_gripperTorque)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    // check if forces need to be enable (happens only once)
    if (statusEnableForcesFirstTime) { enableForces(true); }

    int error = 0;
    m_prevGripperTorque = a_gripperTorque;
    return (error);
}


//===========================================================================
/*!
    Send a force [N] and a torque [N*m] and gripper torque [N*m] to the haptic device.

    \fn     int cDeltaDevice::setForceAndTorqueAndGripper(cVector3d& a_force,
            cVector3d& a_torque, double a_gripperTorque)
    \param  a_force  Force command.
    \param  a_torque  Torque command.
    \param  a_gripperTorque  Gripper torque command.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::setForceAndTorqueAndGripper(cVector3d& a_force, cVector3d& a_torque, double a_gripperTorque)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    // check if forces need to be enable (happens only once)
    if (statusEnableForcesFirstTime) { enableForces(true); }

    // apply force and gripper torque
    if (!m_specifications.m_actuatedRotation)
    {
        if (sdhdSetForceAndGripperForce)
        {
            dhdSetForceAndGripperForce(a_force.x, a_force.y, a_force.z, a_gripperTorque, m_deviceID);
        }
        else if (sdhdSetForce)
        {
            dhdSetForce(a_force.x, a_force.y, a_force.z, m_deviceID);
        }
    }
    else if ((sdhdSetForce) && (sdhdSetTorque) && (m_specifications.m_actuatedRotation))
    {
        dhdSetForce(a_force.x, a_force.y, a_force.z, m_deviceID);
        dhdSetTorque(a_torque.x, a_torque.y, a_torque.z, m_deviceID);
    }

    else if (sdhdSetForce)
    {
        dhdSetForce(a_force.x, a_force.y, a_force.z, m_deviceID);
    }

    int error = 0;
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperTorque = a_gripperTorque;
    return (error);
}


//===========================================================================
/*!
    Read the status of the user switch [1 = \e ON / 0 = \e OFF].

    \fn     int cDeltaDevice::getUserSwitch(int a_switchIndex, bool& a_status)
    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    int error = 0;
    if (getUserSwitch(m_deviceID) == 0)
	{
		a_status = false;
	}
	else
	{
		a_status = true;
	};
    return (error);
}


//===========================================================================
/*!
    Read the user switch of the end-effector.
    This function implements a small filter to avoid reading glitches.

    \fn         int cDeltaDevice::getUserSwitch(int a_deviceID)
    \param      a_deviceID  device ID.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::getUserSwitch(int a_deviceID)
{
    // check if the system is available
    if (!m_systemAvailable) return (-1);

    const long SWITCHTIMEOUT = 10000; // [us]
    const double SWITCHTIMEOUT_SECONDS = ((double)(SWITCHTIMEOUT)) / 1e6;

    // check device id.
    if ((a_deviceID < 0) || (a_deviceID > 7))
    {
        return (0);
    }

    // check time
    if (m_userSwitchClock[a_deviceID].getCurrentTimeSeconds() < SWITCHTIMEOUT_SECONDS)
    {
        return (m_userSwitchStatus[a_deviceID]);
    }
    else
    {
        // check if DHD-API call is available
        if (!sdhdGetButton) return (m_userSwitchStatus[a_deviceID]);

        // timeout has occurred, we read the status again
        int switchStatus;
        switchStatus = dhdGetButton(0, (char)a_deviceID);

        // if value equals to zero, check once more time
        if (switchStatus == 1)
        {
            m_userSwitchStatus[a_deviceID] = 1;
			m_userSwitchCount[a_deviceID] = 0;
        }
		else
		{
			m_userSwitchCount[a_deviceID]++;
			if (m_userSwitchCount[a_deviceID] > 6)
			{
				m_userSwitchStatus[a_deviceID] = 0;
				m_userSwitchCount[a_deviceID] = 0;
			}
		}

        m_userSwitchClock[a_deviceID].reset();
        m_userSwitchClock[a_deviceID].start();

        return (m_userSwitchStatus[a_deviceID]);
    }
}


//===========================================================================
/*!
    Enable/Disable the motors of the omega.x device.
    This function overrides the force button located at the base of the
    device or on the controller panel.

    \fn         int cDeltaDevice::enableForces(bool a_value)
    \param      a_value  force status.
    \return     Return 0 if no error occurred.
*/
//===========================================================================
int cDeltaDevice::enableForces(bool a_value)
{
    // check if DHD-API call is available
    if (!sdhdEnableExpertMode) return (-1);
    if (!sdhdEnableForce) return (-1);
    if (!sdhdDisableExpertMode) return (-1);

    if (a_value)
    {
        // enable forces
        dhdEnableExpertMode();
        dhdEnableForce(DHD_ON, m_deviceID);
        dhdDisableExpertMode();
        statusEnableForcesFirstTime = false;
    }
    else
    {
        // disable forces
        dhdEnableExpertMode();
        dhdEnableForce(DHD_OFF, m_deviceID);
        dhdDisableExpertMode();
    }

    return (0);
}

//---------------------------------------------------------------------------
#endif //_ENABLE_DELTA_DEVICE_SUPPORT
//---------------------------------------------------------------------------
