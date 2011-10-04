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
    \version   2.0.0 $Rev: 266 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "devices/CFalconDevice.h"
//---------------------------------------------------------------------------
#if defined(_ENABLE_FALCON_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

#ifndef DLLVERSIONINFO
typedef struct _DllVersionInfo
{
    DWORD cbSize;
    DWORD dwMajorVersion;
    DWORD dwMinorVersion;
    DWORD dwBuildNumber;
    DWORD dwPlatformID;
}DLLVERSIONINFO;
#endif

#ifndef DLLGETVERSIONPROC
typedef int (FAR WINAPI *DLLGETVERSIONPROC) (DLLVERSIONINFO *);
#endif
//---------------------------------------------------------------------------

HINSTANCE hdFalconDLL = NULL;
HINSTANCE hdFalconDriverDLL = NULL;

int (__stdcall *hdFalconGetNumDevices)  ();
int (__stdcall *hdFalconOpen)           (int a_deviceID);
int (__stdcall *hdFalconClose)          (int a_deviceID);
int (__stdcall *hdFalconGetPosition)    (int a_deviceID,
					 double *a_posX,
                     double *a_posY,
					 double *a_posZ);
int (__stdcall *hdFalconGetRotation)   (int a_deviceID,
                                        double *a_rot00,
                                        double *a_rot01,
                                        double *a_rot02,
                                        double *a_rot10,
                                        double *a_rot11,
                                        double *a_rot12,
                                        double *a_rot20,
                                        double *a_rot21,
                                        double *a_rot22);

int (__stdcall *hdFalconGetButtons)    (int a_deviceID);

int (__stdcall *hdFalconSetForce)      (int a_deviceID,
                                        double *a_forceX,
                                        double *a_forceY,
                                        double *a_forceZ);


// Initialize dhd dll reference count
int cFalconDevice::m_dllcount = 0;

//---------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cFalconDevice.

    \fn     cFalconDevice::cFalconDevice(unsigned int a_deviceNumber)
*/
//===========================================================================
cFalconDevice::cFalconDevice(unsigned int a_deviceNumber)
{
    // set specifications
    m_specifications.m_manufacturerName              = "Novint Technologies";
    m_specifications.m_modelName                     = "Falcon";
    m_specifications.m_maxForce                      = 8.0;     // [N]
    m_specifications.m_maxForceStiffness             = 3000.0;  // [N/m]
    m_specifications.m_maxTorque                     = 0.0;     // [N*m]
    m_specifications.m_maxTorqueStiffness            = 0.0;     // [N*m/Rad]
    m_specifications.m_maxGripperTorque              = 0.0;     // [N]
    m_specifications.m_maxLinearDamping              = 20.0;    // [N/(m/s)]
    m_specifications.m_maxGripperTorqueStiffness     = 0.0;     // [N*m/m]
    m_specifications.m_workspaceRadius               = 0.04;    // [m]
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = false;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = true;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;

    // device is not yet available or ready
    m_driverInstalled = false;
    m_systemAvailable = false;
    m_systemReady = false;

    // check if Falcon drivers installed
    if (m_dllcount == 0)
    {
        // load Falcon dll
        hdFalconDriverDLL = LoadLibrary("hdl.dll");

        // check if file exists
        if (hdFalconDriverDLL == NULL) { return; }

        // check if multi device is supported
        FARPROC testFunction = NULL;
        testFunction = GetProcAddress(hdFalconDriverDLL, "_hdlCountDevices@0");
        if (testFunction == NULL)
        {
            // failed, old version is installed.
            printf("Error - Please update the drivers of your Novint Falcon haptic device.\n");
            return;
        }
    }

    // the Falcon drivers are installed
    m_driverInstalled = true;

    // load dll library
    if (m_dllcount == 0)
    {
        hdFalconDLL = LoadLibrary("hdFalcon.dll");
    }

    // check if DLL loaded correctly
    if (hdFalconDLL == NULL)
    {
        return;
    }


    // load different callbacks
    hdFalconGetNumDevices = (int (__stdcall*)(void))
                            GetProcAddress(hdFalconDLL, "hdFalconGetNumDevices");

    hdFalconOpen         = (int (__stdcall*)(int))
                            GetProcAddress(hdFalconDLL, "hdFalconOpen");

    hdFalconClose        = (int (__stdcall*)(int))
                            GetProcAddress(hdFalconDLL, "hdFalconClose");

    hdFalconGetPosition  = (int (__stdcall*)(int,
                                             double*, double*, double*))
                            GetProcAddress(hdFalconDLL, "hdFalconGetPosition");

    hdFalconGetRotation  = (int (__stdcall*)(int,
                                             double*, double*, double*,
                                             double*, double*, double*,
                                             double*, double*, double*))
                            GetProcAddress(hdFalconDLL, "hdFalconGetRotation");

    hdFalconGetButtons        = (int (__stdcall*)(int))
                            GetProcAddress(hdFalconDLL, "hdFalconGetButtons");

    hdFalconSetForce     = (int (__stdcall*)(int,
                                             double*,
                                             double*,
                                             double*))
                            GetProcAddress(hdFalconDLL, "hdFalconSetForce");


    // get the number ID of the device we wish to communicate with
    m_deviceID = a_deviceNumber;

    // get the number of Force Dimension devices connected to this computer
    int numDevices = hdFalconGetNumDevices();

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
    }

    // increment counter
    m_dllcount++;
}


//===========================================================================
/*!
    Destructor of cFalconDevice.

    \fn         cFalconDevice::~cFalconDevice()
*/
//===========================================================================
cFalconDevice::~cFalconDevice()
{
    // close connection to device
    if (m_systemReady)
    {
        close();
    }

    m_dllcount--;

    if ((m_dllcount == 0) && (hdFalconDLL != NULL))
    {
        FreeLibrary(hdFalconDLL);
        hdFalconDLL = NULL;
    }
}


//===========================================================================
/*!
    Open connection to Falcon haptic device.

    \fn     int cFalconDevice::open()
*/
//===========================================================================
int cFalconDevice::open()
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // check if the system is available
    if (!m_systemAvailable) return (-1);

    // if system is already opened then return
    if (m_systemReady) return (0);

    // try to open the device
    hdFalconOpen(m_deviceID);

    // update device status
    m_systemReady = true;

    // success
    return (0);
}


//===========================================================================
/*!
    Close connection to Falcon haptic device.

    \fn     int cFalconDevice::close()
*/
//===========================================================================
int cFalconDevice::close()
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // check if the system has been opened previously
    if (!m_systemReady) return (-1);

    // yes, the device is open so let's close it
    int result = hdFalconClose(m_deviceID);

    // update status
    m_systemReady = false;

    // exit
    return (result);
}


//===========================================================================
/*!
    Calibrate Falcon haptic device.

    This function does nothing right now; the a_resetEncoders parameter is ignored.

    \fn     int cFalconDevice::initialize(const bool a_resetEncoders = false)
    \param  a_resetEncoders Ignored; exists for forward compatibility.
    \return Always 0
*/
//===========================================================================
int cFalconDevice::initialize(const bool a_resetEncoders)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // reset encoders
    return (0);
}


//===========================================================================
/*!
    Returns the number of devices available from this class of device.

    \fn     unsigned int cFalconDevice::getNumDevices();
    \return  Returns the result
*/
//===========================================================================
unsigned int cFalconDevice::getNumDevices()
{
    // check if drivers are installed
    if (!m_driverInstalled) return (0);

    int numDevices = hdFalconGetNumDevices();
    return (numDevices);
}


//===========================================================================
/*!
    Read the position of the device. Units are meters [m].

    \fn     int cFalconDevice::getPosition(cVector3d& a_position)
    \param  a_position  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFalconDevice::getPosition(cVector3d& a_position)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    double x,y,z;
    int error = hdFalconGetPosition(m_deviceID, &x, &y, &z);

    // add a small offset for zero centering
    x = x + 0.01;
    a_position.set(x, y, z);
    estimateLinearVelocity(a_position);
    return (error);
}


//===========================================================================
/*!
    Send a force [N] to the Falcon haptic device.

    \fn     int cFalconDevice::setForce(cVector3d& a_force)
    \param  a_force  Force command to be applied to device.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFalconDevice::setForce(cVector3d& a_force)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    int error = hdFalconSetForce(m_deviceID, &a_force.x, &a_force.y, &a_force.z);
    m_prevForce = a_force;
    return (error);
}


//===========================================================================
/*!
    Read the status of the user switch [1 = \e ON / 0 = \e OFF].

    \fn     int cFalconDevice::getUserSwitch(int a_switchIndex, bool& a_status)
    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cFalconDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);
    
    bool result = false;
    int button = hdFalconGetButtons(m_deviceID);

    switch (a_switchIndex)
    {
        case 0:
			if (button & 1) { result = true; }
            break;

        case 1:
            if (button & 2) { result = true; }
            break;

        case 2:
            if (button & 3) { result = true; }
            break;

        case 3:
            if (button & 4) { result = true; }
            break;
    }

	// return result
	a_status = result;

    return (0);
}

//---------------------------------------------------------------------------
#endif //_ENABLE_FALCON_DEVICE_SUPPORT
//---------------------------------------------------------------------------
