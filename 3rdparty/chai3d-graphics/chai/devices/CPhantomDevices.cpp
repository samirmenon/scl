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
    \author    Federico Barbagli
    \author    Francois Conti
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "devices/CPhantomDevices.h"
//---------------------------------------------------------------------------
#if defined(_ENABLE_PHANTOM_DEVICE_SUPPORT)
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------
// WIN32
//---------------------------------------------------------------------------
#if defined(_WIN32)
HINSTANCE hdPhantomDLL = NULL;
HINSTANCE hdPhantomDriverDLL = NULL;

int (__stdcall *hdPhantomGetNumDevices)  ();

int (__stdcall *hdPhantomOpen)           (int a_deviceID);

int (__stdcall *hdPhantomClose)          (int a_deviceID);

int (__stdcall *hdPhantomGetPosition)    (int a_deviceID,
                                          double *a_posX,
                                          double *a_posY,
                                          double *a_posZ);

int (__stdcall *hdPhantomGetLinearVelocity)(int a_deviceID,
                                            double *a_velX,
                                            double *a_velY,
                                            double *a_velZ);

int (__stdcall *hdPhantomGetRotation)    (int a_deviceID,
                                          double *a_rot00,
                                          double *a_rot01,
                                          double *a_rot02,
                                          double *a_rot10,
                                          double *a_rot11,
                                          double *a_rot12,
                                          double *a_rot20,
                                          double *a_rot21,
                                          double *a_rot22);

int (__stdcall *hdPhantomGetButtons)     (int a_deviceID);

int (__stdcall *hdPhantomSetForce)       (int a_deviceID,
                                          double *a_forceX,
			                              double *a_forceY,
			                              double *a_forceZ);

int (__stdcall *hdPhantomSetTorque)      (int a_deviceID,
                                          double *a_torqueX,
			                              double *a_torqueY,
			                              double *a_torqueZ);

int (__stdcall *hdPhantomGetWorkspaceRadius)(int a_deviceID,
							                 double *a_workspaceRadius);

int (__stdcall *hdPhantomGetType)(int a_deviceID,
                                  char* a_typeName);


// Initialize dhd dll reference count
int cPhantomDevice::m_dllcount = 0;
#endif

//---------------------------------------------------------------------------
// LINUX
//---------------------------------------------------------------------------
#if defined(_LINUX)

// include files
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

// maximum number of Phantom devices
#define PHANTOM_NUM_DEVICES_MAX 2

// stucture used to store data related to each device entity
struct CPhantomDeviceStatus
{
	HHD handle;
	double position[3];
	double linearVelocity[3];
	double rotation[9];
	double force[3];
	double torque[3];
	int    button;
	bool   enabled;
	bool   initialized;
	double workspaceRadius;
};

// has lib been initialized
bool initPhantomLIB = false;

// table containing information for each device.
CPhantomDeviceStatus phantomDevices[PHANTOM_NUM_DEVICES_MAX];

// predefined value that expresses the absence of a Phantom.
int numPhantomDevices = 0;

// has servo controller been started yet
bool servoStarted = false;

// main servo controller callback
HDCallbackCode servoCallbackHandle;

// initialize phantom library
void init();

// initialize servo controller
void initServo();

// functions that communicate data with the Phantom devices
int hdPhantomGetNumDevices(); 

int hdPhantomOpen(int a_deviceID);

int hdPhantomClose(int a_deviceID);

int hdPhantomGetPosition(int a_deviceID, 
												 double *a_posX,
												 double *a_posY,
												 double *a_posZ);

int hdPhantomGetLinearVelocity(int a_deviceID, 
															 double *a_velX,
		    											 double *a_velY,
															 double *a_velZ);

int hdPhantomGetRotation(int a_deviceID, 
												 double *a_rot00,
												 double *a_rot01,
												 double *a_rot02,
												 double *a_rot10,
												 double *a_rot11,
												 double *a_rot12,
												 double *a_rot20,
												 double *a_rot21,
												 double *a_rot22);

int hdPhantomGetButtons(int a_deviceID);

int hdPhantomSetForce(int a_deviceID, 
											double *a_forceX,
											double *a_forceY,
											double *a_forceZ);

int hdPhantomSetTorque(int a_deviceID, 
											 double *a_torqueX,
											 double *a_torqueY,
											 double *a_torqueZ);

int hdPhantomGetWorkspaceRadius(int a_deviceID, double *a_workspaceRadius);

int hdPhantomGetType(int a_deviceID, 
										 char* a_typeName)

HDCallbackCode HDCALLBACK servophantomDevices(void* pUserData);

#endif

#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cPhantomDevice.
    No servo loop is yet created, encoders are NOT reset.

    \fn     cPhantomDevice::cPhantomDevice(unsigned int a_deviceNumber)
    \param  a_deviceNumber  Index number to the ith Phantom device
*/
//===========================================================================
cPhantomDevice::cPhantomDevice(unsigned int a_deviceNumber)
{
    // default specification setup
    m_specifications.m_manufacturerName              = "Sensable Technologies";
    m_specifications.m_modelName                     = "PHANTOM";
    m_specifications.m_maxForce                      = 6.0;     // [N]
    m_specifications.m_maxForceStiffness             = 1000.0;  // [N/m]
    m_specifications.m_maxTorque                     = 0.0;     // [N*m]
    m_specifications.m_maxTorqueStiffness            = 0.0;     // [N*m/Rad]
    m_specifications.m_maxGripperTorque              = 0.0;     // [N]
    m_specifications.m_maxGripperTorqueStiffness     = 0.0;     // [N*m/m]
    m_specifications.m_maxLinearDamping              = 8.0;    // [N/(m/s)]
    m_specifications.m_workspaceRadius               = 0.10;    // [m];
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = true;
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

    // check if Phantom drivers installed
#if defined(_WIN32)
    if (m_dllcount == 0)
    {
        hdPhantomDriverDLL = LoadLibrary("HD.dll");
        if (hdPhantomDriverDLL == NULL) { return; }
    }

    // the Phantom drivers are installed
    m_driverInstalled = true;

    // load dll library
    if (m_dllcount == 0)
    {
        hdPhantomDLL = LoadLibrary("hdPhantom.dll");
    }

    // check if DLL loaded correctly
    if (hdPhantomDLL == NULL)
    {
        return;
    }

    // load different callbacks
    hdPhantomGetNumDevices = (int (__stdcall*)(void))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetNumDevices");

    hdPhantomOpen         = (int (__stdcall*)(int))
                            GetProcAddress(hdPhantomDLL, "hdPhantomOpen");

    hdPhantomClose        = (int (__stdcall*)(int))
                            GetProcAddress(hdPhantomDLL, "hdPhantomClose");

    hdPhantomGetPosition  = (int (__stdcall*)(int,
                                              double*, double*, double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetPosition");

    hdPhantomGetLinearVelocity  = (int (__stdcall*)(int,
                                                    double*, double*, double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetLinearVelocity");

    hdPhantomGetRotation  = (int (__stdcall*)(int,
                                              double*, double*, double*,
                                              double*, double*, double*,
                                              double*, double*, double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetRotation");

    hdPhantomGetButtons   = (int (__stdcall*)(int))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetButtons");

    hdPhantomSetForce     = (int (__stdcall*)(int,
                                              double*,
                                              double*,
                                              double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomSetForce");
    hdPhantomSetTorque    = (int (__stdcall*)(int,
                                              double*,
                                              double*,
                                              double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomSetTorque");
    hdPhantomGetWorkspaceRadius = (int (__stdcall*)(int,
                                                    double*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetWorkspaceRadius");

    hdPhantomGetType      = (int (__stdcall*)(int,
                                              char*))
                            GetProcAddress(hdPhantomDLL, "hdPhantomGetType");
#endif

#if defined(_LINUX)
    // initialize open haptics
    void init()

    // the Phantom drivers are installed
    m_driverInstalled = true;
#endif

    // get the number ID of the device we wish to communicate with
    m_deviceID = a_deviceNumber;

    // get the number of Force Dimension devices connected to this computer
    int numDevices = hdPhantomGetNumDevices();

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

    // read information related to the device
    hdPhantomGetWorkspaceRadius(m_deviceID, &m_specifications.m_workspaceRadius);

    // read the device model
    char name[255];
    hdPhantomGetType(m_deviceID, &name[0]);
    m_specifications.m_modelName = name;

    /////////////////////////////////////////////////////////////////////
    // Define specifications given the device model
    /////////////////////////////////////////////////////////////////////

    if (m_specifications.m_modelName == "PHANTOM Omni")
    {
        m_specifications.m_maxForce                      = 4.0;     // [N]
        m_specifications.m_maxForceStiffness             = 700.0;   // [N/m]
        m_specifications.m_maxLinearDamping              = 5.0;     // [N/(m/s)]
    }

    // increment counter
    m_dllcount++;
}


//===========================================================================
/*!
    Destructor of cPhantomDevice.

    \fn     cPhantomDevice::~cPhantomDevice()
*/
//===========================================================================
cPhantomDevice::~cPhantomDevice()
{
    // close connection to device
    if (m_systemReady)
    {
        close();
    }

    m_dllcount--;

#if defined(_WIN32)
    if ((m_dllcount == 0) && (hdPhantomDLL != NULL))
    {
        FreeLibrary(hdPhantomDLL);
        hdPhantomDLL = NULL;
    }
#endif
}


//===========================================================================
/*!
    Open connection to phantom device.

    \fn     int cPhantomDevice::open()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cPhantomDevice::open()
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // check if the system is available
    if (!m_systemAvailable) return (-1);

    // if system is already opened then return
    if (m_systemReady) return (0);

    // try to open the device
    hdPhantomOpen(m_deviceID);

    // update device status
    m_systemReady = true;

    // success
    return (0);
}


//===========================================================================
/*!
    Close connection to phantom device.

    \fn     int cPhantomDevice::close()
    \return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cPhantomDevice::close()
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // check if the system has been opened previously
    if (!m_systemReady) return (-1);

    // yes, the device is open so let's close it
    int result = hdPhantomClose(m_deviceID);

    // update status
    m_systemReady = false;

    // exit
    return (result);
}


//===========================================================================
/*!
    Initialize the phantom device.
    
    For desktops and omnis, the a_resetEncoders parameter is ignored.
    For premiums, if you specify a_resetEncoders as true, you should
    be holding the Phantom in its rest position when this is called.
    
    \fn     int cPhantomDevice::initialize(const bool a_resetEncoders=false)
    \param  a_resetEncoders Should I re-zero the encoders?  (affects premiums only...)
    \return Return 0 if operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cPhantomDevice::initialize(const bool a_resetEncoders)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // exit
    return 0;
}


//===========================================================================
/*!
    Returns the number of devices available from this class of device.

    \fn      unsigned int cPhantomDevice::getNumDevices()
    \return  Returns the result
*/
//===========================================================================
unsigned int cPhantomDevice::getNumDevices()
{
    // check if drivers are installed
    if (!m_driverInstalled) return (0);

    // read number of devices
    int numDevices = hdPhantomGetNumDevices();
    return (numDevices);
}


//===========================================================================
/*!
    Read the position of the device. Units are meters [m].

    \fn     int cPhantomDevice::getPosition(cVector3d& a_position)
    \param  a_position  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::getPosition(cVector3d& a_position)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    double x,y,z;
    int error = hdPhantomGetPosition(m_deviceID, &x, &y, &z);
    a_position.set(x, y, z);
    estimateLinearVelocity(a_position);
    return (error);
}


//===========================================================================
/*!
    Read the linear velocity of the device. Units are in [m/s].

    \fn     int cPhantomDevice::getLinearVelocity(cVector3d& a_linearVelocity)
    \param  a_linearVelocity  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::getLinearVelocity(cVector3d& a_linearVelocity)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    int error = -1;

    /*
    int error = -1;
    double vx,vy,vz;
    error = hdPhantomGetLinearVelocity(m_deviceID, &vx, &vy, &vz);

    m_linearVelocity.set(vx, vy, vz);
    a_linearVelocity = m_linearVelocity;
    */

    a_linearVelocity = m_linearVelocity;

    return (error);
}


//===========================================================================
/*!
    Read the orientation frame of the device end-effector

    \fn     int cPhantomDevice::getRotation(cMatrix3d& a_rotation)
    \param  a_rotation  Return value.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    double rot[3][3];
    int error = hdPhantomGetRotation(m_deviceID,
                                     &rot[0][0],
                                     &rot[0][1],
                                     &rot[0][2],
                                     &rot[1][0],
                                     &rot[1][1],
                                     &rot[1][2],
                                     &rot[2][0],
                                     &rot[2][1],
                                     &rot[2][2]);
    a_rotation.set(rot[0][0], rot[0][1], rot[0][2],
                   rot[1][0], rot[1][1], rot[1][2],
                   rot[2][0], rot[2][1], rot[2][2]);
                   
    estimateAngularVelocity(a_rotation);
    return (error);
}


//===========================================================================
/*!
    Send a force [N] to the haptic device

    \fn     int cPhantomDevice::setForce(cVector3d& a_force)
    \param  a_force  Force command to be applied to device.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::setForce(cVector3d& a_force)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    int error = hdPhantomSetForce(m_deviceID, &a_force.x, &a_force.y, &a_force.z);
    m_prevForce = a_force;
    return (error);
}


//===========================================================================
/*!
    Send a torque [N*m] to the haptic device

    \fn     int cPhantomDevice::setTorque(cVector3d& a_torque)
    \param  a_torque Force command to be applied to device.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::setTorque(cVector3d& a_torque)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    int error = hdPhantomSetTorque(m_deviceID, &a_torque.x, &a_torque.y, &a_torque.z);
    m_prevTorque = a_torque;
    return (error);
}


//===========================================================================
/*!
    Read the status of the user switch [\b true = \b ON / \b false = \b OFF].

    \fn     int cPhantomDevice::getUserSwitch(int a_switchIndex, bool& a_status)
    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);
    
    int result = 0;
    int button = hdPhantomGetButtons(m_deviceID);

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

//==========================================================================
// LINUX SUPPORT
//==========================================================================
#if defined(_LINUX)

//==========================================================================
/*!
  Initialize OpenHaptics
*/
//==========================================================================
void init()
{
	if (!initPhantomLIB)
	{
		initPhantomLIB = true;

		for (int i=0; i<PHANTOM_NUM_DEVICES_MAX; i++)
		{
			// init button data
			phantomDevices[i].button = 0;

			// init position data
			phantomDevices[i].position[0] = 0.0;
			phantomDevices[i].position[1] = 0.0;
			phantomDevices[i].position[2] = 0.0;

			// init rotation data
			phantomDevices[i].rotation[0] = 1.0;
			phantomDevices[i].rotation[1] = 0.0;
			phantomDevices[i].rotation[2] = 0.0;
			phantomDevices[i].rotation[3] = 0.0;
			phantomDevices[i].rotation[4] = 1.0;
			phantomDevices[i].rotation[5] = 0.0;
			phantomDevices[i].rotation[6] = 0.0;
			phantomDevices[i].rotation[7] = 0.0;
			phantomDevices[i].rotation[8] = 1.0;
			
			// init force data
			phantomDevices[i].force[0] = 0.0;
			phantomDevices[i].force[1] = 0.0;
			phantomDevices[i].force[2] = 0.0;
			
			// init torque data
			phantomDevices[i].torque[0] = 0.0;
			phantomDevices[i].torque[1] = 0.0;
			phantomDevices[i].torque[2] = 0.0;

			// init enable/disable data
			phantomDevices[i].enabled = false;

			// init phantom api initialized
			phantomDevices[i].initialized = false;
		}

		//------------------------------------------------------------------
		// INITIALIZE DEVICES
		//------------------------------------------------------------------	
		HDErrorInfo error;
		numPhantomDevices = 0;

		// search for a first device
		HHD hHD0 = hdInitDevice(HD_DEFAULT_DEVICE);

		// check if device is available
		if (HD_DEVICE_ERROR(error = hdGetError())) 
		{
			return(true);
		}

		// enable forces
		hdMakeCurrentDevice(hHD0);
		hdEnable(HD_FORCE_OUTPUT);

		// add device to list
		phantomDevices[0].handle = hHD0;
		phantomDevices[0].enabled = true;
		numPhantomDevices++;

		// search for a possible second device
		HHD hHD1 = hdInitDevice("Phantom2");

		// check if device is available
		if (HD_DEVICE_ERROR(error = hdGetError())) 
		{
			return(true);
		}

		// enable forces
		hdMakeCurrentDevice(hHD1);
		hdEnable(HD_FORCE_OUTPUT);

		// add device to list
		phantomDevices[1].handle = hHD0;
		phantomDevices[1].enabled = true;
		numPhantomDevices++;
	}
	return (true);
}


//==========================================================================
/*!
  Retrieves the number of devices of type phantom.
*/
//==========================================================================
int hdPhantomGetNumDevices() 
{
	return (numPhantomDevices);
}


//==========================================================================
/*!
  Open a connection to the device selected by a_deviceID.
*/
//==========================================================================
int hdPhantomOpen(int a_deviceID)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// init device
	if (!phantomDevices[a_deviceID].initialized)
	{
		phantomDevices[a_deviceID].initialized = true;
	}

	// enable device
	phantomDevices[a_deviceID].enabled = true;

	// return result
	return (phantomDevices[a_deviceID].handle);
}


//==========================================================================
/*!
  Closes a connection to the device selected by a_deviceID.
*/
//==========================================================================
int hdPhantomClose(int a_deviceID)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// disable device
	phantomDevices[a_deviceID].enabled = false;
}


//==========================================================================
/*!
  Returns the position of the device end-effector.
*/
//==========================================================================
int hdPhantomGetPosition(int a_deviceID, 
												double *a_posX,
												double *a_posY,
												double *a_posZ)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { initServo(); }

	// check if enabled
	if (!phantomDevices[a_deviceID].enabled) { return (-1); }

	// get position
	*a_posX = phantomDevices[a_deviceID].position[2];
	*a_posY = phantomDevices[a_deviceID].position[0];
	*a_posZ = phantomDevices[a_deviceID].position[1];

	// success
	return (0);
}


//==========================================================================
/*!
  Returns the linear velocity of the device end-effector.
*/
//==========================================================================
int hdPhantomGetLinearVelocity(int a_deviceID, 
															double *a_velX,
		    											double *a_velY,
															double *a_velZ)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { initServo(); }

	// check if enabled
	if (!phantomDevices[a_deviceID].enabled) { return (-1); }

	// get position
	*a_velX = phantomDevices[a_deviceID].linearVelocity[2];
	*a_velY = phantomDevices[a_deviceID].linearVelocity[0];
	*a_velZ = phantomDevices[a_deviceID].linearVelocity[1];

	// success
	return (0);
}


//==========================================================================
/*!
  Returns the orientation matrix (frame) of the device end-effector.
*/
//==========================================================================
int hdPhantomGetRotation(int a_deviceID, 
						double *a_rot00,
						double *a_rot01,
						double *a_rot02,
						double *a_rot10,
						double *a_rot11,
						double *a_rot12,
						double *a_rot20,
						double *a_rot21,
						double *a_rot22)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { initServo(); }

	// check if enabled
	if (!phantomDevices[a_deviceID].enabled) { return (-1); }

	// return rotation matrix
	/*
	*a_rot00 = phantomDevices[a_deviceID].rotation[8];	
	*a_rot01 = phantomDevices[a_deviceID].rotation[2];	
	*a_rot02 = phantomDevices[a_deviceID].rotation[5];	
	*a_rot10 = phantomDevices[a_deviceID].rotation[6];	
	*a_rot11 = phantomDevices[a_deviceID].rotation[0];	
	*a_rot12 = phantomDevices[a_deviceID].rotation[3];	
	*a_rot20 = phantomDevices[a_deviceID].rotation[7];	
	*a_rot21 = phantomDevices[a_deviceID].rotation[1];	
	*a_rot22 = phantomDevices[a_deviceID].rotation[4];	
	*/

	// read value from matrix and correct matrix to be orthogonal
	// unfortunately there seems be some precision errors coming
	// from the OpenHaptics library.
	cVector3d v0, v1, v2;
	v0.set( phantomDevices[a_deviceID].rotation[8],
			phantomDevices[a_deviceID].rotation[6],
			phantomDevices[a_deviceID].rotation[7]);
	v1.set( phantomDevices[a_deviceID].rotation[2],
			phantomDevices[a_deviceID].rotation[0],
			phantomDevices[a_deviceID].rotation[1]);
	v0.normalize();
	v1.normalize();
	v0.crossr(v1, v2);
	v2.crossr(v0, v1);

	*a_rot00 = v0.x;	
	*a_rot01 = v1.x; 
	*a_rot02 = v2.x; 
	*a_rot10 = v0.y; 
	*a_rot11 = v1.y;
	*a_rot12 = v2.y;
	*a_rot20 = v0.z; 
	*a_rot21 = v1.z; 
	*a_rot22 = v2.z;

	// success
	return (0);
}


//==========================================================================
/*!
  Read the values of each end-effector user button.
*/
//==========================================================================
int hdPhantomGetButtons(int a_deviceID)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { initServo(); }

	// check if enabled
	if (!phantomDevices[a_deviceID].enabled) { return (-1); }

	// return value
	return (phantomDevices[a_deviceID].button);
}


//==========================================================================
/*!
	Send a force to the device
*/
//==========================================================================
int hdPhantomSetForce(int a_deviceID, 
											double *a_forceX,
											double *a_forceY,
											double *a_forceZ)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { initServo(); }

	// check if enabled
	if (!phantomDevices[a_deviceID].enabled) { return (-1); }

	// set force
	phantomDevices[a_deviceID].force[2] = *a_forceX;
	phantomDevices[a_deviceID].force[0] = *a_forceY;
	phantomDevices[a_deviceID].force[1] = *a_forceZ;

	// success
	return (0);
}


//==========================================================================
/*!
	Send a torque to the device
*/
//==========================================================================
int hdPhantomSetTorque(int a_deviceID, 
												double *a_torqueX,
												double *a_torqueY,
												double *a_torqueZ)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { initServo(); }

	// check if enabled
	if (!phantomDevices[a_deviceID].enabled) { return (-1); }

	// set torque
	phantomDevices[a_deviceID].torque[2] = *a_torqueX;
	phantomDevices[a_deviceID].torque[0] = *a_torqueY;
	phantomDevices[a_deviceID].torque[1] = *a_torqueZ;

	// success
	return (0);
}


//==========================================================================
/*!
	Send a force to the device
*/
//==========================================================================
int hdPhantomGetWorkspaceRadius(int a_deviceID, double *a_workspaceRadius)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { initServo(); }

	// check if enabled
	if (!phantomDevices[a_deviceID].enabled) { return (-1); }

	// retrieve handle
	HHD hHD = phantomDevices[a_deviceID].handle;

	// activate ith device
	hdMakeCurrentDevice(hHD);

	// read workspace of device
	double size[6];
	hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, size);
	double sizeX = size[3] - size[0];
	double sizeY = size[4] - size[1];
	double sizeZ = size[5] - size[2];
	double radius = 0.5 * sqrt(sizeX * sizeX +
				     		   sizeY * sizeY +
					           sizeZ * sizeZ);

	// convert value to [m]
	phantomDevices[a_deviceID].workspaceRadius = 0.001 * radius;

	// return estimated workspace radius
	*a_workspaceRadius = phantomDevices[a_deviceID].workspaceRadius;

	// success
	return (0);
}


//==========================================================================
/*!
	Read the name type of the device
*/
//==========================================================================
int hdPhantomGetType(int a_deviceID, 
										 char* a_typeName)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { initServo(); }

	// check if enabled
	if (!phantomDevices[a_deviceID].enabled) { return (-1); }

	// retrieve handle
	HHD hHD = phantomDevices[a_deviceID].handle;

	// activate ith device
	hdMakeCurrentDevice(hHD);

	// read device model
	const char* typeName = hdGetString(HD_DEVICE_MODEL_TYPE);
	strcpy(a_typeName, typeName);
}


//==========================================================================
/*
	servo controller callback

	\fn     HDLServoOpExitCode servophantomDevices(void* pUserData)
	\param  pUserData pointer to user data information (not used here)
*/
//==========================================================================
HDCallbackCode HDCALLBACK servophantomDevices(void* pUserData)
{
	for (int i=0; i<PHANTOM_NUM_DEVICES_MAX; i++)
	{
		// for each activated phantom device
		if (phantomDevices[i].enabled)
		{
			// retrieve handle
			HHD hHD = phantomDevices[i].handle;

			// activate ith device
			hdMakeCurrentDevice(hHD);

			// start sending commands
			hdBeginFrame(hHD);
			
			// retrieve the position and orientation of the end-effector.
			double frame[16];
			hdGetDoublev(HD_CURRENT_TRANSFORM, frame);

			// convert position from [mm] to [m] 
			frame[12] = frame[12] * 0.001;
			frame[13] = frame[13] * 0.001;
			frame[14] = frame[14] * 0.001;

			phantomDevices[i].position[0] = frame[12];
			phantomDevices[i].position[1] = frame[13];
			phantomDevices[i].position[2] = frame[14];

			phantomDevices[i].rotation[0] = frame[0];
			phantomDevices[i].rotation[1] = frame[1];
			phantomDevices[i].rotation[2] = frame[2];
			phantomDevices[i].rotation[3] = frame[4];
			phantomDevices[i].rotation[4] = frame[5];
			phantomDevices[i].rotation[5] = frame[6];
			phantomDevices[i].rotation[6] = frame[8];
			phantomDevices[i].rotation[7] = frame[9];
			phantomDevices[i].rotation[8] = frame[10];

			// read linear velocity
			double vel[3];
			hdGetDoublev(HD_CURRENT_VELOCITY, vel);

			// convert position from [mm] to [m] 
			vel[0] = vel[0] * 0.001;
			vel[1] = vel[1] * 0.001;
			vel[2] = vel[2] * 0.001;
			
			phantomDevices[i].linearVelocity[0] = vel[0];
			phantomDevices[i].linearVelocity[1] = vel[1];
			phantomDevices[i].linearVelocity[2] = vel[2];

			// read user buttons
			int buttons;
			hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
			phantomDevices[i].button = buttons;

			// send force to end-effector
			double force[3];
			force[0] = phantomDevices[i].force[0];
			force[1] = phantomDevices[i].force[1];
			force[2] = phantomDevices[i].force[2];
			hdSetDoublev(HD_CURRENT_FORCE, force);

			// send torque to end-effector
			double torque[3];
			torque[0] = phantomDevices[i].torque[0];
			torque[1] = phantomDevices[i].torque[1];
			torque[2] = phantomDevices[i].torque[2];
			hdSetDoublev(HD_CURRENT_TORQUE, torque);

			// flush commands
			hdEndFrame(hHD);
		}
	}

	return (HD_CALLBACK_CONTINUE);
}


//==========================================================================
/*
	initialize servo controller

	\fn     void initServo()
*/
//==========================================================================
void initServo()
{
	HDErrorInfo error;
	if (!servoStarted)
	{
		// servo controller has been started
		servoStarted = true;

		// create callback
		HDCallbackCode servoCallbackHandle = hdScheduleAsynchronous(
		servophantomDevices, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);

		// start scheduler
		hdStartScheduler();
	}
}

//---------------------------------------------------------------------------
#endif // _LINUX
//---------------------------------------------------------------------------
#endif // _ENABLE_PHANTOM_DEVICE_SUPPORT
//---------------------------------------------------------------------------

