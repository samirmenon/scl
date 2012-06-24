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
	\author    Federico Barbagli
	\author    Francois Conti
	\version   $MAJOR.$MINOR.$RELEASE $Rev: 799 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CPhantomDevices.h"
//---------------------------------------------------------------------------
#if defined(C_ENABLE_PHANTOM_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------
// WIN32
//---------------------------------------------------------------------------
#if defined(WIN32)
HINSTANCE hdPhantomDLL = NULL;
HINSTANCE hdPhantomDriverDLL = NULL;

int (__stdcall *hdPhantomGetNumDevices)  ();

int (__stdcall *hdPhantomOpen)           (const int a_deviceID);

int (__stdcall *hdPhantomClose)          (const int a_deviceID);

int (__stdcall *hdPhantomGetPosition)    (const int a_deviceID,
										  double *a_posX,
										  double *a_posY,
										  double *a_posZ);

int (__stdcall *hdPhantomGetLinearVelocity)(const int a_deviceID,
											double *a_velX,
											double *a_velY,
											double *a_velZ);

int (__stdcall *hdPhantomGetRotation)    (const int a_deviceID,
										  double *a_rot00,
										  double *a_rot01,
										  double *a_rot02,
										  double *a_rot10,
										  double *a_rot11,
										  double *a_rot12,
										  double *a_rot20,
										  double *a_rot21,
										  double *a_rot22);

int (__stdcall *hdPhantomGetButtons)     (const int a_deviceID);

int (__stdcall *hdPhantomSetForce)       (const int a_deviceID,
										  const double *a_forceX,
										  const double *a_forceY,
										  const double *a_forceZ);

int (__stdcall *hdPhantomSetTorque)      (int a_deviceID,
										  const double *a_torqueX,
										  const double *a_torqueY,
										  const double *a_torqueZ);

int (__stdcall *hdPhantomSetForceAndTorque) (int a_deviceID,
											 const double *a_forceX,
											 const double *a_forceY,
											 const double *a_forceZ,
											 const double *a_torqueX,
											 const double *a_torqueY,
											 const double *a_torqueZ);

int (__stdcall *hdPhantomGetWorkspaceRadius)(const int a_deviceID,
											 double *a_workspaceRadius);

int (__stdcall *hdPhantomGetType)(int a_deviceID,
								  char* a_typeName);

// initialize servo controller
void (__stdcall *hdPhantomStartServo)();

// stop servo controller
void (__stdcall *hdPhantomStopServo)();


#endif

// Initialize dhd dll reference count
int cPhantomDevice::m_dllcount = 0;

//---------------------------------------------------------------------------
// LINUX
//---------------------------------------------------------------------------
#if defined(LINUX)

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

// initialize phantom library
void init();

// initialize servo controller
void hdPhantomStartServo();

// stop servo controller
void hdPhantomStopServo();

// has lib been initialized?
bool initPhantomLIB = false;

// table containing information for each device.
CPhantomDeviceStatus phantomDevices[PHANTOM_NUM_DEVICES_MAX];

// predefined value that expresses the absence of a Phantom.
int numPhantomDevices = 0;

// has servo controller been started yet
bool servoStarted = false;

// main servo controller callback
HDCallbackCode servoCallbackHandle;

// functions that communicate data with the Phantom devices
int hdPhantomGetNumDevices(); 

int hdPhantomOpen(const int a_deviceID);

int hdPhantomClose(const int a_deviceID);

int hdPhantomGetPosition(const int a_deviceID, 
						 double *a_posX,
						 double *a_posY,
						 double *a_posZ);

int hdPhantomGetLinearVelocity(const int a_deviceID, 
							   double *a_velX,
							   double *a_velY,
							   double *a_velZ);

int hdPhantomGetRotation(const int a_deviceID, 
						 double *a_rot00,
						 double *a_rot01,
						 double *a_rot02,
						 double *a_rot10,
						 double *a_rot11,
						 double *a_rot12,
						 double *a_rot20,
						 double *a_rot21,
						 double *a_rot22);

int hdPhantomGetButtons(const int a_deviceID);

int hdPhantomSetForce(const int a_deviceID, 
					  const double *a_forceX,
					  const double *a_forceY,
					  const double *a_forceZ);

int hdPhantomSetTorque(const int a_deviceID, 
					   const double *a_torqueX,
					   const double *a_torqueY,
					   const double *a_torqueZ);

int hdPhantomSetForceAndTorque) (const int a_deviceID,
								 const double *a_forceX,
								 const double *a_forceY,
								 const double *a_forceZ,
								 const double *a_torqueX,
								 const double *a_torqueY,
								 const double *a_torqueZ);

int hdPhantomGetWorkspaceRadius(const int a_deviceID, double *a_workspaceRadius);

int hdPhantomGetType(const int a_deviceID, 
					 char* a_typeName);

HDCallbackCode HDCALLBACK servoPhantomDevices(void* pUserData);

#endif


//---------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//===========================================================================
/*!
	Constructor of cPhantomDevice.
	No servo loop is yet created, encoders are NOT reset.

	\param  a_deviceNumber  Index number to the ith Phantom device
*/
//===========================================================================
cPhantomDevice::cPhantomDevice(unsigned int a_deviceNumber)
{
	// default specification setup
    m_specifications.m_model                         = C_HAPTIC_DEVICE_PHANTOM_OTHER;
	m_specifications.m_manufacturerName              = "Sensable Technologies";
	m_specifications.m_modelName                     = "PHANTOM";
	m_specifications.m_maxLinearForce                = 6.0;     // [N]
	m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]
	m_specifications.m_maxGripperForce               = 0.0;     // [N]
	m_specifications.m_maxLinearStiffness            = 1000.0;  // [N/m]
	m_specifications.m_maxAngularStiffness           = 0.0;     // [N*m/Rad]
	m_specifications.m_maxGripperLinearStiffness     = 1000.0;  // [N/m]
	m_specifications.m_maxLinearDamping              = 8.0;     // [N/(m/s)]
	m_specifications.m_maxAngularDamping			 = 0.0;     // [N*m/(Rad/s)]
	m_specifications.m_maxGripperAngularDamping	     = 0.0;     // [N*m/(Rad/s)]
	m_specifications.m_workspaceRadius               = 0.10;    // [m];
	m_specifications.m_sensedPosition                = true;
	m_specifications.m_sensedRotation                = true;
	m_specifications.m_sensedGripper                 = false;
	m_specifications.m_actuatedPosition              = true;
	m_specifications.m_actuatedRotation              = false;
	m_specifications.m_actuatedGripper               = false;
	m_specifications.m_leftHand                      = true;
	m_specifications.m_rightHand                     = true;
    m_specifications.m_positionOffset.set(0.0, 0.0, 0.0);

	// device is not yet available or ready
	m_driverInstalled = false;
	m_deviceAvailable = false;
	m_deviceReady = false;

	// check if Phantom drivers installed
#if defined(WIN32)
	if (m_dllcount == 0)
	{
		hdPhantomDriverDLL = LoadLibrary("HD.dll");
		if (hdPhantomDriverDLL == NULL) { return; }
	}

	// load dll library
	if (m_dllcount == 0)
	{
#if defined (WIN32)
		hdPhantomDLL = LoadLibrary("hdPhantom32.dll");
#endif;
#if defined (WIN64)
		hdPhantomDLL = LoadLibrary("hdPhantom64.dll");
#endif;
	}

	// check if DLL loaded correctly
	if (hdPhantomDLL == NULL)
	{
		return;
	}

	// load different callbacks
	hdPhantomGetNumDevices = (int (__stdcall*)(void))GetProcAddress(hdPhantomDLL, "hdPhantomGetNumDevices");
	if (!hdPhantomGetNumDevices) return;

	hdPhantomOpen = (int (__stdcall*)(const int))GetProcAddress(hdPhantomDLL, "hdPhantomOpen");
	if (!hdPhantomOpen) return;

	hdPhantomClose = (int (__stdcall*)(const int))GetProcAddress(hdPhantomDLL, "hdPhantomClose");
	if (!hdPhantomClose) return;

	hdPhantomGetPosition = (int (__stdcall*)(const int, double*, double*, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetPosition");
	if (!hdPhantomGetPosition) return;

	hdPhantomGetLinearVelocity = (int (__stdcall*)(const int, double*, double*, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetLinearVelocity");
	if (!hdPhantomGetLinearVelocity) return;

	hdPhantomGetRotation = (int (__stdcall*)(const int, double*, double*, double*, double*, double*, double*, double*, double*, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetRotation");
	if (!hdPhantomGetRotation) return;

	hdPhantomGetButtons = (int (__stdcall*)(const int))GetProcAddress(hdPhantomDLL, "hdPhantomGetButtons");
	if (!hdPhantomGetButtons) return;

	hdPhantomSetForce = (int (__stdcall*)(const int, const double*, const double*, const double*))GetProcAddress(hdPhantomDLL, "hdPhantomSetForce");
	if (!hdPhantomSetForce) return;

	hdPhantomSetTorque = (int (__stdcall*)(const int, const double*, const double*, const double*))GetProcAddress(hdPhantomDLL, "hdPhantomSetTorque");
	if (!hdPhantomSetTorque) return;

	hdPhantomSetForceAndTorque = (int (__stdcall*)(const int, const double*, const double*, const double*, const double*, const double*, const double*))GetProcAddress(hdPhantomDLL, "hdPhantomSetForceAndTorque");
	if (!hdPhantomSetForceAndTorque) return;

	hdPhantomGetWorkspaceRadius = (int (__stdcall*)(const int, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetWorkspaceRadius");
	if (!hdPhantomGetWorkspaceRadius) return;

	hdPhantomGetType = (int (__stdcall*)(const int, char*))GetProcAddress(hdPhantomDLL, "hdPhantomGetType");
	if (!hdPhantomGetType) return;

	hdPhantomStartServo = (void (__stdcall*)(void))GetProcAddress(hdPhantomDLL, "hdPhantomStartServo");
	if (!hdPhantomStartServo) return;

	hdPhantomStopServo = (void (__stdcall*)(void))GetProcAddress(hdPhantomDLL, "hdPhantomStopServo");
	if (!hdPhantomStopServo) return;

	// if we got to here, we can consider that the Phantom drivers are installed
	m_driverInstalled = true;
#endif

#if defined(LINUX)
	// initialize open haptics
	init();

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
		m_deviceAvailable = false;
	}
	else
	{
		// yes, this ID leads to an existing device
		m_deviceAvailable = true;
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
        m_specifications.m_model                         = C_HAPTIC_DEVICE_PHANTOM_OMNI;
		m_specifications.m_maxLinearForce                = 4.0;     // [N]
		m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]
		m_specifications.m_maxGripperForce               = 0.0;     // [N]
		m_specifications.m_maxLinearStiffness            = 700.0;   // [N/m]
		m_specifications.m_maxAngularStiffness           = 0.0;     // [N*m/Rad]
		m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N/m]
		m_specifications.m_maxLinearDamping              = 5.0;     // [N/(m/s)]
		m_specifications.m_maxAngularDamping			 = 0.0;     // [N*m/(Rad/s)]
		m_specifications.m_maxGripperAngularDamping	     = 0.0;     // [N*m/(Rad/s)]
		m_specifications.m_workspaceRadius               = 0.10;    // [m];
		m_specifications.m_sensedPosition                = true;
		m_specifications.m_sensedRotation                = true;
		m_specifications.m_sensedGripper                 = false;
		m_specifications.m_actuatedPosition              = true;
		m_specifications.m_actuatedRotation              = false;
		m_specifications.m_actuatedGripper               = false;
		m_specifications.m_leftHand                      = true;
		m_specifications.m_rightHand                     = true;
        m_specifications.m_positionOffset.set(0.0, 0.0, 0.0);
    }

	if (m_specifications.m_modelName == "PHANTOM Premium 1.5 6DOF")
    {
        m_specifications.m_model                         = C_HAPTIC_DEVICE_PHANTOM_15_6DOF;
		m_specifications.m_maxLinearForce                = 4.0;     // [N]
		m_specifications.m_maxAngularTorque              = 0.188;   // [N*m]
		m_specifications.m_maxGripperForce               = 0.0;     // [N]
		m_specifications.m_maxLinearStiffness            = 800.0;   // [N/m]
		m_specifications.m_maxAngularStiffness           = 0.3;     // [N*m/Rad]
		m_specifications.m_maxGripperLinearStiffness     = 0.0;     // [N/m]
		m_specifications.m_maxLinearDamping              = 5.0;     // [N/(m/s)]
		m_specifications.m_maxAngularDamping			 = 0.004;   // [N*m/(Rad/s)]
		m_specifications.m_maxGripperAngularDamping	     = 0.0;     // [N*m/(Rad/s)]
		m_specifications.m_workspaceRadius               = 0.10;    // [m];
		m_specifications.m_sensedPosition                = true;
		m_specifications.m_sensedRotation                = true;
		m_specifications.m_sensedGripper                 = false;
		m_specifications.m_actuatedPosition              = true;
		m_specifications.m_actuatedRotation              = true;
		m_specifications.m_actuatedGripper               = false;
		m_specifications.m_leftHand                      = true;
		m_specifications.m_rightHand                     = true;
        m_specifications.m_positionOffset.set(0.0, 0.0, -0.10);
    }
	// increment counter
	m_dllcount++;
}


//===========================================================================
/*!
	Destructor of cPhantomDevice.
*/
//===========================================================================
cPhantomDevice::~cPhantomDevice()
{
	// close connection to device
	if (m_deviceReady)
	{
		close();
	}

	m_dllcount--;

#if defined(WIN32)
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

	\return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cPhantomDevice::open()
{
	// check if drivers are installed
	if (!m_driverInstalled) return (-1);

	// check if the system is available
	if (!m_deviceAvailable) return (-1);

	// if system is already opened then return
	if (m_deviceReady) return (0);

	// try to open the device
	hdPhantomOpen(m_deviceID);

	// update device status
	m_deviceReady = true;

	// success
	return (0);
}


//===========================================================================
/*!
	Close connection to phantom device.

	\return Return 0 is operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cPhantomDevice::close()
{
	// check if drivers are installed
	if (!m_driverInstalled) return (-1);

	// check if the system has been opened previously
	if (!m_deviceReady) return (-1);

	// yes, the device is open so let's close it
	int result = hdPhantomClose(m_deviceID);

	 // update status
	m_deviceReady = false;

	// possibly turn off servo loop
	hdPhantomStopServo();

	// exit
	return (result);
}


//===========================================================================
/*!
	Calibrate the phantom device.

	\return Return 0 if operation succeeds, -1 if an error occurs.
*/
//===========================================================================
int cPhantomDevice::calibrate()
{
	// check if drivers are installed
	if (!m_driverInstalled) 
		return (-1);

	// exit
	return 0;
}


//===========================================================================
/*!
	Returns the number of devices available from this class of device.

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
    a_position = m_specifications.m_positionOffset + cVector3d(x, y, z);
    estimateLinearVelocity(a_position);
    return (error);
}


//===========================================================================
/*!
	Read the linear velocity of the device. Units are in [m/s].

	\param  a_linearVelocity  Return value.

	\return Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::getLinearVelocity(cVector3d& a_linearVelocity)
{
	// check if drivers are installed
	if (!m_driverInstalled) return (-1);

	int error = -1;

    // Note: The velocity signal computed by the Sensable API is pretty noisy,
    // so we avoid using it here.
    /* 
	double vx,vy,vz;
	error = hdPhantomGetLinearVelocity(m_deviceID, &vx, &vy, &vz);
    m_linearVelocity.set(vx, vy, vz);
	*/

    a_linearVelocity = m_linearVelocity;
	
	return (error); 
}


//===========================================================================
/*!
	Read the orientation frame of the device end-effector

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

	\param  a_force  Force command to be applied to device.

	\return Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::setForce(const cVector3d& a_force)
{
	// check if drivers are installed
	if (!m_driverInstalled) return (-1);

	// send force command
	cVector3d force = a_force;
	int error = hdPhantomSetForce(m_deviceID, &force(0) , &force(1) , &force(2) );
	if (error != 0) { return (error); }

	// store new commanded values
	m_prevForce = a_force;

	// success
	return (0);
}


//===========================================================================
/*!
	Send a force [N] and torque [N*m] to the haptic device.

	\param  a_force Force command to be applied to device.
	\param  a_torque Torque command to be applied to device.

	\return  Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::setForceAndTorque(const cVector3d& a_force, const cVector3d& a_torque)
{
    // check if drivers are installed
    if (!m_driverInstalled) return (-1);

    // send force and torque command
    int error = hdPhantomSetForceAndTorque(m_deviceID, &a_force(0), &a_force(1), &a_force(2), &a_torque(0), &a_torque(1), &a_torque(2));
    if (error != 0) { return (error); }

    // store new commanded values
    m_prevForce  = a_force;
    m_prevTorque = a_torque;

    // success
    return (0);
}


//===========================================================================
/*!
	Read the status of the user switch [\b true = \b ON / \b false = \b OFF].

	\param  a_switchIndex  index number of the switch.
	\param  a_status result value from reading the selected input switch.

	\return Return 0 if no error occurred.
*/
//===========================================================================
int cPhantomDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
	// check if drivers are installed
	if (!m_driverInstalled) return (-1);
	
	bool result = false;
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
#if defined(LINUX)

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
			return;
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
			return;
		}

		// enable forces
		hdMakeCurrentDevice(hHD1);
		hdEnable(HD_FORCE_OUTPUT);

		// add device to list
		phantomDevices[1].handle = hHD1;
		phantomDevices[1].enabled = true;
		numPhantomDevices++;
	}
	return;
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
int hdPhantomOpen(const int a_deviceID)
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
int hdPhantomClose(const int a_deviceID)
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
int hdPhantomGetPosition(const int a_deviceID, 
						 double *a_posX,
						 double *a_posY,
						 double *a_posZ)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { hdPhantomStartServo(); }

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
int hdPhantomGetLinearVelocity(const int a_deviceID, 
                               double *a_velX,
                               double *a_velY,
                               double *a_velZ)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { hdPhantomStartServo(); }

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
int hdPhantomGetRotation(const int a_deviceID, 
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
	if (!servoStarted) { hdPhantomStartServo(); }

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

	*a_rot00 = v0(0) ;	
	*a_rot01 = v1(0) ; 
	*a_rot02 = v2(0) ; 
	*a_rot10 = v0(1) ; 
	*a_rot11 = v1(1) ;
	*a_rot12 = v2(1) ;
	*a_rot20 = v0(2) ; 
	*a_rot21 = v1(2) ; 
	*a_rot22 = v2(2) ;

	// success
	return (0);
}


//==========================================================================
/*!
  Read the values of each end-effector user button.
*/
//==========================================================================
int hdPhantomGetButtons(const int a_deviceID)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { hdPhantomStartServo(); }

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
int hdPhantomSetForce(const int a_deviceID, 
					  const double *a_forceX,
					  const double *a_forceY,
					  const double *a_forceZ)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { hdPhantomStartServo(); }

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
int hdPhantomSetTorque(const int a_deviceID, 
					   const double *a_torqueX,
					   const double *a_torqueY,
					   const double *a_torqueZ)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { hdPhantomStartServo(); }

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
int hdPhantomGetWorkspaceRadius(const int a_deviceID, 
                                double *a_workspaceRadius)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { hdPhantomStartServo(); }

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
int hdPhantomGetType(const int a_deviceID, 
					 char* a_typeName)
{
	// check id
	if ((a_deviceID < 0) || (a_deviceID >= numPhantomDevices)) { return (-1); }

	// check if servo started
	if (!servoStarted) { hdPhantomStartServo(); }

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

	\param  pUserData pointer to user data information (not used here)
*/
//==========================================================================
HDCallbackCode HDCALLBACK servoPhantomDevices(void* pUserData)
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
		start servo controller
*/
//==========================================================================
void hdPhantomStartServo()
{
	HDErrorInfo error;
	if (!servoStarted)
	{
		// servo controller has been started
		servoStarted = true;

		// create callback
		HDCallbackCode servoCallbackHandle = hdScheduleAsynchronous(
		servoPhantomDevices, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);

		// start scheduler
		hdStartScheduler();
	}
}


//==========================================================================
/*
		stop  servo controller
*/
//==========================================================================
void hdPhantomStopServo()
{
	if (servoStarted)
	{
		// check if any device is enabled
		bool deviceEnabled = false;
		for (int i=0; i<PHANTOM_NUM_DEVICES_MAX; i++)
		{
		   deviceEnabled = deviceEnabled || phantomDevices[i].enabled;
		}

		// if a device is still enabled, do not stop servo loop
		if (deviceEnabled) { return; };

		// stop servo controller
		servoStarted = false;
		hdStopScheduler();
	}
}

//---------------------------------------------------------------------------
#endif // LINUX
//---------------------------------------------------------------------------
#endif // C_ENABLE_PHANTOM_DEVICE_SUPPORT
//---------------------------------------------------------------------------

