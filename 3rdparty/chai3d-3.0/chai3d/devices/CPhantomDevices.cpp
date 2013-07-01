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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1080 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CPhantomDevices.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_PHANTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

// Number of instances for this class of devices currently using the libraries.
unsigned int cPhantomDevice::s_libraryCounter = 0;

// Allocation table for devices of this class.
bool cPhantomDevice::s_allocationTable[C_MAX_DEVICES] = {false, false, false, false,
    false, false, false, false,
    false, false, false, false,
    false, false, false, false}; 

//------------------------------------------------------------------------------
// WIN32
//------------------------------------------------------------------------------
#if defined(WIN32)
HINSTANCE hdPhantomDLL = NULL;
HINSTANCE hdPhantomDriverDLL = NULL;

extern "C"
{

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

}

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// LINUX
//------------------------------------------------------------------------------
#if defined(LINUX)

void * HDSO = NULL;
void * PhantomSO = NULL;

// functions that communicate data with the Phantom devices
int  (*hdPhantomGetNumDevices      ) (void);
int  (*hdPhantomOpen               ) (const int);
int  (*hdPhantomClose              ) (const int);
int  (*hdPhantomGetPosition        ) (const int, double*, double*, double*);
int  (*hdPhantomGetLinearVelocity  ) (const int, double*, double*, double*);
int  (*hdPhantomGetRotation        ) (const int, double*, double*, double*, double*, double*, double*, double*, double*, double*);
int  (*hdPhantomGetButtons         ) (const int);
int  (*hdPhantomSetForce           ) (const int, const double*, const double*, const double*);
int  (*hdPhantomSetTorque          ) (const int, const double*, const double*, const double*);
int  (*hdPhantomSetForceAndTorque  ) (const int, const double*, const double*, const double*, const double*, const double*, const double*);
int  (*hdPhantomGetWorkspaceRadius ) (const int, double*);
int  (*hdPhantomGetType            ) (const int, char*);
void (*hdPhantomStartServo         ) (void);
void (*hdPhantomStopServo          ) (void);
//------------------------------------------------------------------------------
#endif  // LINUX
//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Open libraries for this class of devices.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::openLibraries() 
{ 
    // increment number of instances using the libraries for this class of devices
    s_libraryCounter++;

    // if libraries are already initialized, then we are done
    if (s_libraryCounter > 1) return (C_SUCCESS); 


    ////////////////////////////////////////////////////////////////////////////
    // initialize libraries
    ////////////////////////////////////////////////////////////////////////////

    // check if Phantom drivers installed
#if defined(WIN32)
    hdPhantomDriverDLL = LoadLibrary("HD.dll");
    if (hdPhantomDriverDLL == NULL) { return (C_ERROR); }
#endif

    // load phantom interface library
#if defined (WIN32)
        hdPhantomDLL = LoadLibrary("hdPhantom32.dll");
#endif
#if defined (WIN64)
        hdPhantomDLL = LoadLibrary("hdPhantom64.dll");
#endif


#if defined(WIN32) | defined(WIN64)

    // check if DLL loaded correctly
    if (hdPhantomDLL == NULL)
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load different callbacks
    hdPhantomGetNumDevices = (int (__stdcall*)(void))GetProcAddress(hdPhantomDLL, "hdPhantomGetNumDevices");
    hdPhantomOpen = (int (__stdcall*)(const int))GetProcAddress(hdPhantomDLL, "hdPhantomOpen");
    hdPhantomClose = (int (__stdcall*)(const int))GetProcAddress(hdPhantomDLL, "hdPhantomClose");
    hdPhantomGetPosition = (int (__stdcall*)(const int, double*, double*, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetPosition");
    hdPhantomGetLinearVelocity = (int (__stdcall*)(const int, double*, double*, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetLinearVelocity");
    hdPhantomGetRotation = (int (__stdcall*)(const int, double*, double*, double*, double*, double*, double*, double*, double*, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetRotation");
    hdPhantomGetButtons = (int (__stdcall*)(const int))GetProcAddress(hdPhantomDLL, "hdPhantomGetButtons");
    hdPhantomSetForce = (int (__stdcall*)(const int, const double*, const double*, const double*))GetProcAddress(hdPhantomDLL, "hdPhantomSetForce");
    hdPhantomSetTorque = (int (__stdcall*)(const int, const double*, const double*, const double*))GetProcAddress(hdPhantomDLL, "hdPhantomSetTorque");
    hdPhantomSetForceAndTorque = (int (__stdcall*)(const int, const double*, const double*, const double*, const double*, const double*, const double*))GetProcAddress(hdPhantomDLL, "hdPhantomSetForceAndTorque");
    hdPhantomGetWorkspaceRadius = (int (__stdcall*)(const int, double*))GetProcAddress(hdPhantomDLL, "hdPhantomGetWorkspaceRadius");
    hdPhantomGetType = (int (__stdcall*)(const int, char*))GetProcAddress(hdPhantomDLL, "hdPhantomGetType");
    hdPhantomStartServo = (void (__stdcall*)(void))GetProcAddress(hdPhantomDLL, "hdPhantomStartServo");
    hdPhantomStopServo = (void (__stdcall*)(void))GetProcAddress(hdPhantomDLL, "hdPhantomStopServo");
    
    // check if all functions were loaded
    if ((!hdPhantomStopServo) ||
        (!hdPhantomOpen) ||
        (!hdPhantomClose) ||
        (!hdPhantomGetPosition) ||
        (!hdPhantomGetLinearVelocity) ||
        (!hdPhantomGetRotation) ||
        (!hdPhantomGetButtons) ||
        (!hdPhantomSetForce) ||
        (!hdPhantomSetTorque) ||
        (!hdPhantomSetForceAndTorque) ||
        (!hdPhantomGetWorkspaceRadius) ||
        (!hdPhantomGetType) ||
        (!hdPhantomStartServo) ||
        (!hdPhantomStopServo)) 
        return (C_ERROR);

#endif

#ifdef LINUX

    // load shared library
    HDSO = dlopen ("libHD.so", RTLD_LAZY|RTLD_GLOBAL);

    // check that it loaded correctly
    if (HDSO == NULL)
    {
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load shared library
    PhantomSO = dlopen ("libhdPhantom.so", RTLD_LAZY|RTLD_GLOBAL);

    // check that it loaded correctly
    if (PhantomSO == NULL)
    {
        printf ("*** CPhantomDevices: %s\n", dlerror());
        dlclose (HDSO);
        HDSO = NULL;
        s_libraryCounter = 0;
        return (C_ERROR);
    }

    // load different callbacks
    *(void**)(&hdPhantomGetNumDevices     ) = dlsym (PhantomSO, "hdPhantomGetNumDevices");
    *(void**)(&hdPhantomOpen              ) = dlsym (PhantomSO, "hdPhantomOpen");
    *(void**)(&hdPhantomClose             ) = dlsym (PhantomSO, "hdPhantomClose");
    *(void**)(&hdPhantomGetPosition       ) = dlsym (PhantomSO, "hdPhantomGetPosition");
    *(void**)(&hdPhantomGetLinearVelocity ) = dlsym (PhantomSO, "hdPhantomGetLinearVelocity");
    *(void**)(&hdPhantomGetRotation       ) = dlsym (PhantomSO, "hdPhantomGetRotation");
    *(void**)(&hdPhantomGetButtons        ) = dlsym (PhantomSO, "hdPhantomGetButtons");
    *(void**)(&hdPhantomSetForce          ) = dlsym (PhantomSO, "hdPhantomSetForce");
    *(void**)(&hdPhantomSetTorque         ) = dlsym (PhantomSO, "hdPhantomSetTorque");
    *(void**)(&hdPhantomSetForceAndTorque ) = dlsym (PhantomSO, "hdPhantomSetForceAndTorque");
    *(void**)(&hdPhantomGetWorkspaceRadius) = dlsym (PhantomSO, "hdPhantomGetWorkspaceRadius");
    *(void**)(&hdPhantomGetType           ) = dlsym (PhantomSO, "hdPhantomGetType");
    *(void**)(&hdPhantomStartServo        ) = dlsym (PhantomSO, "hdPhantomStartServo");
    *(void**)(&hdPhantomStopServo         ) = dlsym (PhantomSO, "hdPhantomStopServo");

    // check if all functions were loaded
    if ((!hdPhantomStopServo) ||
            (!hdPhantomOpen) ||
            (!hdPhantomClose) ||
            (!hdPhantomGetPosition) ||
            (!hdPhantomGetLinearVelocity) ||
            (!hdPhantomGetRotation) ||
            (!hdPhantomGetButtons) ||
            (!hdPhantomSetForce) ||
            (!hdPhantomSetTorque) ||
            (!hdPhantomSetForceAndTorque) ||
            (!hdPhantomGetWorkspaceRadius) ||
            (!hdPhantomGetType) ||
            (!hdPhantomStartServo) ||
            (!hdPhantomStopServo)) {
        printf ("*** CPhantomDevices: %s\n", dlerror());
        return (C_ERROR);
    }

#endif

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    Close libraries for this class of devices.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool cPhantomDevice::closeLibraries() 
{ 
    // sanity check
    if (s_libraryCounter < 1) return (C_ERROR);

    // decrement library counter; exit if other objects are still using libraries
    s_libraryCounter--;
    if (s_libraryCounter > 0) return (C_SUCCESS);

    // free libraries
    #if defined(WIN32) | defined(WIN64)
    if ((s_libraryCounter == 0) && (hdPhantomDLL != NULL))
    {
        FreeLibrary(hdPhantomDLL);
        FreeLibrary(hdPhantomDriverDLL);
        hdPhantomDLL = NULL;
        hdPhantomDriverDLL = NULL;
    }
    #endif

    // free libraries
    #ifdef LINUX
    if ((s_libraryCounter == 0) && (PhantomSO != NULL))
    {
        dlclose (PhantomSO);
        dlclose (HDSO);
        PhantomSO = NULL;
        HDSO = NULL;
    }
    #endif

    // exit
    return (C_SUCCESS); 
}


//==============================================================================
/*!
    Get number of haptic devices available for this class of device.

    \return  Number of available haptic devices.
*/
//==============================================================================
unsigned int cPhantomDevice::getNumDevices()
{
    // open libraries
    if (openLibraries() == false)
    {
        //printf ("*** hdPhantom: open libraries failed\n");
        return (0);
    }

    // get device number
    int result = hdPhantomGetNumDevices();

    // close libraries
    closeLibraries();

    // return result
    return (result);
}



//==============================================================================
/*!
    Constructor of cPhantomDevice.
    No servo loop is yet created, encoders are NOT reset.

    \param  a_deviceNumber  Index number to the ith Phantom device
*/
//==============================================================================
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
    m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
    m_specifications.m_sensedPosition                = true;
    m_specifications.m_sensedRotation                = true;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = true;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = true;
    m_specifications.m_rightHand                     = true;

    // device is not yet available or ready
    m_deviceAvailable = false;
    m_deviceReady = false;

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
        m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
        m_specifications.m_sensedPosition                = true;
        m_specifications.m_sensedRotation                = true;
        m_specifications.m_sensedGripper                 = false;
        m_specifications.m_actuatedPosition              = true;
        m_specifications.m_actuatedRotation              = false;
        m_specifications.m_actuatedGripper               = false;
        m_specifications.m_leftHand                      = true;
        m_specifications.m_rightHand                     = true;
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
        m_specifications.m_gripperMaxAngleRad            = cDegToRad(0.0);
        m_specifications.m_sensedPosition                = true;
        m_specifications.m_sensedRotation                = true;
        m_specifications.m_sensedGripper                 = false;
        m_specifications.m_actuatedPosition              = true;
        m_specifications.m_actuatedRotation              = true;
        m_specifications.m_actuatedGripper               = false;
        m_specifications.m_leftHand                      = true;
        m_specifications.m_rightHand                     = true;
    }
}


//==============================================================================
/*!
    Destructor of cPhantomDevice.
*/
//==============================================================================
cPhantomDevice::~cPhantomDevice()
{
    // close device
    if (m_deviceReady)
    {
        close();
    }

    // release device
    if (m_deviceAvailable)
    {
        s_allocationTable[m_deviceNumber] = false;
    }

    // close libraries
    closeLibraries();
}


//==============================================================================
/*!
    Open connection to Phantom device.

    \return Return C_SUCCESS is operation succeeds, C_ERROR if an error occurs.
*/
//==============================================================================
bool cPhantomDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_SUCCESS);

    // try to open the device
    hdPhantomOpen(m_deviceID);

    // update device status
    m_deviceReady = true;

    // success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    Close connection to phantom device.

    \return Return C_SUCCESS is operation succeeds, C_ERROR if an error occurs.
*/
//==============================================================================
bool cPhantomDevice::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady) return (C_ERROR);

    // yes, the device is open so let's close it
    int result = hdPhantomClose(m_deviceID);

     // update status
    m_deviceReady = false;

    // possibly turn off servo loop
    hdPhantomStopServo();

    // exit
    return (result);
}


//==============================================================================
/*!
    Calibrate the phantom device.

    \return Return C_SUCCESS if operation succeeds, C_ERROR if an error occurs.
*/
//==============================================================================
bool cPhantomDevice::calibrate(bool a_forceCalibration)
{
    // check if drivers are installed
    if (!m_deviceReady) 
        return (C_ERROR);

    // exit
    return (C_SUCCESS);
}


//==============================================================================
/*!
    Read the position of the device. Units are meters [m].

    \param  a_position  Return value.

    \return Return C_SUCCESS if no error occurred.
*/
//==============================================================================
bool cPhantomDevice::getPosition(cVector3d& a_position)
{
    // check if drivers are installed
    if (!m_deviceReady) return (C_ERROR);

    double x,y,z;
    int error = hdPhantomGetPosition(m_deviceID, &x, &y, &z);
    a_position.set(x, y, z);

    // offset adjustment depending of device type
    if (m_specifications.m_model == C_HAPTIC_DEVICE_PHANTOM_15_6DOF)
    {
        a_position.add(0.0, 0.0, -0.10);
    }

    estimateLinearVelocity(a_position);
    return (error);
}


//==============================================================================
/*!
    Read the orientation frame of the device end-effector

    \param  a_rotation  Return value.

    \return Return C_SUCCESS if no error occurred.
*/
//==============================================================================
bool cPhantomDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if drivers are installed
    if (!m_deviceReady) return (C_ERROR);

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

    return (C_SUCCESS);
}


//==============================================================================
/*!
    Send a force [N] to the haptic device

    \param  a_force  Force command to be applied to device.

    \return Return C_SUCESS if no error occurred.
*/
//==============================================================================
bool cPhantomDevice::setForce(const cVector3d& a_force)
{
    // check if drivers are installed
    if (!m_deviceReady) return (C_ERROR);

    // send force command
    cVector3d force = a_force;
    int error = hdPhantomSetForce(m_deviceID, &force(0) , &force(1) , &force(2) );
    if (error == -1) { return (C_ERROR); }

    // store new commanded values
    m_prevForce = a_force;

    // success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    Send a force [N] and torque [N*m] to the haptic device.

    \param  a_force Force command to be applied to device.
    \param  a_torque Torque command to be applied to device.

    \return  Return C_SUCCESS if no error occurred.
*/
//==============================================================================
bool cPhantomDevice::setForceAndTorque(const cVector3d& a_force, const cVector3d& a_torque)
{
    // check if drivers are installed
    if (!m_deviceReady) return (C_ERROR);

    // send force and torque command
    int error = hdPhantomSetForceAndTorque(m_deviceID, &a_force(0), &a_force(1), &a_force(2), &a_torque(0), &a_torque(1), &a_torque(2));
    if (error == -1) { return (C_ERROR); }

    // store new commanded values
    m_prevForce  = a_force;
    m_prevTorque = a_torque;

    // success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    Read the status of the user switch [__true__ = \b ON / __false__ = \b OFF].

    \param  a_switchIndex  index number of the switch.
    \param  a_status result value from reading the selected input switch.

    \return Return C_SUCCESS if no error occurred.
*/
//==============================================================================
bool cPhantomDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    // check if drivers are installed
    if (!m_deviceReady) return (C_ERROR);
    
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

    return (C_SUCCESS);
}

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_PHANTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------

