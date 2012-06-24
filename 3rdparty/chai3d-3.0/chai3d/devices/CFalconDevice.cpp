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
 \author    Francois Conti
 \author    Sebastien Grange
 \version   $MAJOR.$MINOR.$RELEASE $Rev: 799 $
 */
//===========================================================================

//---------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CFalconDevice.h"
//---------------------------------------------------------------------------
#if defined(C_ENABLE_FALCON_DEVICE_SUPPORT)
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

// HDL prototypes
static int          (__stdcall *hdlCountDevices)      (void)                                = NULL;
static int          (__stdcall *hdlInitIndexedDevice) (const int, const char*)              = NULL;
static void         (__stdcall *hdlUninitDevice)      (int)                                 = NULL;
static void         (__stdcall *hdlMakeCurrent)       (int)                                 = NULL;
static void         (__stdcall *hdlStart)             (void)                                = NULL;
static void         (__stdcall *hdlStop)              (void)                                = NULL;
static int          (__stdcall *hdlCreateServoOp)     (int (_cdecl *)(void *), void*, bool) = NULL;
static void         (__stdcall *hdlDestroyServoOp)    (int)                                 = NULL;
static void         (__stdcall *hdlToolPosition)      (double*)                             = NULL;
static void         (__stdcall *hdlToolButtons)       (int*)                                = NULL;
static void         (__stdcall *hdlSetToolForce)      (double*)                             = NULL;
static unsigned int (__stdcall *hdlGetState)          (void)                                = NULL;

// global (to the file) handles
static HINSTANCE hdlDLL = NULL;

// servo loop callback
int __cdecl
hdlServoOp (void *param)
{
    static int count = 0;
    HANDLE* evt = (HANDLE*)(param);
    SetEvent (*evt);
    return 1;
}

// Initialize dhd dll reference count
int cFalconDevice::m_opencount = 0;
int cFalconDevice::m_handles[FALCON_NUM_DEVICES_MAX] = { 0, 0, 0, 0, 0, 0, 0, 0 };

//---------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//===========================================================================
/*!
     Constructor of cFalconDevice.
 */
//===========================================================================
cFalconDevice::cFalconDevice(unsigned int a_deviceNumber)
{
    // set specifications
    m_specifications.m_model                         = C_HAPTIC_DEVICE_FALCON;
    m_specifications.m_manufacturerName              = "Novint Technologies";
    m_specifications.m_modelName                     = "Falcon";
    m_specifications.m_maxLinearForce                =    8.0;   // [N]
    m_specifications.m_maxAngularTorque              =    0.0;   // [N*m]
    m_specifications.m_maxGripperForce               =    0.0;   // [N]
    m_specifications.m_maxLinearStiffness            = 3000.0;   // [N/m]
    m_specifications.m_maxAngularStiffness           =    0.0;   // [N*m/Rad]
    m_specifications.m_maxGripperLinearStiffness     =    0.0;   // [N*m/Rad]
    m_specifications.m_maxLinearDamping              =   20.0;   // [N/(m/s)]
    m_specifications.m_maxAngularDamping             =    0.0;   // [N*m/(Rad/s)]
    m_specifications.m_maxGripperAngularDamping      =    0.0;   // [N*m/(Rad/s)]
    m_specifications.m_workspaceRadius               =    0.04;  // [m]
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
    m_deviceAvailable = false;
    m_deviceReady     = false;

    // initialize and store device handles if not already done
    if (m_opencount == 0)
    {
        // if Falcon configuration does not exist, silently ignore
        if (getenv ("NOVINT_DEVICE_SUPPORT") == NULL)
        {
            m_driverInstalled = false;
            return;
        }

        // if Novint configuration folder exists, look for DLL
        else
        {
            // retrieve path to HDAL
            string HDLPath = string(getenv ("NOVINT_DEVICE_SUPPORT"));

            // test for Falcon support
            hdlDLL = LoadLibrary ((HDLPath+string("\\bin\\hdl.dll")).c_str());

            // if DLL does not exist, silently ignore
            if (hdlDLL == NULL)
            {
                m_driverInstalled = false;
                return;
            }

            // otherwise import DLL entry points
            else
            {
                // check if multi device is supported
                if (GetProcAddress (hdlDLL, "_hdlCountDevices@0") != NULL)
                {
                    // load relevant DLL exports
                  if ( (hdlCountDevices       = (int          (__stdcall*)(void))                                GetProcAddress (hdlDLL, "_hdlCountDevices@0"))      == NULL) { m_driverInstalled = false; return; }
                    if ((hdlInitIndexedDevice = (int          (__stdcall*)(const int, const char*))              GetProcAddress (hdlDLL, "_hdlInitIndexedDevice@8")) == NULL) { m_driverInstalled = false; return; }
                    if ((hdlUninitDevice      = (void         (__stdcall*)(int))                                 GetProcAddress (hdlDLL, "_hdlUninitDevice@4"))      == NULL) { m_driverInstalled = false; return; }
                    if ((hdlMakeCurrent       = (void         (__stdcall*)(int))                                 GetProcAddress (hdlDLL, "_hdlMakeCurrent@4"))       == NULL) { m_driverInstalled = false; return; }
                    if ((hdlStart             = (void         (__stdcall*)(void))                                GetProcAddress (hdlDLL, "_hdlStart@0"))             == NULL) { m_driverInstalled = false; return; }
                    if ((hdlStop              = (void         (__stdcall*)(void))                                GetProcAddress (hdlDLL, "_hdlStop@0"))              == NULL) { m_driverInstalled = false; return; }
                    if ((hdlCreateServoOp     = (int          (__stdcall*)(int (_cdecl *)(void *), void*, bool)) GetProcAddress (hdlDLL, "_hdlCreateServoOp@12"))    == NULL) { m_driverInstalled = false; return; }
                    if ((hdlDestroyServoOp    = (void         (__stdcall*)(int))                                 GetProcAddress (hdlDLL, "_hdlDestroyServoOp@4"))    == NULL) { m_driverInstalled = false; return; }
                    if ((hdlToolPosition      = (void         (__stdcall*)(double*))                             GetProcAddress (hdlDLL, "_hdlToolPosition@4"))      == NULL) { m_driverInstalled = false; return; }
                    if ((hdlToolButtons       = (void         (__stdcall*)(int*))                                GetProcAddress (hdlDLL, "_hdlToolButtons@4"))       == NULL) { m_driverInstalled = false; return; }
                    if ((hdlSetToolForce      = (void         (__stdcall*)(double*))                             GetProcAddress (hdlDLL, "_hdlSetToolForce@4"))      == NULL) { m_driverInstalled = false; return; }
                    if ((hdlGetState          = (unsigned int (__stdcall*)(void))                                GetProcAddress (hdlDLL, "_hdlGetState@0"))          == NULL) { m_driverInstalled = false; return; }

                    // initialize all Falcon handles found
                    int deviceCount = hdlCountDevices ();
                    for (int i=0; i<deviceCount; i++)
                        m_handles[i] = hdlInitIndexedDevice (i, HDLPath.c_str ());
                    if (deviceCount > 0)
                        hdlStart ();
                }

                // failed, old version is installed
                else
                {
                    printf("ERROR: Please update the drivers of your Novint Falcon haptic device.\n");
                    return;
                }
            }
        }
    }

    // check if the Falcon drivers are installed
    if (hdlCountDevices != NULL) m_driverInstalled = true;
    else
    {
      m_driverInstalled = false;
      return;
    }

    // get the number ID of the device we wish to communicate with
    m_deviceID = a_deviceNumber;

    // check if such device is available
    if ((a_deviceNumber + 1) > (unsigned int)(hdlCountDevices()))
        m_deviceAvailable = false;
    else
        m_deviceAvailable = true;

    // increment counter
    m_opencount++;
}


//===========================================================================
/*!
     Destructor of cFalconDevice.
 */
//===========================================================================
cFalconDevice::~cFalconDevice()
{
    // close connection to device
    if (m_deviceReady)
        close();

    // if this is the last device to destruct, cleanup
    m_opencount--;
    if ((m_opencount == 0) && (hdlDLL != NULL))
    {
        int deviceCount = hdlCountDevices();
        if (deviceCount > 0) hdlStop();
        for (int i=0; i<deviceCount; i++)
            hdlUninitDevice (m_handles[i]);
        FreeLibrary (hdlDLL);
        hdlDLL = NULL;
    }
}


//===========================================================================
/*!
     Open connection to Falcon haptic device.
 */
//===========================================================================
int cFalconDevice::open()
{
    // check if drivers are installed
    if (!m_driverInstalled)
        return (-1);

    // check if the system is available
    if (!m_deviceAvailable)
        return (-1);

    // if system is already opened then return
    if (m_deviceReady)
        return (0);

    // check id
    if ((m_deviceID < 0) || (m_deviceID > FALCON_NUM_DEVICES_MAX))
        return (-1);

    // enable device
    m_servoEvent = CreateEvent(NULL, false, false, NULL);
    m_servoHandle = hdlCreateServoOp(hdlServoOp, (void*)(&m_servoEvent), false);

    // update device status
    m_deviceReady = true;

    // success
    return (0);
}


//===========================================================================
/*!
     Close connection to Falcon haptic device.
 */
//===========================================================================
int cFalconDevice::close()
{
    // check if drivers are installed
    if (!m_driverInstalled)
        return (-1);

    // check if the system has been opened previously
    if (!m_deviceReady)
        return (-1);

    // yes, the device is open so let's close it
    hdlDestroyServoOp(m_servoHandle);
    CloseHandle(m_servoEvent);

    // update status
    m_deviceReady = false;

    // exit
    return (0);
}


//===========================================================================
/*!
    Calibrate Falcon haptic device.

    \return Always 0
 */
//===========================================================================
int cFalconDevice::calibrate()
{
    // check if drivers are installed
    if (!m_driverInstalled)
        return (-1);

    // success
    return (0);
}


//===========================================================================
/*!
     Returns the number of devices available from this class of device.

     \return  Returns the result
 */
//===========================================================================
unsigned int cFalconDevice::getNumDevices()
{
    // check if drivers are installed
    if (!m_driverInstalled)
        return (0);

    // read number of devices
    return hdlCountDevices();
}


//===========================================================================
/*!
     Read the position of the device. Units are meters [m].

     \param  a_position  Return value.

     \return Return 0 if no error occurred.
 */
//===========================================================================
int cFalconDevice::getPosition(cVector3d& a_position)
{
    // check if drivers are installed
    if (!m_driverInstalled)
        return (-1);

    // check if enabled
    if (!m_deviceReady)
        return (-1);

    // get position
    double pos[3];
    hdlMakeCurrent(m_handles[m_deviceID]);
    hdlToolPosition(pos);

    // add a small offset for zero centering
    pos[2] += 0.01;
    a_position.set(pos[2], pos[0], pos[1]);
    estimateLinearVelocity(a_position);

    // success
    return (0);
}


//===========================================================================
/*!
     Send a force [N] to the Falcon haptic device.

     \param  a_force  Force command to be applied to device.

     \return Return 0 if no error occurred.
 */
//===========================================================================
int cFalconDevice::setForce(const cVector3d& a_force)
{
    // check if drivers are installed
    if (!m_driverInstalled)
        return (-1);

    // check if enabled
    if (!m_deviceReady)
        return (-1);

    // define force in Falcon frame
    double f[3];
    f[2] = a_force(0) ;
    f[0] = a_force(1) ;
    f[1] = a_force(2) ;

    // synchronize with a servo thread in order to save system resources
    WaitForSingleObject(m_servoEvent, 1000);

    // write force to device
    hdlMakeCurrent(m_handles[m_deviceID]);
    hdlSetToolForce(f);
    m_prevForce = a_force;

    // success
    return (0);
}


//===========================================================================
/*!
     Read the status of the user switch [1 = \e ON / 0 = \e OFF].

     \param  a_switchIndex  index number of the switch.
     \param  a_status result value from reading the selected input switch.

     \return Return 0 if no error occurred.
 */
//===========================================================================
int cFalconDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    // check if drivers are installed
    if (!m_driverInstalled)
        return (-1);

    // check if enabled
    if (!m_deviceReady)
        return (-1);

    // retrieve button data from device
    int button;
    hdlMakeCurrent(m_handles[m_deviceID]);
    hdlToolButtons(&button);

    // translate button index
    bool result = false;
    switch (a_switchIndex)
    {
        case 0:
            if (button & 1)
                result = true;
            break;

        case 1:
            if (button & 2)
                result = true;
            break;

        case 2:
            if (button & 4)
                result = true;
            break;

        case 3:
            if (button & 8)
                result = true;
            break;
    }

    // return result
    a_status = result;

    // success
    return (0);
}

//---------------------------------------------------------------------------
#endif //C_ENABLE_FALCON_DEVICE_SUPPORT
//---------------------------------------------------------------------------
