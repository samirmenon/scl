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
    \author    Force Dimension - www.forcedimension.com
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 931 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CHydraDeviceH
#define CHydraDeviceH
//------------------------------------------------------------------------------
#if defined(C_ENABLE_HYDRA_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CHydraDevice.h

    \brief
    <b> Devices </b> \n 
    Razer Hydra Devices.
*/
//==============================================================================

#if defined(WIN32) | defined(WIN64)

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/* devices */
#define DHD_DEVICE_NONE              0
#define DHD_DEVICE_3DOF             31
#define DHD_DEVICE_6DOF             61
#define DHD_DEVICE_6DOF_500         62
#define DHD_DEVICE_DELTA3           63
#define DHD_DEVICE_DELTA6           64
#define DHD_DEVICE_OMEGA            32
#define DHD_DEVICE_OMEGA3           33
#define DHD_DEVICE_OMEGA33          34
#define DHD_DEVICE_OMEGA33_LEFT     36
#define DHD_DEVICE_OMEGA331         35
#define DHD_DEVICE_OMEGA331_LEFT    37
#define DHD_DEVICE_FALCON           60
#define DHD_DEVICE_CONTROLLER       81
#define DHD_DEVICE_CONTROLLER_HR    82
#define DHD_DEVICE_CUSTOM           91
#define DHD_DEVICE_DLR331          102
#define DHD_DEVICE_DLR331_LEFT     103
#define DHD_DEVICE_SIGMA331        104
#define DHD_DEVICE_SIGMA331_LEFT   105

/* status */
#define DHD_ON                     1
#define DHD_OFF                    0

/* device count */
#define DHD_MAX_DEVICE             4

/* TimeGuard return value */
#define DHD_TIMEGUARD              1

/* status count */
#define DHD_MAX_STATUS            15

/* status codes */
#define DHD_STATUS_POWER           0
#define DHD_STATUS_CONNECTED       1
#define DHD_STATUS_STARTED         2
#define DHD_STATUS_RESET           3
#define DHD_STATUS_IDLE            4
#define DHD_STATUS_FORCE           5
#define DHD_STATUS_BRAKE           6
#define DHD_STATUS_TORQUE          7
#define DHD_STATUS_WRIST_DETECTED  8
#define DHD_STATUS_ERROR           9
#define DHD_STATUS_GRAVITY        10
#define DHD_STATUS_TIMEGUARD      11
#define DHD_STATUS_ROTATOR_RESET  12
#define DHD_STATUS_REDUNDANCY     13
#define DHD_STATUS_FORCEOFFCAUSE  14

/* buttons count */
#define DHD_MAX_BUTTONS            8

/* velocity estimator computation mode */
#define DHD_VELOCITY_WINDOWING     0
#define DHD_VELOCITY_AVERAGING     1

#endif  // DOXYGEN_SHOULD_SKIP_THIS
#endif  // WIN32

//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cDeltaDevice
    \ingroup    devices  
    
    \brief  
    cDeltaDevice implements an interface to  the omega(0)  and delta(0) 
    haptic devices from Force Dimension.
*/
//==============================================================================
class cDeltaDevice : public cGenericHapticDevice
{
  public:

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
    //! Constructor of cDeltaDevice.
    cDeltaDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cDeltaDevice.
    virtual ~cDeltaDevice();


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    //! Open connection to haptic device (0 indicates success).
    int open();

    //! Close connection to haptic device (0 indicates success).
    int close();

    //! Calibrate haptic device (0 indicates success).
    int calibrate(bool force=false);

    //! Returns the number of devices available from this class of device.
    unsigned int getNumDevices();

    //! Read the position of the device. Units are meters [m].
    int getPosition(cVector3d& a_position);

    //! Read the linear velocity of the device. Units are meters per second [m/s].
    int getLinearVelocity(cVector3d& a_linearVelocity);

    //! Read the orientation frame of the device end-effector.
    int getRotation(cMatrix3d& a_rotation);

    //! Read the gripper angle in radian.
    int getGripperAngleRad(double& a_angle);

    //! Send a force [N] to the haptic device.
    int setForce(const cVector3d& a_force);

    //! Send a force [N] and a torque [N*m] to the haptic device.
    int setForceAndTorque(const cVector3d& a_force, const cVector3d& a_torque);

    //! Send a force [N] and a torque [N*m] and force [N] to the gripper.
    int setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);

    //! read the status of the user switch [\b true = \b ON / \b false = \b OFF].
    int getUserSwitch(int a_switchIndex, bool& a_status);


    //--------------------------------------------------------------------------
    // METHODS RESTRICTED TO FORCE DIMENSION DEVICES ONLY:
    //--------------------------------------------------------------------------

    //! Return the Force Dimension type of the current device.
    int getDeviceType() { return m_deviceType; }

    //! Overrides the force button switch located at the base of the device.
    int enableForces(bool a_value);

  protected:

    //! Reference count used to control access to the dhd dll.
    static int m_activeDeltaDevices;

    //! Device number in the list of Force Dimension devices connected to the computer
    int m_deviceNumber;

    //! Device ID number.
    int m_deviceID;

    //! Which FD device is actually instantiated here?
    int m_deviceType;

    //! structure for modeling a low-pass filter user switches.
    int m_userSwitchCount[8];

    //! Last state of user switch.
    int m_userSwitchStatus[8];

    //! Timeguard for user switch.
    cPrecisionClock m_userSwitchClock[8];

    //! Have forces been enable yet since the connection to the device was opened?
    bool statusEnableForcesFirstTime;

    //! status of DHD API calls.
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
    static bool sdhdGetDeviceCount;
    static bool sdhdGetDeviceID;
    static bool sdhdGetSystemType;
    static bool sdhdOpenID;
    static bool sdhdClose;
    static bool sdhdReset;
    static bool sdhdGetButton;
    static bool sdhdGetPosition;
    static bool sdhdGetLinearVelocity;
    static bool sdhdGetOrientationRad;
    static bool sdhdSetTorque;
    static bool sdhdGetOrientationFrame;
    static bool sdhdSetForce;
    static bool sdhdSetForceAndTorque;
    static bool sdhdSetForceAndGripperForce;
    static bool sdhdSetForceAndTorqueAndGripperForce;
    static bool sdhdGetGripperThumbPos;
    static bool sdhdGetGripperFingerPos;
    static bool sdhdGetGripperAngleRad;
    static bool sdhdEnableExpertMode;
    static bool sdhdDisableExpertMode;
    static bool sdhdEnableForce;
    static bool sdhdIsLeftHanded;
    static bool sdhdSetBaseAngleZDeg;
    static bool sdhdSetVelocityThreshold;
    static bool sdrdOpenID;
    static bool sdrdClose;
    static bool sdrdIsInitialized;
    static bool sdrdAutoInit;
    static bool sdrdStop;
    #endif  // DOXYGEN_SHOULD_SKIP_THIS
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif //_DISABLE_DELTA_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
