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
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1064 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGenericHapticDeviceH
#define CGenericHapticDeviceH
//------------------------------------------------------------------------------
#include "devices/CGenericDevice.h"
#include "math/CMaths.h"
#include "system/CGlobals.h"
#include "timers/CPrecisionClock.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGenericHapticDevice.h

    \brief
    <b> Devices </b> \n 
    Haptic Device Base Class.
*/
//==============================================================================


//------------------------------------------------------------------------------
// GENERAL CONSTANTS
//------------------------------------------------------------------------------
//! Filter property used for velocity estimator. 
const int       C_DEVICE_HISTORY_SIZE            = 200;      // [number of samples]

//! Minimum time between two device status acquisitions.
const double    C_DEVICE_MIN_ACQUISITION_TIME    = 0.0001;   // [s]
//------------------------------------------------------------------------------


//==============================================================================
/*!
    Defines the list of devices currently supported by CHAI3D. 
*/
//==============================================================================
enum cHapticDeviceModel
{
    C_HAPTIC_DEVICE_VIRTUAL,
    C_HAPTIC_DEVICE_DELTA_3,
    C_HAPTIC_DEVICE_DELTA_6,
    C_HAPTIC_DEVICE_OMEGA_3,
    C_HAPTIC_DEVICE_OMEGA_6,
    C_HAPTIC_DEVICE_OMEGA_7,
    C_HAPTIC_DEVICE_SIGMA_7,
    C_HAPTIC_DEVICE_SIGMA_6P,
    C_HAPTIC_DEVICE_FALCON,
    C_HAPTIC_DEVICE_PHANTOM_OMNI,
    C_HAPTIC_DEVICE_PHANTOM_15_6DOF,
    C_HAPTIC_DEVICE_PHANTOM_OTHER,
    C_TRACKER_DEVICE_SIXENSE,
    C_HAPTIC_DEVICE_CUSTOM
};


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \struct     cTimestampValue
    \ingroup    devices  

    \brief
    Provides a structure to store a double value with a time stamp.
*/
//==============================================================================
struct cTimestampValue
{
    //! Time in seconds when value data was acquired.
    double m_time;

    //! Value data.
    double m_value;
};


//==============================================================================
/*!
    \struct     cTimestampPos
    \ingroup    devices 
    
    \brief
    Provides a structure to store a position value with a time stamp.
*/
//==============================================================================
struct cTimestampPos
{
    //! Time in seconds when position data was acquired.
    double m_time;

    //! Position data.
    cVector3d m_pos;
};


//==============================================================================
/*!
    \struct     cTimestampRot
    \ingroup    devices  

    \brief
    Provides a structure to store an rotation matrix with a time stamp.
*/
//==============================================================================
struct cTimestampRot
{
    //! Time in seconds when rotation matrix data data was acquired.
    double m_time;

    //! Rotation matrix data.
    cMatrix3d m_rot;
};


//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//==============================================================================
/*!
    \struct     cHapticDeviceInfo
    \ingroup    devices  

    \brief
    Specifications for haptic devices.

    \details
    Provides a structure which stores technical specifications about a
    haptic device.
*/
//==============================================================================
struct cHapticDeviceInfo
{
    //! Haptic device model.
    cHapticDeviceModel m_model;

    //! Name of the haptic device model.
    std::string m_modelName;

    //! Name of the manufacturer.
    std::string m_manufacturerName;

    //! Maximum continuous force in [N] that can be generated by the device in translation.
    double m_maxLinearForce;

    //! Maximum continuous torque in [N*m] that can be generated by the device in orientation.
    double m_maxAngularTorque;

    //! Maximum continuous force in [N] that can be produced by the gripper.
    double m_maxGripperForce;

    //! Maximum closed loop linear stiffness [N/m] for a simulation running at 1 KHz.
    double m_maxLinearStiffness;

    //! Maximum closed loop angular stiffness [N*m/rad] for a simulation running at 1 KHz.
    double m_maxAngularStiffness;

    //! Maximum closed loop gripper stiffness [N/m] for a simulation running at 1 KHz.
    double m_maxGripperLinearStiffness;

    //! Maximum recommended linear damping factor Kv when using the getVelocity() method from the device class.
    double m_maxLinearDamping;

    //! Maximum recommended angular damping factor Kv when using the getAngularVelocity() method from the device class.
    double m_maxAngularDamping;

    //! Maximum recommended angular damping factor Kv when using the getGripperAngularVelocity() method from the device class.
    double m_maxGripperAngularDamping;

    //! Radius which describes the largest sphere (3D devices) or circle (2D Devices) which can be enclosed inside the physical workspace of the device.
    double m_workspaceRadius;

    //! Maximum open angle of the gripper [rad].
    double m_gripperMaxAngleRad;

    //! If __true__ then device supports position sensing (x,y,z axis), __false__ otherwise.
    bool m_sensedPosition;

    //! If __true__ then device supports rotation sensing. (i.e stylus, pen), __false__ otherwise.
    bool m_sensedRotation;

    //! If __true__ then device supports a sensed gripper interface, __false__ otherwise.
    bool m_sensedGripper;

    //! If __true__ then device provides actuation capabilities for translation degrees of freedom (x,y,z axis), __false__ otherwise.
    bool m_actuatedPosition;

    //! If __true__ then device provides actuation capabilities for orientation degrees of freedom (i.e stylus, wrist, pen), __false__ otherwise.
    bool m_actuatedRotation;

    //! If __true__ then device provides an actuated gripper, __false__ otherwise.
    bool m_actuatedGripper;

    //! If __true__ then the device can be used for left hands, __false__ otherwise.
    bool m_leftHand;

    //! If __true__ then the device can be used for right hands, __false__ otherwise.
    bool m_rightHand;
};


//==============================================================================
/*!
    \class      cGenericHapticDevice
    \ingroup    devices  

    \brief
    Abstract class for haptic and tracking devices.

    \details
    cGenericHapticDevice describes a virtual class from which all
    haptic devices are derived. 
*/
//==============================================================================
class cGenericHapticDevice : public cGenericDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericHapticDevice.
    cGenericHapticDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cGenericHapticDevice.
    virtual ~cGenericHapticDevice() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL COMMANDS:
    //--------------------------------------------------------------------------
 
public:

    //! Open connection to haptic device.
    virtual bool open() { return (C_ERROR); }

    //! Close connection to haptic device.
    virtual bool close() { return (C_ERROR); }

    //! Calibrate haptic device.
    virtual bool calibrate(bool a_forceCalibration = false) { return (m_deviceReady); }

    //! Read position of haptic device. Units are meters [m].
    virtual bool getPosition(cVector3d& a_position) { a_position.zero(); return (m_deviceReady); }

    //! Read linear velocity of haptic device. Units are meters per second [m/s].
    virtual bool getLinearVelocity(cVector3d& a_linearVelocity) { a_linearVelocity = m_linearVelocity; return (m_deviceReady); }

    //! Read orientation frame (3x3 matrix) of the haptic device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation) { a_rotation.identity(); return (m_deviceReady); }

    //! Read angular velocity of haptic device. Units are in radians per second [rad/s].
    virtual bool getAngularVelocity(cVector3d& a_angularVelocity) { a_angularVelocity = m_angularVelocity; return (m_deviceReady); }

    //! Read position and orientation of haptic device. Results is passed through a transformation matrix (4x4).
    virtual bool getTransform(cTransform& a_transform);

    //! Read gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! Read gripper angle in degrees [deg].
    inline bool getGripperAngleDeg(double& a_angle) { double angle; bool result = getGripperAngleRad(angle); a_angle = cRadToDeg(angle); return (result); }

    //! Read angular velocity of the gripper. Units are in radians per second [rad/s].
    virtual bool getGripperAngularVelocity(double& a_gripperAngularVelocity) { a_gripperAngularVelocity = m_gripperAngularVelocity; return (m_deviceReady); }

    //! Send force [N] to haptic device.
    virtual bool setForce(const cVector3d& a_force) { cSleepMs(1); return (m_deviceReady); }

    //! Send force [N] and torque [N*m] to haptic device.
    virtual bool setForceAndTorque(const cVector3d& a_force, const cVector3d& a_torque) { return (setForce(a_force)); }

    //! Send force [N], torque [N*m], and gripper force [N] to haptic device.
    virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce) { return (setForceAndTorque(a_force, a_torque)); }

    //! Read sensed force [N] from haptic device.
    virtual bool getForce(cVector3d& a_force) { a_force = m_prevForce; return (m_deviceReady); }

    //! Read sensed torque [N*m] from haptic device.
    virtual bool getTorque(cVector3d& a_torque) { a_torque = m_prevTorque; return (m_deviceReady); }

    //! Read sensed torque [N*m] from force gripper.
    virtual bool getGripperForce(double& a_gripperForce) { a_gripperForce = m_prevGripperForce; return (m_deviceReady); }

    //! Read status of user switch [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitch(int a_switchIndex, bool& a_status) { a_status = false; return (m_deviceReady); }

    //! Read technical specifications of haptic device.
    cHapticDeviceInfo getSpecifications() { return (m_specifications); }

    //! Enable or disable virtual gripper switch.
    virtual void setEnableGripperUserSwitch(const bool a_status) { m_gripperUserSwitchEnabled = a_status; }

    //! Return the status of the virtual gripper user switch. If __true__, then gripper is used to emulate a user switch. Return __false__ otherwise. 
    virtual bool getEnableGripperUserSwitch() const { return (m_gripperUserSwitchEnabled); }


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public: 

    //! Get number of haptic devices available for this class of devices.
    static unsigned int getNumDevices() { return (0); }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - GENERAL:
    //--------------------------------------------------------------------------

public:

    //! Technical specifications of haptic device.
    cHapticDeviceInfo m_specifications;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - CURRENT VALUES:
    //--------------------------------------------------------------------------

protected:

    //! Last force sent to haptic device.
    cVector3d m_prevForce;

    //! Last torque sent to haptic device.
    cVector3d m_prevTorque;

    //! Last gripper force sent to haptic device.
    double m_prevGripperForce;

    //! Last estimated linear velocity.
    cVector3d m_linearVelocity;

    //! Last estimated angular velocity.
    cVector3d m_angularVelocity;

    //! Last estimated gripper angular velocity.
    double m_gripperAngularVelocity;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - VELOCITY ESTIMATION:
    //--------------------------------------------------------------------------

protected:

    //! History position data of the device.
    cTimestampPos m_historyPos[C_DEVICE_HISTORY_SIZE];

    //! History orientation data of the device.
    cTimestampRot m_historyRot[C_DEVICE_HISTORY_SIZE];

    //! History position of device gripper.
    cTimestampValue m_historyGripper[C_DEVICE_HISTORY_SIZE];

    //! Current index position in history data table.
    int m_indexHistoryPos;

    //! Current index position in history data table.
    int m_indexHistoryRot;

    //! Current index position in history data table.
    int m_indexHistoryGripper;

    //! Last index position used to compute velocity.
    int m_indexHistoryPosWin;

    //! Last index position used to compute velocity.
    int m_indexHistoryRotWin;

    //! Last index position used to compute velocity.
    int m_indexHistoryGripperWin;

    //! Window time interval for measuring linear velocity.
    double m_linearVelocityWindowSize;

    //! Window time interval for measuring angular velocity.
    double m_angularVelocityWindowSize;

    //! Window time interval for measuring gripper velocity.
    double m_gripperVelocityWindowSize;

    //! General clock used to compute velocity signals.
    cPrecisionClock m_clockGeneral;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - GRIPPER USER SWITCH:
    //--------------------------------------------------------------------------

protected:

    //! If __true__ then virtual gripper user switch is enabled.
    bool m_gripperUserSwitchEnabled;

    //! Position of the gripper when the user encounters the virtual switch.
    double m_gripperUserSwitchAngleStart;

    //! Position of the gripper when the virtual switch is enabled and the "click" occurs.
    double m_gripperUserSwitchAngleClick;

    //! Maximum force level at the force gripper when the "click" occurs.
    double m_gripperUserSwitchForceClick;

    //! Force level when the gripper is completely closed after the "click" event has occurred.
    double m_gripperUserSwitchForceEngaged;

    //! Virtual gripper angle in radians [rad].
    double m_virtualGripperAngle;

    //! Virtual gripper minimum angle in radians [rad].
    double m_virtualGripperAngleMin;

    //! Virtual gripper minimum angle in radians [rad].
    double m_virtualGripperAngleMax;

    //! Virtual speed value used for simulating the opening and closing of the virtual gripper [rad/s].
    double m_virtualGripperAngularVelocity;

    //! Clock for computing the position of the virtual gripper [rad/s].
    cPrecisionClock m_virtualGripperClock;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - VELOCITY ESTIMATION:
    //--------------------------------------------------------------------------

protected:

    //! Estimate linear velocity of handle by passing the latest position.
    void estimateLinearVelocity(cVector3d& a_newPosition);

    //! Estimate angular velocity of handle by passing the latest orientation frame.
    void estimateAngularVelocity(cMatrix3d& a_newRotation);

    //! Estimate velocity of gripper by passing the latest gripper position.
    void estimateGripperVelocity(double a_newGripperPosition);


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - GRIPPER USER SWITCH:
    //--------------------------------------------------------------------------

protected:

    //! Computer virtual gripper force.
    double computeGripperUserSwitchForce(const double& a_gripperAngle,
                                         const double& a_gripperAngularVelocity);

    //! Read status of gripper user switch. Return __true__ if virtual user switch is engaged, __false_ otherwise.
    bool getGripperUserSwitch();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - DEVICE LIBRARY INITIALIZATION:
    //--------------------------------------------------------------------------

protected:

    //! Open libraries for this class of devices.
    static bool openLibraries() { return (C_SUCCESS); }

    //! Close libraries for this class of devices.
    static bool closeLibraries() { return (C_SUCCESS); }
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
