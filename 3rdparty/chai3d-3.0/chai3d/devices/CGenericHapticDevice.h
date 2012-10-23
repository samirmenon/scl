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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 839 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericHapticDeviceH
#define CGenericHapticDeviceH
//---------------------------------------------------------------------------
#include "devices/CGenericDevice.h"
#include "math/CMaths.h"
#include "timers/CPrecisionClock.h"
//---------------------------------------------------------------------------
#include <string>
//===========================================================================
/*!
    \file       CGenericHapticDevice.h

    \brief
    <b> Devices </b> \n 
    Haptic Device Base Class.
*/
//===========================================================================

//---------------------------------------------------------------------------
//! Filter property used for velocity estimator. 
const int       C_DEVICE_HISTORY_SIZE            = 200;      // [number of samples]

//! Minimum time between two devioce status acquisitions.
const double    C_DEVICE_MIN_ACQUISITION_TIME    = 0.0001;   // [s]
//---------------------------------------------------------------------------

//! Haptic device models.
enum cHapticDeviceModel
{
    C_HAPTIC_DEVICE_VIRTUAL,
    C_HAPTIC_DEVICE_DELTA_3,
    C_HAPTIC_DEVICE_DELTA_6,
    C_HAPTIC_DEVICE_OMEGA_3,
    C_HAPTIC_DEVICE_OMEGA_6,
    C_HAPTIC_DEVICE_OMEGA_7,
    C_HAPTIC_DEVICE_SIGMA_7,
    C_HAPTIC_DEVICE_FALCON,
    C_HAPTIC_DEVICE_PHANTOM_OMNI,
    C_HAPTIC_DEVICE_PHANTOM_15_6DOF,
    C_HAPTIC_DEVICE_PHANTOM_OTHER,
    C_HAPTIC_DEVICE_CUSTOM,
    C_HAPTIC_DEVICE_BFR
};

//===========================================================================
/*!
    \struct     cTimestampValue
    \ingroup    devices  

    \brief
    Provides a structure to store a double value with a time
    stamp.
*/
//===========================================================================
struct cTimestampValue
{
    //! Time when the following data was acquired
    double m_time;

    //! Gripper position
    double m_value;
};


//===========================================================================
/*!
    \struct     cTimestampPos
    \ingroup    devices 
    
    \brief
    Provides a structure to store a position with a time stamp.
*/
//===========================================================================
struct cTimestampPos
{
    //! Time when the following data was acquired
    double m_time;

    //! Position information
    cVector3d m_pos;
};


//===========================================================================
/*!
    \struct     cTimestampRot
    \ingroup    devices  

    \brief
    Provides a structure to store an orientation frame with a time stamp.
*/
//===========================================================================
struct cTimestampRot
{
    //! Time when the following data was acquired
    double m_time;

    //! Rotation information
    cMatrix3d m_rot;
};


//===========================================================================
/*!
    \struct     cHapticDeviceInfo
    \ingroup    devices  

    \brief
    Provides a structure which can hold technical specifications about a
    particular haptic device.
*/
//===========================================================================
struct cHapticDeviceInfo
{
    //! Haptic device model.
    cHapticDeviceModel m_model;

    //! Name of the device model. `delta`, `omega` or `phantom` for instance.
    std::string m_modelName;

    //! Name of the manufacturer of the device.
    std::string m_manufacturerName;

    //! Maximum force in [N] that can be produced by the device in translation.
    double m_maxLinearForce;

    //! Maximum torque in [N*m] that can be produced by the device in orientation.
    double m_maxAngularTorque;

    //! Maximum force in [N] that can be produced by the gripper.
    double m_maxGripperForce;

    //! Maximum closed loop linear stiffness [N/m] for a simulation running at 1 KhZ.
    double m_maxLinearStiffness;

    //! Maximum closed loop angular stiffness [N*m/rad] for a simulation running at 1 KhZ.
    double m_maxAngularStiffness;

    //! Maximum closed loop gripper stiffness [N*m] for a simulation running at 1 KhZ.
    double m_maxGripperLinearStiffness;

    //! Maximum recommended linear damping factor Kv when using the getVelocity() method from the device class.
    double m_maxLinearDamping;

    //! Maximum recommended angular damping factor Kv when using the getAngularVelocity() method from the device class.
    double m_maxAngularDamping;

    //! Maximum recommended angular damping factor Kv when using the getGripperAngularVelocity() method from the device class.
    double m_maxGripperAngularDamping;

    //! Radius which describes the largest sphere (3D devices) or circle (2D Devices) which can be enclosed inside the physical workspace of the device.
    double m_workspaceRadius;

    //! If \b true then device supports position sensing (x,y,z axis).
    bool m_sensedPosition;

    //! If \b true then device supports rotation sensing. (i.e stylus, pen).
    bool m_sensedRotation;

    //! If \b true then device supports a sensed gripper interface.
    bool m_sensedGripper;

    //! If \b true then device provides actuation capabilities on the translation degrees of freedom. (x,y,z axis).
    bool m_actuatedPosition;

    //! If \b true then device provides actuation capabilities on the orientation degrees of freedom. (i.e stylus, pen).
    bool m_actuatedRotation;

    //! If \b true then device provides actuation capabilities on the gripper.
    bool m_actuatedGripper;

    //! If \b true then the device can used for left hands.
    bool m_leftHand;

    //! If \b true then the device can used for right hands.
    bool m_rightHand;

    //! Position offset between origin returned by the haptic device and origin of the physiqual workspace.
    cVector3d m_positionOffset;
};


//===========================================================================
/*!
    \class      cGenericHapticDevice
    \ingroup    devices  

    \brief  
    cGenericHapticDevice describes a virtual class from which all
    2D or 3D point contact haptic devices are derived. These include
    for instance the delta(0) , omega(0)  or Phantom haptic devices.
*/
//===========================================================================
class cGenericHapticDevice : public cGenericDevice
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGenericHapticDevice.
    cGenericHapticDevice();

    //! Destructor of cGenericHapticDevice.
    virtual ~cGenericHapticDevice() {};


    //-----------------------------------------------------------------------
    // METHODS - GENERAL COMMANDS:
    //-----------------------------------------------------------------------
    //! Open connection to haptic device (0 indicates success).
    virtual int open() { return (-1); }

    //! Close connection to haptic device (0 indicates success).
    virtual int close() { return (-1); }

    //! Calibrate haptic device (0 indicates success).
    virtual int calibrate() { return (-1); }

    //! Read the position of the device. Units are meters [m].
    virtual int getPosition(cVector3d& a_position) { a_position.zero(); return (0); }

    //! Read the linear velocity of the device. Units are meters per second [m/s].
    virtual int getLinearVelocity(cVector3d& a_linearVelocity) { a_linearVelocity = m_linearVelocity; return (0); }

    //! Read the orientation frame of the device end-effector.
    virtual int getRotation(cMatrix3d& a_rotation) { a_rotation.identity(); return (0); }

    //! Read the angular velocity of the device. Units are in radians per second [m/s].
    virtual int getAngularVelocity(cVector3d& a_angularVelocity) { a_angularVelocity = m_angularVelocity; return (0); }

    //! Read the position and orientation of the device through a transformation matrix.
    virtual int getTransform(cTransform& a_transform);

    //! Read the gripper angle in radian.
    virtual int getGripperAngleRad(double& a_angle);

    //! Read the gripper angle in degrees.
    inline int getGripperAngleDeg(double& a_angle) { double angle; int result = getGripperAngleRad(angle); a_angle = cRadToDeg(angle); return (result); }

    //! Read the angular velocity of the gripper. Units are in radians per second [m/s].
    virtual int getGripperAngularVelocity(double& a_gripperAngularVelocity) { a_gripperAngularVelocity = m_gripperAngularVelocity; return (0); }

    //! Send a force [N] to the haptic device.
    virtual int setForce(const cVector3d& a_force) { cSleepMs(1); return (0); }

    //! Send a a force [N] and a torque [N*m] to the haptic device.
    virtual int setForceAndTorque(const cVector3d& a_force, const cVector3d& a_torque) { return (setForce(a_force)); }

    //! Send a torque [N*m] to the gripper.
    virtual int setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce) { return (setForceAndTorque(a_force, a_torque)); }

    //! Read a sensed force [N] from the haptic device.
    virtual int getForce(cVector3d& a_force) { a_force = m_prevForce; return (0); }

    //! Read a sensed torque [N*m] from the haptic device.
    virtual int getTorque(cVector3d& a_torque) { a_torque = m_prevTorque; return (0); }

    //! Read a sensed torque [N*m] from the gripper.
    virtual int getGripperForce(double& a_gripperForce) { a_gripperForce = m_prevGripperForce; return (0); }

    //! Read the status of the user switch [\b true = \b ON / \b false = \b OFF].
    virtual int getUserSwitch(int a_switchIndex, bool& a_status) { a_status = false; return (0); }

    //! Get the specifications of the current device.
    cHapticDeviceInfo getSpecifications() { return (m_specifications); }

	//! Enable or Disable virtual gripper button.
	virtual void setEnableGripperUserSwitch(const bool a_status) { m_gripperUserSwitchEnabled = a_status; }

	//! Returns the status of the virtual gripper button.
	virtual bool getEnableGripperUserSwitch() const { return (m_gripperUserSwitchEnabled); }


    //-----------------------------------------------------------------------
    // MEMBERS - GENERAL:
    //-----------------------------------------------------------------------

	//! Technical specifications of the current haptic device.
    cHapticDeviceInfo m_specifications;


  protected:

    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Previous sent force to the haptic device.
    cVector3d m_prevForce;

    //! Previous sent torque to the haptic device.
    cVector3d m_prevTorque;

    //! Previous sent gripper torque to the haptic device.
    double m_prevGripperForce;

    //! Last estimated linear velocity.
    cVector3d m_linearVelocity;

    //! Last estimated angular velocity.
    cVector3d m_angularVelocity;

    //! Last estimated gripper angular velocity.
    double m_gripperAngularVelocity;

    //! History position data of the device.
    cTimestampPos m_historyPos[C_DEVICE_HISTORY_SIZE];

    //! History orientation data of the device.
    cTimestampRot m_historyRot[C_DEVICE_HISTORY_SIZE];

    //! History position of the device gripper.
    cTimestampValue m_historyGripper[C_DEVICE_HISTORY_SIZE];

    //! Current index position in History data table.
    int m_indexHistoryPos;

    //! Current index position in History data table.
    int m_indexHistoryRot;

    //! Current index position in History data table.
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
    double m_gripperLinearVelocityWindowSize;

    //! General clock when the device was started.
    cPrecisionClock m_clockGeneral;

	//! If \b true then virtual gripper user switch is activated.
	bool m_gripperUserSwitchEnabled;

	//! Position of the gripper when the user begins to touch the virtual switch.
	double m_gripperUserSwitchAngleStart;

	//! Position of the gripper when the virtual switch is enabled. (Click).
	double m_gripperUserSwitchAngleClick;

	//! Maximum force level of the switch when the "click" occurs.
	double m_gripperUserSwitchForceClick;

	//! Force level when the gripper is completely closed after the "click" event has occured.
	double m_gripperUserSwitchForceEngaged;

	//! Virtual gripper angle in radians.
	double m_virtualGripperAngle;

	//! Virtual gripper minimum angle in radians. [rad]
	double m_virtualGripperAngleMin;

	//! Virtual gripper minimum angle in radians. [rad]
	double m_virtualGripperAngleMax;

	//! Speed for opening and closing the virtual gripper. [rad/s]
	double m_virtualGripperAngularVelocity;

	//! Clock for computing the position of the virtual gripper. [rad/s]
	cPrecisionClock m_virtualGripperClock;


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Estimate the linear velocity by passing the latest position.
    void estimateLinearVelocity(cVector3d& a_newPosition);

    //! Estimate the angular velocity by passing the latest orientation frame.
    void estimateAngularVelocity(cMatrix3d& a_newRotation);

    //! Estimate the velocity of the gripper by passing the latest gripper position.
    void estimateGripperVelocity(double a_newGripperPosition);

	//! Computer virtual gripper force.
	double computeGripperUserSwitchForce(const double& a_gripperAngle,
										 const double& a_gripperAngularVelocity);

	//! Read status of gripper user switch.
	bool getGripperUserSwitch();
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
