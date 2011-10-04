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
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericHapticDeviceH
#define CGenericHapticDeviceH
//---------------------------------------------------------------------------
#include "devices/CGenericDevice.h"
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "timers/CPrecisionClock.h"
//---------------------------------------------------------------------------

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
const int       CHAI_DEVICE_HISTORY_SIZE            = 200;      // [number of samples]

//! Minimum time between two devioce status acquisitions.
const double    CHAI_DEVICE_MIN_ACQUISITION_TIME    = 0.0001;   // [s]
//---------------------------------------------------------------------------

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
    //! Name of the device model. "delta, "omega" or "phantom" for instance.
    string m_modelName;

    //! Name of the manufacturer of the device.
    string m_manufacturerName;

    //! Maximum force in [N] that can be produced by the device in translation.
    double m_maxForce;

    //! Maximum torque in [N*m] that can be produced by the device in orientation.
    double m_maxTorque;

    //! Maximum force in [N*m] that can be produced by the gripper.
    double m_maxGripperTorque;

    //! Maximum closed loop force stiffness [N/m] for a simulation running at 1 KhZ.
    double m_maxForceStiffness;

    //! Maximum closed loop torque stiffness [N*m/rad] for a simulation running at 1 KhZ.
    double m_maxTorqueStiffness;

    //! Maximum closed loop gripper torque stiffness [N*m/rad] for a simulation running at 1 KhZ.
    double m_maxGripperTorqueStiffness;

    //! Maximum recommended linear damping factor Kv when using the getVelocity() method from the device class
    double m_maxLinearDamping;

    //! Radius which describes the largest sphere (3D devices) or circle (2D Devices) which can be enclosed inside the physical workspace of the device.
    double m_workspaceRadius;

    //! If \b true then device supports position sensing (x,y,z axis).
    bool m_sensedPosition;

    //! If \b true thhen device supports rotation sensing. (i.e stylus, pen).
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

    //! If \b true then the device can used for left hands.
    bool m_rightHand;
};


//===========================================================================
/*!
    \class      cGenericHapticDevice
    \ingroup    devices  

    \brief  
    cGenericHapticDevice describes a virtual class from which all
    2D or 3D point contact haptic devices are derived. These include
    for instance the delta.x, omega.x or Phantom haptic devices.
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
    virtual int open() { return -1; }

    //! Close connection to haptic device (0 indicates success).
    virtual int close() { return -1; }

    //! Initialize or calibrate haptic device (0 indicates success).
    virtual int initialize(const bool a_resetEncoders=false) { return -1; }

    //! Send a generic command to the haptic device (0 indicates success).
    virtual int command(int a_command, void* a_data);

    //! Read the position of the device. Units are meters [m].
    virtual int getPosition(cVector3d& a_position) { a_position.zero(); return (0); }

    //! Read the linear velocity of the device. Units are meters per second [m/s].
    virtual int getLinearVelocity(cVector3d& a_linearVelocity) { a_linearVelocity = m_linearVelocity; return (0); }

    //! Read the orientation frame of the device end-effector.
    virtual int getRotation(cMatrix3d& a_rotation) { a_rotation.identity(); return (0); }

    //! Read the angular velocity of the device. Units are in radians per second [m/s].
    virtual int getAngularVelocity(cVector3d& a_angularVelocity) { a_angularVelocity = m_angularVelocity; return (0); }

    //! Read the gripper angle in radian.
    virtual int getGripperAngleRad(double& a_angle) { a_angle = 0; return (0); }

    //! Read the angular velocity of the gripper. Units are in radians per second [m/s].
    virtual int getGripperVelocity(double& a_gripperVelocity) { a_gripperVelocity = m_gripperVelocity; return (0); }

    //! Send a force [N] to the haptic device.
    virtual int setForce(cVector3d& a_force) { return (0); }

    //! Read a sensed force [N] from the haptic device.
    virtual int getForce(cVector3d& a_force) { a_force = m_prevForce; return (0); }

    //! Send a torque [N*m] to the haptic device.
    virtual int setTorque(cVector3d& a_torque) { return (0); }

    //! Read a sensed torque [N*m] from the haptic device.
    virtual int getTorque(cVector3d& a_torque) { a_torque = m_prevTorque; return (0); }

    //! Send a torque [N*m] to the gripper.
    virtual int setGripperTorque(double a_gripperTorque) { return (0); }

    //! Read a sensed torque [N*m] from the gripper.
    virtual int getGripperTorque(double a_gripperTorque) { a_gripperTorque = m_prevGripperTorque; return (0); }

    //! Send a force [N], a torque [N*m] and a gripper torque [N*m] to the haptic device.
    virtual int setForceAndTorqueAndGripper(cVector3d& a_force, cVector3d& a_torque, double a_gripperTorque);

    //! read the status of the user switch [\b true = \b ON / \b false = \b OFF].
    virtual int getUserSwitch(int a_switchIndex, bool& a_status) { a_status = false; return (0); }

    //! Get the specifications of the current device.
    cHapticDeviceInfo getSpecifications() { return (m_specifications); }


  protected:

	//-----------------------------------------------------------------------
    // MEMBERS:
	//-----------------------------------------------------------------------

    //! Technical specifications of the current haptic device.
    cHapticDeviceInfo m_specifications;

    //! Previous sent force to the haptic device.
    cVector3d m_prevForce;

    //! Previous sent torque to the haptic device.
    cVector3d m_prevTorque;

    //! Previous sent gripper torque to the haptic device.
    double m_prevGripperTorque;

    //! Last estimated linear velocity.
    cVector3d m_linearVelocity;

    //! Last estimated angular velocity.
    cVector3d m_angularVelocity;

    //! Last estimated gripper velocity.
    double m_gripperVelocity;

    //! History position data of the device.
    cTimestampPos m_historyPos[CHAI_DEVICE_HISTORY_SIZE];

    //! History orientation data of the device.
    cTimestampRot m_historyRot[CHAI_DEVICE_HISTORY_SIZE];

    //! History position of the device gripper.
    cTimestampValue m_historyGripper[CHAI_DEVICE_HISTORY_SIZE];

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
    double m_gripperVelocityWindowSize;

    //! General clock when the device was started.
    cPrecisionClock m_clockGeneral;


	//-----------------------------------------------------------------------
    // METHODS:
	//-----------------------------------------------------------------------

    //! Estimate the linear velocity by passing the latest position.
    void estimateLinearVelocity(cVector3d& a_newPosition);

    //! Estimate the angular velocity by passing the latest orientation frame.
    void estimateAngularVelocity(cMatrix3d& a_newRotation);

    //! Estimate the velocity of the gripper by passing the latest gripper position.
    void estimateGripperVelocity(double a_newGripperPosition);
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
