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
    \version   2.0.0 $Rev: 221 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cGenericHapticDevice.
    Initialize basic parameters of generic haptic device.

    \fn     cGenericHapticDevice::cGenericHapticDevice()
*/
//===========================================================================
cGenericHapticDevice::cGenericHapticDevice() : cGenericDevice()
{
    m_specifications.m_manufacturerName              = "not defined";
    m_specifications.m_modelName                     = "not defined";
    m_specifications.m_maxForce                      = 0.1; // [N]
    m_specifications.m_maxForceStiffness             = 0.1; // [N/m]
    m_specifications.m_maxTorque                     = 0.1; // [N*m]
    m_specifications.m_maxTorqueStiffness            = 0.1; // [N*m/Rad]
    m_specifications.m_maxGripperTorque              = 0.1; // [N]
    m_specifications.m_maxGripperTorqueStiffness     = 0.1; // [N*m/m]
    m_specifications.m_maxLinearDamping              = 0.0; // [N/(m/s)]
    m_specifications.m_workspaceRadius               = 0.1; // [m]
    m_specifications.m_sensedPosition                = false;
    m_specifications.m_sensedRotation                = false;
    m_specifications.m_sensedGripper                 = false;
    m_specifications.m_actuatedPosition              = false;
    m_specifications.m_actuatedRotation              = false;
    m_specifications.m_actuatedGripper               = false;
    m_specifications.m_leftHand                      = false;
    m_specifications.m_rightHand                     = false;

    m_prevForce.zero();
    m_prevTorque.zero();
    m_prevGripperTorque = 0.0;

    m_angularVelocity.zero();
    m_gripperVelocity = 0.0;

    // start the general clock of the device
    m_clockGeneral.reset();
    m_clockGeneral.start();

    // reset history tables
    double time = m_clockGeneral.getCurrentTimeSeconds();

    m_indexHistoryPos       = 0;
    m_indexHistoryPosWin    = CHAI_DEVICE_HISTORY_SIZE-1;
    for (int i=0; i<CHAI_DEVICE_HISTORY_SIZE; i++)
    {
        m_historyPos[i].m_pos.zero();
        m_historyPos[i].m_time = time;
    }

    m_indexHistoryRot       = 0;
    m_indexHistoryPosWin    = CHAI_DEVICE_HISTORY_SIZE-1;
    for (int i=0; i<CHAI_DEVICE_HISTORY_SIZE; i++)
    {
        m_historyRot[i].m_rot.identity();
        m_historyRot[i].m_time = time;
    }

    m_indexHistoryGripper   = 0;
    m_indexHistoryPosWin    = CHAI_DEVICE_HISTORY_SIZE-1;
    for (int i=0; i<CHAI_DEVICE_HISTORY_SIZE; i++)
    {
        m_historyGripper[i].m_value = 0.0;
        m_historyGripper[i].m_time = time;
    }

    // Window time interval for measuring linear velocity
    m_linearVelocityWindowSize  = 0.015; // [s]

    // Window time interval for measuring angular velocity
    m_angularVelocityWindowSize  = 0.015; // [s]

    // Window time interval for measuring gripper velocity
    m_gripperVelocityWindowSize  = 0.015; // [s]
}


//===========================================================================
/*!
    Set command for the haptic device

    \fn         int cGenericHapticDevice::command(int a_command, void* a_data)
    \param      a_command  Selected command.
    \param      a_data  Pointer to the corresponding data structure.
    \return     Return status of command.
*/
//===========================================================================
int cGenericHapticDevice::command(int a_command, void* a_data)
{
    // temp variables
    int result;

    // check if the device is open
    if (m_systemReady)
    {
        switch (a_command)
        {
            // read position of end-effector
            case CHAI_CMD_GET_POS_3D:
            {
                cVector3d temp;
                result = getPosition(temp);
                cVector3d* position = (cVector3d *) a_data;
                *position = temp;
            }
            break;

            // read normalized position of end-effector
            case CHAI_CMD_GET_POS_NORM_3D:
            {
                cVector3d temp;
                result = getPosition(temp);
                temp.div(m_specifications.m_workspaceRadius);
                cVector3d* position = (cVector3d *) a_data;
                *position = temp;
            }
            break;

            // read orientation of end-effector
            case CHAI_CMD_GET_ROT_MATRIX:
            {
                cMatrix3d temp;
                result = getRotation(temp);
                cMatrix3d* rotation = (cMatrix3d *) a_data;
                *rotation = temp;
            }
            break;

            // set force to end-effector
            case CHAI_CMD_SET_FORCE_3D:
            {
                cVector3d* force = (cVector3d *) a_data;
                result = setForce(*force);
            }
            break;

            // set torque to end-effector
            case CHAI_CMD_SET_TORQUE_3D:
            {
                cVector3d* torque = (cVector3d *) a_data;
                result = setTorque(*torque);
            }
            break;

            // read user switch from end-effector
            case CHAI_CMD_GET_SWITCH_0:
            {
                bool* status = (bool *) a_data;
                bool temp;
                result = getUserSwitch(0, temp);
                *status = temp;
            }
            break;

            // read user switch from end-effector
            case CHAI_CMD_GET_SWITCH_MASK:
            {
                // Force the result to be 0 or 1, since bit 0 should carry button 0's value
                bool* status = (bool *) a_data;
                bool temp;
                result = getUserSwitch(0, temp);
                *status = temp;
            }
            break;

            // function is not implemented
            default:
                result = CHAI_MSG_NOT_IMPLEMENTED;
        }
    }
    else
    {
        result = CHAI_MSG_SYSTEM_NOT_READY;
    }
    return (result);
}


//===========================================================================
/*!
    Send a force [N] and a torque [N*m] and gripper force [N] to the
    haptic device.

    \fn     int cGenericHapticDevice::setForceAndTorqueAndGripper(cVector3d& a_force,
            cVector3d& a_torque, double a_gripperTorque)
    \param  a_force  Force command.
    \param  a_torque  Torque command.
    \param  a_gripperTorque  Gripper torque command.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cGenericHapticDevice::setForceAndTorqueAndGripper(cVector3d& a_force, cVector3d& a_torque,
                  double a_gripperTorque)
{
    int error0, error1, error2;

    // send force command
    error0 = setForce(a_force);

    // send torque command
    error1 = setTorque(a_force);

    // send gripper command
    error2 = setGripperTorque(a_gripperTorque);

    // return status
    if ((error0 != 0) || (error1 != 0) || (error2 != 0))
    {
        // an error has occurred
        return (-1);
    }
    else
    {
        // success
        return (0);
    }
}


//===========================================================================
/*!
    Estimate the linear velocity by passing the latest position.

    \fn     void cGenericHapticDevice::estimateLinearVelocity(cVector3d& a_newPosition)
    \param  a_newPosition  New position of the device.
    \return Return 0 if no error occurred.
*/
//===========================================================================
void cGenericHapticDevice::estimateLinearVelocity(cVector3d& a_newPosition)
{
    // get current time
    double time = m_clockGeneral.getCurrentTimeSeconds();

    // check the time interval between the current and previous sample
    if ((time - m_historyPos[m_indexHistoryPos].m_time) < CHAI_DEVICE_MIN_ACQUISITION_TIME)
    {
        return;
    }

    // store new value
    m_indexHistoryPos = (m_indexHistoryPos + 1) % CHAI_DEVICE_HISTORY_SIZE;
    m_historyPos[m_indexHistoryPos].m_time = time;
    m_historyPos[m_indexHistoryPos].m_pos  = a_newPosition;

    // search table to find a sample that occurred before current time
    // minus time window interval
    int i=0;
    bool completed = false;
    while ((i < CHAI_DEVICE_HISTORY_SIZE) && (!completed))
    {
        double interval = time - m_historyPos[m_indexHistoryPosWin].m_time;
        if ((interval < m_linearVelocityWindowSize) || (i == (CHAI_DEVICE_HISTORY_SIZE-1)))
        {
            // compute result
            cVector3d result;
            m_historyPos[m_indexHistoryPos].m_pos.subr(m_historyPos[m_indexHistoryPosWin].m_pos, result);
            if (interval > 0)
            {
                result.divr(interval, m_linearVelocity);
                completed = true;
            }
            else
            {
                completed = true;
            }
        }
        else
        {
            m_indexHistoryPosWin = (m_indexHistoryPosWin + 1) % CHAI_DEVICE_HISTORY_SIZE;
        }
    }
}


//===========================================================================
/*!
    Estimate the angular velocity by passing the latest orientation frame.

    \fn     void cGenericHapticDevice::estimateAngularVelocity(cMatrix3d& a_newRotation)
    \param  a_newRotation  New orientation frame of the device.
*/
//===========================================================================
void cGenericHapticDevice::estimateAngularVelocity(cMatrix3d& a_newRotation)
{
    // TODO: TO BE COMPLETED!
    m_angularVelocity.zero();
}


//===========================================================================
/*!
    Estimate the velocity of the gripper by passing the latest gripper position.

    \fn     void cGenericHapticDevice::estimateGripperVelocity(double a_newGripperPosition)
    \param  a_newGripperPosition  New position of the gripper.
*/
//===========================================================================
void cGenericHapticDevice::estimateGripperVelocity(double a_newGripperPosition)
{
    // get current time
    double time = m_clockGeneral.getCurrentTimeSeconds();

    // check the time interval between the current and previous sample
    if ((time - m_historyGripper[m_indexHistoryGripper].m_time) < CHAI_DEVICE_MIN_ACQUISITION_TIME)
    {
        return;
    }

    // store new value
    m_indexHistoryGripper = (m_indexHistoryGripper + 1) % CHAI_DEVICE_HISTORY_SIZE;
    m_historyGripper[m_indexHistoryGripper].m_time  = time;
    m_historyGripper[m_indexHistoryGripper].m_value = a_newGripperPosition;

    // search table to find a sample that occurred before current time
    // minus time window interval
    int i=0;
    bool completed = false;
    while ((i < CHAI_DEVICE_HISTORY_SIZE) && (!completed))
    {
        double interval = time - m_historyGripper[m_indexHistoryGripperWin].m_time;
        if ((interval < m_gripperVelocityWindowSize) || (i == (CHAI_DEVICE_HISTORY_SIZE-1)))
        {
            // compute result
            if (interval > 0)
            {
                m_gripperVelocity = (m_historyGripper[m_indexHistoryGripper].m_value - 
                                     m_historyGripper[m_indexHistoryGripperWin].m_value) / interval;
                completed = true;
            }
            else
            {
                completed = true;
            }
        }
        else
        {
            m_indexHistoryGripperWin = (m_indexHistoryGripperWin + 1) % CHAI_DEVICE_HISTORY_SIZE;
        }
    }
}

