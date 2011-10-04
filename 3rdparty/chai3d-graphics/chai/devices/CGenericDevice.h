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
    \version   2.0.0 $Rev: 250 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericDeviceH
#define CGenericDeviceH
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "devices/CCallback.h"
#include <string>
#include <stdio.h>
//---------------------------------------------------------------------------
using std::string;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGenericDevice.h

    \brief 
    <b> Devices </b> \n 
    Device Base Class.
*/
//===========================================================================

//---------------------------------------------------------------------------
// GENERIC DEVICE or BOARD:
//---------------------------------------------------------------------------
//! Purpose:    Query device to check if is operating correctly.
//! iData:      integer type value. 1 means device is ok
//!             0 means device is not ready.
const int   CHAI_CMD_GET_DEVICE_STATE   = 1010;

//---------------------------------------------------------------------------
// GENERIC I/O BOARDS:
//---------------------------------------------------------------------------
/*!
    \b Purpose:  Read the value of an encoder N.
    \b Data:     Integer type value.
*/
//! Reference to encoder 0.
const int   CHAI_CMD_GET_ENCODER_0      = 1020;

//! Reference to encoder 1.
const int   CHAI_CMD_GET_ENCODER_1      = 1021;

//! Reference to encoder 2.
const int   CHAI_CMD_GET_ENCODER_2      = 1022;

//! Reference to encoder 3.
const int   CHAI_CMD_GET_ENCODER_3      = 1023;

//! Reference to encoder 4.
const int   CHAI_CMD_GET_ENCODER_4      = 1024;

//! Reference to encoder 5.
const int   CHAI_CMD_GET_ENCODER_5      = 1025;

//! Reference to encoder 6.
const int   CHAI_CMD_GET_ENCODER_6      = 1026;

//! Reference to encoder 7.
const int   CHAI_CMD_GET_ENCODER_7      = 1027;


/*!
    \b Purpose:  Reset the value of an encoder N.
    \b Data:     Integer type value.
*/
//! Reference to reset signal encoder 0.
const int   CHAI_CMD_RESET_ENCODER_0    = 1040;

//! Reference to reset signal encoder 1.
const int   CHAI_CMD_RESET_ENCODER_1    = 1041;

//! Reference to reset signal encoder 2.
const int   CHAI_CMD_RESET_ENCODER_2    = 1042;

//! Reference to reset signal encoder 3.
const int   CHAI_CMD_RESET_ENCODER_3    = 1043;

//! Reference to reset signal encoder 4.
const int   CHAI_CMD_RESET_ENCODER_4    = 1044;

//! Reference to reset signal encoder 5.
const int   CHAI_CMD_RESET_ENCODER_5    = 1045;

//! Reference to reset signal encoder 6.
const int   CHAI_CMD_RESET_ENCODER_6    = 1046;

//! Reference to reset signal encoder 7.
const int   CHAI_CMD_RESET_ENCODER_7    = 1047;


/*!
    \b Purpose:  Set value to a DAC.
    \b Data:     Integer type value.
*/

//! Reference to DAC 0.
const int   CHAI_CMD_SET_DAC_0          = 1030;

//! Reference to DAC 1.
const int   CHAI_CMD_SET_DAC_1          = 1031;

//! Reference to DAC 2.
const int   CHAI_CMD_SET_DAC_2          = 1032;

//! Reference to DAC 3.
const int   CHAI_CMD_SET_DAC_3          = 1033;

//! Reference to DAC 4.
const int   CHAI_CMD_SET_DAC_4          = 1034;

//! Reference to DAC 5.
const int   CHAI_CMD_SET_DAC_5          = 1035;

//! Reference to DAC 6.
const int   CHAI_CMD_SET_DAC_6          = 1036;

//! Reference to DAC 7.
const int   CHAI_CMD_SET_DAC_7          = 1037;

//---------------------------------------------------------------------------
// GENERIC POINT CONTACT 3/6 DOF HAPTIC DEVICES:
//---------------------------------------------------------------------------

/*!
    \b Purpose:  Read position (px, py, pz) in meters [m] of 3d point contact device. \n
    \b Data:     cVector3d type value.
*/
const int   CHAI_CMD_GET_POS_3D         = 2000;

/*!
    \b Purpose: Read normalized position (px, py, pz) of 3d point contact device.
                typically the value of each component of the vector position will
                be included in the interval [-1,1], accounting for the maximum
                usable workspace of the device. \n
    \b Data:    cVector3d type value.
*/
const int   CHAI_CMD_GET_POS_NORM_3D    = 2001;

/*!
    \b Purpose: Read velocity (vx, vy, vz) of 3d point contact device in [m/s]. \n
    \b Data:    cVector3d type value.
*/
const int   CHAI_CMD_GET_VEL_3D         = 2002;

/*!
    \b Purpose: Set a force (fx, fy, fz) to a 3d point contact device (in Newtons [N]).\n
    \b Data:    cVector3d type value.
*/
const int   CHAI_CMD_SET_FORCE_3D       = 2010;

/*!
    \b Purpose: Set a normalized force (fx, fy, fz) to a 3d point contact device.
                A normalized force has a maximum length of 1.0 corresponding
                to the highest force that the device can generate.\n
    \b Data:    cVector3d type value.
*/
const int   CHAI_CMD_SET_FORCE_NORM_3D  = 2011;

/*!
    \b Purpose: Set a force (fx, fy, fz) and a torque (tx, ty, tz) to a 6d point contact device.\n
    \b Data:    array of 2 cVector3d type value.  Units are [N] and [N*mm].
*/
const int   CHAI_CMD_SET_FORCE_TORQUE_3D       = 2012;

/*!
    \b Purpose: Read orientation angles (ax, ay, az) of a 3d wrist or stylus.\n
    \b Data:    cVector3d type value.
*/
const int   CHAI_CMD_GET_ROT_ANGLES     = 2020;

/*!
    \b Purpose: Read orientation matrix of a 3d wrist or stylus.\n
    \b Data:    cMatrix3d type value.
*/
const int   CHAI_CMD_GET_ROT_MATRIX     = 2021;

/*!
    \b Purpose: Set a torque (tx, ty, tz) to a 3d wrist or stylus.\n
    \b Data:    cVector3d type value.  Units are N*mm.
*/
const int   CHAI_CMD_SET_TORQUE_3D      = 2030;

/*!
    \b Purpose: Read status of user switch 0.\n
    \b Data:    Integer type value.
*/
const int   CHAI_CMD_GET_SWITCH_0       = 2041;

/*!
    \b Purpose: Read status of user switch 1.\n
    \b Data:    Integer type value.
*/
const int   CHAI_CMD_GET_SWITCH_1       = 2042;

/*!
    \b Purpose: Read status of user switch 2.\n
    \b Data:    Integer type value.
*/
const int   CHAI_CMD_GET_SWITCH_2       = 2043;

/*!
    \b Purpose: Reads all switches into a bit mask with bit 0 = button 0, etc.\n
    \b Data:    Integer type value.
*/
const int   CHAI_CMD_GET_SWITCH_MASK       = 2044;

/*!
    \b Purpose: Get the scale factor from normalized coordinates to mm\n
    \b Data:    double scale factor... mm = scale * normalized_coords
*/
const int   CHAI_CMD_GET_NORMALIZED_SCALE_FACTOR = 2045;


//===========================================================================
/*!
    \brief  The following constants define the possible return values
            of the method cGenericDevice:: command().
*/
//===========================================================================

//! Error message - no error occurred.
const int   CHAI_MSG_OK                 =   0;

//! Error message - and error has occurred.
const int   CHAI_MSG_ERROR              =  -1;

//! Error message - this command is not supported.
const int   CHAI_MSG_NOT_IMPLEMENTED    =  -2;

//! Error message - system is not ready.
const int   CHAI_MSG_SYSTEM_NOT_READY   =  -3;


//===========================================================================
/*!
    \class      cGenericDevice
    \ingroup    devices  

    \brief  
    cGenericDevice Provides an general interface to communicate with hardware 
    devices. A number of constants define a set of generic commands supported 
    by the cGenericDevice:command method. For each generic command, we describe 
    the data type and information that must be passed by parameter for 
    \e index and \e data. \e command contains of course the command number 
    corresponding to the following list of command constants.

*/
//===========================================================================
class cGenericDevice
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGenericDevice.
    cGenericDevice();

    //! Destructor of cGenericDevice.
    virtual ~cGenericDevice() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Open connection to device (0 indicates success).
    virtual int open() { return -1; }

    //! Close connection to device (0 indicates success).
    virtual int close() { return -1; }

    //! Initialize or calibrate device (0 indicates success).
    virtual int initialize(const bool a_resetEncoders=false) { return -1; }

    //! Send a command to the device (0 indicates success).
    virtual int command(int a_command, void* a_data) { return (CHAI_MSG_NOT_IMPLEMENTED); }

    //! Returns the number of devices available from this class of device.
    virtual unsigned int getNumDevices() { return (0); }

    //! Returns true if the device is available for communication.
    bool isSystemAvailable() { return (m_systemAvailable); }

    //! Returns true if the connection to the device has been created.
    bool isSystemReady() { return (m_systemReady); }

    //! Ask the device to call me back periodically.
    virtual bool setCallback(cCallback* a_callback);

  protected:
    //! Flag that indicates if the hardware device is available to the computer.
    bool m_systemAvailable;

    //! Flag that indicates if connection to system was opened successfully.
    bool m_systemReady;

    //! A callback method for this device (or zero if none has been registered).
    cCallback* m_callback;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
