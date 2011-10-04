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
#ifndef CFalconDeviceH
#define CFalconDeviceH
//---------------------------------------------------------------------------
#if defined(_ENABLE_FALCON_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CFalconDevice.h

    \brief
    <b> Devices </b> \n 
    Falcon Haptic Device.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cFalconDevice
    \ingroup    devices  

    \brief  
    cFalconDevice describes an interface to the Falcon haptic device
    from Novint Technlogies.
*/
//===========================================================================
class cFalconDevice : public cGenericHapticDevice
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cFalconDevice.
    cFalconDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cFalconDevice.
    virtual ~cFalconDevice();

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Open connection to haptic device (0 indicates success).
    int open();

    //! Close connection to haptic device (0 indicates success).
    int close();

    //! Initialize or calibrate haptic device (0 indicates success).
    int initialize(const bool a_resetEncoders=false);

    //! Returns the number of devices available from this class of device.
    unsigned int getNumDevices();

    //! Read the position of the device. Units are meters [m].
    int getPosition(cVector3d& a_position);

    //! Send a force [N] to the haptic device.
    int setForce(cVector3d& a_force);

    //! read the status of the user switch [\b true = \b ON / \b false = \b OFF].
    int getUserSwitch(int a_switchIndex, bool& a_status);


  private:

    //! counter.
    static int m_dllcount;

    //! Device ID number.
    int m_deviceID;

    //! Are the Falcon drivers installed and available.
    bool m_driverInstalled;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif //_ENABLE_FALCON_DEVICE_SUPPORT
//---------------------------------------------------------------------------
