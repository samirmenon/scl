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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 799 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericDeviceH
#define CGenericDeviceH
//---------------------------------------------------------------------------
#include "system/CGlobals.h"
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

//===========================================================================
/*!
    \class      cGenericDevice
    \ingroup    devices  

    \brief  
    cGenericDevice Provides an general interface to communicate with hardware 
    devices. 

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
    virtual int calibrate() { return -1; }

    //! Returns the number of devices available from this class of device.
    virtual unsigned int getNumDevices() { return (0); }

    //! Returns true if the device is available for communication.
    bool isDeviceAvailable() { return (m_deviceAvailable); }

    //! Returns true if the connection to the device has been created.
    bool isDeviceReady() { return (m_deviceReady); }

    //! Ask the device to call me back periodically.
    virtual bool setCallback(cCallback* a_callback);

  protected:
    //! Flag that indicates if the hardware device is available to the computer.
    bool m_deviceAvailable;

    //! Flag that indicates if connection to device was opened successfully.
    bool m_deviceReady;

    //! A callback method for this device (or zero if none has been registered).
    cCallback* m_callback;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
