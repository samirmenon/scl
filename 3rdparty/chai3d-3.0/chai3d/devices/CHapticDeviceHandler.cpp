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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 819 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "devices/CHapticDeviceHandler.h"
//---------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#include <process.h>
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cHapticDeviceHandler.
*/
//===========================================================================
cHapticDeviceHandler::cHapticDeviceHandler()
{
    // clear number of devices
    m_numDevices = 0;

    // create a null haptic device. a pointer to this device is returned
    // if no device is found. this insures that applications which forget
    // to address the case when no device is connected start sending commands
    // to a NULL pointer...
    m_nullHapticDevice = new cGenericHapticDevice();

    // clear device table
    unsigned int i;
    for (i=0; i<C_MAX_HAPTIC_DEVICES; i++)
    {
        m_devices[i] = NULL;
    }

    // search for available haptic devices
    update();
}


//===========================================================================
/*!
    Destructor of cHapticDeviceHandler.
*/
//===========================================================================
cHapticDeviceHandler::~cHapticDeviceHandler()
{
    // clear current list of devices
    int unsigned i;
    for (i=0; i<C_MAX_HAPTIC_DEVICES; i++)
    {
        if (m_devices[i] != NULL)
        {
            delete m_devices[i];
        }
        m_devices[i] = NULL;
    }
}


//===========================================================================
/*!
    Updates information regarding the devices that are connected to 
    your computer.
*/
//===========================================================================
void cHapticDeviceHandler::update()
{
    // temp variables
    int index, count;
    cGenericHapticDevice* device;

    // clear current list of devices
    m_numDevices = 0;
    for (unsigned int i=0; i<C_MAX_HAPTIC_DEVICES; i++)
    {
        if (m_devices[i] != NULL)
        {
            delete m_devices[i];
        }
        m_devices[i] = NULL;
    }

    //-----------------------------------------------------------------------
    // search for Force Dimension devices
    //-----------------------------------------------------------------------
    #if defined(C_ENABLE_DELTA_DEVICE_SUPPORT)

    // reset index number
    index = 0;

    // check for how many devices of this type that are available
    // and store the first one if available
    device = new cDeltaDevice(index);
    count = device->getNumDevices();
    if (count > 0)
    {
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }
    else
    {
        delete device;
    }

    //  open all remaining devices
    for (int i=1; i<count; i++)
    {
        index++;
        device = new cDeltaDevice(index);
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }

    #endif

    //-----------------------------------------------------------------------
    // search for Novint Falcon device
    //-----------------------------------------------------------------------
    #if defined(C_ENABLE_FALCON_DEVICE_SUPPORT)

    // reset index number
    index = 0;

    // create a first device of this class
    device = new cFalconDevice();

    // check for how many devices of this type that are available
    count = device->getNumDevices();

    // if there are one or more devices available, then store them in the device table
    if (count > 0)
    {
        // store first device
        m_devices[m_numDevices] = device;
        device->open();
        m_numDevices++;

        // search for other devices
        if (count > 1)
        {
            for (int i=1; i<count; i++)
            {
                index++;
                device = new cFalconDevice(index);
                device->open();
                m_devices[m_numDevices] = device;
                m_numDevices++;
            }
        }
    }
    else
    {
        delete device;
    }

    #endif


    //-----------------------------------------------------------------------
    // search for Sensable Technologies devices
    //-----------------------------------------------------------------------
    #if defined(C_ENABLE_PHANTOM_DEVICE_SUPPORT)

    // reset index number
    index = 0;

    // create a first device of this class
    device = new cPhantomDevice(index);

    // check for how many devices of this type that are available
    count = device->getNumDevices();

    // if there are one or more devices available, then store them in the device table
    if (count > 0)
    {
        // store first device
        m_devices[m_numDevices] = device;
        m_numDevices++;

        // search for other devices
        if (count > 1)
        {
            for (int i=1; i<count; i++)
            {
                index++;
                device = new cPhantomDevice(index);
                m_devices[m_numDevices] = device;
                m_numDevices++;
            }
        }
    }
    else
    {
        delete device;
    }

    #endif

    //-----------------------------------------------------------------------
    // search for MyCustom device
    //-----------------------------------------------------------------------
    #if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)

    // reset index number
    index = 0;

    // create a first device of this class
    device = new cMyCustomDevice(index);

    // check for how many devices of this type that are available
    count = device->getNumDevices();

    // if there are one or more devices available, then store them in the device table
    if (count > 0)
    {
        // store first device
        m_devices[m_numDevices] = device;
        m_numDevices++;

        // search for other devices
        if (count > 1)
        {
            for (int i=1; i<count; i++)
            {
                index++;
                device = new cMyCustomDevice(index);
                m_devices[m_numDevices] = device;
                m_numDevices++;
            }
        }
    }
    else
    {
        delete device;
    }

    #endif

    //-----------------------------------------------------------------------
    // search for CHAI3D Virtual Device
    // Note:
    // Virtual devices should always be listed last. The desired behavior
    // is that an application first searches for physical devices. If none
    // are found, it may launch a virtual device
    //-----------------------------------------------------------------------
    #if defined(C_ENABLE_VIRTUAL_DEVICE_SUPPORT)

    // reset index number
    index = 0;

    // create a first device of this class
    device = new cVirtualDevice();

    // check for how many devices of this type that are available
    count = device->getNumDevices();

    // if there are one or more devices available, then store it in the device table
    if (count > 0)
    {
        // store first device
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }

    // if no devices have been found then we try to launch a virtual haptic device
    else if (m_numDevices == 0)
    {
        // delete previous device
        delete device;

        // we try to launch the virtual device.
        spawnlp(_P_NOWAIT, "VirtualDevice.exe", "VirtualDevice.exe", NULL);
        cSleepMs(750);

        // create again a first device of this class
        device = new cVirtualDevice();

        // check for how many devices of this type that are available
        count = device->getNumDevices();

        // if there are one or more devices available, then store it in the device table
        if (count > 0)
        {
            // store first device
            m_devices[m_numDevices] = device;
            m_numDevices++;
        }
    }
    #endif
}


//===========================================================================
/*!
    Returns the specifications of the ith device.

    \param  a_deviceSpecifications  Returned result
    \param  a_index   Index number of the device.

    \return Return 0 if no error occurred.
*/
//===========================================================================
int cHapticDeviceHandler::getDeviceSpecifications(cHapticDeviceInfo& a_deviceSpecifications, unsigned int a_index)
{
    if (a_index < m_numDevices)
    {
        a_deviceSpecifications = m_devices[a_index]->getSpecifications();
        return (0);
    }
    else
    {
        return (-1);
    }
}


//===========================================================================
/*!
    Returns a handle to the ith device if available.

    \param  a_hapticDevice  Handle to device
    \param  a_index   Index number of the device.

    \return Return 0 if no error occurred.
*/
//===========================================================================
int cHapticDeviceHandler::getDevice(cGenericHapticDevice*& a_hapticDevice, 
								    unsigned int a_index)
{
    if (a_index < m_numDevices)
    {
        a_hapticDevice = m_devices[a_index];
        return (0);
    }
    else
    {
        a_hapticDevice = m_nullHapticDevice;
        return (-1);
    }
}



