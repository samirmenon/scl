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
    \version   2.0.0 $Rev: 244 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "devices/CHapticDeviceHandler.h"
//---------------------------------------------------------------------------
#include "extras/CExtras.h"

#if defined(_WIN32)
#include <process.h>
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cHapticDeviceHandler.

    \fn     cHapticDeviceHandler::cHapticDeviceHandler()
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
    int i;
    for (i=0; i<CHAI_MAX_HAPTIC_DEVICES; i++)
    {
        m_devices[i] = NULL;
    }

    // search for available haptic devices
    update();
}


//===========================================================================
/*!
    Destructor of cHapticDeviceHandler.

    \fn     cHapticDeviceHandler::~cHapticDeviceHandler()
*/
//===========================================================================
cHapticDeviceHandler::~cHapticDeviceHandler()
{
    // clear current list of devices
    int i;
    for (i=0; i<CHAI_MAX_HAPTIC_DEVICES; i++)
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

    \fn     void cHapticDeviceHandler::update()
*/
//===========================================================================
void cHapticDeviceHandler::update()
{
    // temp variables
    int index, count, i;
    cGenericHapticDevice* device;

    // clear current list of devices
    m_numDevices = 0;
    for (i=0; i<CHAI_MAX_HAPTIC_DEVICES; i++)
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
    #if defined(_ENABLE_DELTA_DEVICE_SUPPORT)

    // reset index number
    index = 0;

    // create a first device of this class
    device = new cDeltaDevice(index);

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
            for (i=1; i<count; i++)
            {
                index++;
                device = new cDeltaDevice(index);
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
    // search for Novint Falcon device
    //-----------------------------------------------------------------------
    #if defined(_ENABLE_FALCON_DEVICE_SUPPORT)

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
            for (i=1; i<count; i++)
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
    // search for MPB Technologies devices
    //-----------------------------------------------------------------------
    #if defined(_ENABLE_MPB_DEVICE_SUPPORT)

    // reset index number
    index = 0;

    // create a first device of this class
    device = new cFreedom6SDevice();

    // check for how many devices of this type that are available
    count = device->getNumDevices();

    // if there is one device available, then store it in the device table
    if (count > 0)
    {
        // store first device
        m_devices[m_numDevices] = device;
        m_numDevices++;
    }
    else
    {
        delete device;
    }

    #endif
    //-----------------------------------------------------------------------
    // search for Sensable Technologies devices
    //-----------------------------------------------------------------------
    #if defined(_ENABLE_PHANTOM_DEVICE_SUPPORT)

    // reset index number
    index = 0;
    bool available = true;

    // if the device is available, add it to the list and keep on searching
    if (available)
    {
        // create a new instance of the device
        device = new cPhantomDevice(index);

        // check if its available
        available = device->isSystemAvailable();
        if (available)
        {
            m_devices[m_numDevices] = device;
            m_numDevices++;
        }
        else
        {
            delete device;
        }
    }

    #endif
    //-----------------------------------------------------------------------
    // search for MyCustom device
    //-----------------------------------------------------------------------
    #if defined(_ENABLE_MY_CUSTOM_DEVICE_SUPPORT)

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
            for (i=1; i<count; i++)
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
    // search for CHAI 3D Virtual Device
    // Note:
    // Virtual devices should always be listed last. The desired behavior
    // is that an application first searches for physical devices. If none
    // are found, it may launch a virtual device
    //-----------------------------------------------------------------------
    #if defined(_ENABLE_VIRTUAL_DEVICE_SUPPORT)

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

    \fn     int cHapticDeviceHandler::getDeviceSpecifications(cHapticDeviceInfo&
            a_deviceSpecifications, unsigned int a_index)
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

    \fn     int cHapticDeviceHandler::getDevice(cGenericHapticDevice*& a_hapticDevice,
            unsigned int a_index)
    \param  a_hapticDevice  Handle to device
    \param  a_index   Index number of the device.
    \return Return 0 if no error occurred.
*/
//===========================================================================
int cHapticDeviceHandler::getDevice(cGenericHapticDevice*& a_hapticDevice, unsigned int a_index)
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



