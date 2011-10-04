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
#include "devices/CGenericDevice.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cGenericDevice.

    \fn  cGenericDevice::cGenericDevice()
*/
//===========================================================================
cGenericDevice::cGenericDevice()
{
    // the device is not yet available
    m_systemAvailable = false;

    // the system is not yet ready to receive commands
    m_systemReady = false;

    // No call back has been defined
    m_callback = NULL;
};


//===========================================================================
/*!
    Ask the device to call me back periodically.  If this device supports
    timed callbacks, this function will return 'true' and will call the
    supplied m_callback method at haptic rates.  If not, this function will
    return 'false', and you should create your own haptic thread.

    \fn     cGenericDevice::setCallback(cCallback* m_callback)
    \param  m_callback  The callback to trigger periodically, or 0 to cancel
                        an existing callback. 
    \return Returns \b true if this device supports callbacks, \b false 
            otherwise.                          
*/
//===========================================================================
bool cGenericDevice::setCallback(cCallback* a_callback)
{
    m_callback = a_callback;
    return (true);
}
