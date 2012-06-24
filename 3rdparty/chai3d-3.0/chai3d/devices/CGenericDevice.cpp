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
#include "devices/CGenericDevice.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cGenericDevice.
*/
//===========================================================================
cGenericDevice::cGenericDevice()
{
    // the device is not yet available
    m_deviceAvailable = false;

    // the system is not yet ready to receive commands
    m_deviceReady = false;

    // No call back has been defined
    m_callback = NULL;
};


//===========================================================================
/*!
    Ask the device to call me back periodically.  If this device supports
    timed callbacks, this function will return 'true' and will call the
    supplied m_callback method at haptic rates.  If not, this function will
    return 'false', and you should create your own haptic thread.

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
