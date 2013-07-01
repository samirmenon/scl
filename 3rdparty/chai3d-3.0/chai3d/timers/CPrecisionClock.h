//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2013, CHAI3D.
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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1055 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CPrecisionClockH
#define CPrecisionClockH
//------------------------------------------------------------------------------
#include "system/CGlobals.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CPrecisionClock.h

    \brief
    <b> Timers </b> \n 
    High precision clock.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cPrecisionClock
    \ingroup    timers

    \brief
    High precision clock

    \details
    __cPrecisionClock__ implements a high-precision clock. All measurements 
    are computed in seconds unless otherwise-specified.\n 
  
    __cPrecisionClock__ behaves just like a real chronograph: It can be started, 
    stopped and restarted at a later time. When a clock is running (__ON__), 
    time is accumulated until the next stop event (__OFF__). The value of a 
    clock can be read by calling method getCurrentTimeSeconds(). When a clock
    is disabled (__OFF__), time is no longer accumulated.
*/
//==============================================================================

class cPrecisionClock
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
 
public:

    //! Constructor of cPrecisionClock.
    cPrecisionClock();

    //! Destructor of cPrecisionClock.
    virtual ~cPrecisionClock() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Reset clock.
    void reset(const double a_currentTime = 0.0);

    //! Start clock with optional reset.
    double start(bool a_resetClock = false);

    //! Stop clock and return elapsed time.
    double stop();

    //! Return __true__ if clock is running (__ON__), otherwise return __false__ if clock is paused (__OFF__).
    bool on() const { return (m_on); };

    //! Read current clock time in seconds.
    double getCurrentTimeSeconds() const;

    //! Set the period in seconds before a _timeout_ occurs (you need to poll for this).
    void setTimeoutPeriodSeconds(const double a_timeoutPeriod);

    //! Read the programmed _timeout_ period is seconds.
    double getTimeoutPeriodSeconds() const { return (m_timeoutPeriod); }

    //! Return __true__ if _timeout_ has occurred, otherwise return  __false__.
    bool timeoutOccurred() const ;

    //! Return __true__ if the high resolution CPU clock is available on this computer, otherwise return __false__.
    bool highResolution() const { return (m_highres); };

    //! Return raw CPU time in seconds.
    double getCPUTimeSeconds() const;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

#if defined(WIN32) | defined(WIN64)
    //! Stores information about CPU high precision clock.
    LARGE_INTEGER m_freq;
#endif

    //! Time accumulated between previous calls to start() and stop().
    double m_timeAccumulated;

    //! CPU time in seconds when clock was started
    double m_timeStart;

    //! Timeout period in seconds.
    double m_timeoutPeriod;

    //! Clock time in seconds when timer was started. 
    double m_timeoutStart;

    //! If __true__, then high precision CPU clock is available.
    bool m_highres;

    //! If __true__, then clock is currently __ON__.
    bool m_on;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
