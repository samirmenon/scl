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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 803 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CPrecisionClockH
#define CPrecisionClockH
//---------------------------------------------------------------------------
#include "system/CGlobals.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CPrecisionClock.h

    \brief
    <b> Timers </b> \n 
    High Precision Clock.
*/
//===========================================================================

//===========================================================================
/*!
  \class      cPrecisionClock
  \ingroup    timers

  \brief
  cPrecisionClock provides a class to manage high-precision time
  measurements. All measurements are computed in seconds unless
  otherwise-specified. \n cPrecisionClock behaves just like a real chronograph:
  It can be started, stopped and restarted a later time. When the clock is running (ON),
  time is accumulated until the next stop event (OFF). The value of the clock can
  be read at any time by calling method getCurrentTimeSeconds(). When the clock
  is stopped (OFF), time is no longer accumulated.
*/
//===========================================================================
class cPrecisionClock
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    
    //! Constructor of cPrecisionClock.
    cPrecisionClock();

    //! Destructor of cPrecisionClock.
    ~cPrecisionClock();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Reset the clock to zero.
    void reset(const double a_currentTime = 0.0);

    //! Start counting time; optionally reset the clock to zero.
    double start(bool a_resetClock = false);

    //! Stop counting time; return the elapsed time.
    double stop();

    //! Return \b true if clock is currently \b on (clock is running), else return \b false (clock is paused).
    bool on() const { return (m_on); };

    //! Read the current clock time (seconds).
    double getCurrentTimeSeconds() const;

    //! Set the period before a "timeout" occurs (you need to poll for this).
    void setTimeoutPeriodSeconds(const double a_timeoutPeriod);

    //! Read the programmed timeout period
    double getTimeoutPeriodSeconds() const { return (m_timeoutPeriod); }

    //! Returns \b true if a timeout has occurred.
    bool timeoutOccurred() const ;

    //! Returns \b true if the high resolution CPU clock are available on this computer.
    bool highResolution() const { return (m_highres); };

    //! If all you want is something that tells you the raw CPU time, this is your function...
    double getCPUTimeSeconds() const;

    //! For backwards-compatibility...
    double getCPUTime() const { return (getCPUTimeSeconds()); }

    //! For backwards-compatibility...
    double getCPUtime() const { return (getCPUTimeSeconds()); }

  private:

#if defined(WIN32) | defined(WIN64)
    //! Stores information about CPU high precision clock.
    LARGE_INTEGER m_freq;
#endif

    //! Time accumulated between previous calls to "start" and "stop".
    double m_timeAccumulated;

    //! CPU time when clock was started. Unit: seconds
    double m_timeStart;

    //! Timeout period. Unit: seconds
    double m_timeoutPeriod;

    //! clock time when timer was started. Unit: seconds
    double m_timeoutStart;

    //! If \b true, a  high precision CPU clock is available.
    bool m_highres;

    //! If \b true, the clock is currently \b on.
    bool m_on;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
