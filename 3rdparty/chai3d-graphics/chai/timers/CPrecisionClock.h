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
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CPrecisionClockH
#define CPrecisionClockH
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
//---------------------------------------------------------------------------
#if defined(_MACOSX) || defined(_LINUX)
#include "sys/time.h"
#endif
//---------------------------------------------------------------------------

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
	\class	    cPrecisionClock
    \ingroup    timers  

	\brief	
    cPrecisionClock provides a class to manage high-precision time 
    measurements.  All measurements are in seconds unless
    otherwise-specified.
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

    //! Reset clock to zero.
    void reset();

    //! Start counting time; optionally reset the clock to zero.
    double start(bool a_resetClock = false);

    //! Stop counting time; return the elapsed time.
    double stop();

    //! Return \b true if timer is currently \b on, else return \b false.
    bool on() { return (m_on); };

    //! Read the current clock time (seconds) (the time that has elapsed since the last call to "start").
    double getCurrentTimeSeconds();

    //! Set the period before a "timeout" occurs (you need to poll for this).
    void setTimeoutPeriodSeconds(double a_timeoutPeriod);

    //! Read the programmed timeout period
    double getTimeoutPeriodSeconds() { return (m_timeoutPeriod); }

    //! Returns \b true if a timeout has occurred.
    bool timeoutOccurred();

    //! Returns \b true if high resolution timers are available on this computer.
    bool highResolution() { return (m_highres); };

    //! If all you want is something that tells you the time, this is your function...
    double getCPUTimeSeconds();

    //! For backwards-compatibility...
    double getCPUTime() { return getCPUTimeSeconds(); }

    //! For backwards-compatibility...
    double getCPUtime() { return getCPUTimeSeconds(); }

  private:

#if defined(_WIN32)
    //! Stores information about CPU high precision clock.
    LARGE_INTEGER m_freq;
#else
    timeval t_start_;
#endif

    //! Time accumulated between previous calls to "start" and "stop".
    double m_timeAccumulated;

    //! CPU time when clock was started.
    double m_timeStart;

    //! Timeout period.
    double m_timeoutPeriod;

    //! clock time when timer was started.
    double m_timeoutStart;

    //! If \b true, a  high precision CPU clock is available.
    bool m_highres;

    //! If \b true, the clock is \b on.
    bool m_on;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
