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
#include "timers/CPrecisionClock.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cPrecisionClock. The clock is initialized to zero.
*/
//===========================================================================
cPrecisionClock::cPrecisionClock()
{
    // clock is currently off
    m_on = false;

#if defined(WIN32) | defined(WIN64)
    // test if high performance clock is available on the local machine. 
    // Some old computers may not offer this feature.
    QueryPerformanceFrequency (&m_freq);
    if (m_freq.QuadPart <= 0)
    {
        m_highres  = false;
    }
    else
    {
        m_highres  = true;
    }
#else
    m_highres = true;
#endif

    // initialize the time counter
    m_timeAccumulated = 0.0;

    // initialize timeout period
    m_timeoutPeriod = 0.0;

    // reset clock
    reset();
}


//===========================================================================
/*!
    Destructor of cPrecisionClock.
*/
//===========================================================================
cPrecisionClock::~cPrecisionClock()
{
}


//===========================================================================
/*!
    Reset the clock to zero.
*/
//===========================================================================
void cPrecisionClock::reset(const double a_currentTime)
{
    // initialize clock
    m_timeAccumulated = a_currentTime;

	// store current CPU time as starting time
    m_timeStart = getCPUTimeSeconds();
}


//===========================================================================
/*!
    Start or restart the clock. To read the latest time
    from the clock, use method getCurrentTime.

    \param      a_resetClock  Should we start counting from zero?

    \return     Returns the current clock time.
*/
//===========================================================================
double cPrecisionClock::start(const bool a_resetClock)
{
 	// store current CPU time as starting time
    m_timeStart = getCPUTimeSeconds();

    if (a_resetClock)
        m_timeAccumulated = 0.0;

    // timer is now on
    m_on = true;

    // return time when timer was started.
    return m_timeAccumulated;
}


//===========================================================================
/*!
    Stop the timer. To resume counting call start().

    \return     Return time in \e seconds.
*/
//===========================================================================
double cPrecisionClock::stop()
{

    // How much time has now elapsed in total running "sessions"?
    m_timeAccumulated += getCPUTimeSeconds() - m_timeStart;

    // stop timer
    m_on = false;

    // return time when timer was stopped
    return getCurrentTimeSeconds();
}


//===========================================================================
/*!
    Set the period in \e microseconds before timeout occurs. Do not forget
    to set the timer on by calling method \e start()

    \param      a_timeoutPeriod  Timeout period in \e seconds.
*/
//===========================================================================
void cPrecisionClock::setTimeoutPeriodSeconds(const double a_timeoutPeriod)
{
    m_timeoutPeriod = a_timeoutPeriod;
}


//===========================================================================
/*!
    Check if timer has expired its timeout period. if so return \b true.

    \return     Return \b true if timeout occurred, otherwise \b false.
*/
//===========================================================================
bool cPrecisionClock::timeoutOccurred() const 
{
    // check if timeout has occurred
    if (getCurrentTimeSeconds() > m_timeoutPeriod)
    {
        return true;
    }
    else
    {
        return false;
    }
}



//===========================================================================
/*!
    Read the current time of timer. Result is returned in \e seconds.

    \return		Return current time in \e seconds
*/
//===========================================================================
double cPrecisionClock::getCurrentTimeSeconds() const 
{
    if (m_on)
    {
        return m_timeAccumulated + getCPUTimeSeconds() - m_timeStart;
    }

    else return m_timeAccumulated;
}


//===========================================================================
/*!
    If all you want is something that tells you the time, this is your function...

    \return     Return cpu clock in \e seconds.
*/
//===========================================================================
double cPrecisionClock::getCPUTimeSeconds() const 
{

#if defined(WIN32) | defined(WIN64)

    if (m_highres)
    {
        __int64 curtime;
        QueryPerformanceCounter( (LARGE_INTEGER *)&curtime );
        return (double)curtime / (double)m_freq.QuadPart;
    }

    else
    {
        return ((double)(GetTickCount())) / 1000.0;
    }

#endif

#ifdef LINUX

    struct timespec time;

    clock_gettime (CLOCK_MONOTONIC, &time);

    return (double)(time.tv_sec + time.tv_nsec*1e-9);

#endif

#ifdef MACOSX

    static double ratio = 0.0;

    if (!ratio) {
      mach_timebase_info_data_t info;
      if(mach_timebase_info(&info) == KERN_SUCCESS) ratio = info.numer * 1e-9 * info.denom;
    }

    return mach_absolute_time() * ratio;

#endif

}

