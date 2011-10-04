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
    \version   2.0.0 $Rev: 201 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "timers/CPrecisionClock.h"
//---------------------------------------------------------------------------
#if defined(_MACOSX) || defined(_LINUX)
#include "sys/time.h"
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cPrecisionClock. Clock is initialized to zero.

    \fn		cPrecisionClock::cPrecisionClock()
*/
//===========================================================================
cPrecisionClock::cPrecisionClock()
{
    // clock is currently off
    m_on = false;

#if defined(_WIN32)
    // test for high performance timer on the local machine. Some old computers
    // may not offer this feature
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
    gettimeofday(&t_start_,NULL);
#endif

    // initialize current time
    m_timeAccumulated = 0.0;

    // initialize timeout
    m_timeoutPeriod = 0.0;


}


//===========================================================================
/*!
    Destructor of cPrecisionClock.

    \fn		cPrecisionClock::~cPrecisionClock()
*/
//===========================================================================
cPrecisionClock::~cPrecisionClock()
{
}


//===========================================================================
/*!
    Reset the clock to zero.

    \fn		void cPrecisionClock::reset()
*/
//===========================================================================
void cPrecisionClock::reset()
{
    // initialize current time of timer
    m_timeAccumulated = 0.0;
    m_timeStart = getCPUTimeSeconds();
}


//===========================================================================
/*!
    Start the clock from its current time value. To read the latest time
    from the clock, use method getCurrentTime.

    \fn         double cPrecisionClock::start(bool a_resetClock)
    \param      a_resetClock  Should we start counting from zero?
    \return     Returns the current clock time.
*/
//===========================================================================
double cPrecisionClock::start(bool a_resetClock)
{
    // store cpu time when timer was started
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

    \fn         long cPrecisionClock::stop()
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

    \fn         void cPrecisionClock::setTimeoutPeriodSeconds(double a_timeoutPeriod)
    \param      a_timeoutPeriod  Timeout period in \e seconds.
*/
//===========================================================================
void cPrecisionClock::setTimeoutPeriodSeconds(double a_timeoutPeriod)
{
    m_timeoutPeriod = a_timeoutPeriod;
}


//===========================================================================
/*!
    Check if timer has expired its timeout period. if so return \b true.

    \fn         bool cPrecisionClock::timeoutOccurred()
    \return     Return \b true if timeout occurred, otherwise \b false.
*/
//===========================================================================
bool cPrecisionClock::timeoutOccurred()
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

    \fn			double cPrecisionClock::getCurrentTimeSeconds()
    \return		Return current time in \e seconds
*/
//===========================================================================
double cPrecisionClock::getCurrentTimeSeconds()
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

    \fn         long cPrecisionClock::getCPUtime()
    \return     Return cpu clock in \e seconds.
*/
//===========================================================================
double cPrecisionClock::getCPUTimeSeconds()
{

    // Windows implementation
#if defined(_WIN32)

    if (m_highres)
    {
        __int64 curtime;
        //Receives the current performance-counter frequency, in counts per second
        QueryPerformanceCounter( (LARGE_INTEGER *)&curtime );
        //Returns seconds since some epoch
        return (double)curtime / (double)m_freq.QuadPart;
    }

    else
    {
        return ((double)(GetTickCount())) / 1000.0;
    }

    // POSIX implementation
#else

    // compute and print the elapsed time in millisec
    double elapsedTime;
    timeval t2;
    gettimeofday(&t2,NULL);
    elapsedTime = (t2.tv_sec - t_start_.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (t2.tv_usec - t_start_.tv_usec) / 1000.0;   // us to ms
    return elapsedTime;

#endif

}

