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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 456 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "timers/CFrequencyCounter.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cFrequencyCounter.

    \param  a_timePeriod  Time period. 
*/
//===========================================================================
cFrequencyCounter::cFrequencyCounter(const double a_timePeriod)
{
    // set time period
    if (a_timePeriod > 0.0)
    {
        m_clock.setTimeoutPeriodSeconds(a_timePeriod);
    }
    else
    {
        m_clock.setTimeoutPeriodSeconds(1.0);
    }

    // reset clock
    m_clock.start(true);
}


//===========================================================================
/*!
    Reset the frequency to zero.
*/
//===========================================================================
void cFrequencyCounter::reset()
{
    // reset counter
    m_counter = 0;

    // reset clock
    m_clock.start(true);
}


//===========================================================================
/*!
    Signal one of more events.

    \param      a_numEvents  Number of new events. In principle 1.

    \return     Returns the last computed frequency.
*/
//===========================================================================
double cFrequencyCounter::signal(const unsigned int a_numEvents)
{
    if (m_clock.timeoutOccurred())
    {
        double time = m_clock.getCurrentTimeSeconds();
        double timeout = m_clock.getTimeoutPeriodSeconds();

        // if timeout period has occured, compute frequency
        m_frequency = (double)(m_counter) / (double)(m_clock.getTimeoutPeriodSeconds());

        // restart clock and set first events
        m_counter = a_numEvents;
        while (time-timeout > 0) time -= timeout;
        m_clock.reset(time);
    }
    else
    {
        m_counter = m_counter + a_numEvents;
    }

    return (m_frequency);
}


//===========================================================================
/*!
    Set the time period of the frequency counter.

    \param      a_timePeriodInSeconds  Time period in seconds.
*/
//===========================================================================
void cFrequencyCounter::setTimePeriod(const double& a_timePeriod)
{
    // set time period
    if (a_timePeriod > 0.0)
    {
        m_clock.setTimeoutPeriodSeconds(a_timePeriod);
    }
    else
    {
        m_clock.setTimeoutPeriodSeconds(1.0);
    }
}
