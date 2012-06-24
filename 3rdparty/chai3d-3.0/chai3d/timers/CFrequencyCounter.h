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
#ifndef CFrequencyCounterH
#define CFrequencyCounterH
//---------------------------------------------------------------------------
#include "timers/CPrecisionClock.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CFrequencyCounter.h

    \brief
    <b> Timers </b> \n 
    High Precision Frequency Counter.
*/
//===========================================================================

//===========================================================================
/*!
  \class      cFrequencyCounter
  \ingroup    timers

  \brief
    cFrequencyCounter provides a class for measuring frequency. Frequency is 
    defined as the number of events of a particular sort occurring in a set 
    period of time. The period of time is adjustable but is set at 1 second
    as a default value.

    By calling the method signal() a counter is incremented. When the internal
    clock  reaches the programmed timeout period (1 second for instance),
    the frequency value is computed and updated.
*/
//===========================================================================
class cFrequencyCounter
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    
    //! Constructor of cFrequencyCounter.
    cFrequencyCounter(const double a_timePeriod = 1.0);

    //! Destructor of cFrequencyCounter.
    virtual ~cFrequencyCounter() {}


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Set period in seconds.
    void setTimePeriod(const double& a_timePeriod);

    //! Start counting time; optionally reset the clock to zero.
    inline double getTimePeriod() const { return (m_clock.getTimeoutPeriodSeconds()); }

    //! Stop counting time; return the elapsed time.
    inline double getFrequency() const { return (m_frequency); }

    //! Reset the frequency counter.
    void reset();

    //! Signal the frequency counter of a new event. The current frequency is returned.
    double signal(const unsigned int a_numEvents = 1);


  private:

    //! High precision clock.
    cPrecisionClock m_clock;

    //! Frequency
    double m_frequency;

    //! Event counter
    unsigned int m_counter;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
