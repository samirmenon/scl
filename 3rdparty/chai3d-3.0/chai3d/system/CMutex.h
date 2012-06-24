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
    \author    Sebastien Grange
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 799 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CMutexH
#define CMutexH
//---------------------------------------------------------------------------
#include "system/CGlobals.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CMutex.h
    \ingroup    system

    \brief
    <b> System </b> \n
    Mutex.
*/
//===========================================================================

//===========================================================================
/*!
    \class	    cMutex
    \ingroup    system

    \brief	
    cThread provides a class to manage threads.
*/
//===========================================================================
class cMutex
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cMutex.
    cMutex();

    //! Destructor of cMutex.
    virtual ~cMutex();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Acquire the mutex
    bool acquire();

    //! Acquire the mutex only if it is not owned by another thread
    bool tryAcquire();

    //! Release the mutex.
    void release();

  protected:

#if defined(WIN32) | defined(WIN64)
    //! Mutex handle
    CRITICAL_SECTION m_mutex;
#endif

#if defined(LINUX) || defined(MACOSX)
    //! Mutex handle
    pthread_mutex_t m_mutex;
#endif
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
