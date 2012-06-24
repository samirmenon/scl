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
#ifndef CThreadH
#define CThreadH
//---------------------------------------------------------------------------
#include "system/CGlobals.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CThread.h
    \ingroup    system

    \brief
    <b> System </b> \n
    Threads.
*/
//===========================================================================

//---------------------------------------------------------------------------
/*!
    Defines thread priority for handling \e graphics and \e haptics
    rendering loops.
*/
//---------------------------------------------------------------------------
enum CThreadPriority
{
    CTHREAD_PRIORITY_GRAPHICS,
    CTHREAD_PRIORITY_HAPTICS
};


//===========================================================================
/*!
    \class	    cThread
    \ingroup    system

    \brief	
    cThread provides a class to manage threads.
*/
//===========================================================================
class cThread
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cThread.
    cThread();

    //! Destructor of cThread.
    virtual ~cThread();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Set the thread parameters (and start it).
    void start(void(*a_function)(void ), const CThreadPriority a_level);
    void start(void(*a_function)(void*), const CThreadPriority a_level, void *arg);

    //! Terminate the thread (not recommended)
    void stop();

    //! Set the thread priority level.
    void setPriority(CThreadPriority a_level);

    //! Get the current thread priority level.
    CThreadPriority getPriority() const { return (m_priorityLevel); }


  protected:

#if defined(WIN32) | defined(WIN64)
    //! Thread handle
    DWORD m_threadId;
#endif

#if defined(LINUX) || defined(MACOSX)
    //! Thread handle
    pthread_t m_handle;
#endif

    //! Pointer to thread function.
    void* m_function;

    //! Thread priority level.
    CThreadPriority m_priorityLevel;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
