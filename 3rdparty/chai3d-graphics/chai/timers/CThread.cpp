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
    \version   2.0.0 $Rev: 245 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CThread.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cThread.

    \fn		cThread::cThread()
*/
//===========================================================================
cThread::cThread()
{
    // no thread function has been defined yet
    m_function = 0;

    // default value for priority level
    m_priorityLevel = CHAI_THREAD_PRIORITY_GRAPHICS;
}


//===========================================================================
/*!
    Destructor of cThread.

    \fn		cThread::~cThread()
*/
//===========================================================================
cThread::~cThread()
{
}


//===========================================================================
/*!
    Creates a thread to execute within the address space of the calling process.
    Parameters include a pointer to the function and its priority level.

    \fn		void cThread::set(void (*a_function)(void), CThreadPriority a_level)
    \param  a_function Pointer to thread function
    \param  a_level Priority level of thread.
*/
//===========================================================================
void cThread::set(void (*a_function)(void), CThreadPriority a_level)
{
    // create thread
#if defined(_WIN32)
    CreateThread(
          0,
          0,
          (LPTHREAD_START_ROUTINE)(a_function),
          0,
          0,
          &m_threadId
      );
#endif

#if defined (_LINUX) || defined (_MACOSX)
    pthread_create(
          &m_handle,
          0,
          (void * (*)(void*)) a_function,
          0
    );
#endif

    // set thread priority level
    setPriority(a_level);
}


//===========================================================================
/*!
    Adjust the priority level of the thread.

    \fn		void cThread::setPriority(CThreadPriority a_level)
    \param  a_level  Priority level of the thread
*/
//===========================================================================
void cThread::setPriority(CThreadPriority a_level)
{
    m_priorityLevel = a_level;

#if defined(_WIN32)
    switch (m_priorityLevel)
    {
        case CHAI_THREAD_PRIORITY_GRAPHICS:
        SetThreadPriority(&m_threadId, THREAD_PRIORITY_NORMAL);
        break;

        case CHAI_THREAD_PRIORITY_HAPTICS:
        SetThreadPriority(&m_threadId, THREAD_PRIORITY_ABOVE_NORMAL);
        break;
    }
#endif

#if defined(_LINUX) || defined(_MACOSX)
    struct sched_param sp;
    memset(&sp, 0, sizeof(struct sched_param));

    switch (m_priorityLevel)
    {
        CHAI_THREAD_PRIORITY_GRAPHICS:
        sp.sched_priority = 5;
        break;

        CHAI_THREAD_PRIORITY_HAPTICS:
        sp.sched_priority = 10;
        break;
    }

    pthread_setschedparam(m_handle, SCHED_RR, &sp);
#endif
}
