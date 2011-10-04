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
    \version   2.0.0 $Rev: 244 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CExtras.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Suspends the execution of the current thread for a specified interval
    defined in milliseconds.

    \fn       void cSleepMs(unsigned int a_interval);
    \param    a_interval  time interval defined in milliseconds.
*/
//===========================================================================
void cSleepMs(unsigned int a_interval)
{
#if defined(_WIN32)
    Sleep(a_interval);
#endif

#if defined(_LINUX)
    struct timespec t;
    t.tv_sec  = a_interval/1000000;
    t.tv_nsec = a_interval%1000000;
    nanosleep (&t, NULL);
#endif

#if defined(_MACOSX)
    struct timespec t;
    t.tv_sec  = a_interval/1000000;
    t.tv_nsec = a_interval%1000000;
    nanosleep (&t, NULL);
#endif
}



