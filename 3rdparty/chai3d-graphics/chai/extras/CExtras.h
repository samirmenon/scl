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
#ifndef CExtrasH
#define CExtrasH
//---------------------------------------------------------------------------
#if defined(_WIN32)
#include "windows.h"
#endif

#if defined(_LINUX)
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/io.h>
#include <sys/mman.h>
#endif

#if defined(_MACOSX)
#include <mach/mach_time.h>
#include <mach/kern_return.h>
#include <sys/mman.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CExtras.h
    \ingroup    extras

    \brief  
    <b> Extras </b> \n 
	Additional Useful Functions.
*/
//===========================================================================

//---------------------------------------------------------------------------
// GENERAL PUPOSE FUNCTIONS:
//---------------------------------------------------------------------------

//! Suspends the execution of the current thread for a specified interval.
void cSleepMs(unsigned int a_interval);

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
