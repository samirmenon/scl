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
    \version   2.0.0 $Rev: 258 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGlobalsH
#define CGlobalsH
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGlobals.h
    \ingroup    extras

    \brief  
    <b> Extras </b> \n 
	General CHAI3D Settings.
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//===========================================================================
// WIN32 / WIN64 OS
//===========================================================================
#if defined(_WIN32)
    //--------------------------------------------------------------------
    // GENERAL
    //--------------------------------------------------------------------
    // general windows
    #include "windows.h"

    // This needs to happen before GLUT gets included
    #include <stdlib.h>

    //--------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------
    #define _ENABLE_CUSTOM_DEVICE_SUPPORT
    #define _ENABLE_DELTA_DEVICE_SUPPORT
    #define _ENABLE_FALCON_DEVICE_SUPPORT
    #define _ENABLE_MPB_DEVICE_SUPPORT
    #define _ENABLE_PHANTOM_DEVICE_SUPPORT
    #define _ENABLE_SENSORAY626_DEVICE_SUPPORT
    #define _ENABLE_SERVOTOGO_DEVICE_SUPPORT
    #define _ENABLE_VIRTUAL_DEVICE_SUPPORT


    //--------------------------------------------------------------------
    // BBCP - BORLAND BUILDER
    //--------------------------------------------------------------------
    #if defined(_BBCP)

        // printf
        #define CHAI_DEBUG_PRINT printf

        // open gl
        #include "GL/glut.h"
        #include "GL/gl.h"
    #endif

    //--------------------------------------------------------------------
    // MSVC - MICROSOFT VISUAL STUDIO
    //--------------------------------------------------------------------
    #if  defined(_MSVC)

        // turn off annoying compiler warnings
        #pragma warning(disable: 4267)
        #pragma warning(disable: 4305)
        #pragma warning(disable: 4786)
        #pragma warning(disable: 4800)
		#pragma warning(disable: 4996)
        
        // printf
        #include <conio.h>
        #define CHAI_DEBUG_PRINT _cprintf

        // open gl
        #include "gl/glut.h"

    #endif

#endif


//===========================================================================
// LINUX OS
//===========================================================================
#if defined(_LINUX)

  //--------------------------------------------------------------------
  // GENERAL
  //--------------------------------------------------------------------
	// printf
	#define CHAI_DEBUG_PRINT printf

	// standard libraries
	#include <stdlib.h>
	#include <string.h>

  #include "GL/freeglut.h"
	// open gl -- NOTE SAMIR EDIT
//	#include "GL/gl.h"
//	#include "GL/glut.h"

	// threads
	#include "pthread.h"

  //--------------------------------------------------------------------
  // HAPTIC DEVICES
  //--------------------------------------------------------------------
	#define _ENABLE_CUSTOM_DEVICE_SUPPORT
	#define _ENABLE_DELTA_DEVICE_SUPPORT

  // disabled devices
  // #define _ENABLE_PHANTOM_DEVICE_SUPPORT

#endif

//===========================================================================
// MAC OS
//===========================================================================
#if defined(_MACOSX)

  //--------------------------------------------------------------------
  // GENERAL
  //--------------------------------------------------------------------
	// printf
	#define CHAI_DEBUG_PRINT printf

	// standard libraries
	#include <stdlib.h>
	#include <string.h>

	// open gl
	#include "GLUT/glut.h"

	// threads
	#include "pthread.h"

  //--------------------------------------------------------------------
  // HAPTIC DEVICES
  //--------------------------------------------------------------------
	#define _ENABLE_CUSTOM_DEVICE_SUPPORT
	#define _ENABLE_DELTA_DEVICE_SUPPORT

#endif


//===========================================================================
// GENERAL
//===========================================================================
//! maximum length of a path
#define CHAI_SIZE_PATH		255

//! maximum length of a object name
#define CHAI_SIZE_NAME		64

//! a large double
#define CHAI_DBL_MAX	    9999999

//---------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

