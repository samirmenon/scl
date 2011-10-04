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
    \version   2.0.0 $Rev: 250 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CConstantsH
#define CConstantsH
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CConstants.h
    \ingroup    math

    \brief
    <b> Math </b> \n 
    General Constants.
*/
//===========================================================================

//---------------------------------------------------------------------------
// SCALARS
//---------------------------------------------------------------------------

//! PI constant.
#define CHAI_PI                 3.14159265358979323846

//! PI constant divided by two.
#define CHAI_PI_DIV_2           1.57079632679489661923

//! Conversion from degrees to radians.
#define CHAI_DEG2RAD	        0.01745329252

//! Conversion from radians to degrees.
#define CHAI_RAD2DEG	        57.2957795131

//! Smallest value near zero for a double.
#define CHAI_TINY               1e-49

//! Small value near zero.
#define CHAI_SMALL              0.000000001

//! Biggest value for a double
#ifdef DBL_MAX
#define CHAI_LARGE              DBL_MAX
#else
#define CHAI_LARGE              1e+49
#endif

//! Biggest value for a float
#ifdef FLOAT_MAX
#define CHAI_LARGE_FLOAT        FLOAT_MAX
#else
#define CHAI_LARGE_FLOAT        1e37
#endif


//---------------------------------------------------------------------------
// VECTORS
//---------------------------------------------------------------------------

//! Zero vector (0,0,0)
#define CHAI_VECTOR_ZERO        cVector3d(0, 0, 0);

//! Unit vector along Axis X (1,0,0)
#define CHAI_VECTOR_X           cVector3d(1, 0, 0);

//! Unit vector along Axis Y (0,1,0)
#define CHAI_VECTOR_Y           cVector3d(0, 1, 0);

//! Unit vector along Axis Z (0,0,1)
#define CHAI_VECTOR_Z           cVector3d(0, 0, 1);

//! Origin (0,0,0)
#define CHAI_ORIGIN             cVector3d(0, 0, 0);


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
