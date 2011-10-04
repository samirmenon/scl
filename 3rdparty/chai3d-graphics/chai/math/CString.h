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
	\author	Dan Morris
    \version   2.0.0 $Rev: 250 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CStringH
#define CStringH
//---------------------------------------------------------------------------
#include <string>
#include <stdio.h>
//---------------------------------------------------------------------------
using std::string;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CString.h
    \ingroup    math

    \brief
    <b> Math </b> \n 
    Strings.
*/
//===========================================================================

//! Compute the length of a string.
int cStringLength(const char* a_string);

//! Convert a boolean into a string.
void cStr(string& a_string, const bool& a_value);

//! Convert an integer into a string.
void cStr(string& a_string, const int& a_value);

//! Convert a float into a string.
void cStr(string& a_string, const float& a_value, const unsigned int a_precision=2);

//! Convert a double into a string.
void cStr(string& a_string, const double& a_value, const unsigned int a_precision=2);


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
