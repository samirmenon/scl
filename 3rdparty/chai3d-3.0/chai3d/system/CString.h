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
	\author	Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 423 $
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
    \ingroup    system

    \brief
    <b> System </b> \n
    Strings.
*/
//===========================================================================

//---------------------------------------------------------------------------
// ANSI STRINGS
//---------------------------------------------------------------------------

//! Compute the length of an ANSI string.
int cStringLength(const char* a_input);


//---------------------------------------------------------------------------
// FILE NAMES
//---------------------------------------------------------------------------

//! Find the file extension of a file.
string cFindFileExtension(const string& a_input, const bool a_includeDot = false);

//! Replace the file extension of a filename.
string cReplaceFileExtension(const string& a_input, const string& a_extension);

//! Find the filename.
string cFindFilename(const string& a_input, const bool a_includeFileExtension = true);

//! Find the directory.
string cFindDirectory(const string& a_input);


//---------------------------------------------------------------------------
// STRING CONVERSIONS
//---------------------------------------------------------------------------

//! Convert a string to lower case.
string cStringToLower(const string& a_input);

//! Convert a boolean into a string.
string cStr(const bool a_value);

//! Convert an integer into a string.
string cStr(const int a_value);

//! Convert a float into a string.
string cStr(const float a_value, const unsigned int a_precision = 2);

//! Convert a double into a string.
string cStr(const double& a_value, const unsigned int a_precision = 2);


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
