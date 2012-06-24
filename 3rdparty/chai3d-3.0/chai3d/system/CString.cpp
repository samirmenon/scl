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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 387 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "system/CString.h"
#include "math/CMaths.h"
//---------------------------------------------------------------------------
#include <iostream>
#include <iomanip>
using namespace std;
//---------------------------------------------------------------------------

//===========================================================================
/*!
	Compute the length of a string up to 255 characters. If the end of string
    cannot be found, then -1 is returned as a result.

	\param		a_string  Input string. Pointer to a char.
	
    \return		Return the length of the string.
*/
//===========================================================================
int cStringLength(const char* a_input)
{
    return (int)(strlen(a_input));
}


//===========================================================================
/*!
    Convert a string into lower case.

    \param  a_string  Input string 
    
    \return  Returns the output string.
*/
//===========================================================================
string cStringToLower(const string& a_input)
{
    string result = a_input;
    transform(result.begin(), result.end(), result.begin(), ::tolower);
    return (result);
}


//===========================================================================
/*!
    Finds the extension in a filename.

    \param  a_input  Input filename.
    \param  a_includeDot  If \b true, include the dot at the beginning of 
                          the extension. (example: ".jpg")
    
    \return  Returns a string containing the extension.
*/
//===========================================================================
string cFindFileExtension(const string& a_input, 
		                  const bool a_includeDot)
{
    int pos = (int)(a_input.find_last_of("."));
    if (pos < 0) return "";
    if (a_includeDot)
    {
        return a_input.substr(pos, a_input.length());
    }
    else
    {
        return a_input.substr(pos+1, a_input.length());
    }
}


//===========================================================================
/*!
    Discards the path component of a filename and returns the filename itself,
    optionally including the extension.

    \param  a_input   Input string containing path and filename
    \param  a_includeExtension  Should the output include the extension?
    
    \return  Returns the output string.
*/
//===========================================================================
string cFindFilename(const string& a_input, 
                     const bool a_includeFileExtension)
{
    string result = a_input;
    int pos = (int)(result.find_last_of("\\"));
    if (pos > -1) result = result.substr(pos+1, result.length()-pos-1);
    pos = (int)(result.find_last_of("/"));
    if (pos > -1) result = result.substr(pos+1, result.length()-pos-1);
    if (!a_includeFileExtension)
    {
        pos = (int)(result.find_last_of("."));
        result = result.substr(0, pos);
    }
    return (result);
}


//===========================================================================
/*!
    Returns the string a_filename by replacing its extension with a new
    string provided by parameter a_extension.

    \param      a_filename The input filename
    \param      a_extension  The extension to replace a_input's extension with

    \return     Returns the output string.
*/
//===========================================================================
string cReplaceFileExtension(const string& a_filename, 
                             const string& a_extension)
{
    string result = a_filename;
    int pos = (int)(result.find_last_of("."));
    if (pos < 0) return a_filename;
    result.replace(pos+1, result.length(), a_extension);
    return (result);
}


//===========================================================================
/*!
    Finds only the _path_ portion of source, and copies it with
    _no_ trailing '\\'.  If there's no /'s or \\'s, writes an
    empty string.

    \param      a_dest    String which will contain the directory name.
    \param      a_source  Input string containing path and filename.
    
    \return     Return \b true for success, \b false if there's no separator.
*/
//===========================================================================
string cFindDirectory(const string& a_input)
{
    return (a_input.substr(0, a_input.length() - cFindFilename(a_input, true).length()));
}


//===========================================================================
/*!
    Convert a \e boolean into a \e string.

    \param    a_value  Input value of type \e boolean.
    
    \return   Return output string.
*/
//===========================================================================
string cStr(const bool a_value)
{
    string result;
    if (a_value) 
        result = "true";
    else 
        result = "false";   
    return (result);
}


//===========================================================================
/*!
    Convert an \e integer into a \e string.

    \param    a_value  Input value of type \e integer.
    
    \return   Return output string.
*/
//===========================================================================
string cStr(const int a_value)
{
    ostringstream result;
    result << a_value;
    return (result.str());
}


//===========================================================================
/*!
    Convert a \e float into a \e string.

    \param    a_value  Input value of type \e float.
    \param    a_precision  Number of digits displayed after the decimal point.

    \return   Return output string.
*/
//===========================================================================
string cStr(const float a_value, 
            const unsigned int a_precision)
{
    ostringstream result;
    result << fixed << setprecision(a_precision) << a_value;
    return (result.str());
}


//===========================================================================
/*!
    Convert a \e double into a \e string.

    \param      a_value  Input value of type \e double.
    \param      a_precision  Number of digits displayed after the decimal point.

    \return     Return output string.
*/
//===========================================================================
string cStr(const double& a_value, 
            const unsigned int a_precision)
{
    ostringstream result;
    result << fixed << setprecision(a_precision) << a_value;
    return (result.str());
}
