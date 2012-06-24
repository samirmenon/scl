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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 423 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CFogH
#define CFogH
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CRenderOptions.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CFog.h
    
    \brief  
    <b> Graphics </b> \n 
    Fog.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cFog
    \ingroup    graphics

    \brief      
    cFog implements support for OpenGL's fog capabilitiy. When fog is enabled,
	objects that are farther from the viewpoint begin to fade into the fog 
	color. You can control the density of the fog, which determines the rate
	at which objects fade as the distance increases, as well as the fog's color.
*/
//===========================================================================
class cFog
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    
    //! Constructor of cFog.
    cFog();

    //! Destructor of cFog.
    virtual ~cFog() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Fog rendering.
    virtual void render(cRenderOptions& a_options);

	//! Enable of disable for property.
	void setEnabled(const bool a_enabled) { m_enabled = a_enabled; }

	//! Returns \b true if fog is enabled, \b false otherwise.
	bool getEnabled() const { return(m_enabled); }

	//! Set fog mode by passing the OpenGL fog mode constant as parameter.
	void setFogMode(const GLint a_fogMode) { m_fogMode = a_fogMode; }

	//! Set fog mode to GL_LINEAR.
	void setFogModeLINEAR() { m_fogMode = GL_LINEAR; }

	//! Set fog mode to GL_EXP.
	void setFogModeEXP() { m_fogMode = GL_EXP; }

	//! Set fog mode to GL_EXP2.
	void setFogModeEXP2() { m_fogMode = GL_EXP2; }

	//! Get the current fog mode.
	GLint getFogMode() { return (m_fogMode); }

	//! Set fog properties.
	void setProperties(const double a_start, 
                       const double a_end, 
                       const double a_density);


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

	//! Fog color.
	cColorf m_color;


  protected:

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

	//! If \b true, then fog is enabled, \b false otherwise.
	bool m_enabled;

	//! Fog mode.
	GLint m_fogMode;

	//! Fog start distance.
	float m_start;

	//! Fog end distance.
	float m_end;

	//! Fog density.
	float m_density;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

