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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 368 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericLightH
#define CGenericLightH
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "math/CMaths.h"
#include "lighting/CShadowMap.h"
#include "world/CGenericObject.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGenericLight.h

    \brief 
    <b> Lighting </b> \n 
    Generic Light Source.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGenericLight
    \ingroup    lighting

    \brief      
    CGenericLight is a base class for describing light sources.
*/
//===========================================================================
class cGenericLight : public cGenericObject
{
  friend class cWorld;

  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cLight.
    cGenericLight(cWorld* a_world);
	
    //! Destructor of cLight.
    virtual ~cGenericLight();


    //-----------------------------------------------------------------------
    // GENERAL METHODS:
    //----------------------------------------------------------------------- 

    //! Enable or disable light source.
    inline void setEnabled(const bool a_enabled) { m_enabled = a_enabled; }

    //! Is this light source enabled?
    inline bool getEnabled() const { return (m_enabled); }

    //! Enable or disable two sided lighting mode.
    void setUseTwoSideLightModel(bool a_useTwoSideLightModel);

    //! Get status of two sided lighting mode.
    bool getUseTwoSideLightModel() const;

    //! Enable of disable graphic representation of light source. (To be used for debugging purposes).
    void setDisplayEnabled(bool a_enabled) { m_displayEnabled = a_enabled; }

    //! Get status about the graphic display representation of the light source.
    bool getDisplayEnabled() const { return (m_displayEnabled); }

    //! Set the OpenGL Light ID number. Assigining these numbers is normally handled by the world in which the light source is located.
    inline void setGLLightNumber(const GLint a_glLightNumber) { m_glLightNumber = a_glLightNumber; }

    //! Get the OpenGL Light ID number for this light source
    inline GLint getGLLightNumber() const { return(m_glLightNumber); }


	//-----------------------------------------------------------------------
    // MEMBERS:
	//-----------------------------------------------------------------------

    //! Ambient light component.
    cColorf m_ambient;
	
    //! Diffuse light component.
    cColorf m_diffuse;
    
	//! Specular light component.
    cColorf m_specular;


	//-----------------------------------------------------------------------
	// METHODS:
	//-----------------------------------------------------------------------
    
    //! Render the lighting properties of this light source in OpenGL.
    virtual void renderLightSource(cRenderOptions& a_options){}

    //! Render a graphical representation (display model) of the light source. (used for debuging purposes typically).
    virtual void render(cRenderOptions& a_options){}


  protected:
  
  	//-----------------------------------------------------------------------
    // MEMBERS:
	//-----------------------------------------------------------------------
	
    //! Parent world in which light source is located.
    cWorld* m_worldParent;

  	//! Enable light source (ON/OFF).
    bool m_enabled;
               
    //! OpenGL reference number for the current light source. This number ranges from 0 to 7.
    GLint m_glLightNumber;

    //! If \b true then a two sided light source model is used.
    GLint m_useTwoSideLightModel;

    //! If \b true then a graphical representation of the light source is rendered.
    bool m_displayEnabled;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

