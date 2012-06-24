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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 403 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CTexture1dH
#define CTexture1dH
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "materials/CGenericTexture.h"
#include "graphics/CImage.h"
#include <string>
#include <stdio.h>
//---------------------------------------------------------------------------

//===========================================================================
/*! 
    \file       CTexture1d.h

    \brief  
    <b> Materials </b> \n 
    1D Texture.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cTexture1d
    \ingroup    materials
    
    \brief      
    cTexture1d describes a 2D bitmap texture used for OpenGL texture-mapping
*/
//===========================================================================
class cTexture1d : public cGenericTexture
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cTexture1d.
    cTexture1d();

    //! Destructor of cTexture1d.
    virtual ~cTexture1d();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Set texture unit where a_textureUnit is GL_TEXTUREi_ARB, where 0i<GL_MAX_TEXTURE_UNITS_ARB.
    inline void setTextureUnit(const GLenum a_textureUnit);

    //! Get texture unit where a_textureUnit is GL_TEXTUREi_ARB, where 0i<GL_MAX_TEXTURE_UNITS_ARB.
    inline GLenum getTextureUnit() const { return (m_textureUnit); }

	//! Create a copy of current object.
	virtual cTexture1d* copy();

    //! Load a texture image file.
    virtual bool loadFromFile(const string& a_fileName);

    //! Save a texture image file.
    virtual bool saveToFile(const string& a_fileName);

    //! Enable texturing and set this texture as the current texture.
    virtual void render(cRenderOptions& a_options);

    //! Call this to force texture re-initialization.
    virtual void markForUpdate() { m_updateTextureFlag = true; }

    //! Set the environment mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE, or -1 for "don't set").
    void setEnvironmentMode(const GLint& a_environmentMode) { m_environmentMode = a_environmentMode; }

    //! Get the environment mode status.
    GLint getEnvironmentMode() { return (m_environmentMode); }

    //! Set the texture wrap mode.
    virtual void setWrapMode(const GLint& a_wrapMode);

    //! Get the texture wrap mode of S.
    GLint getWrapSmode() const { return (m_wrapSmode); }

    //! Set the magnification function.
    void setMagnificationFunction(const GLint a_magnificationFunction);

    //! Get current magnification function.
    GLint getMagnificationFunction() const { return (m_magnificationFunction); }

    //! Set the minification function.
    void setMinifyingFunction(const GLint a_minifyingFunction);

    //! Get current magnification function.
    GLint getMinifyingFunction() const { return (m_minifyingFunction); }

    //! Set spherical mapping mode \b ON or \b OFF.
    void setSphericalMappingEnabled(const bool a_enabled) { m_useSphericalMapping = a_enabled; }

    //! Get the status of the spherical mapping mode.
    bool getSphericalMappingEnabled() const { return (m_useSphericalMapping); }

    //! Enable or disable Mipmaps.
    void setUseMipmaps(bool a_useMipmaps);

    //! Get the status of Mipmaps mode.
    bool getUseMipmaps() { return (m_useMipmaps); }

    // ! Set an existing image to use as texture
    bool setImage (cImage *a_image);

    //! Image loader (use this to get data about the texture itself).
    cImage* m_image;

    //! Environmental color.
    cColorf m_color;


  private:

	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------
	
	//! Reset internal variables. This function should be called only by constructors.
    void reset();

    //! Initialize GL texture.
    void update();


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

  public:

    //! If \b true, texture bitmap has not yet been sent to video card.
    bool m_updateTextureFlag;

    //! OpenGL texture ID number.
    GLuint m_textureID;

    //! Texture wrap parameter along S (\e GL_REPEAT or \e GL_CLAMP).
    GLint m_wrapSmode;

    //! Texture magnification function. (\e GL_NEAREST or \e GL_LINEAR).
    GLint m_magnificationFunction;

    //! Texture minifying function. (\e GL_NEAREST or \e GL_LINEAR).
    GLint m_minifyingFunction;

    //! If \b true, we use GLU to build mipmaps.
    bool m_useMipmaps;

    //! If \b true, we use spherical mapping.
    bool m_useSphericalMapping;

    //! OpenGL texture mode (\e GL_MODULATE, \e GL_DECAL, \e GL_BLEND, \e GL_REPLACE).
    GLint m_environmentMode;

    //! Texture unit number
    GLenum m_textureUnit;


  protected:

    //! Texture magnification function when Mipmap is OFF.
    GLint m_magnificationFunctionMipmapsOFF;

    //! Texture mignifying function when Mipmap is OFF.
    GLint m_minifyingFunctionMipmapsOFF;

    //! Texture magnification function when Mipmap is ON.
    GLint m_magnificationFunctionMipmapsON;

    //! Texture mignifying function when Mipmap is ON.
    GLint m_minifyingFunctionMipmapsON;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

