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
    \author    Dan Morris
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CTexture2DH
#define CTexture2DH
//---------------------------------------------------------------------------
#include "files/CImageLoader.h"
#include "graphics/CColor.h"
#include "graphics/CGenericTexture.h"
#include <string>
#include <stdio.h>
//---------------------------------------------------------------------------

//===========================================================================
/*! 
    \file       CTexture2D.h

    \brief  
    <b> Graphics </b> \n 
    2D Texture.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cTexture2D
    \ingroup    graphics
    
    \brief      
    cTexture2D describes a 2D bitmap texture used for OpenGL texture-mapping
*/
//===========================================================================
class cTexture2D : public cGenericTexture
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cTexture2D.
    cTexture2D();

    //! Destructor of cTexture2D.
    ~cTexture2D();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Load an image file (CHAI currently supports 24-bit .bmp and 32-bit .tga files)
    bool loadFromFile(const char* a_fileName);

    //! Enable texturing and set this texture as the current texture.
    void render();

    //! Call this to force texture re-initialization.
    void markForUpdate() { m_updateTextureFlag = true; }

    //! Set the environment mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE, or -1 for "don't set").
    void setEnvironmentMode(const GLint& a_environmentMode) { m_environmentMode = a_environmentMode; }

    //! Get the environment mode status.
    GLint getEnvironmentMode() { return (m_environmentMode); }

    //! Set the texture wrap mode.
    void setWrapMode(const GLint& a_wrapSmode, const GLint& a_wrapTmode);

    //! Get the texture wrap mode of S.
    GLint getWrapSmode() { return (m_wrapSmode); }

    //! Get the texture wrap mode of T.
    GLint getWrapTmode() { return (m_wrapSmode); }

    //! Set the magnification function.
    void setMagnificationFunction(GLint a_magnificationFunction);

    //! Get current magnification function.
    GLint getMagnificationFunction() { return (m_magnificationFunction); }

    //! Set the minification function.
    void setMinifyingFunction(GLint a_minifyingFunction);

    //! Get current magnification function.
    GLint getMinifyingFunction() { return (m_minifyingFunction); }

    //! Set spherical mapping mode \b ON or \b OFF.
    void setSphericalMappingEnabled(bool a_enabled) { m_useSphericalMapping = a_enabled; }

    //! Get the status of the spherical mapping mode.
    bool getSphericalMappingEnabled() { return (m_useSphericalMapping); }

    //! Image loader (use this to get data about the texture itself).
    cImageLoader m_image;

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

    //! If \b true, texture bitmap has not yet been sent to video card.
    bool m_updateTextureFlag;

    //! OpenGL texture ID number.
    GLuint m_textureID;

    //! Texture wrap parameter along S (\e GL_REPEAT or \e GL_CLAMP).
    GLint m_wrapSmode;

    //! Texture wrap parameter along T (\e GL_REPEAT or \e GL_CLAMP).
    GLint m_wrapTmode;

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
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

