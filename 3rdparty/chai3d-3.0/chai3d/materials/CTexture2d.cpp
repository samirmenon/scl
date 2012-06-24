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
    \author    Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 492 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "materials/CTexture2d.h"
//---------------------------------------------------------------------------
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    A texture contains a 2D bitmap which can be projected onto the
    polygons of a 3D solid.

    \fn         cTexture2d::cTexture2d()
*/
//===========================================================================
cTexture2d::cTexture2d()
{
    // set default texture unit
    m_textureUnit = GL_TEXTURE1_ARB;

	// create image
	m_image = new cImage();

    // initialize internal variables
    reset();    
}


//===========================================================================
/*!
    Destructor of cTexture2d.

    \fn         cTexture2d::~cTexture2d()
*/
//===========================================================================
cTexture2d::~cTexture2d()
{
    if (m_textureID != 0)
    {
        glDeleteTextures(1, &m_textureID);
        m_textureID = 0;
    }
}


//===========================================================================
/*!
    Creates a copy of itself.

    \fn     cTexture2d* cTexture2d::copy()
    \return Return pointer to new object.
*/
//===========================================================================
cTexture2d* cTexture2d::copy()
{
    // create new instance
    cTexture2d* obj = new cTexture2d();

    // create copy of image data
    obj->m_image = m_image->copy();

    // copy all variables
    obj->m_enabled                  = m_enabled;
    obj->m_updateTextureFlag        = m_updateTextureFlag;
    obj->m_textureID                = m_textureID;
    obj->m_wrapSmode                = m_wrapSmode;
    obj->m_wrapTmode                = m_wrapTmode;
    obj->m_magnificationFunction    = m_magnificationFunction;
    obj->m_minifyingFunction        = m_minifyingFunction;
    obj->m_useMipmaps               = m_useMipmaps;
    obj->m_useSphericalMapping      = m_useSphericalMapping;
    obj->m_environmentMode          = m_environmentMode;

    // return
    return (obj);
}


//===========================================================================
/*!
    Reset internal variables. This function should be called only by constructors.

    \fn         void cTexture2d::reset()
*/
//===========================================================================
void cTexture2d::reset()
{
    // id number provided by OpenGL once texture is stored in graphics
    // card memory
    m_textureID = 0;

    // texture has not yet been rendered
    m_updateTextureFlag = true;

    // Tile the texture in X. (GL_REPEAT or GL_CLAMP)
    m_wrapSmode = GL_REPEAT;

    // Tile the texture in Y. (GL_REPEAT or GL_CLAMP)
    m_wrapTmode = GL_REPEAT;

    // set environmental mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE)
    m_environmentMode = GL_MODULATE;

    // set environmental color
    m_color.set(1.0, 1.0, 1.0, 0.0);

    // set spherical mode
    m_useSphericalMapping = false;

    // use mipmaps
    m_useMipmaps = false;

    // default settings
    m_magnificationFunctionMipmapsOFF   = GL_LINEAR;
    m_minifyingFunctionMipmapsOFF       = GL_LINEAR;
    m_magnificationFunctionMipmapsON    = GL_LINEAR;
    m_minifyingFunctionMipmapsON        = GL_LINEAR_MIPMAP_LINEAR;

    // set the magnification function. (GL_NEAREST or GL_LINEAR)
    m_magnificationFunction = m_magnificationFunctionMipmapsOFF;

    // set the minifying function. (GL_NEAREST or GL_LINEAR)
    m_minifyingFunction = m_minifyingFunctionMipmapsOFF; 
}


//===========================================================================
/*!
      Enable texturing and set this texture as the current texture

      \fn       void cTexture2d::render(cRenderOptions& a_options)
	  \param	a_options Rendering options
*/
//===========================================================================
void cTexture2d::render(cRenderOptions& a_options)
{
	// check if texture is enabled
	if (!m_enabled) { return; }

	// check if materials should be rendered
	if (!a_options.m_render_textures) { return; }

	// check image texture
    if (m_image->isInitialized() == 0) return;

    // Only check residency in memory if we weren't going to
    // update the texture anyway...
    if (m_updateTextureFlag == false)
    {
        GLboolean texture_is_resident;
        glAreTexturesResident(1, &m_textureID, &texture_is_resident);

        if (texture_is_resident == false)
        {
            m_updateTextureFlag = true;
        }
    }
    
	// setup texture settings
    glActiveTextureARB(GL_TEXTURE1_ARB);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE_EXT);

    glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB_EXT, GL_REPLACE);
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB_EXT, GL_PREVIOUS_EXT);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_RGB_EXT, GL_SRC_COLOR);

    glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_ALPHA_EXT, GL_ADD_SIGNED_EXT);
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_ALPHA_EXT, GL_PREVIOUS_EXT);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_ALPHA_EXT, GL_SRC_ALPHA);
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE1_ALPHA_EXT, GL_TEXTURE);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND1_ALPHA_EXT, GL_ONE_MINUS_SRC_ALPHA);

    // enable texturing
    glEnable(GL_TEXTURE_2D);

    // setup texture or update
    if (m_updateTextureFlag)
    {
        // update texture map
        update();
        m_updateTextureFlag = false;
    }
    else
    {
        // make this the current texture
        glBindTexture(GL_TEXTURE_2D, m_textureID);
    }

    // enable or disable spherical mapping
    if (m_useSphericalMapping)
    {
        glEnable(GL_TEXTURE_GEN_S);
        glEnable(GL_TEXTURE_GEN_T);
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
        glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
    }
    else
    {
        glDisable(GL_TEXTURE_GEN_S);
        glDisable(GL_TEXTURE_GEN_T);
    }
    
    // Sets the wrap parameter for texture coordinate s to either
    // GL_CLAMP or GL_REPEAT.
    glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_WRAP_S, m_wrapSmode);
    glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_WRAP_T, m_wrapTmode);

    // Set the texture magnification function to either GL_NEAREST or GL_LINEAR.
    glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_MAG_FILTER, m_magnificationFunction);

    // Set the texture minifying function to either GL_NEAREST or GL_LINEAR.
    glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_MIN_FILTER, m_minifyingFunction);

    // set the environment mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE)
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_environmentMode);

    // set the environmental color
    glTexEnvfv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, &m_color.pColor()[0]);

    // set default vertex color which combined with the texture (white).
    glColor4f(1.0, 1.0, 1.0, 1.0);
}


//===========================================================================
/*!
      Load a texture image file.

      \fn       bool cTexture2d::loadFromFile(const string& a_fileName)
      \param    a_fileName  Filename.
*/
//===========================================================================
bool cTexture2d::loadFromFile(const string& a_fileName)
{
    return (m_image->loadFromFile(a_fileName));
}


//===========================================================================
/*!
      Save current texture to file.

      \fn       bool cTexture2d::saveToFile(const string& a_fileName)
      \param    a_fileName  Filename
*/
//===========================================================================
bool cTexture2d::saveToFile(const string& a_fileName)
{
    return (m_image->saveToFile(a_fileName));
}


//===========================================================================
/*!
      Generate texture from memory data, to prepare for rendering.

      \fn         void cTexture2d::update()
*/
//===========================================================================
void cTexture2d::update()
{
    if (m_textureID != 0)
    {
        // Deletion can make for all kinds of new hassles, particularly
        // when re-initializing a whole display context, since opengl
        // automatically starts re-assigning texture ID's.
        glDeleteTextures(1, &m_textureID);
        m_textureID = 0;
    }

    // Generate a texture ID and bind to it
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &m_textureID);
    glBindTexture(GL_TEXTURE_2D, m_textureID);

    glPixelStorei(GL_UNPACK_ALIGNMENT,   4);
    glPixelStorei(GL_UNPACK_ROW_LENGTH,  0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS,   0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

    glTexImage2D(GL_TEXTURE_2D,
                    0,
                    GL_RGBA,
                    m_image->getWidth(),
                    m_image->getHeight(),
                    0,
                    m_image->getFormat(),
                    m_image->getType(),
                    m_image->getData()
        );
   
    if (m_useMipmaps)
    {
        glEnable (GL_TEXTURE_2D);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
}


//===========================================================================
/*!
      Sets the wrap parameter for texture coordinate s to either GL_CLAMP or
      GL_REPEAT. GL_CLAMP causes s coordinates to be clamped to the
      range [0,1] and is useful for preventing wrapping artifacts when mapping
      a single image onto an object. GL_REPEAT causes the integer part of the
      s coordinate to be ignored; OpenGL uses only the fractional part, thereby
      creating a repeating pattern. Border texture elements are accessed only
      if wrapping is set to GL_CLAMP. Initially, GL_TEXTURE_WRAP_S is set
      to GL_REPEAT.

      \fn       void cTexture2d::setWrapMode(const GLint& a_wrapMode)
      \param    a_wrapMode  value shall be either GL_REPEAT or GL_CLAMP
*/
//===========================================================================
void cTexture2d::setWrapMode(const GLint& a_wrapMode)
{
    m_wrapSmode = a_wrapMode;
	m_wrapTmode = a_wrapMode;
}

