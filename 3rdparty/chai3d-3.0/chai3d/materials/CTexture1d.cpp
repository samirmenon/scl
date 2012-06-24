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
#include "materials/CTexture1d.h"
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

    \fn         cTexture1d::cTexture1d()
*/
//===========================================================================
cTexture1d::cTexture1d()
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
    Destructor of cTexture1d.

    \fn         cTexture1d::~cTexture1d()
*/
//===========================================================================
cTexture1d::~cTexture1d()
{
    if (m_textureID != (unsigned int)-1)
    {
        glDeleteTextures(1, &m_textureID);
        m_textureID = (unsigned int)-1;
    }
}


//===========================================================================
/*!
    Creates a copy of itself.

    \fn     cTexture1d* cTexture2d::copy()
    \return Return pointer to new object.
*/
//===========================================================================
cTexture1d* cTexture1d::copy()
{
    // create new instance
    cTexture1d* obj = new cTexture1d();

    // create copy of image data
    obj->m_image = m_image->copy();

    // copy all variables
    obj->m_enabled                  = m_enabled;
    obj->m_updateTextureFlag        = m_updateTextureFlag;
    obj->m_textureID                = m_textureID;
    obj->m_wrapSmode                = m_wrapSmode;
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

    \fn         void cTexture1d::
    reset()
*/
//===========================================================================
void cTexture1d::reset()
{
    // id number provided by OpenGL once texture is stored in graphics
    // card memory
    m_textureID = (unsigned int)-1;

    // texture has not yet been rendered
    m_updateTextureFlag = true;

    // Tile the texture in X. (GL_REPEAT or GL_CLAMP)
    m_wrapSmode = GL_REPEAT;

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

      \fn       void cTexture1d::render(cRenderOptions& a_options)
	  \param	a_options Rendering options
*/
//===========================================================================
void cTexture1d::render(cRenderOptions& a_options)
{
	// check if shadow is enabled
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
    glEnable(GL_TEXTURE_1D);

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
        glBindTexture(GL_TEXTURE_1D, m_textureID);
    }

    // enable or disable spherical mapping
    if (m_useSphericalMapping)
    {
        glEnable(GL_TEXTURE_GEN_S);
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
    }
    else
    {
        glDisable(GL_TEXTURE_GEN_S);
    }
    
    // Sets the wrap parameter for texture coordinate s to either
    // GL_CLAMP or GL_REPEAT.
    glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_WRAP_S, m_wrapSmode);

    // Set the texture magnification function to either GL_NEAREST or GL_LINEAR.
    glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_MAG_FILTER, m_magnificationFunction);

    // Set the texture minifying function to either GL_NEAREST or GL_LINEAR.
    glTexParameteri(GL_TEXTURE_1D ,GL_TEXTURE_MIN_FILTER, m_minifyingFunction);

    // set the environment mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE)
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_environmentMode);

    // make this the current texture
    glBindTexture(GL_TEXTURE_1D, m_textureID);

    // set the environmental color
    glTexEnvfv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, &m_color.pColor()[0]);

    // set default vertex color which combined with the texture (white).
    glColor4f(1.0, 1.0, 1.0, 1.0);
}


//===========================================================================
/*!
      Load a texture image file.

      \fn       bool cTexture1d::loadFromFile(const string& a_fileName)
      \param    a_fileName  Filename.
*/
//===========================================================================
bool cTexture1d::loadFromFile(const string& a_fileName)
{
    return (m_image->loadFromFile(a_fileName));
}


//===========================================================================
/*!
      Save current texture to file.

      \fn       bool cTexture1d::saveToFile(const string& a_fileName)
      \param    a_fileName  Filename
*/
//===========================================================================
bool cTexture1d::saveToFile(const string& a_fileName)
{
    return (m_image->saveToFile(a_fileName));
}


//===========================================================================
/*!
      Generate texture from memory data, to prepare for rendering.

      \fn         void cTexture1d::update()
*/
//===========================================================================
void cTexture1d::update()
{
    if (m_textureID != (unsigned int)-1)
    {
        // Deletion can make for all kinds of new hassles, particularly
        // when re-initializing a whole display context, since opengl
        // automatically starts re-assigning texture ID's.
        glDeleteTextures(1,&m_textureID);
        m_textureID = (unsigned int)-1;
    }

    // Generate a texture ID and bind to it
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1,&m_textureID);
    glBindTexture(GL_TEXTURE_2D, m_textureID);

    glPixelStorei(GL_UNPACK_ALIGNMENT,   4);
    glPixelStorei(GL_UNPACK_ROW_LENGTH,  0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS,   0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);

    glTexImage1D(GL_TEXTURE_1D,
                    0,
                    GL_RGBA,
                    m_image->getWidth() * m_image->getHeight(),
                    0,
                    m_image->getFormat(),
                    m_image->getType(),
                    m_image->getData()
        );

    if (m_useMipmaps)
    {
        glEnable (GL_TEXTURE_1D);
        glGenerateMipmap(GL_TEXTURE_1D);
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

      \fn       void cTexture1d::setWrapMode(const GLint& a_wrapMode)
      \param    a_wrapMode  value shall be either GL_REPEAT or GL_CLAMP
*/
//===========================================================================
void cTexture1d::setWrapMode(const GLint& a_wrapMode)
{
    m_wrapSmode = a_wrapMode;
}


//===========================================================================
/*!
    The texture magnification function is used when the pixel being textured
    maps to an area less than or equal to one texture element.
    It sets the texture magnification function to either GL_NEAREST or GL_LINEAR.

    \fn       void cTexture1d::setMagnificationFunction(const GLint a_magnificationFunction)
    \param    a_magnificationFunction  value shall be either GL_NEAREST or GL_LINEAR.
*/
//==========================================================================
void cTexture1d::setMagnificationFunction(const GLint a_magnificationFunction)
{
    m_magnificationFunction = a_magnificationFunction;

    if (m_useMipmaps)
    {
             
            m_magnificationFunctionMipmapsON = m_magnificationFunction;
    }
    else
    {
            m_magnificationFunctionMipmapsOFF = m_magnificationFunction;
    }
}


//===========================================================================
/*!
    The texture minifying function is used whenever the pixel being textured
    maps to an area greater than one texture element. There are six defined
    minifying functions. Two of them use the nearest one or nearest four
    texture elements to compute the texture value. The other four use mipmaps.
    A mipmap is an ordered set of arrays representing the same image at
    progressively lower resolutions. If the texture has dimensions 2nx2m
    there are max(n, m) + 1 mipmaps. The first mipmap is the original texture,
    with dimensions 2nx2m. Each subsequent mipmap has dimensions 2k1x2l1 where 2
    kx2l are the dimensions of the previous mipmap, until either k = 0 or l = 0.
    At that point, subsequent mipmaps have dimension 1x2l1 or 2k1x1 until the
    final mipmap, which has dimension 1x1. Mipmaps are defined using
    glTexImage1D or glTexImage2D with the level-of-detail argument indicating
    the order of the mipmaps. Level 0 is the original texture; level bold
    max(n, m) is the final 1x1 mipmap.

    \fn       void cTexture1d::setMinifyingFunction(const GLint a_minifyingFunction);
    \param    a_minifyingFunction  value shall be either GL_NEAREST or GL_LINEAR.
*/
//==========================================================================
void cTexture1d::setMinifyingFunction(const GLint a_minifyingFunction)
{
    m_minifyingFunction = a_minifyingFunction;

    if (m_useMipmaps)
    {
             
            m_minifyingFunctionMipmapsON = m_minifyingFunction;
    }
    else
    {
            m_minifyingFunctionMipmapsOFF = m_minifyingFunction;
    }
}


//===========================================================================
/*!
      Enable or Disable Mipmaps. Please note that this function does not 
      work with all graphics cards! (ATI graphic cards seem to have problems)

      \fn       void cTexture1d::setUseMipmaps(bool a_useMipmaps) 
      \param    a_useMipmaps  Mipmaps status.
*/
//===========================================================================
void cTexture1d::setUseMipmaps(bool a_useMipmaps) 
{ 
    m_useMipmaps = a_useMipmaps;
    
    if (m_useMipmaps)
    {
        m_minifyingFunction     = m_minifyingFunctionMipmapsON;
        m_magnificationFunction = m_magnificationFunctionMipmapsON;
    }
    else
    {
        m_minifyingFunction     = m_minifyingFunctionMipmapsOFF;
        m_magnificationFunction = m_magnificationFunctionMipmapsOFF;
    }
}


//===========================================================================
/*!
      Cleanly remove existing image and set a new, existing image as the bitmap source

      \fn       void cBitmap::setImage(cImage *a_image);
      \param    a_image a pointer to the new image
*/
//===========================================================================
bool cTexture1d::setImage(cImage *a_image)
{
    if (!a_image) return false;

    if (m_image)
    {
        delete m_image;
    }

    m_image = a_image;

    return (true);
}
