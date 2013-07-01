//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2013, CHAI3D.
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
//==============================================================================

//------------------------------------------------------------------------------
#include "materials/CTexture2d.h"
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cTexture2d.
*/
//==============================================================================
cTexture2d::cTexture2d()
{
    // set default texture unit
    m_textureUnit = GL_TEXTURE1_ARB;

    // create image
    m_image = new cImage();

    // initialize internal variables
    reset();    
}


//==============================================================================
/*!
    Destructor of cTexture2d.
*/
//==============================================================================
cTexture2d::~cTexture2d()
{
    if (m_textureID != 0)
    {
        #ifdef C_USE_OPENGL
        glDeleteTextures(1, &m_textureID);
        #endif

        m_textureID = 0;
    }
}


//==============================================================================
/*!
    Creates a copy of itself.

    \return Return pointer to new object.
*/
//==============================================================================
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
    obj->m_magFunction              = m_magFunction;
    obj->m_minFunction              = m_minFunction;
    obj->m_useMipmaps               = m_useMipmaps;
    obj->m_useSphericalMapping      = m_useSphericalMapping;
    obj->m_environmentMode          = m_environmentMode;

    // return
    return (obj);
}


//==============================================================================
/*!
    Reset internal variables. This function should be called only by constructors.
*/
//==============================================================================
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
    m_magFunctionMipmapsOFF = GL_LINEAR;
    m_minFunctionMipmapsOFF = GL_LINEAR;
    m_magFunctionMipmapsON  = GL_LINEAR;
    m_minFunctionMipmapsON  = GL_LINEAR_MIPMAP_LINEAR;

    // set the magnification function. (GL_NEAREST or GL_LINEAR)
    m_magFunction = m_magFunctionMipmapsOFF;

    // set the minifying function. (GL_NEAREST or GL_LINEAR)
    m_minFunction = m_minFunctionMipmapsOFF; 
}


//==============================================================================
/*!
      Enable texturing and set this texture as the current texture

      \param	a_options Rendering options
*/
//==============================================================================
void cTexture2d::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

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
        if (m_textureID != 0)
        {
            if (glIsTexture(m_textureID) == false)
            {
                m_textureID = 0;
                m_updateTextureFlag = true;
            }
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
    glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_MAG_FILTER, m_magFunction);

    // Set the texture minifying function to either GL_NEAREST or GL_LINEAR.
    glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_MIN_FILTER, m_minFunction);

    // set the environment mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE)
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_environmentMode);

    // set the environmental color
    glTexEnvfv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_COLOR, &m_color.pColor()[0]);

    // set default vertex color which combined with the texture (white).
    glColor4f(1.0, 1.0, 1.0, 1.0);

#endif
}


//==============================================================================
/*!
      Load a texture image file.

      \param    a_fileName  Filename.
*/
//==============================================================================
bool cTexture2d::loadFromFile(const string& a_fileName)
{
    return (m_image->loadFromFile(a_fileName));
}


//==============================================================================
/*!
      Save current texture to file.

      \param    a_fileName  Filename
*/
//==============================================================================
bool cTexture2d::saveToFile(const string& a_fileName)
{
    return (m_image->saveToFile(a_fileName));
}


//==============================================================================
/*!
      Generate texture from memory data, to prepare for rendering.
*/
//==============================================================================
void cTexture2d::update()
{
#ifdef C_USE_OPENGL

    if (m_textureID == 0)
    {
        glGenTextures(1, &m_textureID);
        glBindTexture(GL_TEXTURE_2D, m_textureID);

        glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_WRAP_S, m_wrapSmode);
        glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_WRAP_T, m_wrapTmode);
        glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_MAG_FILTER, m_magFunction);
        glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_MIN_FILTER, m_minFunction);

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
    }
    else
    {
        glBindTexture(GL_TEXTURE_2D, m_textureID);

        glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_WRAP_S, m_wrapSmode);
        glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_WRAP_T, m_wrapTmode);
        glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_MAG_FILTER, m_magFunction);
        glTexParameteri(GL_TEXTURE_2D ,GL_TEXTURE_MIN_FILTER, m_minFunction);

        glTexSubImage2D(GL_TEXTURE_2D, 
                        0, 
                        0, 
                        0, 
                        m_image->getWidth(), 
                        m_image->getHeight(), 
                        m_image->getFormat(), 
                        m_image->getType(), 
                        m_image->getData());
    }
    
    /*
    glPixelStorei(GL_UNPACK_ALIGNMENT,   4);
    glPixelStorei(GL_UNPACK_ROW_LENGTH,  0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS,   0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
    */

    if (m_useMipmaps)
    {
        glEnable (GL_TEXTURE_2D);
        glGenerateMipmap(GL_TEXTURE_2D);
    }

#endif
}


//==============================================================================
/*!
      Sets the wrap parameter for texture coordinate s to either GL_CLAMP or
      GL_REPEAT. GL_CLAMP causes s coordinates to be clamped to the
      range [0,1] and is useful for preventing wrapping artifacts when mapping
      a single image onto an object. GL_REPEAT causes the integer part of the
      s coordinate to be ignored; OpenGL uses only the fractional part, thereby
      creating a repeating pattern. Border texture elements are accessed only
      if wrapping is set to GL_CLAMP. Initially, GL_TEXTURE_WRAP_S is set
      to GL_REPEAT.

      \param    a_wrapMode  value shall be either GL_REPEAT or GL_CLAMP
*/
//==============================================================================
void cTexture2d::setWrapMode(const GLint& a_wrapMode)
{
    m_wrapSmode = a_wrapMode;
    m_wrapTmode = a_wrapMode;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
