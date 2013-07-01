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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 425 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CTexture2dH
#define CTexture2dH
//------------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "materials/CTexture1d.h"
#include "graphics/CImage.h"
#include <string>
#include <stdio.h>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*! 
    \file   CTexture2D.h

    \brief  
    <b> Materials </b> \n 
    2D Texture.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cTexture2d
    \ingroup    materials
    
    \brief      
    2D Texture Map

    \details      
    cTexture1d describes a 2D bitmap texture used for OpenGL texture-mapping.
*/
//==============================================================================
class cTexture2d : public cTexture1d
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cTexture2d.
    cTexture2d();

    //! Destructor of cTexture2d.
    virtual ~cTexture2d();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Create a copy of current object.
    virtual cTexture2d* copy();

    //! Load an texture image file.
    virtual bool loadFromFile(const std::string& a_fileName);

    //! Save a texture image file.
    virtual bool saveToFile(const std::string& a_fileName);

    //! Enable texturing and set this texture as the current texture.
    virtual void render(cRenderOptions& a_options);

    //! Call this to force texture re-initialization.
    virtual void markForUpdate() { m_updateTextureFlag = true; }

    //! Set the texture wrap mode.
    virtual void setWrapMode(const GLint& a_wrapMode);

    //! Get the texture wrap mode of T.
    GLint getWrapTmode() const { return (m_wrapSmode); }


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! Reset internal variables. This function should be called only by constructors.
    void reset();

    //! Initialize GL texture.
    void update();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Texture wrap parameter along T (\e GL_REPEAT or \e GL_CLAMP).
    GLint m_wrapTmode;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

