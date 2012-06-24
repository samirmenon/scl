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
#ifndef CGenericTextureH
#define CGenericTextureH
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CRenderOptions.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGenericTexture.h
    
    \brief  
    <b> Materials </b> \n 
    Texture Base Class.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGenericTexture
    \ingroup    materials

    \brief      
    cGenericTexture implements a base class for handling OpenGL textures.
*/
//===========================================================================
class cGenericTexture
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    
      //! Constructor of cGenericTexture.
    cGenericTexture() { m_enabled = true; };

    //! Destructor of cGenericTexture.
    virtual ~cGenericTexture() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

	//! Create a copy of current object.
    virtual cGenericTexture* copy() { return (NULL); }

    //! Load a texture file.
    virtual bool loadFromFile(const string& a_fileName) { return (false); }

    //! Save a texture image file.
    virtual bool saveToFile(const string& a_fileName) { return (false); }

    //! Enable texturing and set this texture as the current texture.
    virtual void render(cRenderOptions& a_options) {};

    //! Call this to force texture re-initialization.
    virtual void markForUpdate() {}

	//! Enable of disable texture map
	void setEnabled(bool a_enabled) { m_enabled = a_enabled; }

	//! Returns \b true if the texture map is enabled, \b false otherwise.
	bool getEnabled() const { return(m_enabled); }


  protected:

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

	//! If \b true, shadow is enabled, \b false otherwise.
	bool m_enabled;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
