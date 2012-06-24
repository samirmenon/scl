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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 435 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CShadowMapH
#define CShadowMapH
//---------------------------------------------------------------------------
#include "graphics/CImage.h"
#include "graphics/CColor.h"
#include "materials/CTexture2d.h"
#include "world/CWorld.h"
#include <string>
#include <stdio.h>
//---------------------------------------------------------------------------

//===========================================================================
/*! 
    \file       CShadowMap.h

    \brief  
    <b> Lighting </b> \n 
    Shadow Map.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cShadowMap
    \ingroup    lighting
    
    \brief      
    cShadowMap implements a shadow map texture. 
*/
//===========================================================================
class cShadowMap : public cTexture2d
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cShadowMap.
    cShadowMap();

    //! Destructor of cShadowMap.
    virtual ~cShadowMap();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Set the size of the shadowmap by defining the width and height in pixels.
    void setSize(const unsigned int a_width, const unsigned int a_height);

    //! Enable texturing and set this texture as the current texture.
    virtual void render(cRenderOptions& a_options);
	
	//! Call this to force texture re-initialization.
    virtual void markForUpdate() { m_updateTextureFlag = true; }

	//! Update the shadow map by passing a reference to the related light source.
	bool updateMap(cWorld* a_world, 
				   const cVector3d& a_lightPos, 
				   const cVector3d& a_lightLookat, 
				   const cVector3d& a_lightUp, 
				   const double a_lightFieldViewAngle,
				   const double a_distanceNear,
				   const double a_distanceFar);

    //! Copy shadowmap from GPU memory to m_image object in RAM memory.
    bool copyMapFromGPUtoImage();

    //! Copy shadowmap from GPU memory to an image object in RAM memory.
    bool copyMapFromGPUtoImage(cImage* a_image);

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------
	
	//! Projection matrix of the light source creating this shadow.
	cTransform m_lightProjectionMatrix;

	//! View matrix of the light source creating this shadow.
	cTransform m_lightViewMatrix;

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

    //! OpenGL framebuffer used to render the shadowmap.
    GLuint m_fbo;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

