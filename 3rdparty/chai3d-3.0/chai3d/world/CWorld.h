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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 774 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CWorldH
#define CWorldH
//---------------------------------------------------------------------------
#include "display/CCamera.h"
#include "world/CGenericObject.h"
#include "graphics/CColor.h"
#include "graphics/CTriangle.h"
#include "graphics/CFog.h"
#include "materials/CTexture2d.h"
#include <vector>
//---------------------------------------------------------------------------
class cGenericLight;
//---------------------------------------------------------------------------
//! The maximum number of lights that we expect OpenGL to support
#define C_MAXIMUM_OPENGL_LIGHT_COUNT 8
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CWorld.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual World.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cWorld
    \ingroup    scenegraph 

    \brief      
    cWorld defines the typical root of the CHAI scene graph. It stores 
    lights,  allocates textures, and serves as the root for scene-wide 
    collision detection.
*/
//===========================================================================
class cWorld : public cGenericObject
{
  friend class cGenericLight;

  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cWorld.
    cWorld();
    
    //! Destructor of cWorld.
    virtual ~cWorld();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Set the background color used when rendering.
    void setBackgroundColor(const GLfloat a_red, 
                            const GLfloat a_green,
                            const GLfloat a_blue);

    //! Set the background color used when rendering.
    void setBackgroundColor(const cColorf& a_color);

    //! Get the background color used when rendering.
    cColorf getBackgroundColor() const { return (m_backgroundColor); }

    //! Enable or disable the rendering of this world's light sources.
    void enableLightSourceRendering(bool enable) { m_renderLightSources = enable; }

    //! Set Shadow Intensity. (0.0 = no shadow - full light. 1.0 = full shadow - no light)
    void setShadowIntensity(double a_intensity) { m_shadowIntensity = cClamp01(a_intensity); }

    // Get shadow intensity
    double getShadowIntensity() { return (m_shadowIntensity); }

    //! Compute collision detection between a ray segment and all objects in this world.
    virtual bool computeCollisionDetection(cVector3d& a_segmentPointA,
                                           cVector3d& a_segmentPointB,
                                           cCollisionRecorder& a_recorder,
                                           cCollisionSettings& a_settings);

    //! Update the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN);

    //! Render OpenGL lights.
    virtual void render(cRenderOptions& a_options);

    //! Get access to a particular light source (between 0 and MAXIMUM_OPENGL_LIGHT_COUNT-1).
    virtual cGenericLight* getLightSource(int index);  
    
    //! It's useful to store the world's modelview matrix, for rendering stuff in "global" coordinates.
    double m_worldModelView[16];
    

protected:

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Add a light source to this world.
    bool addLightSource(cGenericLight* a_light);

    //! Remove a light source from this world.
    bool removeLightSource(cGenericLight* a_light);


    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

public:
    //! Background color. Default color is black.
    cColorf m_backgroundColor;
    
    //! Fog property
    cFog* m_fog;

    //! List of textures.
    vector<cTexture2d*> m_textures;
    
    //! List of light sources.
    vector<cGenericLight*> m_lights;
    
protected:

    //! Should I render my light sources, or just use the current OpenGL light state?
    bool m_renderLightSources;    

    //! Shadow intensity. Values ranges between 0.0 and 1.0 (0.0 = no shadow - full light. 1.0 = full shadow - no light).
    double m_shadowIntensity;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

