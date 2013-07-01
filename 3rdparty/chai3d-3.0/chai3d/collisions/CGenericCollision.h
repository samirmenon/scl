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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 995 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGenericCollisionH
#define CGenericCollisionH
//------------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file   cGenericCollision.h
    
    \brief  
    <b> Collision Detection </b> \n
    Base Class.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cGenericCollision
    \ingroup    collisions  
    
    \brief    
    cGenericCollision is an abstract class for collision-detection
    algorithms for meshes with line segments.
*/
//==============================================================================
class cGenericCollision
{
  public:

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cGenericCollision.
    cGenericCollision();

    //! Destructor of cGenericCollision.
    virtual ~cGenericCollision() {};


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    //! Do any necessary initialization, such as building trees.
    virtual void initialize(const double a_radius = 0) {};

    //! Provide a visual representation of the method.
    virtual void render(cRenderOptions& a_options) {};

    //! Return the triangles intersected by the given segment, if any.
    virtual bool computeCollision(cGenericObject* a_object,
                                  cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings)
                                  { return (false); }
    
    double getTriangleBoundaryRadius() const { return (m_radiusAroundTriangles); }

    //! Set level of collision tree to display.
    void setDisplayDepth(const int a_depth) { m_displayDepth = a_depth; }

    //! Read level of collision tree being displayed.
    double getDisplayDepth() const { return (m_displayDepth); }


    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

    //! Color properties of the collision object.
    cColorf m_color;


  protected:

    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

    /*!
        Level of collision tree to render... negative values force rendering
        up to and including this level, positive values render _just_ this level.
    */
    int m_displayDepth;
    
    /*! 
        Radius boundary around triangles. This value is must be equal or larger
        than the physical radius of the proxy. 
    */
    double m_radiusAroundTriangles;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
