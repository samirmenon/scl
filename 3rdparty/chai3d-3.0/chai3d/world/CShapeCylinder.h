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
    \author    Sebastien Grange
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1067 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CShapeCylinderH
#define CShapeCylinderH
//------------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
//------------------------------------------------------------------------------
#ifdef C_USE_OPENGL
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CShapeCylinder.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Shape - Cylinder.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cShapeCylinder
    \ingroup    scenegraph

    \brief
    3D Shape Cylinder

    \details
    Implementation of a virtual cylinder shape.
*/
//==============================================================================
class cShapeCylinder: public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cShapeCylinder.
    cShapeCylinder(const double a_baseRadius, 
                   const double a_topRadius, 
                   const double a_height, 
                   cMaterial* a_material = NULL);
    
    //! Destructor of cShapeCylinder.
    virtual ~cShapeCylinder() {};


    //--------------------------------------------------------------------------
    // VIRTUAL PUBLIC METHODS: (cGenericObject)
    //--------------------------------------------------------------------------

public:

    //! Create a copy of itself.
    virtual cShapeCylinder* copy(const bool a_duplicateMaterialData = false,
                                 const bool a_duplicateTextureData = false, 
                                 const bool a_duplicateMeshData = false,
                                 const bool a_buildCollisionDetector = false);

    //! Render object in OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! Update bounding box of current object.
    virtual void updateBoundaryBox();

    //! Scaling the object by a defined factor.
    virtual void scaleObject(const double& a_scaleFactor);

    //! Update the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS: (cShapeCylinder)
    //--------------------------------------------------------------------------

public:

    //! Set base radius of cylinder.
    void setBaseRadius(double a_baseRadius) { m_baseRadius = cAbs(a_baseRadius); updateBoundaryBox(); invalidateDisplayList();}

    //! Set top radius of cylinder.
    void setTopRadius(double a_topRadius) { m_topRadius = cAbs(a_topRadius); updateBoundaryBox(); invalidateDisplayList();}

    //! Set height of cylinder.
    void setHeight(double a_height) { m_height = cAbs(a_height); updateBoundaryBox(); invalidateDisplayList();}

    //! Get base radius of cylinder.
    inline double getBaseRadius() const { return (m_baseRadius); }

    //! Get top radius of cylinder.
    inline double getTopRadius() const { return (m_topRadius); }

    //! Get height of cylinder.
    inline double getHeight() const { return (m_height); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS: (cShapeCylinder)
    //--------------------------------------------------------------------------

protected:

    //! base radius of cylinder.
    double m_baseRadius;

    //! top radius of cylinder.
    double m_topRadius;

    //! height of cylinder.
    double m_height;

    //! rendering object.
    GLUquadricObj *m_quadric;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
