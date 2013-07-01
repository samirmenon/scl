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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1067 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CShapeTorusH
#define CShapeTorusH
//------------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CShapeTorus.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Shape - Torus.
*/
//==============================================================================

//==============================================================================
/*!    
    \class      cShapeTorus
    \ingroup    scenegraph

    \brief
    3D Shape Torus

    \details
    cShapeTorus describes a simple torus potential field.
*/
//==============================================================================
class cShapeTorus : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cShapeTorus.
    cShapeTorus(const double& a_innerRadius, 
                const double& a_outerRadius, 
                cMaterial* a_material = NULL);

    //! Destructor of cShapeTorus.
    virtual ~cShapeTorus() {};


    //--------------------------------------------------------------------------
    // VIRTUAL PUBLIC METHODS: (cGenericObject)
    //--------------------------------------------------------------------------

public:

    //! Create a copy of itself.
    virtual cShapeTorus* copy(const bool a_duplicateMaterialData = false,
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
    // PUBLIC METHODS: (cShapeTorus)
    //--------------------------------------------------------------------------

public:

    //! Set inside and outside radius of torus.
    void setSize(const double& a_innerRadius, 
                 const double& a_outerRadius);

    //! Get inside radius of torus.
    inline double getInnerRadius() const { return (m_innerRadius); }

    //! Get inside radius of torus.
    inline double getOuterRadius() const { return (m_outerRadius); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS: (cShapeTorus)
    //--------------------------------------------------------------------------

protected:

    //! Inside radius of torus.
    double m_innerRadius;

    //! Outside radius of torus.
    double m_outerRadius;

    //! Resolution of the graphical model.
    unsigned int m_resolution;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
