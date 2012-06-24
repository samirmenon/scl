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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 738 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CShapeSphereH
#define CShapeSphereH
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CShapeSphere.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Shape - Sphere.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cShapeSphere
    \ingroup    scenegraph

    \brief      
    Implementation of a virtual sphere shape.
*/
//===========================================================================
class cShapeSphere : public cGenericObject
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cShapeSphere.
    cShapeSphere(const double& a_radius, 
                 cMaterial* a_material = NULL);
    
    //! Destructor of cShapeSphere.
    virtual ~cShapeSphere() {};


    //-----------------------------------------------------------------------
    // VIRTUAL METHODS: (cGenericObject)
    //-----------------------------------------------------------------------

    //! Create a copy of itself.
    virtual cShapeSphere* copy(const bool a_duplicateMaterialData = false,
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

    //-----------------------------------------------------------------------
    // METHODS: (cShapeSphere)
    //-----------------------------------------------------------------------

    //! Set radius of sphere.
    void setRadius(const double& a_radius);

    //! Get radius of sphere.
    inline double getRadius() const { return (m_radius); }


  protected:
    
    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! radius of sphere.
    double m_radius;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
