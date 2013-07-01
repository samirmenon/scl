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
#ifndef CShapeBoxH
#define CShapeBoxH
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
    \file       CShapeBox.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Shape - Box.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cShapeBox
    \ingroup    scenegraph

    \brief
    3D Shape Box.

    \details
    Implementation of a virtual box shape.
*/
//==============================================================================
class cShapeBox : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cShapeBox.
    cShapeBox(const double& a_sizeX, 
              const double& a_sizeY, 
              const double& a_sizeZ,
              cMaterial* a_material = NULL);
    
    //! Destructor of cShapeBox.
    virtual ~cShapeBox() {};


    //--------------------------------------------------------------------------
    // PUBLIC VIRTUAL METHODS: (cGenericObject)
    //--------------------------------------------------------------------------

public:

    //! Create a copy of itself.
    virtual cShapeBox* copy(const bool a_duplicateMaterialData = false,
                            const bool a_duplicateTextureData = false, 
                            const bool a_duplicateMeshData = false,
                            const bool a_buildCollisionDetector = true);

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
    // PUBLIC METHODS: (cShapeBox)
    //--------------------------------------------------------------------------

public:

    //! Set the dimensions of the box along each axis.
    void setSize(const double& a_sizeX, 
                 const double& a_sizeY, 
                 const double& a_sizeZ);

    //! Get box length along axis X.
    inline double getSizeX() const { return (2.0 * m_hSizeX); }

    //! Get box length along axis Y.
    inline double getSizeY() const { return (2.0 * m_hSizeY); }

    //! Get box length along axis Z.
    inline double getSizeZ() const { return (2.0 * m_hSizeZ); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS: (cShapeBox)
    //--------------------------------------------------------------------------

  protected:

    //! half length of box along axis Z.
    double m_hSizeX;

    //! half length of box along axis Y.
    double m_hSizeY;

    //! half length of box along axis Z.
    double m_hSizeZ;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
