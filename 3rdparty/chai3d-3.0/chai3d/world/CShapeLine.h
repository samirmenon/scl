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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 846 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CShapeLineH
#define CShapeLineH
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "materials/CMaterial.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CShapeLine.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Shape - Line.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cShapeLine
    \ingroup    scenegraph

    \brief      
    cShapeLine describes a simple line potential field
*/
//===========================================================================
class cShapeLine : public cGenericObject
{  
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    public:

    //! Constructor of cShapeLine.
    cShapeLine();

    //! Constructor of cShapeLine.
    cShapeLine(const cVector3d& a_pointA, const cVector3d& a_pointB);

    //! Destructor of cShapeLine.
    virtual ~cShapeLine() {};


    //-----------------------------------------------------------------------
    // VIRTUAL METHODS: (cGenericObject)
    //-----------------------------------------------------------------------

    public:

    //! Create a copy of itself.
    virtual cShapeLine* copy(const bool a_duplicateMaterialData = false,
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

    //! Set line width.
    inline void setLineWidth(const double a_lineWidth) { m_lineWidth = cAbs(a_lineWidth); }

    //! Get line width.
    inline double getLineWidth() const { return (m_lineWidth); }

    //! Specify line stipple pattern.
    void setLineStipple(const GLint a_stippleFactor, const GLushort a_stipplePattern);

    //! Get line stipple factor value.
    GLint getLineStippleFactor() { return (m_stippleFactor); }

    //! Get line stipple pattern value.
    GLushort  getLineStipplePattern() { return (m_stipplePattern); }


    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    public:

    //! Point A of line.
    cVector3d m_pointA;

    //! Point A of line.
    cVector3d m_pointB;

    //! Color of point A of line.
    cColorf m_colorPointA;

    //! Color of point B of line.
    cColorf m_colorPointB;

    //! Specifies a multiplier for each bit in the line stipple pattern.
    GLint m_stippleFactor;

    //! Specifies a 16-bit integer whose bit pattern determine which fragments of a line will be drawn when the line is rasterized.
    GLushort   m_stipplePattern;


    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    protected:

    //! Line width
    double m_lineWidth;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
