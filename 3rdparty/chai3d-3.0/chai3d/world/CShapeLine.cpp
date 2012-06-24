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
#include "world/CShapeLine.h"
#include "graphics/CDraw3D.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cShapeLine.

    \fn     cShapeLine::cShapeLine()
*/
//===========================================================================
cShapeLine::cShapeLine()
{
    // default line width
    m_lineWidth = 1.0;

    // initialize line with start and end points.
    m_pointA.zero();
    m_pointB.zero();

    // set color properties
    m_colorPointA.set(0.7f, 0.7f, 0.7f, 1.0f);
    m_colorPointB.set(0.7f, 0.7f, 0.7f, 1.0f);

    // initialize stippling settings
    m_stippleFactor = 1;
    m_stipplePattern = 0xFFFF;
};


//===========================================================================
/*!
    Constructor of cShapeLine.

    \fn     cShapeLine::cShapeLine(const cVector3d& a_pointA, const cVector3d& a_pointB)
	\param	a_pointA  Point A of line.
	\param	a_pointB  Point B of line.
*/
//===========================================================================
cShapeLine::cShapeLine(const cVector3d& a_pointA, const cVector3d& a_pointB)
{
    // default line width
    m_lineWidth = 1.0;

    // initialize line with start and end points.
    m_pointA.copyfrom(a_pointA);
    m_pointB.copyfrom(a_pointB);

    // set color properties
    m_colorPointA.set(1.0, 1.0, 1.0, 1.0);
    m_colorPointB.set(1.0, 1.0, 1.0, 1.0);
};


//===========================================================================
/*!
    Create a copy of itself.

    \fn     cShapeLine* cShapeLine::copy(const bool a_duplicateMaterialData,
                             const bool a_duplicateTextureData, 
                             const bool a_duplicateMeshData,
                             const bool a_buildCollisionDetector)

    \param      a_duplicateMaterialData  If \b true, material (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateTextureData  If \b true, texture data (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateMeshData  If \b true, mesh data (if available) is duplicated, otherwise it is shared.
    \param      a_buildCollisionDetector  If \b true, collision detector (if available) is duplicated, otherwise it is shared.

	\return		Return new object.
*/
//===========================================================================
cShapeLine* cShapeLine::copy(const bool a_duplicateMaterialData,
                             const bool a_duplicateTextureData, 
                             const bool a_duplicateMeshData,
                             const bool a_buildCollisionDetector)
{
    // create new instance
    cShapeLine* obj = new cShapeLine(m_pointA, m_pointB);
	obj->m_colorPointA = m_colorPointA;
	obj->m_colorPointB = m_colorPointB;
    obj->m_stippleFactor = m_stippleFactor;
    obj->m_stipplePattern = m_stipplePattern;

    // copy generic properties
    copyGenericProperties(obj, a_duplicateMaterialData, a_duplicateTextureData);

    // return
    return (obj);
}


//===========================================================================
/*!
    Render line in OpenGL

    \fn       void cShapeLine::render(cRenderOptions& a_options)

    \param    a_options  Render options.
*/
//===========================================================================
void cShapeLine::render(cRenderOptions& a_options)
{
	/////////////////////////////////////////////////////////////////////////
	// Render parts that are always opaque
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
	{
		// disable lighting
		glDisable(GL_LIGHTING);

        // set line width
        glLineWidth(m_lineWidth);

        // set stipple settings
        if (m_stipplePattern == 0xFFFF)
        {
            glDisable(GL_LINE_STIPPLE);
        }
        else
        {
            glLineStipple(m_stippleFactor, m_stipplePattern);
            glEnable(GL_LINE_STIPPLE);
        }


		// draw line
		glBegin(GL_LINES);
			m_colorPointA.render();
			glVertex3dv(&m_pointA(0) );
			m_colorPointB.render();
			glVertex3dv(&m_pointB(0) );
		glEnd();

       
		// restore OpenGL to default value
		glEnable(GL_LIGHTING);
        glDisable(GL_LINE_STIPPLE);
	}
}


//===========================================================================
/*!
    From the position of the tool, search for the nearest point located
    at the surface of the current object. Decide if the point is located inside
    or outside of the object

    \fn     void cShapeLine::computeLocalInteraction(const cVector3d& a_toolPos,
                                                      const cVector3d& a_toolVel,
                                                      const unsigned int a_IDN)
    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void cShapeLine::computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN)
{
    // the tool can never be inside the line
    m_interactionInside = false;

    // if both point are equal
    m_interactionProjectedPoint = cProjectPointOnSegment(a_toolPos,
                                                         m_pointA,
                                                         m_pointB);
}


//===========================================================================
/*!
    Update bounding box of current object.

    \fn       void cShapeLine::updateBoundaryBox()
*/
//===========================================================================
void cShapeLine::updateBoundaryBox()
{
    m_boundaryBoxMin.set(cMin(m_pointA(0) , m_pointB(0) ), 
                         cMin(m_pointA(1) , m_pointB(1) ), 
                         cMin(m_pointA(2) , m_pointB(2) ));

    m_boundaryBoxMax.set(cMax(m_pointA(0) , m_pointB(0) ), 
                         cMax(m_pointA(1) , m_pointB(1) ), 
                         cMax(m_pointA(2) , m_pointB(2) ));
}


//===========================================================================
/*!
    Scale line with a uniform scale factor.

    \fn       void cShapeLine::scaleObject(const double& a_scaleFactor)
    \param    a_scaleFactor  Scale factor.
*/
//===========================================================================
void cShapeLine::scaleObject(const double& a_scaleFactor)
{
    m_pointA.mul(a_scaleFactor);
    m_pointB.mul(a_scaleFactor);
    updateBoundaryBox();
}


//===========================================================================
/*!
    Specify the line stipple pattern. 
    
    a_stippleFactor Specifies a multiplier for each bit in the line stipple 
    pattern. If factor is 3, for example, each bit in the pattern is used 
    three times before the next bit in the pattern is used. factor is 
    clamped to the range [1, 256] and defaults to 1.

    a_stipplePattern specifies a 16-bit integer whose bit pattern determines
    which fragments of a line will be drawn when the line is rasterized.
    Bit zero is used first; the default pattern is all 1's.


    \param    a_stippleFactor  Scale factor.
    \param    a_stipplePattern  Scale factor.
*/
//===========================================================================
void cShapeLine::setLineStipple(const GLint a_stippleFactor, 
    const GLushort a_stipplePattern)
{
    m_stippleFactor = a_stippleFactor;
    m_stipplePattern = a_stipplePattern;
}