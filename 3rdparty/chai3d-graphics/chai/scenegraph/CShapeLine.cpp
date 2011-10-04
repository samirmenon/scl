//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "scenegraph/CShapeLine.h"
#include "graphics/CDraw3D.h"
//---------------------------------------------------------------------------

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
    // initialize line with start and end points.
    m_pointA.copyfrom(a_pointA);
    m_pointB.copyfrom(a_pointB);

    // set color properties
    m_ColorPointA.set(1.0, 1.0, 1.0, 1.0);
    m_ColorPointB.set(1.0, 1.0, 1.0, 1.0);
};


//===========================================================================
/*!
    Render sphere in OpenGL

    \fn       void cShapeLine::render(const int a_renderMode)
    \param    a_renderMode  See cGenericObject::render()
*/
//===========================================================================
void cShapeLine::render(const int a_renderMode)
{
    //-----------------------------------------------------------------------
    // Conditions for object to be rendered
    //-----------------------------------------------------------------------

    if((a_renderMode == CHAI_RENDER_MODE_TRANSPARENT_FRONT_ONLY) ||
       (a_renderMode == CHAI_RENDER_MODE_TRANSPARENT_BACK_ONLY))
    {
        return;
    }

    //-----------------------------------------------------------------------
    // Rendering code here
    //-----------------------------------------------------------------------

    glDisable(GL_LIGHTING);

    // draw line
    glBegin(GL_LINES);
        m_ColorPointA.render();
        glVertex3dv(&m_pointA.x);
        m_ColorPointB.render();
        glVertex3dv(&m_pointB.x);
    glEnd();

    glEnable(GL_LIGHTING);
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
    m_boundaryBoxMin.set(cMin(m_pointA.x, m_pointB.x), 
                         cMin(m_pointA.y, m_pointB.y), 
                         cMin(m_pointA.z, m_pointB.z));

    m_boundaryBoxMax.set(cMax(m_pointA.x, m_pointB.x), 
                         cMax(m_pointA.y, m_pointB.y), 
                         cMax(m_pointA.z, m_pointB.z));
}


//===========================================================================
/*!
    Scale object of defined scale factor

    \fn       void cShapeLine::scaleObject(const cVector3d& a_scaleFactors)
    \param    a_scaleFactors Scale factor
*/
//===========================================================================
void cShapeLine::scaleObject(const cVector3d& a_scaleFactors)
{
    m_pointA.x = a_scaleFactors.x * m_pointA.x;
    m_pointA.y = a_scaleFactors.y * m_pointA.y;
    m_pointA.z = a_scaleFactors.z * m_pointA.z;

    m_pointB.x = a_scaleFactors.x * m_pointB.x;
    m_pointB.y = a_scaleFactors.y * m_pointB.y;
    m_pointB.z = a_scaleFactors.z * m_pointB.z;
}
