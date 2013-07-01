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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 699 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "widgets/CLevel.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cLevel.

    \fn         cLevel::cLevel()
*/
//==============================================================================
cLevel::cLevel()
{
    // use vertex colors for line segments
    m_useVertexColors = true;

    // disable material properties
    m_useMaterialProperty = false;

    // use display list to accelerate rendering
    m_useDisplayList = true;

    // set the default number of increments
    m_numIncrements = 100;

    // set display mode
    m_flagSingleIncrementDisplay = false;

    // default width
    setSize(40);

    // set color for activated increments
    m_colorActive.setBlueCornflower();

    // set color for activated increments
    m_colorInactive = m_colorActive;
    m_colorInactive.mul(0.3f);

    // set range
    m_minValue = 0.0;
    m_maxValue = 100.0;

    // set default value
    setValue(50);
}


//==============================================================================
/*!
    Set the width of the cLevel object.

    \fn         void cLevel::setSize(const double a_size)

    \param      a_size  Size.
*/
//==============================================================================
void cLevel::setSize(const double a_size)
{
    // sanity test
    m_size = cAbs(a_size);

    // update level
    update();

    // set value
    setValue(m_value);
}


//==============================================================================
/*!
    Set the number of increments. This value can vary between 2 and 200.

    \fn         void cLevel::setNumIncrements(const int a_numIncrements)

    \param      a_numIncrements  Number of increments.
*/
//==============================================================================
void cLevel::setNumIncrements(const int a_numIncrements)
{
    // clamp value
    m_numIncrements = cClamp(a_numIncrements, 2, 200);

    // update level
    update();

    // set value
    setValue(m_value);
}


//==============================================================================
/*!
    Set the range of input values which are used to command the 
    highlighed segments of cLevel.

    \fn         void cLevel::setRange(const double a_minValue, 
                      const double a_maxValue)

    \param      a_minValue  Minimum value.
    \param      a_maxValue  Maximum value.
*/
//==============================================================================
void cLevel::setRange(const double a_minValue, 
                      const double a_maxValue)
{
    // sanity check
    if (a_minValue == a_maxValue)
    {
        return;
    }

    // store values
    m_minValue = cMin(a_minValue, a_maxValue);
    m_maxValue = cMax(a_minValue, a_maxValue);

    // set value
    setValue(m_value);
}


//==============================================================================
/*!
    Set a new value. The value is compared to the range defined by 
    m_minValue and m_maxValue. The segment are highlighted accordingly.

    \fn         void cLevel::setValue(const double a_value)

    \param      a_value  New value.
*/
//==============================================================================
void cLevel::setValue(const double a_value)
{
    // sanity check
    if (m_numIncrements > (int)(4 * m_vertices->size()))
    {
        update();
    }

    // init variables
    int j=0;

    // clamp value within range
    double value = cClamp(a_value, m_minValue, m_maxValue);
    
    // compute level based on value and range
    int level = (int)((double)(m_numIncrements) * (value - m_minValue) / (m_maxValue - m_minValue));
    if (level > (m_numIncrements-1))
    {
        level = m_numIncrements-1;
    }
    
    // process all increments
    for (int i=0; i<m_numIncrements; i++)
    {
        if ((i == level) || ((i < level) && (!m_flagSingleIncrementDisplay)))
        {
            getVertex(j)->setColor(m_colorActive); j++;
            getVertex(j)->setColor(m_colorActive); j++;
            getVertex(j)->setColor(m_colorActive); j++;
            getVertex(j)->setColor(m_colorActive); j++;
        }
        else
        {
            getVertex(j)->setColor(m_colorInactive); j++;
            getVertex(j)->setColor(m_colorInactive); j++;
            getVertex(j)->setColor(m_colorInactive); j++;
            getVertex(j)->setColor(m_colorInactive); j++;
        }
    }

    // update display list
    invalidateDisplayList();
}


//==============================================================================
/*!
    update polygons of cLevel object

    \fn         void cLevel::update()
*/
//==============================================================================
void cLevel::update()
{
    // clear all triangles
    clear();

    // set dimensions
    m_width = m_size;
    double height = (double)((int)(m_size) / 40) + 1;
    double h2 = 2 * height;
    double y = 0;
    cVector3d n(0,0,1);

    // create polygons
    for (int i=0; i<m_numIncrements; i++)
    {
        // compute position of vertices
        cVector3d v0(0, y, 0.0);
        cVector3d v1(m_width, y, 0.0);
        cVector3d v2(m_width, y+height, 0.0);
        cVector3d v3(0, y+height, 0.0);

        // create new vertices
        int vertexIndex0 = newVertex(v0);
        int vertexIndex1 = newVertex(v1);
        int vertexIndex2 = newVertex(v2);
        int vertexIndex3 = newVertex(v3);

        // define vertex normal
        cVertex* vertex;
        vertex = getVertex(vertexIndex0);
        vertex->setNormal(n);
        vertex = getVertex(vertexIndex1);
        vertex->setNormal(n);
        vertex = getVertex(vertexIndex2);
        vertex->setNormal(n);
        vertex = getVertex(vertexIndex3);
        vertex->setNormal(n);

        // create triangles
        newTriangle(vertexIndex0, vertexIndex1, vertexIndex2);
        newTriangle(vertexIndex0, vertexIndex2, vertexIndex3);

        // increment offset
        y = y + h2;
    }

    m_height = y;

    // update boundary box
    updateBoundaryBox();
}

//==============================================================================
/*!
    Update bounding box of current object.

    \fn       void void cBitmap::updateBoundaryBox()
 */
//==============================================================================
void cLevel::updateBoundaryBox()
{
    m_boundaryBoxMin.set(0.0, 0.0, 0.0);
    m_boundaryBoxMax.set(m_width,  m_height, 0.0);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
