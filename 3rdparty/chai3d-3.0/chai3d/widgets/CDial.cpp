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
#include "widgets/CDial.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cDial.

    \fn     cDial::cDial()
*/
//==============================================================================
cDial::cDial()
{
    // set the default number of increments
    m_numIncrements = 72;

    // set display mode
    m_flagSingleIncrementDisplay = false;

    // default width
    setSize(100);

    update();

    setValue(40);
}


//==============================================================================
/*!
    Set a new value 0. The value is compared to the range defined by 
    m_minValue and m_maxValue. The segment are highlighted accordingly.

    \fn         void cLevel::setValue0(const double a_value0)

    \param      a_value0  New value.
*/
//==============================================================================
void cDial::setValue(const double a_value)
{
    m_value0 = m_minValue;
    
    cLevel::setValue(a_value);

    m_value1 = m_value;
}



//==============================================================================
/*!
    Set a new value 0. The value is compared to the range defined by 
    m_minValue and m_maxValue. The segment are highlighted accordingly.

    \fn         void cLevel::setValue0(const double a_value0)

    \param      a_value0  New value.
*/
//==============================================================================
void cDial::setValue0(const double a_value)
{
    // sanity check
    if (m_numIncrements > (int)(4 * m_vertices->size()))
    {
        update();
    }

    // init variables
    int j=0;

    // clamp value within range
    m_value0 = cClamp(a_value, m_minValue, m_maxValue);
    
    // compute level based on value and range
    int level0 = (int)((double)(m_numIncrements) * (m_value0 - m_minValue) / (m_maxValue - m_minValue));
    int level1 = (int)((double)(m_numIncrements) * (m_value1 - m_minValue) / (m_maxValue - m_minValue));

  
    // process increments
    if (m_flagSingleIncrementDisplay)
    {
        for (int i=0; i<m_numIncrements; i++)
        {
            if ((i == level0) || (i == level1))
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
    }
    else
    {
        if (level1 >= level0)
        {
            for (int i=0; i<m_numIncrements; i++)
            {
                if ((i >= level0) && (i <= level1))
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
        }
        else
        {
            for (int i=0; i<m_numIncrements; i++)
            {
                if ((i <= level1) || (i >= level0))
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
        }
    }

    // update display list
    invalidateDisplayList();
}


//==============================================================================
/*!
    Set a new value 0. The value is compared to the range defined by 
    m_minValue and m_maxValue. The segment are highlighted accordingly.

    \fn         void cLevel::setValue0(const double a_value0)

    \param      a_value0  New value.
*/
//==============================================================================
void cDial::setValue1(const double a_value)
{
    // clamp value within range
    m_value1 = cClamp(a_value, m_minValue, m_maxValue);

    setValue0(m_value0);
}


//==============================================================================
/*!
    Set both values. The segments are highlighted accordingly.

    \fn         void cDial::setValues(const double a_value0, 
                                      const double a_value1)

    \param      a_value0  Value 0.
    \param      a_value0  Value 1.
*/
//==============================================================================
void cDial::setValues(const double a_value0, 
                      const double a_value1)
{
    // clamp value within range
    m_value0 = cClamp(a_value0, m_minValue, m_maxValue);
    m_value1 = cClamp(a_value1, m_minValue, m_maxValue);

    // update increments
    setValue0(m_value0);
}


//==============================================================================
/*!
    Set the dimensions of the cDial object.

    \fn         void cDial::setSize(const double a_size)

    \param      a_size  Size.
*/
//==============================================================================
void cDial::setSize(const double a_size)
{
    // sanity test
    m_size = cAbs(a_size);

    // update level
    update();

    // set value
    setValues(m_value0, m_value1);
}


//==============================================================================
/*!
    update polygons of cLevel object

    \fn         void cLevel::update()
*/
//==============================================================================
void cDial::update()
{
    // clear all triangles
    clear();

    // set dimension
    double r0 = 0.7 * (0.5 * m_size);
    double r1 = 1.0 * (0.5 * m_size);

    double a = (double)(2*C_PI) / (double)(m_numIncrements);
    cVector3d n(0,0,1);
    double angle = 0;

    double cos0 = 1.0;
    double sin0 = 0.0;

    // create polygons
    for (int i=0; i<m_numIncrements; i++)
    {
        // compute position of vertices

        double cos1 = cos(angle+a);
        double sin1 = sin(angle+a);

        cVector3d v0(-r0 * sin0, -r0 * cos0, 0.0);
        cVector3d v1(-r1 * sin0, -r1 * cos0, 0.0);
        cVector3d v2(-r0 * sin1, -r0 * cos1, 0.0);
        cVector3d v3(-r1 * sin1, -r1 * cos1, 0.0);

        cos0 = cos1;
        sin0 = sin1;

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
        newTriangle(vertexIndex2, vertexIndex1, vertexIndex3);

        // increment offset
        angle = angle + a;
    }

    // update boundary box
    updateBoundaryBox();
}

//==============================================================================
/*!
    Update bounding box of current object.

    \fn       void void cBitmap::updateBoundaryBox()
 */
//==============================================================================
void cDial::updateBoundaryBox()
{
    double s = 0.5 * m_size;
    m_boundaryBoxMin.set(-s, -s, 0.0);
    m_boundaryBoxMax.set( s,  s, 0.0);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
