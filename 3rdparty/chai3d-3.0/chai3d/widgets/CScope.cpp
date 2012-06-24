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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 699 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "widgets/CScope.h"
//---------------------------------------------------------------------------
#include "graphics/CPrimitives.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cScope.

    \fn     cScope::cScope()
*/
//===========================================================================
cScope::cScope()
{
    // clear signals
    clearSignals();

    // set default margin values    
    m_marginLeft    = 10;
    m_marginRight   = 10;
    m_marginTop     = 10;
    m_marginBottom  = 10;

    // set defailt radius values
    m_radiusTopLeft =       10;
    m_radiusTopRight =      10;
    m_radiusBottomLeft =    10;
    m_radiusBottomRight =   10;

    // set default color signals
    m_colorSignal0.setBlueCornflower();
    m_colorSignal1.setGreenMediumSea();
    m_colorSignal2.setPinkDeep();
    m_colorSignal3.setYellowLemonChiffon();

    // default line width
    m_lineWidth = 1.0;

    // default panel background color settings
    m_colorPanelTopLeft.setGrayLevel(0.3f);
    m_colorPanelTopRight.setGrayLevel(0.3f);
    m_colorPanelBottomLeft.setGrayLevel(0.2f);
    m_colorPanelBottomRight.setGrayLevel(0.2f);

    // set a default size
    setSize(600, 100);
}


//===========================================================================
/*!
    Set values for signals 0, 1, 2 and 3.

    \fn         void cScope::setSignalValues(const double a_signalValue0,
                                             const double a_signalValue1,
                                             const double a_signalValue2,
                                             const double a_signalValue3)

    \param      a_signalValue0  Value for signal 0.
    \param      a_signalValue1  Value for signal 1.
    \param      a_signalValue2  Value for signal 2.
    \param      a_signalValue3  Value for signal 3.
*/
//===========================================================================
void cScope::setSignalValues(const double a_signalValue0,
                             const double a_signalValue1,
                             const double a_signalValue2,
                             const double a_signalValue3)
{
    m_index1 = (m_index1 + 1) % C_SCOPE_MAX_SAMPLES;

    if (m_index1 == m_index0)
    {
        m_index0 = (m_index0 + 1) % C_SCOPE_MAX_SAMPLES;
    }
    

    // set signal 0
    if (m_signalEnabled[0])
    {
        double value = cClamp(a_signalValue0, m_minValue, m_maxValue);
        m_signals[0][m_index1] = (int)((double)(m_height) * (value - m_minValue) / (m_maxValue - m_minValue));
    }
    else
    {
        m_signals[0][m_index1] = 0;
    }

    // set signal 1
    if (m_signalEnabled[1])
    {
        double value = cClamp(a_signalValue1, m_minValue, m_maxValue);
        m_signals[1][m_index1] = (int)((double)(m_height) * (value - m_minValue) / (m_maxValue - m_minValue));
    }
    else
    {
        m_signals[1][m_index1] = 0;
    }

    // set signal 2
    if (m_signalEnabled[2])
    {
        double value = cClamp(a_signalValue2, m_minValue, m_maxValue);
        m_signals[2][m_index1] = (int)((double)(m_height) * (value - m_minValue) / (m_maxValue - m_minValue));
    }
    else
    {
        m_signals[2][m_index1] = 0;
    }

    // set signal 3
    if (m_signalEnabled[3])
    {
        double value = cClamp(a_signalValue3, m_minValue, m_maxValue);
        m_signals[3][m_index1] = (int)((double)(m_height) * (value - m_minValue) / (m_maxValue - m_minValue));
    }
    else
    {
        m_signals[3][m_index1] = 0;
    }

    // initialization case
    if ((m_index0 == 0) && (m_index1 == 1))
    {
        for (int i=0; i<4; i++)
        m_signals[i][0] = m_signals[i][1];
    }
}


//===========================================================================
/*!
    Enable/Disable signals. On enabled signals are displayed to the Scope.

    \fn         void cScope::setSignalEnabled(const bool a_signalEnabled0,
                                              const bool a_signalEnabled1,
                                              const bool a_signalEnabled2,
                                              const bool a_signalEnabled3)

    \param      a_signalEnabled0  Status for signal 0.
    \param      a_signalEnabled1  Status for signal 1.
    \param      a_signalEnabled2  Status for signal 2.
    \param      a_signalEnabled3  Status for signal 3.
*/
//===========================================================================
void cScope::setSignalEnabled(const bool a_signalEnabled0,
                              const bool a_signalEnabled1,
                              const bool a_signalEnabled2,
                              const bool a_signalEnabled3)
{
    m_signalEnabled[0] = a_signalEnabled0;
    m_signalEnabled[1] = a_signalEnabled1;
    m_signalEnabled[2] = a_signalEnabled2;
    m_signalEnabled[3] = a_signalEnabled3;
}


//===========================================================================
/*!
    Reset all signals from scope.

    \fn         void cScope::clearSignals()
*/
//===========================================================================
void cScope::clearSignals()
{
    m_index0 = 0;
    m_index1 = 0;
}


//===========================================================================
/*!
    Set the minimum and maximum values are displayed by the scope.

    \fn         void cScope::setRange(const double a_minValue, 
                                      const double a_maxValue)

    \param      a_minValue  Minimum value.
    \param      a_maxValue  Maximum value.
*/
//===========================================================================
void cScope::setRange(const double a_minValue, 
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
}


//===========================================================================
/*!
    Set the size of the scope by defining the width and height.

    \fn         void cScope::setSize(const int a_width, 
                                     const int a_height)

    \param      a_width  Width of scope.
    \param      a_height  Height of scope.
*/
//===========================================================================
void cScope::setSize(const int a_width, 
                     const int a_height)
{
    // set dimension of scope.
    m_width = cMin(cAbs(a_width), C_SCOPE_MAX_SAMPLES);
    m_height = cAbs(a_height);

    // adjust dimension of Panel
    clear();
    cPanel::setPanelSize(m_width, m_height);
}


//===========================================================================
/*!
    Render scope in OpenGL.

    \fn       void cScope::render(cRenderOptions& a_options)
    \param	a_options  Rendering options.
*/
//===========================================================================
void cScope::render(cRenderOptions& a_options)
{
    // render background
    cMesh::render(a_options);


	/////////////////////////////////////////////////////////////////////////
	// Render parts that are always opaque
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        if (m_index1 == m_index0) { return; }

	    // disable lighting
	    glDisable(GL_LIGHTING);

        // set line width
        glLineWidth(m_lineWidth);

        // render signal 0
        for (int i=0; i<4; i++)
        {
            if (m_signalEnabled[i])
            {
                switch (i)
                {
                    case 0: m_colorSignal0.render(); break;
                    case 1: m_colorSignal1.render(); break;
                    case 2: m_colorSignal2.render(); break;
                    case 3: m_colorSignal3.render(); break;
                }

                int x = m_width;
                int i0 = m_index1;
                int i1 = i0-1;
                if (i1 < 0)
                {
                    i1 = C_SCOPE_MAX_SAMPLES-1;
                }

                glBegin(GL_LINES);
                while ((i0 != m_index0) && (x>0))
                {
                    glVertex3d(x, m_signals[i][i0], 0.0);
                    glVertex3d(x-1, m_signals[i][i1], 0.0);  
                    i0 = i1;
                    i1--;
                    if (i1 < 0)
                    {
                        i1 = C_SCOPE_MAX_SAMPLES-1;
                    }
                    x--;
                }
                glEnd();
            }
        }

	    // restore lighting to default value
	    glEnable(GL_LIGHTING);
    }
}


//===========================================================================
/*!
    Update bounding box of current object.

    \fn       void void cBitmap::updateBoundaryBox()
 */
//===========================================================================
void cScope::updateBoundaryBox()
{
    m_boundaryBoxMin.set(0.0, 0.0, 0.0);
    m_boundaryBoxMax.set(m_width, m_height, 0.0);
}