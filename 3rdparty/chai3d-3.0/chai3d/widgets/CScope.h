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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 733 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CScopeH
#define CScopeH
//------------------------------------------------------------------------------
#include "widgets/CPanel.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
const int C_SCOPE_MAX_SAMPLES = 3000;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CScope.h

    \brief 
    <b> Widgets </b> \n 
    2D scope.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cScope
    \ingroup    widgets

    \brief      
    Implementation of a 2D scope to display signals.
*/
//==============================================================================
class cScope : public cPanel
{    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    public:

    //! Constructor of cScope.
    cScope();

    //! Destructor of cScope.
    virtual ~cScope() {};


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    public:

    //! Color settings for signal 0.
    cColorf m_colorSignal0;

    //! Color settings for signal 1.
    cColorf m_colorSignal1;

    //! Color settings for signal 2.
    cColorf m_colorSignal2;

    //! Color settings for signal 3.
    cColorf m_colorSignal3;

    //! Set line width.
    inline void setLineWidth(const double a_lineWidth) { m_lineWidth = cAbs(a_lineWidth); }

    //! Get line width.
    inline double getLineWidth() const { return (m_lineWidth); }

    //! Set values for signals 0, 1, 2, and 3.
    void setSignalValues(const double a_signalValue0 = 0,
                         const double a_signalValue1 = 0,
                         const double a_signalValue2 = 0,
                         const double a_signalValue3 = 0);

    //! Enable/Disable signals
    void setSignalEnabled(const bool a_signalEnabled0 = true,
                          const bool a_signalEnabled1 = true,
                          const bool a_signalEnabled2 = true,
                          const bool a_signalEnabled3 = true);

    //! Clear all signals
    void clearSignals();

    //! Set the range of input values which command the increment lines.
    virtual void setRange(const double a_minValue, 
                          const double a_maxValue); 

    //! Get minimum possible value from range.
    inline double getRangeMin() const { return (m_minValue); }

    //! Get maximum possible from range.
    inline double getRangeMax() const { return (m_maxValue); }

    //! Set the size of the scope.
    void setSize(const int a_width, 
                 const int a_height);

    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

    protected:

    //! Range - minimum value.
    double m_minValue;

    //! Range - maximum value.
    double m_maxValue;

    //! Data for signal 0.
    int m_signals[4][C_SCOPE_MAX_SAMPLES];

    //! Status about signals
    bool m_signalEnabled[4];

    //! Height of scope.
    int m_height;

    //! Width of scope.
    int m_width;

    //! Index of first sample
    unsigned int m_index0;

    //! Index of last sample
    unsigned int m_index1;

    //! Width used to render lines
    double m_lineWidth;


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    protected:

    //! Render background in OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! Update the bounding box of this object.
    virtual void updateBoundaryBox();
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
