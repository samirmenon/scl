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
#ifndef CLevelH
#define CLevelH
//------------------------------------------------------------------------------
#include "widgets/CGenericWidget.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CLevel.h

    \brief 
    <b> Widgets </b> \n 
    2D Level.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cLevel
    \ingroup    widgets

    \brief      
    Implementation of a 2D level.
*/
//==============================================================================
class cLevel : public cGenericWidget
{    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    public:

    //! Constructor of cLevel.
    cLevel();

    //! Destructor of cLevel.
    virtual ~cLevel() {};


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    public:
   
    //! Get width of widget.
    virtual double getWidth() const { return (m_width); }

    //! Get height of widget.
    virtual double getHeight() const { return (m_height); }

    //! Set the base width of the cLevel object. The height is defined by the number of increments.
    virtual void setSize(const double a_size);

    //! Get size of base width.
    inline double getSize() const { return (m_size); }

    //! Set the number of increments. The value can range from 2 to 200.
    virtual void setNumIncrements(const int a_numIncrements);

    //! Get the number of incrments.
    inline int getNumIncrements() const { return ((int)m_size); }

    //! Color of activated increment lines.
    cColorf m_colorActive;

    //! Color of inactivated increment lines.
    cColorf m_colorInactive;

    //! Set the range of input values which command the increment lines.
    virtual void setRange(const double a_minValue, const double a_maxValue); 

    //! Get minimum possible value from range.
    inline double getRangeMin() const { return (m_minValue); }

    //! Get maximum possible from range.
    inline double getRangeMax() const { return (m_maxValue); }

    //! Set value.
    virtual void setValue(const double a_value);

    //! Get value.
    double getValue() const { return (m_value); }

    //! Use single increment display. Only 1 lines is activated at a time
    inline void setSingleIncrementDisplay(const bool a_singleIncrementDisplay) { m_flagSingleIncrementDisplay = a_singleIncrementDisplay; }

    //! Use single increment display. Only 1 lines is activated at a time
    inline bool getSingleIncrementDisplay() const { return (m_flagSingleIncrementDisplay); }



    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

    protected:

    //! Number of increments.
    int m_numIncrements;

    //! Size value which defines the width and height of the cLevel object.
    double m_size;

    //! Width of object.
    double m_width;

    //! Height of object
    double m_height;

    //! Range - minimum value.
    double m_minValue;

    //! Range - maximum value
    double m_maxValue;

    //! Current value.
    double m_value;

    //! If __true__, then the single segment display is activated.
    bool m_flagSingleIncrementDisplay;


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    protected:

    //! Update model
    virtual void update();

    //! Update the bounding box of this object.
    virtual void updateBoundaryBox();
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
