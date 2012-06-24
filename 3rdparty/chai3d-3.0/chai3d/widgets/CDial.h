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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 733 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDialH
#define CDialH
//---------------------------------------------------------------------------
#include "widgets/CLevel.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CDial.h

    \brief 
    <b> Widgets </b> \n 
    2D dial.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cDial
    \ingroup    widgets

    \brief      
    Implementation of a 2D dial.
*/
//===========================================================================
class cDial : public cLevel
{    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    public:

    //! Constructor of cDial.
    cDial();

    //! Destructor of cDial.
    virtual ~cDial() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    public:

    //! Get width of widget.
    virtual double getWidth() const { return (m_size); }

    //! Get height of widget.
    virtual double getHeight() const { return (m_size); }

    //! Set the diameter of the cDial object.
    virtual void setSize(const double a_size);

    //! Set value.
    virtual void setValue(const double a_value);

    //! Set first value.
    virtual void setValue0(const double a_value0);

    //! Get first value.
    inline double getValue0() const { return (m_value0); }

    //! Set second value.
    virtual void setValue1(const double a_value1);

    //! Get second value.
    inline double getValue1() const { return (m_value1); }

    //! Set both values.
    virtual void setValues(const double a_value0, 
                           const double a_value1);


    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    protected:

    //! Value 0
    double m_value0;

    //! Value 1
    double m_value1;


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    protected:

    //! Update model
    virtual void update();

    //! Update the bounding box of this object.
    virtual void updateBoundaryBox();
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
