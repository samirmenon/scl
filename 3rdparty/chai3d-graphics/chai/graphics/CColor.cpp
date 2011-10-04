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
    \author    Dan Morris
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "graphics/CColor.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// GENERAL COLOR CONSTANTS
//---------------------------------------------------------------------------

//! Color Red.
cColorf CHAI_COLOR_RED(1.0f,0.0f,0.0f,1.0f);

//! Color Green.
cColorf CHAI_COLOR_GREEN(0.0f,1.0f,0.0f,1.0f);

//! Color Blue.
cColorf CHAI_COLOR_BLUE(0.0f,0.0f,1.0f,1.0f);

//! Color Yellow.
cColorf CHAI_COLOR_YELLOW(1.0f,1.0f,0.0f,1.0f);

//! Color Cyan.
cColorf CHAI_COLOR_CYAN(0.0f,1.0f,1.0f,1.0f);

//! Color Magenta.
cColorf CHAI_COLOR_MAGENTA(1.0f,0.0f,1.0f,1.0f);

//! Color Black.
cColorf CHAI_COLOR_BLACK(0.0f,0.0f,0.0f,1.0f);

//! Color White.
cColorf CHAI_COLOR_WHITE(1.0f,1.0f,1.0f,1.0f);


//===========================================================================
/*!
    Converts current color to cColorb format.

    \fn     cColorb cColorf::getColorb(void) const
*/
//===========================================================================

cColorb cColorf::getColorb(void) const
{
    cColorb color(   (GLubyte)(m_color[0] * (GLfloat)0xff),
                     (GLubyte)(m_color[1] * (GLfloat)0xff),
                     (GLubyte)(m_color[2] * (GLfloat)0xff),
                     (GLubyte)(m_color[3] * (GLfloat)0xff) );
    return (color);
}


//===========================================================================
/*!
    Converts current color to cColorf format.

    \fn     cColorf cColorb::getColorf(void) const
*/
//===========================================================================
cColorf cColorb::getColorf(void) const
{
    cColorf color (   (GLfloat)m_color[0] / (GLfloat)0xff,
                      (GLfloat)m_color[1] / (GLfloat)0xff,
                      (GLfloat)m_color[2] / (GLfloat)0xff,
                      (GLfloat)m_color[3] / (GLfloat)0xff );
    return color;
}


