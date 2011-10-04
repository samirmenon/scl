//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2004 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   2.0.0 $Rev: 198 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CLabel.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
      Constructor of cLabel.

      \fn       cLabel::cLabel()
*/
//===========================================================================
cLabel::cLabel()
{
    m_string = "";
    m_font = cFont::createFont();
    m_font->setFontFace("helvetica12");
}


//===========================================================================
/*!
      Destructor of cLabel.

      \fn       cLabel::~cLabel()
*/
//===========================================================================
cLabel::~cLabel()
{

}


//===========================================================================
/*!
      Render the label in OpenGL.

      \fn       void cLabel::render(const int a_renderMode)
	  \param	a_renderMode  Rendering mode.
*/
//===========================================================================
void cLabel::render(const int a_renderMode)
{
    // disable lighting  properties
    glDisable(GL_LIGHTING);

    // render font color
    m_fontColor.render();

    // draw fonts
    glRasterPos2f(0, 0);

    // render string
    m_font->renderString(m_string.c_str());

    // enable lighting  properties
    glEnable(GL_LIGHTING);
}


