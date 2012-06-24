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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 760 $
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
cLabel::cLabel(cFont* a_font)
{
    m_fontScale = 1.0;
    m_string = "";
    m_font = a_font;
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
	Create a copy of itself.

	\fn			cLabel* cLabel::copy(const bool a_duplicateMaterialData,
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
cLabel* cLabel::copy(const bool a_duplicateMaterialData,
					 const bool a_duplicateTextureData, 
					 const bool a_duplicateMeshData,
					 const bool a_buildCollisionDetector)
{
	cLabel* obj = new cLabel(m_font);

    // copy generic properties
    copyGenericProperties(obj, a_duplicateMaterialData, a_duplicateTextureData);

	// copy properties (cLabel)
	obj->m_fontScale = m_fontScale;
	obj->m_string = m_string;
	obj->m_fontColor = m_fontColor;

	// return object
	return (obj);
}


//===========================================================================
/*!
      Render the label in OpenGL.

      \fn       void cLabel::render(cRenderOptions& a_options)
	  \param	a_options  Rendering options.
*/
//===========================================================================
void cLabel::render(cRenderOptions& a_options)
{
	/////////////////////////////////////////////////////////////////////////
	// Render parts that are always opaque
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        // disable lighting  properties
        glDisable(GL_LIGHTING);

        // render font color
        m_fontColor.render();

        // render string
        m_font->renderString(m_string, m_fontColor, m_fontScale, a_options);

        // enable lighting  properties
        glEnable(GL_LIGHTING);
    }
}


//===========================================================================
/*!
      Set String.

      \fn       void cLabel::setString(const string a_string)
	  \param	a_string  String.
*/
//===========================================================================
void cLabel::setString(const string a_string)
{
    // copy string
    m_string = a_string;

    // adjust size of boundary box
    updateBoundaryBox();
}


//===========================================================================
/*!
      Set the font scale factor.

      \fn       void cLabel::setFontScale(const double a_scale
	  \param	a_scale  Scale factor.
*/
//===========================================================================
void cLabel::setFontScale(const double a_scale) 
{ 
    // update scale factor
    m_fontScale = cAbs(a_scale); 

    // adjust size of boundary box
    updateBoundaryBox();
}


//===========================================================================
/*!
      Get width of current string in pixels.

      \fn       double cLabel::getWidth() const
	  \return   Return length of string in pixels.
*/
//===========================================================================
double cLabel::getWidth() const
{
    if (m_font == NULL)
    {
        return (0);
    }
    else
    {
        return (m_fontScale * m_font->getStringWidth(m_string));
    }
}


//===========================================================================
/*!
      Get height of current string in pixels.

      \fn       double cLabel::getHeight() const
	  \return   Return width of string in pixels.
*/
//===========================================================================
double cLabel::getHeight() const
{
    if (m_font == NULL)
    {
        return (0);
    }
    else
    {
        return (m_fontScale * m_font->getPointSize());
    }
}


//===========================================================================
/*!
    Update bounding box of current object.

    \fn       void void cBitmap::updateBoundaryBox()
 */
//===========================================================================
void cLabel::updateBoundaryBox()
{
    m_boundaryBoxMin.set(0.0, 0.0, 0.0);
    m_boundaryBoxMax.set(m_fontScale * getHeight(), m_fontScale * getWidth(), 0.0);
}