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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 995 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "widgets/CBitmap.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
      Constructor of cBitmap.

      \fn       cBitmap::cBitmap()
*/
//==============================================================================
cBitmap::cBitmap()
{
	// create image structure
	m_image = new cImage();

    // initialize zoom factors
    m_zoomWidth = 1.0;
    m_zoomHeight = 1.0;

    // do not used transparency
    m_useTransparency = false;
}


//==============================================================================
/*!
      Destructor of cBitmap.

      \fn       cBitmap::~cBitmap()
*/
//==============================================================================
cBitmap::~cBitmap()
{
}


//==============================================================================
/*!
    Create a copy of itself.

    \fn			cBitmap* cBitmap::copy(const bool a_duplicateMaterialData,
                       const bool a_duplicateTextureData, 
                       const bool a_duplicateMeshData,
                       const bool a_buildCollisionDetector)

    \param      a_duplicateMaterialData  If __true__, material (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateTextureData  If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateMeshData  If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param      a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.

	\return		Return new object.
*/
//==============================================================================
cBitmap* cBitmap::copy(const bool a_duplicateMaterialData,
                       const bool a_duplicateTextureData, 
                       const bool a_duplicateMeshData,
                       const bool a_buildCollisionDetector)
{
    // create new instance
    cBitmap* obj = new cBitmap();

    // copy generic properties
    copyGenericProperties(obj, a_duplicateMaterialData, a_duplicateTextureData);

	// copy cBitmap properties
    obj->m_zoomWidth = m_zoomWidth;
	obj->m_zoomHeight = m_zoomHeight;
    obj->m_useTransparency = m_useTransparency;

	// copy image data
	if (a_duplicateTextureData)
	{
		obj->m_image = m_image->copy();
	}
	else
	{
		obj->m_image = m_image;
	}

    // update boundary box
    obj->updateBoundaryBox();

    // return
    return (obj);
}


//==============================================================================
/*!
      Render bitmap in OpenGL

      \fn       void cBitmap::render(cRenderOptions& a_options)

	  \param	a_options  Rendering options.
*/
//==============================================================================
void cBitmap::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    glDisable(GL_LIGHTING);

    // transparency is used
    if (m_useTransparency)
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }

    // transparency is not used
    else
    {
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }

    if (m_image->getData() != NULL)
    {
        glRasterPos2i(1,1);
        glPixelZoom(m_zoomWidth, m_zoomHeight);
        
        glDrawPixels(m_image->getWidth(),
                     m_image->getHeight(),
                     m_image->getFormat(),
                     m_image->getType(),
                     m_image->getData());

        glPixelZoom(1.0, 1.0);
    }

    // restore OpenGL state
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);

#endif
}


//==============================================================================
/*!
    Update bounding box of current object.

    \fn       void void cBitmap::updateBoundaryBox()
 */
//==============================================================================
void cBitmap::updateBoundaryBox()
{
    m_boundaryBoxMin.zero();

    if (m_image != NULL)
    {
        m_boundaryBoxMax.set(m_zoomWidth * m_image->getWidth(),
                             m_zoomHeight * m_image->getHeight(),
                             0);
    }
    else
    {
        m_boundaryBoxMax.zero();
    }
}


//==============================================================================
/*!
      Set zoom factors for the horizontal and vertical directions

      \fn       void cBitmap::setZoom(const float a_zoomWidth, 
				      const float a_zoomHeight)

      \param    a_zoomHorizontal Zoom factor along the horizontal direction
      \param    a_zoomVertical Zoom factor along the vertical direction
*/
//==============================================================================
void cBitmap::setZoom(const float a_zoomWidth, 
				      const float a_zoomHeight)
{
    m_zoomWidth = a_zoomWidth;
    m_zoomHeight = a_zoomHeight;
}


//==============================================================================
/*!
      Cleanly remove existing image and set a new, existing image as the bitmap source

      \fn       void cBitmap::setImage(cImage *a_image);
      \param    a_image a pointer to the new image
*/
//==============================================================================
bool cBitmap::setImage(cImage *a_image)
{
    if (!a_image) return false;

    if (m_image)
    {
        delete m_image;
    }

    m_image = a_image;

    return true;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
