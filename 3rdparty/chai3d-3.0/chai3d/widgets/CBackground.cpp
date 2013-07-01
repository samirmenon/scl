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
#include "widgets/CBackground.h"
using namespace std;
//------------------------------------------------------------------------------
#include "display/CCamera.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cBackground.

    \fn     cBackground::cBackground()
*/
//==============================================================================
cBackground::cBackground()
{
    // create vertices at each corner
    m_vertexTL = newVertex(0.0, 1.0, 0.0);
    m_vertexTR = newVertex(1.0, 1.0, 0.0);
    m_vertexBL = newVertex(0.0, 0.0, 0.0);
    m_vertexBR = newVertex(1.0, 0.0, 0.0);

    cVertex* vertexTL = getVertex(m_vertexTL);
    cVertex* vertexTR = getVertex(m_vertexTR);
    cVertex* vertexBL = getVertex(m_vertexBL);
    cVertex* vertexBR = getVertex(m_vertexBR);

    // create triangles by connecting above vertices
    newTriangle(m_vertexTL, m_vertexBL, m_vertexBR);
    newTriangle(m_vertexTL, m_vertexBR, m_vertexTR);

    // adjust normal at each vertex
    cVector3d normal(0.0, 0.0, 1.0);
    vertexTL->m_normal = normal;
    vertexTR->m_normal = normal;
    vertexBL->m_normal = normal;
    vertexBR->m_normal = normal;

    // disable material properties, enable colors
    setUseMaterial(true);
    setUseVertexColors(true);
    setUseTransparency(false);
    setMaintainAspectRatio(true);

    // set default color
    cColorf color(1.0, 1.0, 1.0);
    setUniformColor(color);
}


//==============================================================================
/*!
    Render the background object in OpenGL.

    \fn       void cBackground::render(cRenderOptions& a_options)
    \param	a_options  Rendering options.
*/
//==============================================================================
void cBackground::render(cRenderOptions& a_options)
{
    // update the width and height of background in case window size has changed
    int w = a_options.m_camera->getDisplayWidth();
    int h = a_options.m_camera->getDisplayHeight();
    update(0, 0, w, h);  

    // render background
    cMesh::render(a_options);
}


//==============================================================================
/*!
    Updates the dimensions of the backgound based on the display dimensions.

    \fn     void cBackground::update(const unsigned int a_bottomLeftX,
                                     const unsigned int a_bottomLeftY,
                                     const unsigned int a_topRightX,
                                     const unsigned int a_topRightY)
    \param  a_bottomLeftX   X coordinate of bottom left point.
    \param  a_bottomLeftY   Y coordinate of bottom left point.
    \param  a_topRightX	    X coordinate of top right point.
    \param  a_topRightY	    Y coordinate of top right point.
*/
//==============================================================================
void cBackground::update(const unsigned int a_bottomLeftX,
                         const unsigned int a_bottomLeftY,
                         const unsigned int a_topRightX,
                         const unsigned int a_topRightY)
{
    // retrieve pointer to vertices
    cVertex* vertexTL = getVertex(m_vertexTL);
    cVertex* vertexTR = getVertex(m_vertexTR);
    cVertex* vertexBL = getVertex(m_vertexBL);
    cVertex* vertexBR = getVertex(m_vertexBR);

    // update position of vertices
    vertexTL->setLocalPos(a_bottomLeftX, a_topRightY,    0.0);
    vertexTR->setLocalPos(a_topRightX,   a_topRightY,    0.0);
    vertexBL->setLocalPos(a_bottomLeftX, a_bottomLeftY,  0.0);
    vertexBR->setLocalPos(a_topRightX,   a_bottomLeftY,  0.0);

	// update texture coordinates
	if (dynamic_cast<cTexture2d*>(m_texture))
	{
		// 2D texture
        if (m_maintainAspectRatio)
        {
            // temp variables
            double ratioImage  = 1.0;
            double ratioWindow = 1.0;
            double wW,hW, wI, hI = 1.0;

            // compute window ratio 
            wW = (double)(a_topRightX - a_bottomLeftX);
            hW = (double)(a_topRightY - a_bottomLeftY);

            if (hW > 0)
            {
                ratioWindow = wW/hW;
            }

            // compute image ratio
            cImage* image = dynamic_cast<cTexture2d*>(m_texture)->m_image;
            if (image != NULL)
            {
                wI = (double)(image->getWidth());
                hI = (double)(image->getHeight());

                if (hI > 0)
                {
                    ratioImage = wI/hI;
                }
            }

            // compute texture coordinates so that image ratio is maintained
            if (ratioWindow > ratioImage)
            {
                double h = (hW / wW) * wI;
                double t = 0.5 * (hI - h) / hI;

		        vertexTL->m_texCoord.set(0.0, 1.0-t, 0.0);	
		        vertexTR->m_texCoord.set(1.0, 1.0-t, 0.0);	
		        vertexBL->m_texCoord.set(0.0, t, 0.0);	
		        vertexBR->m_texCoord.set(1.0, t, 0.0);
            }
            else
            {
                double w = (wW / hW) * hI;
                double t = 0.5 * (wI - w) / wI;

		        vertexTL->m_texCoord.set(t, 1.0, 0.0);	
		        vertexTR->m_texCoord.set(1.0-t, 1.0, 0.0);	
		        vertexBL->m_texCoord.set(t, 0.0, 0.0);	
		        vertexBR->m_texCoord.set(1.0-t, 0.0, 0.0);
            }
        }
        else
        {
		    vertexTL->m_texCoord.set(0.0, 1.0, 0.0);	
		    vertexTR->m_texCoord.set(1.0, 1.0, 0.0);	
		    vertexBL->m_texCoord.set(0.0, 0.0, 0.0);	
		    vertexBR->m_texCoord.set(1.0, 0.0, 0.0);
        }
	}
	
	else if (dynamic_cast<cTexture1d*>(m_texture))
	{
		// horizontal 1D texture
		if (m_texture->m_image->getHeight() == 1)
		{
			vertexTL->m_texCoord.set(0.0, 0.0, 0.0);	
			vertexTR->m_texCoord.set(0.0, 1.0, 0.0);	
			vertexBL->m_texCoord.set(0.0, 0.0, 0.0);	
			vertexBR->m_texCoord.set(0.0, 1.0, 0.0);	
		}
		// vertical 1D texture
		else
		{
			vertexTL->m_texCoord.set(1.0, 0.0, 0.0);	
			vertexTR->m_texCoord.set(1.0, 0.0, 0.0);	
			vertexBL->m_texCoord.set(0.0, 0.0, 0.0);	
			vertexBR->m_texCoord.set(0.0, 0.0, 0.0);
		}
	}

    // update size
    m_widthBackground = a_topRightX - a_bottomLeftX;
    m_heightBackground = a_topRightY - a_bottomLeftY;
}


//==============================================================================
/*!
    Define an uniform color for the current background.

    \fn       void cBackground::setUniformColor(cColorf a_color)
    \param	a_color  Color.
*/
//==============================================================================
void cBackground::setUniformColor(cColorf a_color)
{
    // retrieve pointer to vertices
    cVertex* vertexTL = getVertex(m_vertexTL);
    cVertex* vertexTR = getVertex(m_vertexTR);
    cVertex* vertexBL = getVertex(m_vertexBL);
    cVertex* vertexBR = getVertex(m_vertexBR);

    // apply color
    vertexTL->m_color = a_color;
    vertexTR->m_color = a_color;
    vertexBL->m_color = a_color;
    vertexBR->m_color = a_color;

	// enable vertex colors
	setUseVertexColors(true);
	setUseMaterial(false);
}


//==============================================================================
/*!
    Define a vertical gradient colored background. The gradient is defined
    by the top and bottom colors.

    \fn       void cBackground::setVerticalLinearGradient(cColorf a_topColor, 
                                                        cColorf a_bottomColor)
    \param	a_topColor  Top color.
    \param	a_bottomColor  Bottom color.
*/
//==============================================================================
void cBackground::setVerticalLinearGradient(cColorf a_topColor, 
                                            cColorf a_bottomColor)
{
    // retrieve pointer to vertices
    cVertex* vertexTL = getVertex(m_vertexTL);
    cVertex* vertexTR = getVertex(m_vertexTR);
    cVertex* vertexBL = getVertex(m_vertexBL);
    cVertex* vertexBR = getVertex(m_vertexBR);

    // apply color
    vertexTL->m_color = a_topColor;
    vertexTR->m_color = a_topColor;
    vertexBL->m_color = a_bottomColor;
    vertexBR->m_color = a_bottomColor;

	// enable vertex colors
	setUseVertexColors(true);
	setUseMaterial(false);
}


//==============================================================================
/*!
    Define a horizontal gradient colored background. The gradient is defined
    by the left and right colors.

    \fn       void cBackground::setHorizontalLinearGradient(cColorf a_leftColor, 
                                                            cColorf a_rightColor)
    \param	a_leftColor  Top color.
    \param	a_rightColor  Bottom color.
*/
//==============================================================================
void cBackground::setHorizontalLinearGradient(cColorf a_leftColor, 
                                              cColorf a_rightColor)
{
    // retrieve pointer to vertices
    cVertex* vertexTL = getVertex(m_vertexTL);
    cVertex* vertexTR = getVertex(m_vertexTR);
    cVertex* vertexBL = getVertex(m_vertexBL);
    cVertex* vertexBR = getVertex(m_vertexBR);

    // apply color
    vertexTL->m_color = a_leftColor;
    vertexTR->m_color = a_rightColor;
    vertexBL->m_color = a_leftColor;
    vertexBR->m_color = a_rightColor;

	// enable vertex colors
	setUseVertexColors(true);
	setUseMaterial(false);
}


//==============================================================================
/*!
    Define a color at each corner of the background. The color distribution
    is interpolated throughout the background.

    \fn       void cBackground::setCornerColors(cColorf a_topLeftColor,
                                                cColorf a_topRightColor, 
                                                cColorf a_bottomLeftColor,    
                                                cColorf a_bottomRightColor)
    \param	a_topLeftColor  Color at top left corner.
    \param	a_topLeftColor  Color at top right corner.
    \param	a_topLeftColor  Color at bottom left corner.
    \param	a_topLeftColor  Color at bottom right corner.
*/
//==============================================================================
void cBackground::setCornerColors(cColorf a_topLeftColor,
                                  cColorf a_topRightColor, 
                                  cColorf a_bottomLeftColor,    
                                  cColorf a_bottomRightColor)
{
    // retrieve pointer to vertices
    cVertex* vertexTL = getVertex(m_vertexTL);
    cVertex* vertexTR = getVertex(m_vertexTR);
    cVertex* vertexBL = getVertex(m_vertexBL);
    cVertex* vertexBR = getVertex(m_vertexBR);

    // apply color
    vertexTL->m_color = a_topLeftColor;
    vertexTR->m_color = a_topRightColor;
    vertexBL->m_color = a_bottomLeftColor;
    vertexBR->m_color = a_bottomRightColor;

	// enable vertex colors
	setUseVertexColors(true);
	setUseMaterial(false);
}


//==============================================================================
/*!
      Load a texture image as background. Depending of the size of the image
      a 1D or 2D texture map is created.

      \fn       bool cBackground::loadFromFile(string a_filename)
	  \param	a_filename  Filename of image.
*/
//==============================================================================
bool cBackground::loadFromFile(string a_filename)
{
	// create image if not yet allocated
	cImage* image = new cImage();
	
	// load file
	bool success = image->loadFromFile(a_filename);
	if (!success) { return (false); }

	// delete current texture
	if (m_texture != NULL)
	{
		delete (m_texture);
	}

	// retrieve dimension and decide if a 1D or 2D should be created
	unsigned int w = image->getWidth();
	unsigned int h = image->getHeight();

	// create 1D or 2D texture depending of image size
	if ((w == 1) || (h == 1))
	{
		m_texture = new cTexture1d();
	}
	else
	{
		m_texture = new cTexture2d();
	}

	// copy image data
	m_texture->m_image->allocate(image->getWidth(), image->getHeight(), image->getFormat());
	image->copyTo(m_texture->m_image);

	// enable texture
	setUseTexture(true);

	// enable vertex colors
	setUseVertexColors(false);
	setUseMaterial(false);

	// discard image
	delete image;

	// success
    return (true);
}



//==============================================================================
/*!
      Load a texture image as background. Depending of the size of the image
      a 1D or 2D texture map is created.

      \fn       bool cBackground::loadFromImage(cImage *a_image)
      \param    a_filename  Filename of image.
*/
//==============================================================================
bool cBackground::loadFromImage(cImage *a_image)
{
    // delete current texture
    if (m_texture != NULL)
    {
        delete (m_texture);
    }

    // retrieve dimension and decide if a 1D or 2D should be created
    unsigned int w = a_image->getWidth();
    unsigned int h = a_image->getHeight();

    // create 1D or 2D texture depending of image size
    if ((w == 1) || (h == 1))
    {
        m_texture = new cTexture1d();
    }
    else
    {
        m_texture = new cTexture2d();
    }

    // copy image data
    m_texture->m_image->allocate(a_image->getWidth(), 
                                 a_image->getHeight(), 
                                 a_image->getFormat());
                                 a_image->copyTo(m_texture->m_image);

    // enable texture
    setUseTexture(true);

    // enable vertex colors
    setUseVertexColors(false);
    setUseMaterial(false);

    // success
    return (true);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
