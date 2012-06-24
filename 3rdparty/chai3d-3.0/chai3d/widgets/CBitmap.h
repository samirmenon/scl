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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 788 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CBitmapH
#define CBitmapH
//---------------------------------------------------------------------------
#include "widgets/CGenericWidget.h"
#include "graphics/CImage.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CBitmap.h

    \brief 
    <b> Widgets </b> \n 
    2D Bitmaps.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cBitmap
    \ingroup    widgets  

    \brief      
    This class provides functionalities to display a bitmap image.
*/
//===========================================================================
class cBitmap : public cGenericWidget
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cBitmap.
    cBitmap();

    //! Destructor of cBitmap.
    virtual ~cBitmap();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Create a copy of current object.
    cBitmap* copy(const bool a_duplicateMaterialData = false,
                  const bool a_duplicateTextureData = false, 
                  const bool a_duplicateMeshData = false,
                  const bool a_buildCollisionDetector = false);

    //! Get width of widget.
    virtual double getWidth() const { return (m_zoomWidth * m_image->getWidth()); }

    //! Get height of widget.
    virtual double getHeight() const { return (m_zoomHeight * m_image->getHeight()); }

    //! Get a pointer to the actual image data... use with care...
    inline unsigned char* getData() { return (m_image->getData()); }

    //! Get width of image.
    inline unsigned int getBitmapWidth() const { return (m_image->getWidth());  }

    //! Get height of image.
    inline unsigned int getBitmapHeight() const { return (m_image->getHeight()); }

    //! Get the format (GL_RGB or GL_RGBA) of the image.
    inline unsigned int getBitmapFormat() const { return (m_image->getFormat()); }

    //! Set zoom factors.
    void setZoom(const float a_zoomWidth, 
		         const float a_zoomHeight);

    //! Get zoom factor along horizontal axis.
    float getZoomHeight() const { return (m_zoomHeight); }

    //! Get zoom factor along vertical axis.
    float getZoomWidth() const { return (m_zoomWidth); }

    // ! Set an existing image to use as a bitmap
    bool setImage (cImage *a_image);

    //! Image loader.
    cImage* m_image;


  private:

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Horizontal zoom factor.
    float m_zoomWidth;

	//! Vertical zoom factor.
	float m_zoomHeight;


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Render texture in OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! Update the bounding box of this object.
    virtual void updateBoundaryBox();
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
