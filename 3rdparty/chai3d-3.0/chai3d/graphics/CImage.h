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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 810 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CImageH
#define CImageH
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CImage.h

    \brief
    <b> Graphics </b> \n 
    2D Image Data Structure.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cImage
    \ingroup    graphics 
    
    \brief    
    cImage provides a class to create support graphic files of the following 
    format: (GL_LUMINANCE, GL_RGB, GL_RGBA).
    Various file formats are also supported for loading and saving images 
    to disk. 
*/
//===========================================================================
class cImage
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Default constructor of cImage
    cImage();

    //! Constructor of cImage. Allocate image by passing width, height, pixel format and pixel type.
    cImage(const unsigned int a_width, 
           const unsigned int a_height,
           const GLenum a_format = GL_RGB,
           const GLenum a_type = GL_UNSIGNED_BYTE);

    //! Destructor of cImage.
    virtual ~cImage();


    //-----------------------------------------------------------------------
    // GENERAL COMMANDS:
    //-----------------------------------------------------------------------

    //! Create a copy of current object.
	cImage* copy();

    //! Allocate a new image by defining its size, pixel format and pixel type.
    bool allocate(const unsigned int a_width, 
                  const unsigned int a_height,
                  const GLenum a_format = GL_RGB,
                  const GLenum a_type = GL_UNSIGNED_BYTE); 

    //! Free image from memory.
    void erase() { cleanup(); }

    //! Returns \b true if the image has been allocated in memory, \b false otherwise.
    inline bool isInitialized() const { return (m_allocated); }

    //! Set the size of image by defining the width and height.
    void setSize(const unsigned int a_width, const unsigned int a_height);

    //! Get width of image.
    inline unsigned int getWidth() const { return (m_width);  }

    //! Get height of image.
    inline unsigned int getHeight() const { return (m_height); }

    //! Get the pixel format of the image (GL_RGB, GL_RGBA, GL_LUMINANCE for instance).
    inline GLenum getFormat() const { return (m_format); }

    //! Get the pixel data type. (GL_UNSIGNED_BYTE, GL_UNSIGNED_INT for instance).
    inline GLenum getType() const { return (m_type); }

    //! Get the number of bits per pixel used to store this image.
    inline unsigned int getBitsPerPixel() const { return (8 * m_bytesPerPixel); }

    //! Get the number of bytes per pixel used to store this image.
    inline unsigned int getBytesPerPixel() const { return (m_bytesPerPixel); }

    //! Get the size in bytes of the current image.
    inline unsigned int getSizeInBytes() const { return (m_bytesPerPixel * m_width * m_height); }

    //! Convert image to a new format.
    bool convert(const unsigned int a_newFormat);

    //! Modify the properties of the image.
    bool setProperties(const unsigned int a_width, 
                       const unsigned int a_height, 
                       const GLenum a_format,
                       const GLenum a_type);

    //! Query the number of bytes per pixel for a given format.
    static int queryBytesPerPixel(const GLenum a_format, 
                                  const GLenum a_type);


	//-----------------------------------------------------------------------
    // METHODS - MANIPULATING PIXELS:
    //-----------------------------------------------------------------------

    //! Clear an image with black pixels.
    void clear();

    //! Clear an image with a defined color.
    void clear(const cColorb& a_color);

    //! Clear an image with a defined color.
    void clear(const unsigned char a_componentR,
               const unsigned char a_componentG,
               const unsigned char a_componentB);

    //! Clear an image with a defined level of gray.
    void clear(const unsigned char a_grayLevel);

    //! Get the color of a pixel by passing its x and y coordinate.
    bool getPixelColor(const unsigned int a_x, 
                       const unsigned int a_y,
                       cColorb& a_color);

    //! Set the color of a pixel.
    void setPixelColor(const unsigned int a_x, 
                       const unsigned int a_y, 
                       const cColorb& a_color);

    //! Set the color of a pixel.
    inline void setPixelColor(const unsigned int a_x, 
							  const unsigned int a_y, 
							  const unsigned char a_r,
							  const unsigned char a_g,
							  const unsigned char a_b) { cColorb color(a_r, a_g, a_b); setPixelColor(a_x, a_y, color); }

    //! Set the gray scale of a pixel.
    void setPixelColor(const unsigned int a_x, 
                       const unsigned int a_y, 
                       const unsigned char a_grayLevel);

    //! Copy a section of this current image to another destination image
    void copyTo(const unsigned int a_sourcePosX,
                const unsigned int a_sourcePosY,
                const unsigned int a_sourceSizeX,
                const unsigned int a_sourceSizeY,
                cImage* a_destImage, 
                const unsigned int a_destPosX = 0,
                const unsigned int a_destPosY = 0);

    //! Copy the entire image to another destination image
    void copyTo(cImage* a_destImage, 
                const unsigned int a_destPosX = 0,
                const unsigned int a_destPosY = 0);

	//! Define a pixel color to be transparent.
	void setTransparentColor(const cColorb &a_color, 
							 const unsigned char a_transparencyLevel);

    //! Define a pixel color to be transparent.
	void setTransparentColor(const unsigned char a_r,
							 const unsigned char a_g,
							 const unsigned char a_b, 
							 const unsigned char a_transparencyLevel);

    //! Define a transparent component to all image pixels.
	void setTransparency(const unsigned char a_transparencyLevel);


	//-----------------------------------------------------------------------
    // METHODS - MEMORY:
    //-----------------------------------------------------------------------
    
    //! Get a pointer to the actual image data. Use with care!
    inline unsigned char* getData() { return (m_data); }

    //! modify the pointer to the actual image data. Use with care!
    void setData(unsigned char* a_data, 
                 const unsigned int a_dataSizeInBytes, 
                 const bool a_dealloc = false);


	//-----------------------------------------------------------------------
    // METHODS - FILES:
    //-----------------------------------------------------------------------

    //! Load image file by passing image path and name as argument.
    bool loadFromFile(const string& a_filename);

    //! Save image file by passing image path and name as argument.
    bool saveToFile(const string& a_filename);

    //! Get the filename from which this image was last loaded or saved.
    string getFilename() const { return (m_filename); }
    

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Reference color for pixels outside the image.
    cColorb m_borderColor;


  protected:

	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Initialize member variables.
    void defaults();

    //! Delete memory and rid ourselves of any image we had previously stored.
    void cleanup();


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Image filename.
    string m_filename;

    //! Width in pixels of the current image.
    unsigned int m_width;
        
    //! Height in pixels of the current image.
    unsigned int m_height;

    //! Pixel format of the image (GL_RGB, GL_RGBA, GL_LUMINANCE for instance).
    GLenum m_format;

    //! Pixel data type. (GL_UNSIGNED_BYTE, GL_UNSIGNED_INT for instance).
    GLenum m_type;

    //! Number of bytes per pixel.
    unsigned int m_bytesPerPixel;

    //! The image data itself.
    unsigned char* m_data;

    //! Size of current image in bytes
    unsigned int m_memorySize;

    //! If \b true, then the image has been allocated in memory, \b false otherwise.
    bool m_allocated;

    //! If \b true, then this object actually performed the memory allocation for this object.
    bool m_responsibleForMemoryAllocation;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
