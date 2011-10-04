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
    \author    Lev Povalahev
    \author    Dan Morris
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CFileLoaderTGAH
#define CFileLoaderTGAH
//---------------------------------------------------------------------------
#include <string>
#include <stdio.h>
#include <fstream>
//---------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS

typedef unsigned char byte;
typedef unsigned int uint;
enum LImageType {itUndefined, itRGB, itRGBA, itGreyscale};

#endif  // DOXYGEN_SHOULD_SKIP_THIS
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CFileLoaderTGA.h

    \brief    
    <b> Files </b> \n 
    TGA Format Image Loader.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cFileLoaderTGA
    \ingroup    files 
    
    \brief
    Class that loads TGA files.
*/
//===========================================================================
class cFileLoaderTGA
{
  public:
   
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Default constructor, does nothing
    cFileLoaderTGA();

    //! Constructs the object and loads the given tga file
    cFileLoaderTGA(const std::string &filename);

    //! Destructor, cleans up the memory
    virtual ~cFileLoaderTGA();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! This method loads a tga file. It clears all the data if needed.
    bool LoadFromFile(const std::string &filename);


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    /*!
        This method returns the depth of the alpha bitplane in the image
        it can be either 0 or 8.
    */
    uint GetAlphaDepth();

    //! This method returns the width of the image.
    uint GetImageWidth();

    //! This method returns the height of the image.
    uint GetImageHeight();

    /*! 
        This method returns the pixel depth of the image. i.e. color
        depth. Can be 0 (no image loaded), 8 or 24. 16 bit images are not supported.
    */
    uint GetPixelDepth();

    /*! 
        This function returns the pointer to the image data. This pointer can be used
        for glTexImage2D, for example.
    */
    byte* GetPixels();

    /*! 
        This function returns an image type. Imagetype can be either itUndefined (when
        no tga file has been loaded, itRGB (when GetAlphaDepth returns 0 and GetPixelDepth
        returns 24, itRGBA (when GetAlphaDepth returns 8 and GetPixelDepth returns 24), or
        itGreyscale (when GetAlphaDepth returns 0 and GetPixelDepth returns 8).
    */
    LImageType GetImageType();

    //! This is the pixel buffer -> the image
    byte *m_pixels;

protected:


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Clears all data
    void Clear();


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! The pixel depth of the image, including the alpha bits
    uint m_pixelDepth;

    //! The depth of the alpha bitplane
    uint m_alphaDepth;

    //! The image height
    uint m_height;

    //! The width of the image
    uint m_width;

    //! Image type, either rgb, rgba or greyscale
    LImageType m_type;

    //! m_loaded is \b true if a file has been loaded
    bool m_loaded;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
