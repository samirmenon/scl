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
    \author    Sebastien Grange
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1065 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "files/CFileImagePNG.h"
//------------------------------------------------------------------------------
#ifdef C_USE_FILE_PNG
//------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------
extern "C" 
{
#include "png.h"
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    PNG decompressor, used by
    \ref bool cLoadPNG(cImage* a_image, void *a_buffer, int a_len) and
    \ref bool cLoadFilePNG(cImage* a_image, string a_filename).
    \param a_png_ptr   a pointer to a PNG engine.
    \param a_info_ptr  a pointer to a PNG information structure.
    \param a_image     the source image to receive the decompressed PNG data.
    \return Returns __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
static bool _decompressPNG(png_structp a_png_ptr, png_infop a_info_ptr, cImage* a_image)
{
    int j;
    int bpp;

    // read in the header
    png_read_info(a_png_ptr, a_info_ptr);
    int width  = png_get_image_width (a_png_ptr, a_info_ptr);
    int height = png_get_image_height(a_png_ptr, a_info_ptr);

    // get image info
    png_uint_32 tmpw, tmph;
    int bit_depth, color_type, interlace_type;
    png_get_IHDR(a_png_ptr, a_info_ptr, &tmpw, &tmph, &bit_depth, &color_type, &interlace_type, NULL, NULL);

    // strip 16 bit/color files down to 8 bits/color
    png_set_strip_16(a_png_ptr);

    // extract multiple pixels with bit depths of 1, 2, and 4 into separate bytes
    png_set_packing(a_png_ptr);

    // expand palette to RGB, grayscale to 8 bits, transparency to alpha
    png_set_expand(a_png_ptr);

    // set the bytes per pixel count
    if (color_type & PNG_COLOR_MASK_COLOR)
        bpp = 3;
    else
        bpp = 1;

    // if there is an alpha channel or some transparency, act accordingly
    if (color_type & PNG_COLOR_MASK_ALPHA || png_get_valid(a_png_ptr, a_info_ptr, PNG_INFO_tRNS))
        bpp += 1;

    // allocate row buffers
    size_t    rowsize = bpp*width;
    png_bytep *row_pointers = new png_bytep[height];
    for (j=0; j<height; j++)
        row_pointers[j] = (png_bytep)png_malloc(a_png_ptr, rowsize);

    // read the entire image in one go
    png_read_image(a_png_ptr, row_pointers);

    // we allocate memory for image. By default we shall use OpenGL's RGB mode.
    if (bpp == 1 && !a_image->allocate(width, height, GL_LUMINANCE))
        return false;
    if (bpp == 2 && !a_image->allocate(width, height, GL_LUMINANCE_ALPHA))
        return false;
    if (bpp == 3 && !a_image->allocate(width, height, GL_RGB))
        return false;
    if (bpp == 4 && !a_image->allocate(width, height, GL_RGBA))
        return false;

    // retrieve pointer to image data
    unsigned char* data = a_image->getData();

    // put rows neatly in the destination buffer
    for (j=0; j<height; j++)
        memcpy(data+bpp*j*width, row_pointers[height-1-j], rowsize);

    // clean up after the read and free any memory allocated
    delete [] row_pointers;
    png_read_end(a_png_ptr, NULL);

    // success
    return true;
}


//==============================================================================
/*!
    PNG compressor, used by
    \ref bool cSavePNG(cImage* a_image, char **a_buffer, int *a_len) and
    \ref bool cSaveFilePNG(cImage* a_image, string a_filename)
    \param a_png_ptr   a pointer to a PNG engine.
    \param a_info_ptr  a pointer to a PNG information structure.
    \param a_image     the source image to compress to PNG data.
    \return Returns __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
static bool _compressPNG(png_structp a_png_ptr, png_infop a_info_ptr, cImage* a_image)
{
    int         bpp;

    // sanity check
    if (a_image == NULL)
        return false;

    // retrieve image size
    unsigned int width  = a_image->getWidth();
    unsigned int height = a_image->getHeight();
    if (!((width > 0) && (height > 0)))
        return false;

    // retrieve pointer to data
    unsigned char* data = a_image->getData();

    // set image information
    switch (a_image->getFormat())
    {
        case GL_LUMINANCE:
            bpp = 1;
            png_set_IHDR(a_png_ptr, a_info_ptr, width, height, 8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
            break;
        case GL_LUMINANCE_ALPHA:
            bpp = 2;
            png_set_IHDR(a_png_ptr, a_info_ptr, width, height, 8, PNG_COLOR_TYPE_GRAY_ALPHA, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
            break;
        case GL_RGB:
            bpp = 3;
            png_set_IHDR(a_png_ptr, a_info_ptr, width, height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
            break;
        case GL_RGBA:
            bpp = 4;
            png_set_IHDR(a_png_ptr, a_info_ptr, width, height, 8, PNG_COLOR_TYPE_RGB_ALPHA, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
            break;
    }

    // write the file header information
    png_write_info(a_png_ptr, a_info_ptr);

    // push image data in png struct
    png_bytep *row_pointers = new png_bytep[height];
    for (unsigned int j=0; j<height; j++)
        row_pointers[height-1-j] = data + bpp*j*width;

    // actually write the image to file
    png_write_image(a_png_ptr, row_pointers);

    // finish writing the rest of the file and cleanup
    delete [] row_pointers;
    png_write_end(a_png_ptr, a_info_ptr);

    return true;
}


//==============================================================================
/*!
    \fn _readFromBuffer(png_structp a_png_ptr, png_bytep a_data, png_size_t a_count)
    Helper function to \ref bool cLoadPNG(cImage* a_image, void *a_buffer, int a_len).
    See libpng documentation on <i>png_set_read_fn()</i> for details.
*/
//==============================================================================

struct _PNGReadBuf
{
    const unsigned char *m_buffer;
    unsigned int         m_len;
    unsigned int         m_count;
};

static void _readFromBuffer(png_structp a_png_ptr, png_bytep a_data, png_size_t a_count)
{
    _PNGReadBuf& buf = *(_PNGReadBuf*)png_get_io_ptr(a_png_ptr);
    memcpy(a_data, buf.m_buffer+buf.m_count, a_count);
    buf.m_count += (unsigned int)(a_count);
}


//==============================================================================
/*!
    \fn _writeToBuffer(png_structp a_png_ptr, png_bytep a_data, png_size_t a_count)
    Helper function to \ref bool cSavePNG(cImage* a_image, char **a_buffer, int *a_len).
    See libpng documentation on <i>png_set_write_fn()</i> for details.
*/
//==============================================================================

struct _PNGWriteBuf
{
    unsigned char *m_buffer;
    unsigned int   m_len;
};

static void _writeToBuffer(png_structp a_png_ptr, png_bytep a_data, png_size_t a_count)
{
    _PNGWriteBuf& buf = *(_PNGWriteBuf*)png_get_io_ptr(a_png_ptr);
    unsigned char *tmp = (unsigned char *)malloc(buf.m_len+a_count);
    memcpy(tmp, buf.m_buffer, buf.m_len);
    memcpy(tmp+buf.m_len, a_data, a_count);
    buf.m_len += (unsigned int)(a_count);
    unsigned char *old = buf.m_buffer;
    buf.m_buffer = tmp;
    if (old) free(old);
}

static void _flushBuffer(png_structp a_png_ptr)
{
}

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Load a PNG image from a file into a cImage structure. 
    If the operation succeeds, then the functions returns __true__ and the 
    image data is loaded into image structure a_image. 
    If the operation fails, then the function returns __false__. 
    In both cases, previous image information stored in a_image is erased.

    \param  a_image  Image structure. 
    \param  a_fileName  Filename.
    \return Returns __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadFilePNG(cImage* a_image, string a_filename)
{
    png_structp  png_ptr;
    png_infop    info_ptr;
    FILE *fp;

    // sanity check
    if (a_image == NULL) 
        return false;

    // open file
    if ((fp = fopen(a_filename.c_str(), "rb")) == NULL)
        return false;

    // initialize the png_struct
    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr)
    {
        fclose(fp);
        return false;
    }

     // allocate memory for image information
     info_ptr = png_create_info_struct(png_ptr);
     if (!info_ptr)
     {
        fclose(fp);
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        return false;
     }

     // set error handling
     if (setjmp(png_jmpbuf(png_ptr)))
     {
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp);
        return false;
     }

     // set up input control for standard C streams
     png_init_io(png_ptr, fp);

     // decompress
     bool ret = _decompressPNG(png_ptr, info_ptr, a_image);

     // cleanup
     png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
     fclose(fp);

     // success
     return ret;
}


//==============================================================================
/*!
    Load a PNG image from a memory buffer into a cImage structure.
    If the operation succeeds, then the functions returns __true__ and the
    image data is loaded into image structure a_image.
    If the operation fails, then the function returns __false__.
    In both cases, previous image information stored in a_image is erased.

    \param  a_image    Image structure.
    \param  a_buffer   Memory buffer containing PNG data.
    \param  a_len      Buffer size in bytes.
    \return Returns __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadPNG(cImage* a_image, const unsigned char *a_buffer, unsigned int a_len)
{
    png_structp  png_ptr;
    png_infop    info_ptr;

    // sanity check
    if (a_image == NULL)
        return false;

    // initialize the png_struct
    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr)
    {
        return false;
    }

    // allocate memory for image information
    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        return false;
    }

    // set error handling
    if (setjmp(png_jmpbuf(png_ptr)))
    {
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        return false;
    }

    // read from buffer
    _PNGReadBuf pngbuf;
    pngbuf.m_buffer = a_buffer;
    pngbuf.m_len    = a_len;
    pngbuf.m_count  = 0;
    png_set_read_fn(png_ptr, &pngbuf, _readFromBuffer);

    // decompress
    bool ret = _decompressPNG(png_ptr, info_ptr, a_image);

    // cleanup
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);

    // success
    return ret;
}


//==============================================================================
/*!
    Save a PNG image from a cImage structure to a file. 
    If the operation succeeds, then the functions returns __true__ and the 
    image data is saved to a file. 
    If the operation fails, then the function returns __false__. 

    \param  a_image  Image structure. 
    \param  a_filename  Filename.
    \return Returns __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSaveFilePNG(cImage* a_image, string a_filename)
{
    FILE       *fp;
    png_structp png_ptr;
    png_infop   info_ptr;

    // open file
    fp = fopen(a_filename.c_str(), "wb");
    if (fp == NULL)
        return false;

    // create and initialize the png_struct
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr)
    {
        fclose(fp);
        return false;
    }

    // allocate/initialize image information struct
    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        fclose(fp);
        png_destroy_write_struct(&png_ptr,  NULL);
        return false;
    }

    // set error handling
    if (setjmp(png_jmpbuf(png_ptr)))
    {
        fclose(fp);
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return false;
    }

    // set up the output control using standard C streams
    png_init_io(png_ptr, fp);

    // compress
    bool ret = _compressPNG(png_ptr, info_ptr, a_image);

    // cleanup
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);

    // success
    return ret;
}


//==============================================================================
/*!
    Save a PNG image from a cImage structure to a memory buffer.
    If the operation succeeds, then the functions returns __true__ and the
    image data is saved to a file.
    If the operation fails, then the function returns __false__.

    \param  a_image  Image structure.
    \param  a_filename  Filename.
    \return Returns __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSavePNG(cImage* a_image, unsigned char **a_buffer, unsigned int *a_len)
{
    png_structp png_ptr;
    png_infop   info_ptr;

    // create and initialize the png_struct
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr)
    {
        return false;
    }

    // allocate/initialize image information struct
    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        png_destroy_write_struct(&png_ptr,  NULL);
        return false;
    }

    // set error handling
    if (setjmp(png_jmpbuf(png_ptr)))
    {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return false;
    }

    // write to buffer
    _PNGWriteBuf pngbuf;
    pngbuf.m_buffer = NULL;
    pngbuf.m_len    = 0;
    png_set_write_fn(png_ptr, &pngbuf, _writeToBuffer, _flushBuffer);

    // compress
    bool ret = _compressPNG(png_ptr, info_ptr, a_image);

    // cleanup
    png_destroy_write_struct(&png_ptr, &info_ptr);

    // retrieve result
    *a_buffer = pngbuf.m_buffer;
    *a_len    = pngbuf.m_len;

    // success
    return ret;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
#endif // C_USE_FILE_PNG
//------------------------------------------------------------------------------





