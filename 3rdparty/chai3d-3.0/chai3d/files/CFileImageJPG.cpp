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
    \author    Sebastien Grange
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 699 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "files/CFileImageJPG.h"
//---------------------------------------------------------------------------
#ifndef NO_DOC
#include <setjmp.h>
extern "C" {
#include "jpeglib.h"
}
struct my_error_mgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};
typedef struct my_error_mgr *my_error_ptr;
METHODDEF (void) my_error_warn (j_common_ptr cinfo) {
    char buffer[JMSG_LENGTH_MAX];
    my_error_ptr myerr = (my_error_ptr) cinfo->err;
    myerr->pub.format_message (cinfo, buffer);
    printf ("jpeg error: %s\n", buffer);
    longjmp (myerr->setjmp_buffer, 1);
}
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Load a JPG image from a file into a cImage structure. 
    If the operation succeeds, then the functions returns \b true and the 
    image data is loaded into image structure a_image. 
    If the operation fails, then the function returns \b false. 
    In both cases, previous image information stored in a_image is erased.

    \fn     bool cLoadFileJPG(cImage* a_image, string a_filename)
    \param  a_image  Image structure. 
    \param  a_fileName  Filename.
    \return Returns \b true in case of success, \b false otherwise.
*/
//===========================================================================
bool cLoadFileJPG(cImage* a_image, string a_filename)
{
    struct jpeg_decompress_struct cinfo;
    struct my_error_mgr           jerr;

    FILE       *infile;
    JSAMPARRAY  buffer;
    int         row_stride;

    // sanity check
    if (a_image == NULL) 
        return false;

    // open file
    if ((infile = fopen(a_filename.c_str(), "rb")) == NULL)
        return false;

    // setup error routines
    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_warn;
    if (setjmp(jerr.setjmp_buffer))
    {
        jpeg_destroy_decompress(&cinfo);
        if (infile != NULL)
            fclose(infile);
        return false;
    }

    // allocate jpeg decompressor
    jpeg_create_decompress(&cinfo);

    // specify data source (the file)
    jpeg_stdio_src(&cinfo, infile);

    // read file parameters with jpeg_read_header()
    jpeg_read_header(&cinfo, TRUE);

    // start decompressor
    jpeg_start_decompress(&cinfo);
    int width  = (int)(cinfo.output_width);
    int height = (int)(cinfo.output_height);

    // we allocate memory for image
    if ((cinfo.out_color_components == 1 && !a_image->allocate(width, height, GL_LUMINANCE)) ||
        (cinfo.out_color_components == 3 && !a_image->allocate(width, height, GL_RGB)))
    {
        fclose(infile);
        return false;
    }

    // retrieve pointer to image data
    unsigned char* data = a_image->getData();

    // allocate temporary buffer
    row_stride = cinfo.output_width * cinfo.output_components;
    buffer =(*cinfo.mem->alloc_sarray)((j_common_ptr) & cinfo, JPOOL_IMAGE, row_stride, 1);

    // while (scan lines remain to be read)
    int line = 0;
    while (cinfo.output_scanline < cinfo.output_height) 
    {
        jpeg_read_scanlines(&cinfo, buffer, 1);
        int index = 3*width*(height-1-line++);
        unsigned char *ptr = buffer[0];
        if (cinfo.out_color_components == 3)
        {
            for (int count=0; count<width; count++)
            {
                for (int channel=0; channel<cinfo.out_color_components; channel++)
                {
                    data[index++] = *ptr++;
                }
            }
        }
    }

    // finish decompression
    jpeg_finish_decompress(&cinfo);

    // release JPEG decompression object
    jpeg_destroy_decompress(&cinfo);

    // cleanup
    fclose (infile);

    return true;
}


//===========================================================================
/*!
    Save a JPG image from a cImage structure to a file. 
    If the operation succeeds, then the functions returns \b true and the 
    image data is saved to a file. 
    If the operation fails, then the function returns \b false. 

    \fn     bool cSaveFileJPG(cImage* a_image, string a_filename)
    \param  a_image  Image structure. 
    \param  a_fileName  Filename.
    \return Returns \b true in case of success, \b false otherwise.
*/
//===========================================================================
bool cSaveFileJPG(cImage* a_image, string a_filename)
{
    const int quality = 80;

    struct jpeg_compress_struct  cinfo;
    FILE                        *pFp;
    struct my_error_mgr          jerr;

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

    // allocate and initialize JPEG compression object
    cinfo.err           = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_warn;
    if (setjmp(jerr.setjmp_buffer))
    {
        jpeg_destroy_compress(&cinfo);
        return false;
    }
    jpeg_create_compress(&cinfo);

    // open file for binary write
    if (NULL == (pFp = fopen(a_filename.c_str(), "wb")))
    {
        jpeg_destroy_compress(&cinfo);
        return false;
    }

    // assign output
    jpeg_stdio_dest(&cinfo, pFp);

    // set parameters for compression
    cinfo.image_width      = width;
    cinfo.image_height     = height;
    switch (a_image->getFormat())
    {
        case GL_RGB:
            cinfo.input_components = 3;
            cinfo.in_color_space   = JCS_RGB;
            break;

        case GL_LUMINANCE:
            cinfo.input_components = 1;
            cinfo.in_color_space   = JCS_GRAYSCALE;
            break;

        // unsupported
        case GL_LUMINANCE_ALPHA:
        case GL_RGBA:
        default:
            fclose(pFp);
            jpeg_destroy_compress(&cinfo);
            return false;
    }

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    
    // start compressor
    jpeg_start_compress(&cinfo, TRUE);

    // write all scanlines at once
    for (int j=height; j>0; j--) 
    {
        unsigned char *ptr = data + cinfo.input_components*width*(j-1);
        jpeg_write_scanlines(&cinfo, &ptr, 1);
    }

    // finish compression
    jpeg_finish_compress(&cinfo);

    // close file
    fclose(pFp);

    // release JPEG compression object
    jpeg_destroy_compress(&cinfo);
  
    return true;
}
