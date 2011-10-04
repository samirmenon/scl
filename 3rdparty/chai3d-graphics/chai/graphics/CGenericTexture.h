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
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericTextureH
#define CGenericTextureH
//---------------------------------------------------------------------------
#include "CColor.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGenericTexture.h
    
    \brief  
    <b> Graphics </b> \n 
    Texture Base Class.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGenericTexture
    \ingroup    graphics

    \brief      
    cGenericTexture implements a base class for handling OpenGL textures.
*/
//===========================================================================
class cGenericTexture
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    
      //! Constructor of cGenericTexture.
    cGenericTexture() {};

    //! Destructor of cGenericTexture.
    virtual ~cGenericTexture() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Load an image file (CHAI currently supports 24-bit .bmp and 32-bit .tga files).
    virtual bool loadFromFile(const char* a_fileName) { return (false); }

    //! Enable texturing and set this texture as the current texture.
    virtual void render() {};

    //! Call this to force texture re-initialization.
    virtual void markForUpdate() {}
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

