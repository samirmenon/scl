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
    \author    Dan Morris
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CFontH
#define CFontH
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include <stdlib.h>
#include <string.h>
//---------------------------------------------------------------------------
//! Default Font type.
#define CHAI_DEFAULT_FONT_FACE "Arial"
//! Default Font size.
#define CHAI_DEFAULT_FONT_SIZE 12.0f
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CFont.h

    \brief 
    <b> Widgets </b> \n 
    Fonts.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cFont
    \ingroup    widgets

    \brief      
    cFont is a generic and pure virtual Font interface, to be subclassed by 
    platform-specific implementations.  For the simplest, most portable 
    approach, use this class and the static method createFont", which 
    returns an actual Font object.  You may also create subclass Font 
    types directly (see below). \n
    Specific implementations can be found later in this file.
*/
//===========================================================================
class cFont
{

public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cFont.
    cFont();

    //! Destructor of cFont.
    virtual ~cFont() {}


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Use this to obtain an actual, non-virtual Font object.
    static cFont* createFont();

    //! Use this to copy data from an existing Font object.
    static cFont* createFont(const cFont* oldFont);

    //! Renders a single-line string.
    virtual int renderString(const char* a_str)=0;

    //! Change the Font face; may require re-initializing the Font.
    virtual void setFontFace(const char* a_faceName);

    //! Get the current Font face.
    virtual void getFontFace(char* a_faceName) const { strcpy(a_faceName,m_fontFace); }

    //! Change the Font size; may require re-initializing the font.
    virtual void setPointSize(const float& a_pointSize) { m_pointSize = a_pointSize; }

    //! Get the current Font size.
    virtual float getPointSize() const { return m_pointSize; }

    //! Get the width of a particular character.
    virtual int getCharacterWidth(const unsigned char& a_char);

  protected:

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! The point size of the Font.
    float m_pointSize;

    //! The Font face name.
    char m_fontFace[255];

    //! The width of each character in our Font.
    int m_char_widths[255];
};


//===========================================================================
/*!
    \class      cGLUTBitmapFont
    \ingroup    widgets

    \brief      
    OpenGL Fonts provided with the GLUT library.
*/
//===========================================================================
class cGLUTBitmapFont : public cFont
{

  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGLUTBitmapFont.
    cGLUTBitmapFont() { }

    //! Destructor of cGLUTBitmapFont.
    virtual ~cGLUTBitmapFont() { }


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Get the width of a particular character.
    virtual int getCharacterWidth(const unsigned char& a_char);

    /*!
        Renders a string, should not contain any newlines. \n
        Returns 0 for success, -1 for error.
    */
    virtual int renderString(const char* a_str);


  protected:

	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Return the index of the current Font in the table of font names.
    int getBestFontMatch();
};


#if defined(_WIN32)

//===========================================================================
/*!
    \class      cWin32BitmapFont
    \ingroup    widgets

    \brief      
    A 2D, texture-based, win32-specific implementation of cFont.
*/
//===========================================================================
class cWin32BitmapFont : public cFont
{

  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cWin32BitmapFont.
    cWin32BitmapFont();

    //! Destructor of cWin32BitmapFont.
    virtual ~cWin32BitmapFont();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    /*!
        Renders a string, optionally a string w/embedded printf specifiers. \n
        Returns 0 for success, -1 for error.
    */
    virtual int renderString(const char* a_str);

    //! Change the Font face; may require re-initializing the Font.
    virtual void setFontFace(const char* a_faceName);

    //! Change the Font size; may require re-initializing the Font.
    virtual void setPointSize(const float& a_pointSize);

    /*! 
        Used to access win32 font information directly; use with care and be aware
        that the font may need reinitialization if you modify options _after_ the
        font is used for rendering.
    */
    virtual LOGFONT* getLogFont() { return &m_logfont; }

    /*!
        If you want an outline Font instead of a solid font, set this to
        \b false before using the Font for rendering.
    */
    bool m_solidFont;

    //! Get the width of a particular character.
    virtual int getCharacterWidth(const unsigned char& a_char);

protected:
    
	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! The base openGL display list used for our Font, or -1 if we're uninitialized.
    int m_bitmap_font_base;

    //! Information about the actual win32 Font.
    LOGFONT m_logfont;

    //! Parameter relevant to outline Fonts only.
    float m_outlineFontDeviation;

    //! Parameter relevant to outline Fonts only.
	float m_outlineFontExtrusion;

    //! Parameter relevant to outline Fonts only.
    bool m_usePolygonsForOutlineFonts;


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Should be called with an active rendering context; returns 0 for success, -1 for error.
    int initialize();

    //! Clean up.
    int uninitialize();
};

//---------------------------------------------------------------------------
#endif // _WIN32
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
