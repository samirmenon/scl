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
    \author    Dan Morris
    \version   2.0.0 $Rev: 253 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CColorH
#define CColorH
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "math/CMaths.h"
//---------------------------------------------------------------------------
struct cColorb;
struct cColorf;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CColor.h
    
    \brief  
    <b> Graphics </b> \n 
    Color Properties.
*/
//===========================================================================

//===========================================================================
/*!
    \struct     cColorf
    \ingroup    graphics

    \brief    
    cColorf describes a color composed of 4 \e GLfloats.
*/
//===========================================================================
struct cColorf
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Default constructor of cColorf (color defaults to opaque white).
    */
    //-----------------------------------------------------------------------
    cColorf()
    {
        // initialize color components R,G,B,A
        m_color[0] = 1.0;
        m_color[1] = 1.0;
        m_color[2] = 1.0;
        m_color[3] = 1.0;
    }


    //-----------------------------------------------------------------------
    /*!
        Constructor of cColorf. Define a color by passing its RGBA components
        as parameters.

        \param    a_red    Red component
        \param    a_green  Green component
        \param    a_blue   Blue component
        \param    a_alpha  Alpha component
    */
    //-----------------------------------------------------------------------
    cColorf(const GLfloat a_red, const GLfloat a_green, const GLfloat a_blue,
            const GLfloat a_alpha = 1.0)
    {
        m_color[0] = cClamp( a_red,   0.0f, 1.0f);
        m_color[1] = cClamp( a_green, 0.0f, 1.0f);
        m_color[2] = cClamp( a_blue,  0.0f, 1.0f);
        m_color[3] = cClamp( a_alpha, 0.0f, 1.0f);
    };


    //-----------------------------------------------------------------------
    /*!
        Destructor of cColorf.
    */
    //-----------------------------------------------------------------------
    ~cColorf() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Set a color by passing its RGBA components as parameters.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
        \param    a_alpha  Alpha component.
    */
    //-----------------------------------------------------------------------
    inline void set(const GLfloat a_red, const GLfloat a_green, const GLfloat a_blue,
                    const GLfloat a_alpha = 1.0)
    {
        m_color[0] = cClamp( a_red,   0.0f, 1.0f);
        m_color[1] = cClamp( a_green, 0.0f, 1.0f);
        m_color[2] = cClamp( a_blue,  0.0f, 1.0f);
        m_color[3] = cClamp( a_alpha, 0.0f, 1.0f);
    };


    //-----------------------------------------------------------------------
    /*!
        Set color by copying three \e GLfloats from an external array, each
        describing one of the RGB components. Alpha is set to \e 1.0.

        \param    a_colorRGB  Pointer to an array of type \e float.
    */
    //-----------------------------------------------------------------------
    inline void setMem3(const GLfloat* a_colorRGB)
    {
        m_color[0] = a_colorRGB[0];
        m_color[1] = a_colorRGB[1];
        m_color[2] = a_colorRGB[2];
        m_color[3] = 1.0;
    }


    //-----------------------------------------------------------------------
    /*!
        Set color by copying four \e GLfloats from an external array, each
        describing one of the RGBA components.

        \param    a_colorRGBA  Pointer to an array of type \e float.
    */
    //-----------------------------------------------------------------------
    inline void setMem4(const GLfloat* a_colorRGBA)
    {
        m_color[0] = a_colorRGBA[0];
        m_color[1] = a_colorRGBA[1];
        m_color[2] = a_colorRGBA[2];
        m_color[3] = a_colorRGBA[3];
    }


    //-----------------------------------------------------------------------
    /*!
          Access the nth component of this color (we provide both const
          and non-const versions so you can use this operator as an l-value
          or an r-value).
    */
    //-----------------------------------------------------------------------
    inline GLfloat operator[](const unsigned int n) const
    {
        if (n<4) return m_color[n];
        else return 0.0f;
    }


    //-----------------------------------------------------------------------
    /*!
          Access the nth component of this color (we provide both const
          and non-const versions so you can use this operator as an l-value
          or an r-value).
    */
    //-----------------------------------------------------------------------
    inline GLfloat& operator[](const unsigned int n)
    {
        if (n<4) return m_color[n];
        else return m_color[0];
    }


    //-----------------------------------------------------------------------
    /*!
          Set the \e red component.

          \param    a_red  Red component.
    */
    //-----------------------------------------------------------------------
    inline void setR(const GLfloat a_red)
    {
        m_color[0] = cClamp( a_red, 0.0f, 1.0f);
    }


    //-----------------------------------------------------------------------
    /*!
          Read the \e red component.
    */
    //-----------------------------------------------------------------------
    inline GLfloat getR() const
    {
        return(m_color[0]);
    }


    //-----------------------------------------------------------------------
    /*!
          Set the \e green component.

          \param    a_green  Green component.
    */
    //-----------------------------------------------------------------------
    inline void setG(const GLfloat a_green)
    {
        m_color[1] = cClamp( a_green, 0.0f, 1.0f);
    }


    //-----------------------------------------------------------------------
    /*!
          Read the \e green component.
    */
    //-----------------------------------------------------------------------
    inline GLfloat getG() const
    {
        return(m_color[1]);
    }


    //-----------------------------------------------------------------------
    /*!
          Set the \e blue component.

          \param    a_blue  Blue component.
    */
    //-----------------------------------------------------------------------
    inline void setB(const GLfloat a_blue)
    {
        m_color[2] = cClamp( a_blue, 0.0f, 1.0f);
    }


    //-----------------------------------------------------------------------
    /*!
          Read the \e blue component.
    */
    //-----------------------------------------------------------------------
    inline GLfloat getB() const
    {
        return(m_color[2]);
    }


    //-----------------------------------------------------------------------
    /*!
          Set the \e alpha component.

          \param    a_alpha  Alpha component.
    */
    //-----------------------------------------------------------------------
    inline void setA(const GLfloat a_alpha)
    {
        m_color[3] = cClamp( a_alpha, 0.0f, 1.0f);
    }


    //-----------------------------------------------------------------------
    /*!
          Read the \e alpha component.
    */
    //-----------------------------------------------------------------------
    inline GLfloat getA() const
    {
        return(m_color[3]);
    }


    //-----------------------------------------------------------------------
    /*!
          Render this color in OpenGL (sets it to be the current color).

          Does not confirm that GL color-tracking is enabled.
    */
    //-----------------------------------------------------------------------
    inline void render() const
    {
        glColor4fv(&m_color[0]);
    };


    //-----------------------------------------------------------------------
    /*!
          Returns a pointer to the raw color array.
    */
    //-----------------------------------------------------------------------
    inline const GLfloat* pColor() const
    {
        return (&m_color[0]);
    }


    //-----------------------------------------------------------------------
    /*!
          Returns this color, converted to \e cColorb format.
    */
    //-----------------------------------------------------------------------
    cColorb getColorb() const;

  public:

    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Color in \e GLfloat format [R,G,B,A].
    GLfloat m_color[4];
};


//===========================================================================
/*!
    \struct     cColorb
    \ingroup    graphics
    
    \brief    
    cColorb describes a color composed of 4 \e bytes.
*/
//===========================================================================
struct cColorb
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Constructor of cColorb.
    */
    //-----------------------------------------------------------------------
    cColorb()
    {
        // initialize color components R,G,B,A
        m_color[0] = 0xff;
        m_color[1] = 0xff;
        m_color[2] = 0xff;
        m_color[3] = 0xff;
    }


    //-----------------------------------------------------------------------
    /*!
        Constructor of cColorb. Define a color by passing its RGBA components.
        as parameters.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
        \param    a_alpha  Alpha component.
    */
    //-----------------------------------------------------------------------
    cColorb(const GLubyte a_red, const GLubyte a_green, const GLubyte a_blue,
            const GLubyte a_alpha = 0xff)
    {
        m_color[0] = a_red;
        m_color[1] = a_green;
        m_color[2] = a_blue;
        m_color[3] = a_alpha;
    };


    //-----------------------------------------------------------------------
    /*!
        Destructor of cColorf.
    */
    //-----------------------------------------------------------------------
    ~cColorb() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Set a color by passing its RGBA components as parameters.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
        \param    a_alpha  Alpha component.
    */
    //-----------------------------------------------------------------------
    inline void set(const GLubyte a_red, const GLubyte a_green, const GLubyte a_blue,
                    const GLubyte a_alpha = 0xff)
    {
        m_color[0] = a_red;
        m_color[1] = a_green;
        m_color[2] = a_blue;
        m_color[3] = a_alpha;
    };


    //-----------------------------------------------------------------------
    /*!
        Set color by copying three floats from an external array, each
        describing an RGB component. Alpha is set to \e 0xff.

        \param    a_colorRGB  Pointer to array of type \e GLubyte.
    */
    //-----------------------------------------------------------------------
    inline void setMem3(const GLubyte* a_colorRGB)
    {
        m_color[0] = a_colorRGB[0];
        m_color[1] = a_colorRGB[1];
        m_color[2] = a_colorRGB[2];
        m_color[3] = 0xff;
    }


    //-----------------------------------------------------------------------
    /*!
        Set color by copying four floats from an external array, each
        describing an RGBA component.

        \param    a_colorRGBA  Pointer to an array of type \e GLubyte.
    */
    //-----------------------------------------------------------------------
    inline void setMem4(const GLubyte* a_colorRGBA)
    {
        m_color[0] = a_colorRGBA[0];
        m_color[1] = a_colorRGBA[1];
        m_color[2] = a_colorRGBA[2];
        m_color[3] = a_colorRGBA[3];
    }


    //-----------------------------------------------------------------------
    /*!
          Set the \e red component.

          \param    a_red  Red component.
    */
    //-----------------------------------------------------------------------
    inline void setR(const GLubyte a_red)
    {
        m_color[0] = a_red;
    }


    //-----------------------------------------------------------------------
    /*!
          Read the \e red component.
    */
    //-----------------------------------------------------------------------
    inline GLfloat getR() const
    {
        return(m_color[0]);
    }


    //-----------------------------------------------------------------------
    /*!
          Set the \e green component.

          \param    a_green  Green component.
    */
    //-----------------------------------------------------------------------
    inline void setG(const GLubyte a_green)
    {
        m_color[1] = a_green;
    }


    //-----------------------------------------------------------------------
    /*!
          Read the \e green component.
    */
    //-----------------------------------------------------------------------
    inline GLfloat getG() const
    {
        return(m_color[1]);
    }


    //-----------------------------------------------------------------------
    /*!
          Set the \e blue component.

          \param    a_blue  Blue component.
    */
    //-----------------------------------------------------------------------
    inline void setB(const GLubyte a_blue)
    {
        m_color[2] = a_blue;
    }


    //-----------------------------------------------------------------------
    /*!
          Read the \e blue component.
    */
    //-----------------------------------------------------------------------
    inline GLfloat getB() const
    {
        return(m_color[2]);
    }


    //-----------------------------------------------------------------------
    /*!
          Set the \e alpha component.

          \param    a_alpha Alpha component.
    */
    //-----------------------------------------------------------------------
    inline void setA(const GLubyte a_alpha)
    {
        m_color[3] = a_alpha;
    }


    //-----------------------------------------------------------------------
    /*!
          Read the \e alpha component.
    */
    //-----------------------------------------------------------------------
    inline GLfloat getA() const
    {
        return(m_color[3]);
    }


    //-----------------------------------------------------------------------
    /*!
          Render this color in OpenGL (sets it to be the current color).
          Does not confirm that GL color-tracking is enabled.
    */
    //-----------------------------------------------------------------------
    inline void render() const
    {
        glColor4bv((const signed char*)(&m_color[0]));
    };


    //-----------------------------------------------------------------------
    /*!
          Return a pointer to the raw color array.

          \return   Return memory location of color array.
    */
    //-----------------------------------------------------------------------
    inline const GLubyte* pColor() const
    {
        return (&m_color[0]);
    }


    //-----------------------------------------------------------------------
    /*!
          Return this color converted to \e cColorf format.
    */
    //-----------------------------------------------------------------------
    cColorf getColorf() const;


  public:

    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Color in \e GLubyte format [R,G,B,A].
    GLubyte m_color[4];
};


//---------------------------------------------------------------------------
// GENERAL CONSTANTS
//---------------------------------------------------------------------------

//! Color Red.
extern cColorf CHAI_COLOR_RED;

//! Color Green.
extern cColorf CHAI_COLOR_GREEN;

//! Color Blue.
extern cColorf CHAI_COLOR_BLUE;

//! Color Yellow.
extern cColorf CHAI_COLOR_YELLOW;

//! Color Cyan.
extern cColorf CHAI_COLOR_CYAN;

//! Color Magenta.
extern cColorf CHAI_COLOR_MAGENTA;

//! Color Black.
extern cColorf CHAI_COLOR_BLACK;

//! Color White.
extern cColorf CHAI_COLOR_WHITE;


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

