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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1064 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CColorH
#define CColorH
//------------------------------------------------------------------------------
#include "math/CMaths.h"
//------------------------------------------------------------------------------
#ifdef C_USE_OPENGL
#include "GL/glew.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
struct cColorb;
struct cColorf;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CColor.h
    
    \brief  
    <b> Graphics </b> \n 
    Color Properties.
*/
//==============================================================================


//==============================================================================
/*!
    Convert a color component from Float type to Unsigned Byte type.

    \param    a_value  Color expressed in Float type (0.0 - 1.0).
    \return   Return converted component in Unsigned Byte type (0x00 - 0xFF).
*/
//==============================================================================
inline GLubyte cColorFtoB(GLfloat a_value) 
{
    return( (GLubyte)(255.0f * cClamp(a_value, 0.0f, 1.0f)) );
}


//==============================================================================
/*!
    Convert a color component from Unsigned Byte type to Float type.

    \param    a_value  Color expressed in Unsigned Byte type (0x00 - 0xFF.
    \return   Return converted component in Float type (0.0 - 1.0).
*/
//==============================================================================
inline GLfloat cColorBtoF(GLubyte a_value) 
{
    return((1.0f / 255.0f) * (GLfloat)(cClamp(a_value, (GLubyte)(0x00), (GLubyte)(0xFF))));
}


//==============================================================================
/*!
    \struct     cColorf
    \ingroup    graphics

    \brief
    Color property. (__float__ representation)

    \details    
    cColorf describes a color property composed of 4 \e __float__ parameters, 
    __R__,__G__,__B__ and __A__ respectively. \n\n

    Colors can be defined by settings each component individually or by using
    the predefined colors from the palette. These colors can be initialized
    by calling the different public methods of the class.\n\n

    For gray scales, you may use method setGrayLevel() that allows the programmer
    to finely adjust the gray scale level from 0.0 - 1.0. \n\n

    The cColorf class allows the programmer to backup the current color and
    define a new one. The method for this task is backup().
    The previous color can later be retrieved by calling the method restore(). \n

    This capability can be very useful when implementing a selection procedure where 
    triangles change color when being being selected by a computer mouse for instance.
    A different color can be applied to the selected triangle and then restored
    to the original color once the selection has been completed.
*/
//==============================================================================
struct cColorf
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Default constructor of cColorf (color defaults to opaque white).
    */
    //--------------------------------------------------------------------------
    cColorf()
    {
        m_color[0] = 1.0f;
        m_color[1] = 1.0f;
        m_color[2] = 1.0f;
        m_color[3] = 1.0f;

        m_backupColor[0] = m_color[0];
        m_backupColor[1] = m_color[1];
        m_backupColor[2] = m_color[2];
        m_backupColor[3] = m_color[3];
        m_flag_color = false;
    }


    //--------------------------------------------------------------------------
    /*!
        Constructor of cColorf. Define a color by passing its RGBA components
        as parameters.

        \param    a_red    Red component
        \param    a_green  Green component
        \param    a_blue   Blue component
        \param    a_alpha  Alpha component
    */
    //--------------------------------------------------------------------------
    cColorf(const GLfloat a_red, 
            const GLfloat a_green, 
            const GLfloat a_blue,
            const GLfloat a_alpha = 1.0f)
    {
        m_color[0] = cClamp( a_red,   0.0f, 1.0f);
        m_color[1] = cClamp( a_green, 0.0f, 1.0f);
        m_color[2] = cClamp( a_blue,  0.0f, 1.0f);
        m_color[3] = cClamp( a_alpha, 0.0f, 1.0f);

        m_backupColor[0] = m_color[0];
        m_backupColor[1] = m_color[1];
        m_backupColor[2] = m_color[2];
        m_backupColor[3] = m_color[3];
        m_flag_color = true;
    };


    //--------------------------------------------------------------------------
    /*!
        Destructor of cColorf.
    */
    //--------------------------------------------------------------------------
    ~cColorf() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Set value to modification flags.

        \param    a_value    Value to be assigned to modification flags.
    */
    //--------------------------------------------------------------------------
    inline void setModificationFlags(const bool a_value) { m_flag_color = a_value; }

    
    //--------------------------------------------------------------------------
    /*!
        Copy modified variables to another material object.

        \param    a_color  Destination color where modifications are copied.
    */
    //--------------------------------------------------------------------------
    inline void copyTo(cColorf& a_color)
    {
        if (m_flag_color)
        {
            a_color.set(m_color[0], m_color[1], m_color[2], m_color[3]);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        Set a color by passing its RGBA components as float parameters.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
        \param    a_alpha  Alpha component.
    */
    //--------------------------------------------------------------------------
    inline void set(const GLfloat a_red, 
                    const GLfloat a_green, 
                    const GLfloat a_blue,
                    const GLfloat a_alpha)
    {
        m_color[0] = cClamp( a_red,   0.0f, 1.0f);
        m_color[1] = cClamp( a_green, 0.0f, 1.0f);
        m_color[2] = cClamp( a_blue,  0.0f, 1.0f);
        m_color[3] = cClamp( a_alpha, 0.0f, 1.0f);
        m_flag_color = true;
    };


    //--------------------------------------------------------------------------
    /*!
        Set a color by passing its RGB components as float parameters.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
    */
    //--------------------------------------------------------------------------
    inline void set(const GLfloat a_red, 
                    const GLfloat a_green, 
                    const GLfloat a_blue)
    {
        m_color[0] = cClamp( a_red,   0.0f, 1.0f);
        m_color[1] = cClamp( a_green, 0.0f, 1.0f);
        m_color[2] = cClamp( a_blue,  0.0f, 1.0f);
        m_flag_color = true;
    };


    //--------------------------------------------------------------------------
    /*!
        Set a color by passing its RGBA components as byte parameters

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
        \param    a_alpha  Alpha component.
    */
    //--------------------------------------------------------------------------
    inline void setb(const GLubyte a_red, 
                     const GLubyte a_green, 
                     const GLubyte a_blue,
                     const GLubyte a_alpha)
    {
        m_color[0] = cColorBtoF(a_red);
        m_color[1] = cColorBtoF(a_green);
        m_color[2] = cColorBtoF(a_blue);
        m_color[3] = cColorBtoF(a_alpha);
        m_flag_color = true;
    };


    //--------------------------------------------------------------------------
    /*!
        Set a color by passing its RGB components as byte parameters.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
    */
    //--------------------------------------------------------------------------
    inline void setb(const GLubyte a_red, 
                     const GLubyte a_green, 
                     const GLubyte a_blue)
    {
        m_color[0] = cColorBtoF(a_red);
        m_color[1] = cColorBtoF(a_green);
        m_color[2] = cColorBtoF(a_blue);
        m_flag_color = true;
    };


    //--------------------------------------------------------------------------
    /*!
        Multiply the RGB components by a scalar.

        \param    a_scale  Scale factor.
    */
    //--------------------------------------------------------------------------
    inline void mul(const GLfloat a_scale)
    {
        m_color[0] = cClamp( a_scale * m_color[0], 0.0f, 1.0f);
        m_color[1] = cClamp( a_scale * m_color[1], 0.0f, 1.0f);
        m_color[2] = cClamp( a_scale * m_color[2], 0.0f, 1.0f);
        m_flag_color = true;
    };


    //--------------------------------------------------------------------------
    /*!
        Set color by copying three \e GLfloats from an external array, each
        describing one of the RGB components. Alpha is set to \e 1.0.

        \param    a_colorRGB  Pointer to an array of type \e float.
    */
    //--------------------------------------------------------------------------
    inline void setMem3(const GLfloat* a_colorRGB)
    {
        m_color[0] = a_colorRGB[0];
        m_color[1] = a_colorRGB[1];
        m_color[2] = a_colorRGB[2];
        m_color[3] = 1.0;
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
        Set color by copying four \e GLfloats from an external array, each
        describing one of the RGBA components.

        \param    a_colorRGBA  Pointer to an array of type \e float.
    */
    //--------------------------------------------------------------------------
    inline void setMem4(const GLfloat* a_colorRGBA)
    {
        m_color[0] = a_colorRGBA[0];
        m_color[1] = a_colorRGBA[1];
        m_color[2] = a_colorRGBA[2];
        m_color[3] = a_colorRGBA[3];
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Set the \e red component.

          \param    a_red  Red component.
    */
    //--------------------------------------------------------------------------
    inline void setR(const GLfloat a_red)
    {
        m_color[0] = cClamp( a_red, 0.0f, 1.0f);
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e red component.
    */
    //--------------------------------------------------------------------------
    inline GLfloat getR() const
    {
        return(m_color[0]);
    }


    //--------------------------------------------------------------------------
    /*!
          Set the \e green component.

          \param    a_green  Green component.
    */
    //--------------------------------------------------------------------------
    inline void setG(const GLfloat a_green)
    {
        m_color[1] = cClamp( a_green, 0.0f, 1.0f);
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e green component.
    */
    //--------------------------------------------------------------------------
    inline GLfloat getG() const
    {
        return(m_color[1]);
    }


    //--------------------------------------------------------------------------
    /*!
          Set the \e blue component.

          \param    a_blue  Blue component.
    */
    //--------------------------------------------------------------------------
    inline void setB(const GLfloat a_blue)
    {
        m_color[2] = cClamp( a_blue, 0.0f, 1.0f);
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e blue component.
    */
    //--------------------------------------------------------------------------
    inline GLfloat getB() const
    {
        return(m_color[2]);
    }


    //--------------------------------------------------------------------------
    /*!
          Set the \e alpha component.

          \param    a_alpha  Alpha component.
    */
    //--------------------------------------------------------------------------
    inline void setA(const GLfloat a_alpha)
    {
        m_color[3] = cClamp( a_alpha, 0.0f, 1.0f);
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e alpha component.
    */
    //--------------------------------------------------------------------------
    inline GLfloat getA() const
    {
        return(m_color[3]);
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e luminance value of this color.
    */
    //--------------------------------------------------------------------------
    inline GLfloat getLuminance() const
    {
        return (0.33333f * (m_color[0] + m_color[1] + m_color[2]));
    }


    //--------------------------------------------------------------------------
    /*!
          Backup the current color. The color is stored in member
          m_backupColor.
    */
    //--------------------------------------------------------------------------
    inline void backup()
    {
        m_backupColor[0] = m_color[0];
        m_backupColor[1] = m_color[1];
        m_backupColor[2] = m_color[2];
        m_backupColor[3] = m_color[3];
    }


    //--------------------------------------------------------------------------
    /*!
          Restore color from backup copy. The color is retrieved from member
          m_backupColor.
    */
    //--------------------------------------------------------------------------
    inline void restore()
    {
        m_color[0] = m_backupColor[0];
        m_color[1] = m_backupColor[1];
        m_color[2] = m_backupColor[2];
        m_color[3] = m_backupColor[3];
    }


    //--------------------------------------------------------------------------
    /*!
          Render this color in OpenGL (sets it to be the current color).

          Does not confirm that GL color-tracking is enabled.
    */
    //--------------------------------------------------------------------------
    inline void render() const
    {
        #ifdef C_USE_OPENGL
        glColor4fv(&m_color[0]);
        #endif
    };


    //--------------------------------------------------------------------------
    /*!
          Returns a pointer to the raw color array.
    */
    //--------------------------------------------------------------------------
    inline const GLfloat* pColor() const
    {
        #ifdef C_USE_OPENGL
        return (&m_color[0]);
        #endif
    }


    //--------------------------------------------------------------------------
    /*!
          Returns this color, converted to \e cColorb format.
    */
    //--------------------------------------------------------------------------
    cColorb getColorb() const;


    //--------------------------------------------------------------------------
    /*!
          Access the nth component of this color (we provide both const
          and non-const versions so you can use this operator as an l-value
          or an r-value).
    */
    //--------------------------------------------------------------------------
    inline GLfloat operator[](const unsigned int n) const
    {
        if (n<4) 
            return (m_color[n]);
        else 
            return (0.0f);
    }


    //--------------------------------------------------------------------------
    /*!
          Access the nth component of this color (we provide both const
          and non-const versions so you can use this operator as an l-value
          or an r-value).
    */
    //--------------------------------------------------------------------------
    inline GLfloat& operator[](const unsigned int n)
    {
        if (n<4) 
            return m_color[n];
        else 
            return m_color[0];
    }


    //--------------------------------------------------------------------------
    /*!
          Compare two colors. Return __true__ if color components (R,G,B) are
          identical. Return __false__ otherwise. The transparency component is
          ignored.
    */
    //--------------------------------------------------------------------------
    inline bool operator==(const cColorf& a_color)
    {
        if (m_color[0] != a_color[0]) { return (false); }
        if (m_color[1] != a_color[1]) { return (false); }
        if (m_color[2] != a_color[2]) { return (false); }
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
          Compare two colors. Return __false__ if color components (R,G,B) are
          identical. Return __true__ otherwise. The transparency component is
          ignored.
    */
    //--------------------------------------------------------------------------
    inline bool operator!=(const cColorf& a_color)
    {
        if (m_color[0] != a_color[0]) { return (true); }
        if (m_color[1] != a_color[1]) { return (true); }
        if (m_color[2] != a_color[2]) { return (true); }
        return (false);
    }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - RED COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Red Indian.
    inline void setRedIndian()              { setb(0xCD, 0x5C, 0x5C); } 

    //! Set color to Light Coral Red. 
    inline void setRedLightCoral()          { setb(0xF0, 0x80, 0x80); }

    //! Set color to Red Salmon.
    inline void setRedSalmon()              { setb(0xFA, 0x80, 0x72); }

    //! Set color to Dark Red Salmon.
    inline void setRedDarkSalmon()          { setb(0xE9, 0x96, 0x7A); }

    //! Set color to Light Red Salmon.
    inline void setRedLightSalmon()         { setb(0xFF, 0xA0, 0x7A); }

    //! Set color to Red Crimson.
    inline void setRedCrimson()             { setb(0xDC, 0x14, 0x3C); }

    //! Set color to Red.
    inline void setRed()                    { setb(0xFF, 0x00, 0x00); }

    //! Set color to Red Fire Brick.
    inline void setRedFireBrick()           { setb(0xB2, 0x22, 0x22); }

    //! Set color to Dark Red.
    inline void setRedDark()                { setb(0x8B, 0x00, 0x00); }

    
    //--------------------------------------------------------------------------
    // PUBLIC METHODS - PINK COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Pink.
    inline void setPink()                   { setb(0xFF, 0xC0, 0xCB); }

    //! Set color to Light Pink.
    inline void setPinkLight()              { setb(0xFF, 0xB6, 0xC); }

    //! Set color to Hot Pink.
    inline void setPinkHot()                { setb(0xFF, 0x69, 0xB4); }

    //! Set color to Deep Pink.
    inline void setPinkDeep()               { setb(0xFF, 0x14, 0x93); }

    //! Set color to Medium Violet Red.
    inline void setPinkMediumVioletRed()    { setb(0xC7, 0x15, 0x85); }

    //! Set color to Pale Violet Red.
    inline void setPinkPaleVioletRed()      { setb(0xDB, 0x70, 0x93); }

   
    //--------------------------------------------------------------------------
    // PUBLIC METHODS - ORANGE COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Orange Light Salmon.
    inline void setOrangeLightSalmon()      { setb(0xFF, 0xA0, 0x7A); }

    //! Set color to Orange Coral.
    inline void setOrangeCoral()            { setb(0xFF, 0x7F, 0x50); }

    //! Set color to Orange Tomato.
    inline void setOrangeTomato()           { setb(0xFF, 0x63, 0x47); }

    //! Set color to Orange Red.
    inline void setOrangeRed()              { setb(0xFF, 0x45, 0x00); }

    //! Set color to Dark Orange.
    inline void setOrangeDark()             { setb(0xFF, 0x8C, 0x00); }

    //! Set color to Orange.
    inline void setOrange()                 { setb(0xFF, 0xA5, 0x00); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - YELLOW COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Gold.
    inline void setYellowGold()             { setb(0xFF, 0xD7, 0x00); }

    //! Set color to Yellow.
    inline void setYellow()                 { setb(0xFF, 0xFF, 0x00); }

    //! Set color to Light Yellow.
    inline void setYellowLight()            { setb(0xFF, 0xFF, 0xE0); }

    //! Set color to Lemon Chiffon.
    inline void setYellowLemonChiffon()     { setb(0xFF, 0xFA, 0xCD); }

    //! Set color to Light Goldenrod.
    inline void setYellowLightGoldenrod()   { setb(0xFA, 0xFA, 0xD); }

    //! Set color to Papaya Whip.
    inline void setYellowPapayaWhip()       { setb(0xFF, 0xEF, 0xD5); }

    //! Set color to Moccasin.
    inline void setYellowMoccasin()         { setb(0xFF, 0xE4, 0xB5); }

    //! Set color to Peach Puff.
    inline void setYellowPeachPuff()        { setb(0xFF, 0xDA, 0xB9); }

    //! Set color to Pale Goldenrod.
    inline void setYellowPaleGoldenrod()    { setb(0xEE, 0xE8, 0xAA); }

    //! Set color to Khaki.
    inline void setYellowKhaki()            { setb(0xF0, 0xE6, 0x8C); }

    //! Set color to Dark Khaki.
    inline void setYellowDarkKhaki()        { setb(0xBD, 0xB7, 0x6B); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - PURPLE COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Lavender.
    inline void setPurpleLavender()         { setb(0xE6, 0xE6, 0xFA); }

    //! Set color to Thistle.
    inline void setPurpleThistle()          { setb(0xD8, 0xBF, 0xD8); }

    //! Set color to Plum.
    inline void setPurplePlum()             { setb(0xDD, 0xA0, 0xDD); }

    //! Set color to Violet.
    inline void setPurpleViolet()           { setb(0xEE, 0x82, 0xEE); }

    //! Set color to Orchid.
    inline void setPurpleOrchid()           { setb(0xDA, 0x70, 0xD6); }

    //! Set color to Fuchsia.
    inline void setPurpleFuchsia()          { setb(0xFF, 0x00, 0xFF); }

    //! Set color to Magenta.
    inline void setPurpleMagenta()          { setb(0xFF, 0x00, 0xFF); }

    //! Set color to Medium Orchid.
    inline void setPurpleMediumOrchid()     { setb(0xBA, 0x55, 0xD3); }

    //! Set color to Medium Purple.
    inline void setPurpleMedium()           { setb(0x93, 0x70, 0xDB); }

    //! Set color to Amethyst.
    inline void setPurpleAmethyst()         { setb(0x99, 0x66, 0xCC); }

    //! Set color to Blue Violet.
    inline void setPurpleBlueViolet()       { setb(0x8A, 0x2B, 0xE2); }

    //! Set color to Dark Violet.
    inline void setPurpleDarkViolet()       { setb(0x94, 0x00, 0xD3); }

    //! Set color to Dark Orchid.
    inline void setPurpleDarkOrchid()       { setb(0x99, 0x32, 0xCC); }

    //! Set color to Dark Magenta.
    inline void setPurpleDarkMagenta()      { setb(0x8B, 0x00, 0x8B); }

    //! Set color to Purple.
    inline void setPurple()                 { setb(0x80, 0x00, 0x80); }

    //! Set color to Indigo.
    inline void setPurpleIndigo()           { setb(0x4B, 0x00, 0x82); }

    //! Set color to Slate Blue.
    inline void setPurpleSlateBlue()        { setb(0x6A, 0x5A, 0xCD); }

    //! Set color to Dark Slate Blue.
    inline void setPurpleDarkSlateBlue()    { setb(0x48, 0x3D, 0x8B); }

    //! Set color to Medium Slate Blue.
    inline void setPurpleMediumSlateBlue()  { setb(0x7B, 0x68, 0xEE); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GREEN COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Green Yellow.
    inline void setGreenYellow()            { setb(0xAD, 0xFF, 0x2F); }

    //! Set color to Chartreuse.
    inline void setGreenChartreuse()        { setb(0x7F, 0xFF, 0x00); }

    //! Set color to Lawn Green.
    inline void setGreenLawn()              { setb(0x7C, 0xFC, 0x00); }

    //! Set color to Lime.
    inline void setGreenLime()              { setb(0x00, 0xFF, 0x00); }

    //! Set color to Lime Green.
    inline void setGreenLimeGreen()         { setb(0x32, 0xCD, 0x32); }

    //! Set color to Pale Green.
    inline void setGreenPale()              { setb(0x98, 0xFB, 0x98); }

    //! Set color to Light Green.
    inline void setGreenLight()             { setb(0x90, 0xEE, 0x90); }

    //! Set color to Medium Spring Green.
    inline void setGreenMediumSpring()      { setb(0x00, 0xFA, 0x9A); }

    //! Set color to Spring Green.
    inline void setGreenSpring()            { setb(0x00, 0xFF, 0x7F); }

    //! Set color to Medium Sea Green.
    inline void setGreenMediumSea()         { setb(0x3C, 0xB3, 0x71); }

    //! Set color to Sea Green.
    inline void setGreenSea()               { setb(0x2E, 0x8B, 0x57); }

    //! Set color to Forest Green.
    inline void setGreenForest()            { setb(0x22, 0x8B, 0x22); }

    //! Set color to Green.
    inline void setGreen()                  { setb(0x00, 0x80, 0x00); }

    //! Set color to Dark Green.
    inline void setGreenDark()              { setb(0x00, 0x64, 0x00); }

    //! Set color to Yellow Green.
    inline void setGreenYellowGreen()       { setb(0x9A, 0xCD, 0x32); }

    //! Set color to Olive Drab.
    inline void setGreenOliveDrab()         { setb(0x6B, 0x8E, 0x23); }

    //! Set color to Olive.
    inline void setGreenOlive()             { setb(0x80, 0x80, 0x00); }

    //! Set color to Dark Olive Green.
    inline void setGreenDarkOlive()         { setb(0x55, 0x6B, 0x2F); }

    //! Set color to Medium Aquamarine.
    inline void setGreenMediumAquamarine()  { setb(0x66, 0xCD, 0xAA); }

    //! Set color to Dark Sea Green.
    inline void setGreenDarkSea()           { setb(0x8F, 0xBC, 0x8F); }

    //! Set color to Light Sea Green.
    inline void setGreenLightSea()          { setb(0x20, 0xB2, 0xAA); }

    //! Set color to Dark Cyan.
    inline void setGreenDarkCyan()          { setb(0x00, 0x8B, 0x8B); }

    //! Set color to Teal.
    inline void setGreenTeal()              { setb(0x00, 0x80, 0x80); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BLUE COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Aqua.
    inline void setBlueAqua()               { setb(0x00, 0xFF, 0xFF); }

    //! Set color to Cyan.
    inline void setBlueCyan()               { setb(0x00, 0xFF, 0xFF); }

    //! Set color to Light Cyan.
    inline void setBlueLightCyan()          { setb(0xE0, 0xFF, 0xFF); }

    //! Set color to Pale Turquoise.
    inline void setBluePaleTurquoise()      { setb(0xAF, 0xEE, 0xEE); }

    //! Set color to Aquamarine.
    inline void setBlueAquamarine()         { setb(0x7F, 0xFF, 0xD4); }

    //! Set color to Turquoise.
    inline void setBlueTurquoise()          { setb(0x40, 0xE0, 0xD0); }

    //! Set color to Medium Turquoise.
    inline void setBlueMediumTurquoise()    { setb(0x48, 0xD1, 0xCC); }

    //! Set color to Dark Turquoise.
    inline void setBlueDarkTurquoise()      { setb(0x00, 0xCE, 0xD1); }

    //! Set color to Cadet Blue.
    inline void setBlueCadet()              { setb(0x5F, 0x9E, 0xA0); }

    //! Set color to Steel Blue.
    inline void setBlueSteel()              { setb(0x46, 0x82, 0xB4); }

    //! Set color to Light Steel Blue.
    inline void setBlueLightSteel()         { setb(0xB0, 0xC4, 0xDE); }

    //! Set color to Powder Blue.
    inline void setBluePowder()             { setb(0xB0, 0xE0, 0xE6); }

    //! Set color to Light Blue.
    inline void setBlueLight()              { setb(0xAD, 0xD8, 0xE6); }

    //! Set color to Sky Blue.
    inline void setBlueSky()                { setb(0x87, 0xCE, 0xEB); }

    //! Set color to Light Sky Blue.
    inline void setBlueLightSky()           { setb(0x87, 0xCE, 0xFA); }

    //! Set color to Deep Sky Blue.
    inline void setBlueDeepSky()            { setb(0x00, 0xBF, 0xFF); }

    //! Set color to Doger Blue.
    inline void setBlueDodger()             { setb(0x1E, 0x90, 0xFF); }

    //! Set color to Cornflower Blue.
    inline void setBlueCornflower()         { setb(0x64, 0x95, 0xED); }

    //! Set color to Medium Slate Blue.
    inline void setBlueMediumSlate()        { setb(0x7B, 0x68, 0xEE); }

    //! Set color to Royal Blue.
    inline void setBlueRoyal()              { setb(0x41, 0x69, 0xE1); }

    //! Set color to Blue.
    inline void setBlue()                   { setb(0x00, 0x00, 0xFF); }

    //! Set color to Medium Blue.
    inline void setBlueMedium()             { setb(0x00, 0x00, 0xCD); }

    //! Set color to Dark Blue.
    inline void setBlueDark()               { setb(0x00, 0x00, 0x8B); }

    //! Set color to Navy.
    inline void setBlueNavy()               { setb(0x00, 0x00, 0x80); }

    //! Set color to Midnight Blue.
    inline void setBlueMidnight()           { setb(0x19, 0x19, 0x70); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BROWN COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Cornsilk.
    inline void setBrownCornsilk()          { setb(0xFF, 0xF8, 0xDC); }

    //! Set color to Blanched Almond.
    inline void setBrownBlanchedAlmond()    { setb(0xFF, 0xEB, 0xCD); }

    //! Set color to Bisque.
    inline void setBrownBisque()            { setb(0xFF, 0xE4, 0xC4); }

    //! Set color to Navajo White.
    inline void setBrownNavajoWhite()       { setb(0xFF, 0xDE, 0xAD); }

    //! Set color to Wheat.
    inline void setBrownWheat()             { setb(0xF5, 0xDE, 0xB3); }

    //! Set color to Burly Wood.
    inline void setBrownBurlyWood()         { setb(0xDE, 0xB8, 0x87); }

    //! Set color to Tan.
    inline void setBrownTan()               { setb(0xD2, 0xB4, 0x8C); }

    //! Set color to Rosy Brown.
    inline void setBrownRosy()              { setb(0xBC, 0x8F, 0x8F); }

    //! Set color to Sandy Brown.
    inline void setBrownSandy()             { setb(0xF4, 0xA4, 0x60); }

    //! Set color to Brown Goldenrod.
    inline void setBrownGoldenrod()         { setb(0xDA, 0xA5, 0x20); }

    //! Set color to Dark Brown Goldenrod.
    inline void setBrownDarkGoldenrod()     { setb(0xB8, 0x86, 0x0B); }

    //! Set color to Peru.
    inline void setBrownPeru()              { setb(0xCD, 0x85, 0x3F); }

    //! Set color to Chocolate.
    inline void setBrownChocolate()         { setb(0xD2, 0x69, 0x1E); }

    //! Set color to Saddle Brown.
    inline void setBrownSaddle()            { setb(0x8B, 0x45, 0x13); }

    //! Set color to Sienna.
    inline void setBrownSienna()            { setb(0xA0, 0x52, 0x2D); }

    //! Set color to Brown.
    inline void setBrown()                  { setb(0xA5, 0x2A, 0x2A); }

    //! Set color to Maroon.
    inline void setBrownMaroon()            { setb(0x80, 0x00, 0x00); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - WHITE COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to White.
    inline void setWhite()                  { setb(0xFF, 0xFF, 0xFF); }

    //! Set color to White Snow.
    inline void setWhiteSnow()              { setb(0xFF, 0xFA, 0xFA); }

    //! Set color to Honeydew.
    inline void setWhiteHoneydew()          { setb(0xF0, 0xFF, 0xF0); }

    //! Set color to Mint Cream.
    inline void setWhiteMintCream()         { setb(0xF5, 0xFF, 0xFA); }

    //! Set color to Azure.
    inline void setWhiteAzure()             { setb(0xF0, 0xFF, 0xFF); }

    //! Set color to Alice Blue.
    inline void setWhiteAliceBlue()         { setb(0xF0, 0xF8, 0xFF); }

    //! Set color to Ghost White.
    inline void setWhiteGhost()             { setb(0xF8, 0xF8, 0xFF); }

    //! Set color to White Smoke.
    inline void setWhiteSmoke()             { setb(0xF5, 0xF5, 0xF5); }

    //! Set color to Seashell.
    inline void setWhiteSeashell()          { setb(0xFF, 0xF5, 0xEE); }

    //! Set color to Beige.
    inline void setWhiteBeige()             { setb(0xF5, 0xF5, 0xDC); }

    //! Set color to Old Lace.
    inline void setWhiteOldLace()           { setb(0xFD, 0xF5, 0xE6); }

    //! Set color to Floral White.
    inline void setWhiteFloral()            { setb(0xFF, 0xFA, 0xF0); }

    //! Set color to Ivory.
    inline void setWhiteIvory()             { setb(0xFF, 0xFF, 0xF0); }

    //! Set color to Antique White.
    inline void setWhiteAntique()           { setb(0xFA, 0xEB, 0xD7); }

    //! Set color to Linen.
    inline void setWhiteLinen()             { setb(0xFA, 0xF0, 0xE6); }

    //! Set color to Lavender Blush.
    inline void setWhiteLavenderBlush()     { setb(0xFF, 0xF0, 0xF5); }

    //! Set color to Misty Rose.
    inline void setWhiteMistyRose()         { setb(0xFF, 0xE4, 0xE1); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRAY COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Gainsboro.
    inline void setGrayGainsboro()          { setb(0xDC, 0xDC, 0xDC); }

    //! Set color to Light Gray.
    inline void setGrayLight()              { setb(0xD3, 0xD3, 0xD3); }

    //! Set color to Silver.
    inline void setGraySilver()             { setb(0xC0, 0xC0, 0xC0); }

    //! Set color to Dark Gray.
    inline void setGrayDark()               { setb(0xA9, 0xA9, 0xA9); }

    //! Set color to Gray.
    inline void setGray()                   { setb(0x80, 0x80, 0x80); }

    //! Set color to Dim Gray.
    inline void setGrayDim()                { setb(0x69, 0x69, 0x69); }

    //! Set color to Light Slate Gray.
    inline void setGrayLightSlate()         { setb(0x77, 0x88, 0x99); }

    //! Set color to Slate Gray.
    inline void setGraySlate()              { setb(0x70, 0x80, 0x90); }

    //! Set color to Dark Slate Gray.
    inline void setGrayDarkSlate()          { setb(0x2F, 0x4F, 0x4F); }

    //! Set color to Black.
    inline void setBlack()                  { setb(0x00, 0x00, 0x00); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - CUSTOM GRAY COLOR:
    //--------------------------------------------------------------------------

public:

    // Set a custom gray level
    inline void setGrayLevel(const GLfloat a_level) { set(a_level, a_level, a_level); }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Current Color in \e GLfloat format [R,G,B,A].
    GLfloat m_color[4];

    //! Backup Color in \e GLfloat format [R,G,B,A].
    GLfloat m_backupColor[4];

    //! Flag to track if related member has been modified.
    bool m_flag_color;
};


//==============================================================================
/*!
    \struct     cColorb
    \ingroup    graphics
    
    \brief
    Color property. (__byte__ representation)

    \details    
    cColorf describes a color property composed of 4 \e __byte__ parameters, 
    __R__,__G__,__B__ and __A__ respectively. \n\n

    Colors can be defined by settings each component individually or by using
    the predefined colors from the palette. These colors can be initialized
    by calling the different public methods of the class.\n\n

    For gray scales, you may use method setGrayLevel() that allows the programmer
    to finely adjust the gray scale level from 0 - 255. \n\n

    The cColorf class allows the programmer to backup the current color and
    define a new one. The method for this task is backup().
    The previous color can later be retrieved by calling the method restore(). \n

    This capability can be very useful when implementing a selection procedure where 
    triangles change color when being being selected by a computer mouse for instance.
    A different color can be applied to the selected triangle and then restored
    to the original color once the selection has been completed.
*/
//==============================================================================
struct cColorb
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Constructor of cColorb.
    */
    //--------------------------------------------------------------------------
    cColorb()
    {
        // initialize color components R,G,B,A
        m_color[0] = 0xff;
        m_color[1] = 0xff;
        m_color[2] = 0xff;
        m_color[3] = 0xff;
        m_flag_color = false;
    }


    //--------------------------------------------------------------------------
    /*!
        Constructor of cColorb. Define a color by passing its RGBA components.
        as parameters.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
        \param    a_alpha  Alpha component.
    */
    //--------------------------------------------------------------------------
    cColorb(const GLubyte a_red, 
            const GLubyte a_green, 
            const GLubyte a_blue,
            const GLubyte a_alpha = 0xff)
    {
        m_color[0] = a_red;
        m_color[1] = a_green;
        m_color[2] = a_blue;
        m_color[3] = a_alpha;
        m_flag_color = true;
    };


    //--------------------------------------------------------------------------
    /*!
        Destructor of cColorf.
    */
    //--------------------------------------------------------------------------
    ~cColorb() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------
public:

    //--------------------------------------------------------------------------
    /*!
        Set value to modification flags.

        \param    a_value    Value to be assigned to modification flags.
    */
    //--------------------------------------------------------------------------
    inline void setModificationFlags(const bool a_value) { m_flag_color = a_value; }

    
    //--------------------------------------------------------------------------
    /*!
        Copy modified variables to another material object.

        \param    a_color  Destination color where modifications are copied.
    */
    //--------------------------------------------------------------------------
    inline void copyTo(cColorb& a_color)
    {
        if (m_flag_color)
        {
            a_color.set(m_color[0], m_color[1], m_color[2], m_color[3]);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        Set a color by passing its RGBA components as parameters.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
        \param    a_alpha  Alpha component.
    */
    //--------------------------------------------------------------------------
    inline void set(const GLubyte a_red, const GLubyte a_green, const GLubyte a_blue,
                    const GLubyte a_alpha = 0xff)
    {
        m_color[0] = a_red;
        m_color[1] = a_green;
        m_color[2] = a_blue;
        m_color[3] = a_alpha;
        m_flag_color = true;
    };


    //--------------------------------------------------------------------------
    /*!
        Set a color by passing its RGBA components as parameters in Float type.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
        \param    a_alpha  Alpha component.
    */
    //--------------------------------------------------------------------------
    inline void setf(const GLfloat a_red, const GLfloat a_green, const GLfloat a_blue,
                     const GLfloat a_alpha = 1.0f)
    {
        m_color[0] = cColorFtoB(a_red);
        m_color[1] = cColorFtoB(a_green);
        m_color[2] = cColorFtoB(a_blue);
        m_color[3] = cColorFtoB(a_alpha);
        m_flag_color = true;
    };


    //--------------------------------------------------------------------------
    /*!
        Set color by copying three floats from an external array, each
        describing an RGB component. Alpha is set to \e 0xff.

        \param    a_colorRGB  Pointer to array of type \e GLubyte.
    */
    //--------------------------------------------------------------------------
    inline void setMem3(const GLubyte* a_colorRGB)
    {
        m_color[0] = a_colorRGB[0];
        m_color[1] = a_colorRGB[1];
        m_color[2] = a_colorRGB[2];
        m_color[3] = 0xff;
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
        Set color by copying four floats from an external array, each
        describing an RGBA component.

        \param    a_colorRGBA  Pointer to an array of type \e GLubyte.
    */
    //--------------------------------------------------------------------------
    inline void setMem4(const GLubyte* a_colorRGBA)
    {
        m_color[0] = a_colorRGBA[0];
        m_color[1] = a_colorRGBA[1];
        m_color[2] = a_colorRGBA[2];
        m_color[3] = a_colorRGBA[3];
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Set the \e red component.

          \param    a_red  Red component.
    */
    //--------------------------------------------------------------------------
    inline void setR(const GLubyte a_red)
    {
        m_color[0] = a_red;
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e red component.
    */
    //--------------------------------------------------------------------------
    inline GLubyte getR() const
    {
        return(m_color[0]);
    }


    //--------------------------------------------------------------------------
    /*!
          Set the \e green component.

          \param    a_green  Green component.
    */
    //--------------------------------------------------------------------------
    inline void setG(const GLubyte a_green)
    {
        m_color[1] = a_green;
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e green component.
    */
    //--------------------------------------------------------------------------
    inline GLubyte getG() const
    {
        return(m_color[1]);
    }


    //--------------------------------------------------------------------------
    /*!
          Set the \e blue component.

          \param    a_blue  Blue component.
    */
    //--------------------------------------------------------------------------
    inline void setB(const GLubyte a_blue)
    {
        m_color[2] = a_blue;
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e blue component.
    */
    //--------------------------------------------------------------------------
    inline GLubyte getB() const
    {
        return(m_color[2]);
    }


    //--------------------------------------------------------------------------
    /*!
          Set the \e alpha component.

          \param    a_alpha Alpha component.
    */
    //--------------------------------------------------------------------------
    inline void setA(const GLubyte a_alpha)
    {
        m_color[3] = a_alpha;
        m_flag_color = true;
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e alpha component.
    */
    //--------------------------------------------------------------------------
    inline GLubyte getA() const
    {
        return(m_color[3]);
    }


    //--------------------------------------------------------------------------
    /*!
          Read the \e luminance value of this color.
    */
    //--------------------------------------------------------------------------
    inline GLubyte getLuminance() const
    {
        return ((GLubyte)(0.33333 * ((float)m_color[0] + (float)m_color[1] + (float)m_color[2])));
    }


    //--------------------------------------------------------------------------
    /*!
          Render this color in OpenGL (sets it to be the current color).
          Does not confirm that GL color-tracking is enabled.
    */
    //--------------------------------------------------------------------------
    inline void render() const
    {
        #ifdef C_USE_OPENGL 
        glColor4bv((const signed char*)(&m_color[0]));
        #endif
    };


    //--------------------------------------------------------------------------
    /*!
          Backup the current color. The color is stored in member
          m_backupColor.
    */
    //--------------------------------------------------------------------------
    inline void backup()
    {
        m_backupColor[0] = m_color[0];
        m_backupColor[1] = m_color[1];
        m_backupColor[2] = m_color[2];
        m_backupColor[3] = m_color[3];
    }


    //--------------------------------------------------------------------------
    /*!
          Restore color from backup copy. The color is retrieved from member
          m_backupColor.
    */
    //--------------------------------------------------------------------------
    inline void restore()
    {
        m_color[0] = m_backupColor[0];
        m_color[1] = m_backupColor[1];
        m_color[2] = m_backupColor[2];
        m_color[3] = m_backupColor[3];
    }


    //--------------------------------------------------------------------------
    /*!
          Return a pointer to the raw color array.

          \return   Return memory location of color array.
    */
    //--------------------------------------------------------------------------
    inline const GLubyte* pColor() const
    {
        return (&m_color[0]);
    }


    //--------------------------------------------------------------------------
    /*!
          Return this color converted to \e cColorf format.
    */
    //--------------------------------------------------------------------------
    cColorf getColorf() const;


    //--------------------------------------------------------------------------
    /*!
          Access the nth component of this color (we provide both const
          and non-const versions so you can use this operator as an l-value
          or an r-value).
    */
    //--------------------------------------------------------------------------
    inline GLubyte operator[](const unsigned int n) const
    {
        if (n<4) 
            return (m_color[n]);
        else 
            return (0x00);
    }


    //--------------------------------------------------------------------------
    /*!
          Access the nth component of this color (we provide both const
          and non-const versions so you can use this operator as an l-value
          or an r-value).
    */
    //--------------------------------------------------------------------------
    inline GLubyte& operator[](const unsigned int n)
    {
        if (n<4) 
            return m_color[n];
        else 
            return m_color[0];
    }


    //--------------------------------------------------------------------------
    /*!
          Compare two colors. Return __true__ if color components (R,G,B) are
          identical. Return __false__ otherwise. The transparency component is
          ignored.
    */
    //--------------------------------------------------------------------------
    inline bool operator==(const cColorb& a_color)
    {
        if (m_color[0] != a_color[0]) { return (false); }
        if (m_color[1] != a_color[1]) { return (false); }
        if (m_color[2] != a_color[2]) { return (false); }
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
          Compare two colors. Return __false__ if color components (R,G,B) are
          identical. Return __true__ otherwise. The transparency component is
          ignored.
    */
    //--------------------------------------------------------------------------
    inline bool operator!=(const cColorb& a_color)
    {
        if (m_color[0] != a_color[0]) { return (true); }
        if (m_color[1] != a_color[1]) { return (true); }
        if (m_color[2] != a_color[2]) { return (true); }
        return (false);
    }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - RED COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Red Indian.
    inline void setRedIndian()              { set(0xCD, 0x5C, 0x5C); } 

    //! Set color to Light Coral Red. 
    inline void setRedLightCoral()          { set(0xF0, 0x80, 0x80); }

    //! Set color to Red Salmon.
    inline void setRedSalmon()              { set(0xFA, 0x80, 0x72); }

    //! Set color to Dark Red Salmon.
    inline void setRedDarkSalmon()          { set(0xE9, 0x96, 0x7A); }

    //! Set color to Light Red Salmon.
    inline void setRedLightSalmon()         { set(0xFF, 0xA0, 0x7A); }

    //! Set color to Red Crimson.
    inline void setRedCrimson()             { set(0xDC, 0x14, 0x3C); }

    //! Set color to Red.
    inline void setRed()                    { set(0xFF, 0x00, 0x00); }

    //! Set color to Red Fire Brick.
    inline void setRedFireBrick()           { set(0xB2, 0x22, 0x22); }

    //! Set color to Dark Red.
    inline void setRedDark()                { set(0x8B, 0x00, 0x00); }

    
    //--------------------------------------------------------------------------
    // PUBLIC METHODS - PINK COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Pink.
    inline void setPink()                   { set(0xFF, 0xC0, 0xCB); }

    //! Set color to Light Pink.
    inline void setPinkLight()              { set(0xFF, 0xB6, 0xC); }

    //! Set color to Hot Pink.
    inline void setPinkHot()                { set(0xFF, 0x69, 0xB4); }

    //! Set color to Deep Pink.
    inline void setPinkDeep()               { set(0xFF, 0x14, 0x93); }

    //! Set color to Medium Violet Red.
    inline void setPinkMediumVioletRed()    { set(0xC7, 0x15, 0x85); }

    //! Set color to Pale Violet Red.
    inline void setPinkPaleVioletRed()      { set(0xDB, 0x70, 0x93); }

   
    //--------------------------------------------------------------------------
    // PUBLIC METHODS - ORANGE COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Orange Light Salmon.
    inline void setOrangeLightSalmon()      { set(0xFF, 0xA0, 0x7A); }

    //! Set color to Orange Coral.
    inline void setOrangeCoral()            { set(0xFF, 0x7F, 0x50); }

    //! Set color to Orange Tomato.
    inline void setOrangeTomato()           { set(0xFF, 0x63, 0x47); }

    //! Set color to Orange Red.
    inline void setOrangeRed()              { set(0xFF, 0x45, 0x00); }

    //! Set color to Dark Orange.
    inline void setOrangeDark()             { set(0xFF, 0x8C, 0x00); }

    //! Set color to Orange.
    inline void setOrange()                 { set(0xFF, 0xA5, 0x00); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - YELLOW COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Gold.
    inline void setYellowGold()             { set(0xFF, 0xD7, 0x00); }

    //! Set color to Yellow.
    inline void setYellow()                 { set(0xFF, 0xFF, 0x00); }

    //! Set color to Light Yellow.
    inline void setYellowLight()            { set(0xFF, 0xFF, 0xE0); }

    //! Set color to Lemon Chiffon.
    inline void setYellowLemonChiffon()     { set(0xFF, 0xFA, 0xCD); }

    //! Set color to Light Goldenrod.
    inline void setYellowLightGoldenrod()   { set(0xFA, 0xFA, 0xD); }

    //! Set color to Papaya Whip.
    inline void setYellowPapayaWhip()       { set(0xFF, 0xEF, 0xD5); }

    //! Set color to Moccasin.
    inline void setYellowMoccasin()         { set(0xFF, 0xE4, 0xB5); }

    //! Set color to Peach Puff.
    inline void setYellowPeachPuff()        { set(0xFF, 0xDA, 0xB9); }

    //! Set color to Pale Goldenrod.
    inline void setYellowPaleGoldenrod()    { set(0xEE, 0xE8, 0xAA); }

    //! Set color to Khaki.
    inline void setYellowKhaki()            { set(0xF0, 0xE6, 0x8C); }

    //! Set color to Dark Khaki.
    inline void setYellowDarkKhaki()        { set(0xBD, 0xB7, 0x6B); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - PURPLE COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Lavender.
    inline void setPurpleLavender()         { set(0xE6, 0xE6, 0xFA); }

    //! Set color to Thistle.
    inline void setPurpleThistle()          { set(0xD8, 0xBF, 0xD8); }

    //! Set color to Plum.
    inline void setPurplePlum()             { set(0xDD, 0xA0, 0xDD); }

    //! Set color to Violet.
    inline void setPurpleViolet()           { set(0xEE, 0x82, 0xEE); }

    //! Set color to Orchid.
    inline void setPurpleOrchid()           { set(0xDA, 0x70, 0xD6); }

    //! Set color to Fuchsia.
    inline void setPurpleFuchsia()          { set(0xFF, 0x00, 0xFF); }

    //! Set color to Magenta.
    inline void setPurpleMagenta()          { set(0xFF, 0x00, 0xFF); }

    //! Set color to Medium Orchid.
    inline void setPurpleMediumOrchid()     { set(0xBA, 0x55, 0xD3); }

    //! Set color to Medium Purple.
    inline void setPurpleMedium()           { set(0x93, 0x70, 0xDB); }

    //! Set color to Amethyst.
    inline void setPurpleAmethyst()         { set(0x99, 0x66, 0xCC); }

    //! Set color to Blue Violet.
    inline void setPurpleBlueViolet()       { set(0x8A, 0x2B, 0xE2); }

    //! Set color to Dark Violet.
    inline void setPurpleDarkViolet()       { set(0x94, 0x00, 0xD3); }

    //! Set color to Dark Orchid.
    inline void setPurpleDarkOrchid()       { set(0x99, 0x32, 0xCC); }

    //! Set color to Dark Magenta.
    inline void setPurpleDarkMagenta()      { set(0x8B, 0x00, 0x8B); }

    //! Set color to Purple.
    inline void setPurple()                 { set(0x80, 0x00, 0x80); }

    //! Set color to Indigo.
    inline void setPurpleIndigo()           { set(0x4B, 0x00, 0x82); }

    //! Set color to Slate Blue.
    inline void setPurpleSlateBlue()        { set(0x6A, 0x5A, 0xCD); }

    //! Set color to Dark Slate Blue.
    inline void setPurpleDarkSlateBlue()    { set(0x48, 0x3D, 0x8B); }

    //! Set color to Medium Slate Blue.
    inline void setPurpleMediumSlateBlue()  { set(0x7B, 0x68, 0xEE); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GREEN COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Green Yellow.
    inline void setGreenYellow()            { set(0xAD, 0xFF, 0x2F); }

    //! Set color to Chartreuse.
    inline void setGreenChartreuse()        { set(0x7F, 0xFF, 0x00); }

    //! Set color to Lawn Green.
    inline void setGreenLawn()              { set(0x7C, 0xFC, 0x00); }

    //! Set color to Lime.
    inline void setGreenLime()              { set(0x00, 0xFF, 0x00); }

    //! Set color to Lime Green.
    inline void setGreenLimeGreen()         { set(0x32, 0xCD, 0x32); }

    //! Set color to Pale Green.
    inline void setGreenPale()              { set(0x98, 0xFB, 0x98); }

    //! Set color to Light Green.
    inline void setGreenLight()             { set(0x90, 0xEE, 0x90); }

    //! Set color to Medium Spring Green.
    inline void setGreenMediumSpring()      { set(0x00, 0xFA, 0x9A); }

    //! Set color to Spring Green.
    inline void setGreenSpring()            { set(0x00, 0xFF, 0x7F); }

    //! Set color to Medium Sea Green.
    inline void setGreenMediumSea()         { set(0x3C, 0xB3, 0x71); }

    //! Set color to Sea Green.
    inline void setGreenSea()               { set(0x2E, 0x8B, 0x57); }

    //! Set color to Forest Green.
    inline void setGreenForest()            { set(0x22, 0x8B, 0x22); }

    //! Set color to Green.
    inline void setGreen()                  { set(0x00, 0x80, 0x00); }

    //! Set color to Dark Green.
    inline void setGreenDark()              { set(0x00, 0x64, 0x00); }

    //! Set color to Yellow Green.
    inline void setGreenYellowGreen()       { set(0x9A, 0xCD, 0x32); }

    //! Set color to Olive Drab.
    inline void setGreenOliveDrab()         { set(0x6B, 0x8E, 0x23); }

    //! Set color to Olive.
    inline void setGreenOlive()             { set(0x80, 0x80, 0x00); }

    //! Set color to Dark Olive Green.
    inline void setGreenDarkOlive()         { set(0x55, 0x6B, 0x2F); }

    //! Set color to Medium Aquamarine.
    inline void setGreenMediumAquamarine()  { set(0x66, 0xCD, 0xAA); }

    //! Set color to Dark Sea Green.
    inline void setGreenDarkSea()           { set(0x8F, 0xBC, 0x8F); }

    //! Set color to Light Sea Green.
    inline void setGreenLightSea()          { set(0x20, 0xB2, 0xAA); }

    //! Set color to Dark Cyan.
    inline void setGreenDarkCyan()          { set(0x00, 0x8B, 0x8B); }

    //! Set color to Teal.
    inline void setGreenTeal()              { set(0x00, 0x80, 0x80); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BLUE COLORS
    //--------------------------------------------------------------------------

public:

    //! Set color to Aqua.
    inline void setBlueAqua()               { set(0x00, 0xFF, 0xFF); }

    //! Set color to Cyan.
    inline void setBlueCyan()               { set(0x00, 0xFF, 0xFF); }

    //! Set color to Light Cyan.
    inline void setBlueLightCyan()          { set(0xE0, 0xFF, 0xFF); }

    //! Set color to Pale Turquoise.
    inline void setBluePaleTurquoise()      { set(0xAF, 0xEE, 0xEE); }

    //! Set color to Aquamarine.
    inline void setBlueAquamarine()         { set(0x7F, 0xFF, 0xD4); }

    //! Set color to Turquoise.
    inline void setBlueTurquoise()          { set(0x40, 0xE0, 0xD0); }

    //! Set color to Medium Turquoise.
    inline void setBlueMediumTurquoise()    { set(0x48, 0xD1, 0xCC); }

    //! Set color to Dark Turquoise.
    inline void setBlueDarkTurquoise()      { set(0x00, 0xCE, 0xD1); }

    //! Set color to Cadet Blue.
    inline void setBlueCadet()              { set(0x5F, 0x9E, 0xA0); }

    //! Set color to Steel Blue.
    inline void setBlueSteel()              { set(0x46, 0x82, 0xB4); }

    //! Set color to Light Steel Blue.
    inline void setBlueLightSteel()         { set(0xB0, 0xC4, 0xDE); }

    //! Set color to Powder Blue.
    inline void setBluePowder()             { set(0xB0, 0xE0, 0xE6); }

    //! Set color to Light Blue.
    inline void setBlueLight()              { set(0xAD, 0xD8, 0xE6); }

    //! Set color to Sky Blue.
    inline void setBlueSky()                { set(0x87, 0xCE, 0xEB); }

    //! Set color to Light Sky Blue.
    inline void setBlueLightSky()           { set(0x87, 0xCE, 0xFA); }

    //! Set color to Deep Sky Blue.
    inline void setBlueDeepSky()            { set(0x00, 0xBF, 0xFF); }

    //! Set color to Doger Blue.
    inline void setBlueDodger()             { set(0x1E, 0x90, 0xFF); }

    //! Set color to Cornflower Blue.
    inline void setBlueCornflower()         { set(0x64, 0x95, 0xED); }

    //! Set color to Medium Slate Blue.
    inline void setBlueMediumSlate()        { set(0x7B, 0x68, 0xEE); }

    //! Set color to Royal Blue.
    inline void setBlueRoyal()              { set(0x41, 0x69, 0xE1); }

    //! Set color to Blue.
    inline void setBlue()                   { set(0x00, 0x00, 0xFF); }

    //! Set color to Medium Blue.
    inline void setBlueMedium()             { set(0x00, 0x00, 0xCD); }

    //! Set color to Dark Blue.
    inline void setBlueDark()               { set(0x00, 0x00, 0x8B); }

    //! Set color to Navy.
    inline void setBlueNavy()               { set(0x00, 0x00, 0x80); }

    //! Set color to Midnight Blue.
    inline void setBlueMidnight()           { set(0x19, 0x19, 0x70); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BROWN COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Cornsilk.
    inline void setBrownCornsilk()          { set(0xFF, 0xF8, 0xDC); }

    //! Set color to Blanched Almond.
    inline void setBrownBlanchedAlmond()    { set(0xFF, 0xEB, 0xCD); }

    //! Set color to Bisque.
    inline void setBrownBisque()            { set(0xFF, 0xE4, 0xC4); }

    //! Set color to Navajo White.
    inline void setBrownNavajoWhite()       { set(0xFF, 0xDE, 0xAD); }

    //! Set color to Wheat.
    inline void setBrownWheat()             { set(0xF5, 0xDE, 0xB3); }

    //! Set color to Burly Wood.
    inline void setBrownBurlyWood()         { set(0xDE, 0xB8, 0x87); }

    //! Set color to Tan.
    inline void setBrownTan()               { set(0xD2, 0xB4, 0x8C); }

    //! Set color to Rosy Brown.
    inline void setBrownRosy()              { set(0xBC, 0x8F, 0x8F); }

    //! Set color to Sandy Brown.
    inline void setBrownSandy()             { set(0xF4, 0xA4, 0x60); }

    //! Set color to Brown Goldenrod.
    inline void setBrownGoldenrod()         { set(0xDA, 0xA5, 0x20); }

    //! Set color to Dark Brown Goldenrod.
    inline void setBrownDarkGoldenrod()     { set(0xB8, 0x86, 0x0B); }

    //! Set color to Peru.
    inline void setBrownPeru()              { set(0xCD, 0x85, 0x3F); }

    //! Set color to Chocolate.
    inline void setBrownChocolate()         { set(0xD2, 0x69, 0x1E); }

    //! Set color to Saddle Brown.
    inline void setBrownSaddle()            { set(0x8B, 0x45, 0x13); }

    //! Set color to Sienna.
    inline void setBrownSienna()            { set(0xA0, 0x52, 0x2D); }

    //! Set color to Brown.
    inline void setBrown()                  { set(0xA5, 0x2A, 0x2A); }

    //! Set color to Maroon.
    inline void setBrownMaroon()            { set(0x80, 0x00, 0x00); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - WHITE COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to White.
    inline void setWhite()                  { set(0xFF, 0xFF, 0xFF); }

    //! Set color to White Snow.
    inline void setWhiteSnow()              { set(0xFF, 0xFA, 0xFA); }

    //! Set color to Honeydew.
    inline void setWhiteHoneydew()          { set(0xF0, 0xFF, 0xF0); }

    //! Set color to Mint Cream.
    inline void setWhiteMintCream()         { set(0xF5, 0xFF, 0xFA); }

    //! Set color to Azure.
    inline void setWhiteAzure()             { set(0xF0, 0xFF, 0xFF); }

    //! Set color to Alice Blue.
    inline void setWhiteAliceBlue()         { set(0xF0, 0xF8, 0xFF); }

    //! Set color to Ghost White.
    inline void setWhiteGhost()             { set(0xF8, 0xF8, 0xFF); }

    //! Set color to White Smoke.
    inline void setWhiteSmoke()             { set(0xF5, 0xF5, 0xF5); }

    //! Set color to Seashell.
    inline void setWhiteSeashell()          { set(0xFF, 0xF5, 0xEE); }

    //! Set color to Beige.
    inline void setWhiteBeige()             { set(0xF5, 0xF5, 0xDC); }

    //! Set color to Old Lace.
    inline void setWhiteOldLace()           { set(0xFD, 0xF5, 0xE6); }

    //! Set color to Floral White.
    inline void setWhiteFloral()            { set(0xFF, 0xFA, 0xF0); }

    //! Set color to Ivory.
    inline void setWhiteIvory()             { set(0xFF, 0xFF, 0xF0); }

    //! Set color to Antique White.
    inline void setWhiteAntique()           { set(0xFA, 0xEB, 0xD7); }

    //! Set color to Linen.
    inline void setWhiteLinen()             { set(0xFA, 0xF0, 0xE6); }

    //! Set color to Lavender Blush.
    inline void setWhiteLavenderBlush()     { set(0xFF, 0xF0, 0xF5); }

    //! Set color to Misty Rose.
    inline void setWhiteMistyRose()         { set(0xFF, 0xE4, 0xE1); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRAY COLORS:
    //--------------------------------------------------------------------------

public:

    //! Set color to Gainsboro.
    inline void setGrayGainsboro()          { set(0xDC, 0xDC, 0xDC); }

    //! Set color to Light Gray.
    inline void setGrayLight()              { set(0xD3, 0xD3, 0xD3); }

    //! Set color to Silver.
    inline void setGraySilver()             { set(0xC0, 0xC0, 0xC0); }

    //! Set color to Dark Gray.
    inline void setGrayDark()               { set(0xA9, 0xA9, 0xA9); }

    //! Set color to Gray.
    inline void setGray()                   { set(0x80, 0x80, 0x80); }

    //! Set color to Dim Gray.
    inline void setGrayDim()                { set(0x69, 0x69, 0x69); }

    //! Set color to Light Slate Gray.
    inline void setGrayLightSlate()         { set(0x77, 0x88, 0x99); }

    //! Set color to Slate Gray.
    inline void setGraySlate()              { set(0x70, 0x80, 0x90); }

    //! Set color to Dark Slate Gray.
    inline void setGrayDarkSlate()          { set(0x2F, 0x4F, 0x4F); }

    //! Set color to Black.
    inline void setBlack()                  { set(0x00, 0x00, 0x00); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - CUSTOM GRAY COLOR
    //--------------------------------------------------------------------------

public:

    // Set a custom gray level
    inline void setGrayLevel(const GLubyte a_level) { set(a_level, a_level, a_level); }

  
    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Color in \e GLubyte format [R,G,B,A].
    GLubyte m_color[4];

    //! Backup Color in \e GLfloat format [R,G,B,A].
    GLubyte m_backupColor[4];

    //! Flag to track if related member has been modified.
    bool m_flag_color;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

