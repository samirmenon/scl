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
#ifndef CBackgroundH
#define CBackgroundH
//------------------------------------------------------------------------------
#include "widgets/CGenericWidget.h"
#include "graphics/CImage.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CBackground.h

    \brief 
    <b> Widgets </b> \n 
    World background.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cBackground
    \ingroup    widgets

    \brief      
    Implementation of a world colored background.
*/
//==============================================================================
class cBackground : public cGenericWidget
{
  public:
    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cBackground.
    cBackground();

    //! Destructor of cBackground.
    virtual ~cBackground() {};


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    //! Get width of widget.
    virtual double getWidth() const { return (m_widthBackground); }

    //! Get height of widget.
    virtual double getHeight() const { return (m_heightBackground); }

    //! Update the dimension of the background in pixel coordinates.
    void update(const unsigned int a_bottomLeftX,
                const unsigned int a_bottomLeftY,
                const unsigned int a_topRightX,
                const unsigned int a_topRightY);

    //! Set uniform color.
    void setUniformColor(cColorf a_color);

    //! Set a vertical gradient color background.
    void setVerticalLinearGradient(cColorf a_topColor, 
                                   cColorf a_bottomColor);

    //! Set a horizontal gradient color background.
    void setHorizontalLinearGradient(cColorf a_leftColor, 
                                     cColorf a_rightColor);

    //! Set a color property at each corner.
    void setCornerColors(cColorf a_topLeftColor,
                         cColorf a_topRightColor, 
                         cColorf a_bottomLeftColor,    
                         cColorf a_bottomRightColor);

    //! Set aspect ratio property of background image.
    inline void setMaintainAspectRatio(bool a_maintainAspectRatio) { m_maintainAspectRatio = a_maintainAspectRatio; }

    //! Get aspect ratio property of background image.
    inline bool getMaintainAspectRatio() { return (m_maintainAspectRatio); }


    //! Load background image from file.
    bool loadFromFile(std::string a_filename);
 
    //! Load background image from image structure. 
    bool loadFromImage(cImage *a_image);


  protected:

    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

    //! Top left vertex.
    unsigned int m_vertexTL;

    //! Top right vertex.
    unsigned int m_vertexTR;

    //! Bottom left vertex.
    unsigned int m_vertexBL;

    //! Bottom right vertex.
    unsigned int m_vertexBR;

    //! Width of background.
    double m_widthBackground;

    //! Height of background.
    double m_heightBackground;

    //! If __true__ then aspect ratio of background image is maintained.
    bool m_maintainAspectRatio;


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    //! Render background in OpenGL.
    virtual void render(cRenderOptions& a_options);
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
