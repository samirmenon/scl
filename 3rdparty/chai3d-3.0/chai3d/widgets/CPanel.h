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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 733 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CPanelH
#define CPanelH
//---------------------------------------------------------------------------
#include "widgets/CGenericWidget.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CPanel.h

    \brief 
    <b> Widgets </b> \n 
    A plain 2D Panel.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cPanel
    \ingroup    widgets

    \brief      
    Implementation of a 2D Panel.
*/
//===========================================================================
class cPanel : public cGenericWidget
{    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    public:

    //! Constructor of cPanel.
    cPanel();

    //! Destructor of cPanel.
    virtual ~cPanel() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    public:

    //! Get width of widget.
    virtual double getWidth() const { return (m_widthPanel); }

    //! Get height of widget.
    virtual double getHeight() const { return (m_heightPanel); }


    //! Set the width, height and radius of each corner of Panel.
    void set(const double& a_width, 
             const double& a_height,
             const double& a_radiusTopLeft = 0,
             const double& a_radiusTopRight = 0,
             const double& a_radiusBottomLeft = 0,
             const double& a_radiusBottomRight = 0);

    //! Set the width and height of Panel.
    void setPanelSize(const double& a_width, 
                      const double& a_height);

    //! Set the radius for each corner of Panel.
    void setCorners(const double& a_radiusTopLeft = 0,
                    const double& a_radiusTopRight = 0,
                    const double& a_radiusBottomLeft = 0,
                    const double& a_radiusBottomRight = 0);

    //! Set top, bottom, left and right margins
    void setMargins(const double& a_marginTop = 0,
                    const double& a_marginBottom = 0,
                    const double& a_marginLeft = 0,
                    const double& a_marginRight = 0);

    //! Set Panel colors.
    void setColorPanel(const cColorf& a_colorPanel);

    //! Set Panel colors.
    //void setColorPanel(const cColorf& a_colorPanel);

    //! Get Panel color.
    inline cColorf getColorTopLeft() const { return (m_colorPanelTopLeft); }

    //! Enable or Disable modelling of Panel
    void setEnabledPanel(const bool a_enabled) { m_enabledPanel = a_enabled; }

    //! Enable or Disable modelling of Panel
    bool getEnabledPanel() {return (m_enabledPanel); }


    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    protected:

    //! Width of Panel 
    double m_widthPanel;

    //! Height of Panel
    double m_heightPanel;

    //! Top margin
    double m_marginTop;

    //! Top margin
    double m_marginBottom;

    //! Top margin
    double m_marginLeft;

    //! Top margin
    double m_marginRight;

    //! Radius of top left corner of Panel.
    double m_radiusTopLeft;

    //! Radius of top right corner of Panel.
    double m_radiusTopRight;

    //! Radius of bottom left corner of Panel.
    double m_radiusBottomLeft;

    //! Radius of bottom right corner of Panel.
    double m_radiusBottomRight;

    //! Number of segments used to render each corner of Panel.
    int m_segmentsPerCorner;

    //! Position along Z-axis when Panel plane is drawn.
    double m_offsetPanelModelZ;

    //! If \b true, Panel modelling is enabled. 
    bool m_enabledPanel;


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    protected:

    //! Update mesh model of Panel.
    virtual void updatePanelModel();

    //! Color of of top left corner of Panel.
    cColorf m_colorPanelTopLeft;

    //! Color of of top right corner of Panel.
    cColorf m_colorPanelTopRight;

    //! Color of of bottom left corner of Panel.
    cColorf m_colorPanelBottomLeft;

    //! Color of of bottom right corner of Panel.
    cColorf m_colorPanelBottomRight;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
