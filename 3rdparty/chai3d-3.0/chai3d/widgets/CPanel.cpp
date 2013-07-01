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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 699 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "widgets/CPanel.h"
//------------------------------------------------------------------------------
#include "graphics/CPrimitives.h"
//------------------------------------------------------------------------------
#include <vector>
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cPanel.

    \fn         cPanel::cPanel()
*/
//==============================================================================
cPanel::cPanel()
{
    // use vertex colors
    m_useVertexColors = true;

    // disable material properties
    m_useMaterialProperty = false;

    // use display list to accelerate rendering
    m_useDisplayList = true;

    // set default colors
    m_colorPanelTopLeft.setGrayLevel(0.40f);
    m_colorPanelTopRight.setGrayLevel(0.35f);
    m_colorPanelBottomLeft.setGrayLevel(0.25f);
    m_colorPanelBottomRight.setGrayLevel(0.20f);

    // set default margin values    
    m_marginLeft =      0.0;
    m_marginRight =     0.0;
    m_marginTop =       0.0;
    m_marginBottom =    0.0;

    // set default radius values
    m_radiusTopLeft =       0.0;
    m_radiusTopRight =      0.0;
    m_radiusBottomLeft =    0.0;
    m_radiusBottomRight =   0.0;

    // number of line segments used to render each corner
    m_segmentsPerCorner = 8;

    // by default, Panel is drawn at position Z=0 along the Z-axis
    m_offsetPanelModelZ = 0.0;

    // set a default panel size
    set(200,        // width
        200,        // height
        8,8,8,8);   // radius for each corner
}


//==============================================================================
/*!
    Set the width, height and radius for each corner of Panel.

    \fn         void cPanel::set(const double& a_width, 
                 const double& a_height,
                 const double& a_radiusTopLeft,
                 const double& a_radiusTopRight,
                 const double& a_radiusBottomLeft,
                 const double& a_radiusBottomRight)

    \param      a_width  Width of Panel.
    \param      a_height  Height of Panel.
    \param      a_radiusTopLeft  Radius of top left corner.
    \param      a_radiusTopRight  Radius of top right corner.
    \param      a_radiusBottomLeft  Radius of bottom left corner.
    \param      a_radiusBottomRight  Radius of bottom right corner.
*/
//==============================================================================
void cPanel::set(const double& a_width, 
                 const double& a_height,
                 const double& a_radiusTopLeft,
                 const double& a_radiusTopRight,
                 const double& a_radiusBottomLeft,
                 const double& a_radiusBottomRight)
{
    // set new values for each corner
    m_radiusTopLeft = cAbs(a_radiusTopLeft);
    m_radiusTopRight = cAbs(a_radiusTopRight);
    m_radiusBottomLeft = cAbs(a_radiusBottomLeft);
    m_radiusBottomRight = cAbs(a_radiusBottomRight);

    // set new values for width and height. 
    double w = cMax(a_radiusTopLeft, a_radiusBottomLeft) + 
               cMax(a_radiusTopRight, a_radiusBottomRight);
    m_widthPanel = cMax(a_width, w);

    double h = cMax(a_radiusTopLeft, a_radiusTopRight) + 
               cMax(a_radiusBottomLeft, a_radiusBottomRight);
    m_heightPanel = cMax(a_height, h);
    
    // update model of panel
    updatePanelModel();
}


//==============================================================================
/*!
    Set the width and height of Panel.

    \fn         void setPanelSize::setSize(const double& a_width, 
                     const double& a_height)

    \param      a_width  Width of Panel.
    \param      a_height  Height of Panel.
*/
//==============================================================================
void cPanel::setPanelSize(const double& a_width, 
                          const double& a_height)
{
    // set new values for width and height.
    double w = cMax(m_radiusTopLeft, m_radiusBottomLeft) + 
               cMax(m_radiusTopRight, m_radiusBottomRight);
    m_widthPanel = cMax(a_width, w);

    double h = cMax(m_radiusTopLeft, m_radiusTopRight) + 
               cMax(m_radiusBottomLeft, m_radiusBottomRight);
    m_heightPanel = cMax(a_height, h);

    // update model of panel
    updatePanelModel();
}


//==============================================================================
/*!
    Set the top, bottom, left and right margins.

    \fn         void cPanel::setMargins(const double& a_marginTop,
                        const double& a_marginBottom,
                        const double& a_marginLeft,
                        const double& a_marginRight)

    \param      a_radiusTopLeft  Radius of top left corner.
    \param      a_radiusTopRight  Radius of top right corner.
    \param      a_radiusBottomLeft  Radius of bottom left corner.
    \param      a_radiusBottomRight  Radius of bottom right corner.
*/
//==============================================================================
void cPanel::setMargins(const double& a_marginTop,
                        const double& a_marginBottom,
                        const double& a_marginLeft,
                        const double& a_marginRight)
{
    // set new values
    m_marginTop = cAbs(a_marginTop);
    m_marginBottom = cAbs(a_marginBottom);
    m_marginLeft = cAbs(a_marginLeft);
    m_marginRight = cAbs(a_marginRight);

    // update model of panel
    updatePanelModel();
}


//==============================================================================
/*!
    Set radius for each corner of Panel.

    \fn         void cPanel::setCorners(const double& a_radiusTopLeft,
                        const double& a_radiusTopRight,
                        const double& a_radiusBottomLeft,
                        const double& a_radiusBottomRight)

    \param      a_radiusTopLeft  Radius of top left corner.
    \param      a_radiusTopRight  Radius of top right corner.
    \param      a_radiusBottomLeft  Radius of bottom left corner.
    \param      a_radiusBottomRight  Radius of bottom right corner.
*/
//==============================================================================
void cPanel::setCorners(const double& a_radiusTopLeft,
                        const double& a_radiusTopRight,
                        const double& a_radiusBottomLeft,
                        const double& a_radiusBottomRight)
{
    // set new values
    m_radiusTopLeft = cAbs(a_radiusTopLeft);
    m_radiusTopRight = cAbs(a_radiusTopRight);
    m_radiusBottomLeft = cAbs(a_radiusBottomLeft);
    m_radiusBottomRight = cAbs(a_radiusBottomRight);

    // check dimension for width and height. adjust accordingly
    double w = cMax(m_radiusTopLeft, m_radiusBottomLeft) + 
               cMax(m_radiusTopRight, m_radiusBottomRight);
    m_widthPanel = cMax(m_widthPanel, w);

    double h = cMax(m_radiusTopLeft, m_radiusTopRight) + 
               cMax(m_radiusBottomLeft, m_radiusBottomRight);
    m_heightPanel = cMax(m_heightPanel, h);

    // update model of panel
    updatePanelModel();
}


//==============================================================================
/*!
   Set color of Panel.

    \fn         void cPanel::setColorPanel(const cColorf& a_colorPanel)

    \param      a_colorPanel  Color of Panel.
*/
//==============================================================================
void cPanel::setColorPanel(const cColorf& a_colorPanel)
{
    // assign new color
    m_colorPanelTopLeft     = a_colorPanel;
    m_colorPanelTopRight    = a_colorPanel;
    m_colorPanelBottomLeft  = a_colorPanel;
    m_colorPanelBottomRight = a_colorPanel;

    // update model of panel
    updatePanelModel();
}


//==============================================================================
/*!
    Set Panel color components for all four corners.

    \param      a_colorPanelTopLeft  Color of top left corner.
    \param      a_colorPanelTopRight  Color of top right corner.
    \param      a_colorPanelBottomLeft  Color of bottom left corner.
    \param      a_colorPanelBottomRight  Color of bottom right corner.
*/
//==============================================================================
void cPanel::setColorPanel(const cColorf& a_colorPanelTopLeft, 
                           const cColorf& a_colorPanelTopRight, 
                           const cColorf& a_colorPanelBottomLeft, 
                           const cColorf& a_colorPanelBottomRight)
{
    // assign new colors
    m_colorPanelTopLeft     = a_colorPanelTopLeft;
    m_colorPanelTopRight    = a_colorPanelTopRight;
    m_colorPanelBottomLeft  = a_colorPanelBottomLeft;
    m_colorPanelBottomRight = a_colorPanelBottomRight;

    // update model of panel
    updatePanelModel();
}

//==============================================================================
/*!
     Assign a transparency level for the scope background.

     \param     a_level  Level of transparency ranging from 0.0 to 1.0.
     \param     a_applyToTextures  If __true__, then apply changes to texture.
     \param     a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cPanel::setTransparencyLevel(const float a_level, 
                                  const bool a_applyToTextures,
                                  const bool a_affectChildren)
{
    // if the transparency level is equal to 1.0, then do not apply transparency
    // otherwise enable it.
    if (a_level < 1.0)
    {
        setUseTransparency(true);
    }
    else
    {
        setUseTransparency(false);
    }

    // apply new value to material
    if (m_material != NULL)
    {
        m_material->setTransparencyLevel(a_level);
    }

    // apply new value to texture
    if (m_texture != NULL)
    {
        if (m_texture->m_image != NULL)
        {
            unsigned char level = (unsigned char)(255.0 * a_level);
            m_texture->m_image->setTransparency(level);   
        }
    }

    // assign transparency level to vertex colors
    m_colorPanelTopLeft.setA(a_level);
    m_colorPanelTopRight.setA(a_level);
    m_colorPanelBottomLeft.setA(a_level);
    m_colorPanelBottomRight.setA(a_level);

    // update panel
    setPanelSize(m_widthPanel, m_heightPanel);

    // apply change to children
    if (a_affectChildren)
    {
	    vector<cGenericObject*>::iterator it;
	    for (it = m_children.begin(); it < m_children.end(); it++)
	    {
            (*it)->setTransparencyLevel(a_level,
                                        a_applyToTextures,
                                        true);
        }
    }
}


//==============================================================================
/*!
    Update mesh of Panel.

    \fn         void cPanel::updatePanelModel()
*/
//==============================================================================
void cPanel::updatePanelModel()
{
    // clear all triangle
    clear();

    // create new model
    cCreatePanel2(this, 
                  m_widthPanel + m_marginLeft + m_marginRight, 
                  m_heightPanel + m_marginTop + m_marginBottom, 
                  m_radiusTopLeft,
                  m_radiusTopRight,
                  m_radiusBottomLeft,
                  m_radiusBottomRight,
                  m_segmentsPerCorner,
                  cVector3d(0.5 * (m_widthPanel + m_marginRight - m_marginLeft), 
                            0.5 * (m_heightPanel + m_marginTop - m_marginBottom), 
                            m_offsetPanelModelZ),
                  cIdentity3d(),
                  m_colorPanelTopLeft,
                  m_colorPanelTopRight,
                  m_colorPanelBottomLeft,
                  m_colorPanelBottomLeft
                  );
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
