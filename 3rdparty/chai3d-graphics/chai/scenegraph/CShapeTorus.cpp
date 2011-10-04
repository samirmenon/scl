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
    \version   2.0.0 $Rev: 264 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "scenegraph/CShapeTorus.h"
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cShapeTorus.

    \fn     cShapeTorus::cShapeTorus(const double& a_insideRadius, 
                                     const double& a_outsideRadius)
    \param  a_insideRadius    Inside radius of torus
    \param  a_outsideRadius   Outside radius of torus
*/
//===========================================================================
cShapeTorus::cShapeTorus(const double& a_insideRadius, const double& a_outsideRadius)
{
    // initialize radius of sphere
    setSize(a_insideRadius, a_outsideRadius);

    // resolution of the graphical model
    m_resolution = 64;

    // set material properties
    m_material.setShininess(100);
    m_material.m_ambient.set((float)0.3, (float)0.3, (float)0.3);
    m_material.m_diffuse.set((float)0.7, (float)0.7, (float)0.7);
    m_material.m_specular.set((float)1.0, (float)1.0, (float)1.0);
    m_material.setStiffness(100.0);
};


//===========================================================================
/*!
    Render sphere in OpenGL

    \fn       void cShapeTorus::render(const int a_renderMode)
    \param    a_renderMode  See cGenericObject::render()
*/
//===========================================================================
void cShapeTorus::render(const int a_renderMode)
{
    //-----------------------------------------------------------------------
    // Conditions for object to be rendered
    //-----------------------------------------------------------------------

    if(((a_renderMode == CHAI_RENDER_MODE_NON_TRANSPARENT_ONLY) &&
        (m_useTransparency == true)) ||
       ((a_renderMode == CHAI_RENDER_MODE_TRANSPARENT_FRONT_ONLY) &&
        (m_useTransparency == false)) ||
       ((a_renderMode == CHAI_RENDER_MODE_TRANSPARENT_BACK_ONLY) &&
        (m_useTransparency == false)))
        {
            return;
        }

    //-----------------------------------------------------------------------
    // Rendering code here
    //-----------------------------------------------------------------------

    // render material properties
    if (m_useMaterialProperty)
    {
        m_material.render();
    }

    // render texture property if defined
    if ((m_texture != NULL) && (m_useTextureMapping))
    {
        m_texture->render();
    }

    // draw sphere
    glutSolidTorus(m_innerRadius, m_outerRadius, m_resolution, m_resolution);

    // turn off texture rendering if it has been used
    glDisable(GL_TEXTURE_2D);
}


//===========================================================================
/*!
    From the position of the tool, search for the nearest point located
    at the surface of the current object. Decide if the point is located inside
    or outside of the object

    \fn     void cShapeTorus::computeLocalInteraction(const cVector3d& a_toolPos,
                                                      const cVector3d& a_toolVel,
                                                      const unsigned int a_IDN)
    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void cShapeTorus::computeLocalInteraction(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN)
{
    cVector3d toolProjection = a_toolPos;
    toolProjection.z = 0;

    // search for the nearest point on the torus medial axis
    if (a_toolPos.lengthsq() > CHAI_SMALL)
    {
        cVector3d pointAxisTorus = cMul(m_outerRadius, cNormalize(toolProjection));

        // compute eventual penetration of tool inside the torus
        cVector3d vectTorusTool = cSub(a_toolPos, pointAxisTorus);

        double distance = vectTorusTool.length();

        // tool is located inside the torus
        if ((distance < m_innerRadius) && (distance > 0.001))
        {
            m_interactionInside = true;
        }

        // tool is located outside the torus
        else
        {
            m_interactionInside = false;
        }

        // compute surface point
        double dist = vectTorusTool.length();
        if (dist > 0)
        {
            vectTorusTool.mul(1/dist);
        }
        vectTorusTool.mul(m_innerRadius);
        pointAxisTorus.addr(vectTorusTool, m_interactionProjectedPoint);
    }
    else
    {
        m_interactionInside = false;
        m_interactionProjectedPoint = a_toolPos;
    }
}


//===========================================================================
/*!
    Update bounding box of current object.

    \fn       void cShapeTorus::updateBoundaryBox()
*/
//===========================================================================
void cShapeTorus::updateBoundaryBox()
{
    m_boundaryBoxMin.set(-m_outerRadius, -m_outerRadius, -(m_outerRadius - m_innerRadius));
    m_boundaryBoxMax.set( m_outerRadius,  m_outerRadius,  (m_outerRadius - m_innerRadius));
}


//===========================================================================
/*!
    Scale the torus with a uniform scale factor

    \fn       void cShapeTorus::scaleObject(const cVector3d& a_scaleFactors)
    \param    a_scaleFactors x,y,z scale factors
*/
//===========================================================================
void cShapeTorus::scaleObject(const cVector3d& a_scaleFactors)
{
    m_outerRadius = a_scaleFactors.x * m_outerRadius;
    m_innerRadius = a_scaleFactors.x * m_innerRadius;
}
