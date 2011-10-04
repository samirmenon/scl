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
#include "scenegraph/CShapeSphere.h"
//---------------------------------------------------------------------------
typedef GLUquadric GLUquadricObj;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cShapeSphere.

    \fn     cShapeSphere::cShapeSphere(const double& a_radius)
    \param  a_radius    Radius of sphere
*/
//===========================================================================
cShapeSphere::cShapeSphere(const double& a_radius)
{
    // initialize radius of sphere
    m_radius = cAbs(a_radius);
    m_texture = NULL;

    // set material properties
    m_material.setShininess(100);
    m_material.m_ambient.set((float)0.3, (float)0.3, (float)0.3);
    m_material.m_diffuse.set((float)0.1, (float)0.7, (float)0.8);
    m_material.m_specular.set((float)1.0, (float)1.0, (float)1.0);
};


//===========================================================================
/*!
    Render sphere in OpenGL

    \fn       void cShapeSphere::render(const int a_renderMode)
    \param    a_renderMode  See cGenericObject::render()
*/
//===========================================================================
void cShapeSphere::render(const int a_renderMode)
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
      printf(" NoSphere. ");
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

    // allocate a new OpenGL quadric object for rendering a sphere
    GLUquadricObj *sphere;
    sphere = gluNewQuadric ();

    // set rendering style
    gluQuadricDrawStyle (sphere, GLU_FILL);

    // set normal-rendering mode
    gluQuadricNormals (sphere, GLU_SMOOTH);

    // render texture property if defined
    if ((m_texture != NULL) && (m_useTextureMapping))
    {
        m_texture->render();

        // generate texture coordinates
        gluQuadricTexture(sphere, GL_TRUE);
    }

    // render a sphere
    gluSphere(sphere, m_radius, 36, 36);

    // delete our quadric object
    gluDeleteQuadric(sphere);

    // turn off texture rendering if it has been used
    glDisable(GL_TEXTURE_2D);
}


//===========================================================================
/*!
    From the position of the tool, search for the nearest point located
    at the surface of the current object. Decide if the point is located inside
    or outside of the object

    \fn     void cShapeSphere::computeLocalInteraction(const cVector3d& a_toolPos,
                                                      const cVector3d& a_toolVel,
                                                      const unsigned int a_IDN)
    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void cShapeSphere::computeLocalInteraction(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN)
{
    // compute distance from center of sphere to tool
    double distance = a_toolPos.length();

    // from the position of the tool, search for the nearest point located
    // on the surface of the sphere
    if (distance > 0)
    {
        m_interactionProjectedPoint = cMul( (m_radius/distance), a_toolPos);
    }
    else
    {
        m_interactionProjectedPoint = a_toolPos;
    }

    // check if tool is located inside or outside of the sphere
    if (distance <= m_radius)
    {
        m_interactionInside = true;
    }
    else
    {
        m_interactionInside = false;
    }
}


//===========================================================================
/*!
    Update bounding box of current object.

    \fn       void cShapeSphere::updateBoundaryBox()
*/
//===========================================================================
void cShapeSphere::updateBoundaryBox()
{
    m_boundaryBoxMin.set(-m_radius, -m_radius, -m_radius);
    m_boundaryBoxMax.set( m_radius,  m_radius,  m_radius);
}


//===========================================================================
/*!
    Scale object of defined scale factor

    \fn       void cShapeSphere::scaleObject(const cVector3d& a_scaleFactors)
    \param    a_scaleFactors Scale factor
*/
//===========================================================================
void cShapeSphere::scaleObject(const cVector3d& a_scaleFactors)
{
    m_radius = a_scaleFactors.x * m_radius;
}
