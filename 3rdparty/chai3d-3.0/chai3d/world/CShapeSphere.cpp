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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 846 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "world/CShapeSphere.h"
//---------------------------------------------------------------------------
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif
//---------------------------------------------------------------------------
typedef GLUquadric GLUquadricObj;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cShapeSphere.

    \fn     cShapeSphere::cShapeSphere(const double& a_radius, 
                                       cMaterial* a_material = NULL)
    \param  a_radius    Radius of sphere
    \param  a_material  Material property to be applied to object.
*/
//===========================================================================
cShapeSphere::cShapeSphere(const double& a_radius, 
                           cMaterial* a_material)
{
    // initialize radius of sphere
    m_radius = cAbs(a_radius);

    // set material properties
    if (a_material == NULL)
    {
        m_material = new cMaterial();
        m_material->setShininess(100);
        m_material->setWhite();
    }
    else
    {
        m_material = a_material;
    }
};


//===========================================================================
/*!
    Create a copy of itself.

    \fn         cShapeSphere* cShapeSphere::copy(const bool a_duplicateMaterialData,
                                 const bool a_duplicateTextureData, 
                                 const bool a_duplicateMeshData,
                                 const bool a_buildCollisionDetector)

    \param      a_duplicateMaterialData  If \b true, material (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateTextureData  If \b true, texture data (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateMeshData  If \b true, mesh data (if available) is duplicated, otherwise it is shared.
    \param      a_buildCollisionDetector  If \b true, collision detector (if available) is duplicated, otherwise it is shared.

	\return		Return new object.
*/
//===========================================================================
cShapeSphere* cShapeSphere::copy(const bool a_duplicateMaterialData,
                                 const bool a_duplicateTextureData, 
                                 const bool a_duplicateMeshData,
                                 const bool a_buildCollisionDetector)
{
    // enable display list
    m_useDisplayList = true;

    // create new instance
    cShapeSphere* obj = new cShapeSphere(m_radius);

    // copy generic properties
    copyGenericProperties(obj, a_duplicateMaterialData, a_duplicateTextureData);

    // return
    return (obj);
}


//===========================================================================
/*!
    Set radius of sphere.

    \fn          void cShapeSphere::setRadius(const double& a_radius) 

    \param      a_radius  Radius of sphere.
*/
//===========================================================================
void cShapeSphere::setRadius(const double& a_radius) 
{ 
    // set new radius
    m_radius = cAbs(a_radius);

    // update bounding box
    updateBoundaryBox();

    // invalidate display list
    invalidateDisplayList();
}


//===========================================================================
/*!
    Render sphere in OpenGL

    \fn       void cShapeSphere::render(cRenderOptions& a_options)
    \param    a_options  Render options
*/
//===========================================================================
void cShapeSphere::render(cRenderOptions& a_options)
{
 	/////////////////////////////////////////////////////////////////////////
	// Render parts that use material properties
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_PARTS_WITH_MATERIALS(a_options, m_useTransparency))
	{
		// render material properties
		if (m_useMaterialProperty)
		{
			m_material->render(a_options);
		}
    
		// allocate a new OpenGL quadric object for rendering a sphere
		GLUquadricObj *sphere;
		sphere = gluNewQuadric ();

		// set rendering style
		gluQuadricDrawStyle (sphere, GLU_FILL);

		// set normal-rendering mode
		gluQuadricNormals (sphere, GLU_SMOOTH);

		// render texture property if defined
        bool usedTexture = false;
		if ((m_texture != NULL) && (m_useTextureMapping))
		{
            // we are using texture
            usedTexture = true;

            // activate texture
			m_texture->render(a_options);

			// generate texture coordinates
			gluQuadricTexture(sphere, GL_TRUE);
		}

		// render a sphere
		gluSphere(sphere, m_radius, 36, 36);

		// delete our quadric object
		gluDeleteQuadric(sphere);

		// turn off texture rendering if it has been used
		if (usedTexture)
		{
			glActiveTextureARB(GL_TEXTURE1_ARB);
			glDisable(GL_TEXTURE_1D);
            glDisable(GL_TEXTURE_2D);
		}
	}
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
    Scale sphere with a uniform scale factor.

    \fn       void cShapeSphere::scaleObject(const double& a_scaleFactor)
    \param    a_scaleFactor  Scale factor.
*/
//===========================================================================
void cShapeSphere::scaleObject(const double& a_scaleFactor)
{
    // update radius
    m_radius *= a_scaleFactor;

    // update bounding box
    updateBoundaryBox();

    // invalidate display list
    invalidateDisplayList();
}
