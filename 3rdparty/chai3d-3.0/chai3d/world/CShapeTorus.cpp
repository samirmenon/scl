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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1067 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CShapeTorus.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cShapeTorus.

    \param  a_innerRadius  Inside radius of torus.
    \param  a_outerRadius  Outside radius of torus.
    \param  a_material  Material property to be applied to object.
*/
//==============================================================================
cShapeTorus::cShapeTorus(const double& a_innerRadius, 
                         const double& a_outerRadius, 
                         cMaterial* a_material)
{
    // enable display list
    m_useDisplayList = true;

    // initialize dimensions of torus
    setSize(a_innerRadius, a_outerRadius);

    // set resolution of graphical model
    m_resolution = 64;

    // set material properties
    if (a_material == NULL)
    {
        m_material = new cMaterial();
        m_material->setShininess(100);
        m_material->m_ambient.set((float)0.3, (float)0.3, (float)0.3);
        m_material->m_diffuse.set((float)0.7, (float)0.7, (float)0.7);
        m_material->m_specular.set((float)1.0, (float)1.0, (float)1.0);
        m_material->setStiffness(100.0);
    }
    else
    {
        m_material = a_material;
    }
};


//==============================================================================
/*!
    Create a copy of itself.

    \param  a_duplicateMaterialData  If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData  If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData  If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.

    \return Return new object.
*/
//==============================================================================
cShapeTorus* cShapeTorus::copy(const bool a_duplicateMaterialData,
                               const bool a_duplicateTextureData, 
                               const bool a_duplicateMeshData,
                               const bool a_buildCollisionDetector)
{
    // create new instance
    cShapeTorus* obj = new cShapeTorus(m_innerRadius, m_outerRadius); 

    // copy generic properties
    copyGenericProperties(obj, a_duplicateMaterialData, a_duplicateTextureData);

    // return
    return (obj);
}


//==============================================================================
/*!
    Set dimensions of torus.

    \param  a_innerRadius  Inside radius of torus.
    \param  a_outerRadius  Outside radius of torus.
*/
//==============================================================================
 void cShapeTorus::setSize(const double& a_innerRadius, 
                           const double& a_outerRadius) 
 {
     // set new dimensions
     m_innerRadius = cAbs(a_innerRadius); 
     m_outerRadius = cAbs(a_outerRadius); 
     
     // update bounding box
     updateBoundaryBox(); 

     // update display list
     invalidateDisplayList(false);
 }



//==============================================================================
/*!
    Render torus in OpenGL

    \param  a_options  Rendering options.
*/
//==============================================================================
void cShapeTorus::render(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

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

        // render texture property if defined
        bool usedTexture = false;
        if ((m_texture != NULL) && (m_useTextureMapping))
        {
            // the torus can only be rendered with texture if environmental mapping
            // is used. This limitation comes from the fact that the current implementation of
            // cDrawSolidTorus() does not generate any texture coordinates. 
            // If you wish to create a textured torus, you may build it using a cMesh class and 
            // the mesh primitives availble in file CPrimitives.h
            if (m_texture->getSphericalMappingEnabled())
            {
                // we are using texture
                usedTexture = true;

                // activate texture
                m_texture->render(a_options);
            }
        }

        // draw torus
        if (!m_displayList.render(m_useDisplayList))
        {
            // create display list if requested
            m_displayList.begin(m_useDisplayList);

            // draw object
            cDrawSolidTorus(m_innerRadius, m_outerRadius, m_resolution, m_resolution);
            
            // finalize display list
            m_displayList.end(true);
        }
  
        // turn off texture rendering if it has been used
        if (usedTexture)
        {
            glActiveTextureARB(GL_TEXTURE1_ARB);
            glDisable(GL_TEXTURE_1D);
            glDisable(GL_TEXTURE_2D);
        }
    }

#endif
}


//==============================================================================
/*!
    From the position of the tool, search for the nearest point located
    at the surface of the current object. Decide if the point is located inside
    or outside of the object

    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//==============================================================================
void cShapeTorus::computeLocalInteraction(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN)
{
    cVector3d toolProjection = a_toolPos;
    toolProjection.z(0.0);
    m_interactionNormal.set(0,0,1);

    // search for the nearest point on the torus medial axis
    if (a_toolPos.lengthsq() > C_SMALL)
    {
        cVector3d pointAxisTorus = cMul(m_outerRadius, cNormalize(toolProjection));

        // compute eventual penetration of tool inside the torus
        cVector3d vectTorusTool = cSub(a_toolPos, pointAxisTorus);

        double distance = vectTorusTool.length();

        // normal
        if (distance > 0.0)
        {
            m_interactionNormal = vectTorusTool;
            m_interactionNormal.normalize();
        }

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
        pointAxisTorus.addr(vectTorusTool, m_interactionPoint);
    }
    else
    {
        m_interactionInside = false;
        m_interactionPoint = a_toolPos;
    }
}


//==============================================================================
/*!
    Update bounding box of current object.
*/
//==============================================================================
void cShapeTorus::updateBoundaryBox()
{
    m_boundaryBoxMin.set(-m_outerRadius, -m_outerRadius, -(m_outerRadius - m_innerRadius));
    m_boundaryBoxMax.set( m_outerRadius,  m_outerRadius,  (m_outerRadius - m_innerRadius));
}


//==============================================================================
/*!
    Scale torus with a uniform scale factor.

    \param  a_scaleFactor  Scale factor.
*/
//==============================================================================
void cShapeTorus::scaleObject(const double& a_scaleFactor)
{
    // update dimensions
    m_outerRadius *= a_scaleFactor;
    m_innerRadius *= a_scaleFactor;

    // update bounding box
    updateBoundaryBox();

    // invalidate display list
    invalidateDisplayList();
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
