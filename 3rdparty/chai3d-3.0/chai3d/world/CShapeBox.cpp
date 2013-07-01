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
    POSSIBILITY OF SUCH DAMAGE. .

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1067 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CShapeBox.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cShapeBox.

    \param  a_sizeX    size of box along axis X.
    \param  a_sizeY    size of box along axis Y.
    \param  a_sizeZ    size of box along axis Z.
    \param  a_material  Material property to be applied to object.
*/
//==============================================================================
cShapeBox::cShapeBox(const double& a_sizeX, 
                     const double& a_sizeY, 
                     const double& a_sizeZ,
                     cMaterial* a_material)
{
    // enable display list
    m_useDisplayList = true;

    // initialize dimension
    setSize(a_sizeX, a_sizeY, a_sizeZ);

    // set material properties
    if (a_material == NULL)
    {
        m_material = new cMaterial();
        m_material->setShininess(100);
        m_material->m_ambient.set ((float)0.3, (float)0.3, (float)0.3);
        m_material->m_diffuse.set ((float)0.1, (float)0.7, (float)0.8);
        m_material->m_specular.set((float)1.0, (float)1.0, (float)1.0);
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
cShapeBox* cShapeBox::copy(const bool a_duplicateMaterialData,
                           const bool a_duplicateTextureData, 
                           const bool a_duplicateMeshData,
                           const bool a_buildCollisionDetector)
{
    // create new instance
    cShapeBox* obj = new cShapeBox(2.0 * m_hSizeX, 
                                   2.0 * m_hSizeY, 
                                   2.0 * m_hSizeZ);

    // copy generic properties
    copyGenericProperties(obj, a_duplicateMaterialData, a_duplicateTextureData);

    // return
    return (obj);
}


//==============================================================================
/*!
    Set the dimensions of the box along each axis.

    \param  a_sizeX  Size of box along axis X.
    \param  a_sizeY  Size of box along axis Y.
    \param  a_sizeZ  Size of box along axis Z.
*/
//==============================================================================
void cShapeBox::setSize(const double& a_sizeX, const double& a_sizeY, const double& a_sizeZ)
{
    // set dimensions
    m_hSizeX = 0.5 * cAbs(a_sizeX);
    m_hSizeY = 0.5 * cAbs(a_sizeY);
    m_hSizeZ = 0.5 * cAbs(a_sizeZ);

    // update bounding box
    updateBoundaryBox();

    // invalidate display list
    invalidateDisplayList(false);
}


//==============================================================================
/*!
    Render box in OpenGL

    \param  a_options  Render options.
*/
//==============================================================================
void cShapeBox::render(cRenderOptions& a_options)
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


        if (!m_displayList.render(m_useDisplayList))
        {
            // create display list if requested
            m_displayList.begin(m_useDisplayList);

            // render box
            glBegin(GL_POLYGON);
                glNormal3d(1.0, 0.0, 0.0);
                glVertex3d(m_hSizeX, m_hSizeY, m_hSizeZ);
                glVertex3d(m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d(m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d(m_hSizeX, m_hSizeY,-m_hSizeZ);
            glEnd();
        
            glBegin(GL_POLYGON);
                glNormal3d(-1.0, 0.0, 0.0);
                glVertex3d(-m_hSizeX, m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY, m_hSizeZ);
            glEnd();

            glBegin(GL_POLYGON);
                glNormal3d(0.0, 1.0, 0.0);
                glVertex3d( m_hSizeX, m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY, m_hSizeZ);
                glVertex3d( m_hSizeX, m_hSizeY, m_hSizeZ);
            glEnd();
        
            glBegin(GL_POLYGON);
                glNormal3d(0.0,-1.0, 0.0);
                glVertex3d( m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d( m_hSizeX,-m_hSizeY,-m_hSizeZ);
            glEnd();

            glBegin(GL_POLYGON);
                glNormal3d(0.0, 0.0, 1.0);
                glVertex3d( m_hSizeX, m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY, m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY, m_hSizeZ);
                glVertex3d( m_hSizeX,-m_hSizeY, m_hSizeZ);
            glEnd();
            glBegin(GL_POLYGON);
                glNormal3d(0.0, 0.0,-1.0);
                glVertex3d( m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX,-m_hSizeY,-m_hSizeZ);
                glVertex3d(-m_hSizeX, m_hSizeY,-m_hSizeZ);
                glVertex3d( m_hSizeX, m_hSizeY,-m_hSizeZ);
            glEnd();

            // finalize display list
            m_displayList.end(true);
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
void cShapeBox::computeLocalInteraction(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN)
{
    // temp variables
    bool inside;
    cVector3d projectedPoint;
    cVector3d normal(1,0,0);
    
    // sign
    double signX = cSign(a_toolPos(0) );
    double signY = cSign(a_toolPos(1) );
    double signZ = cSign(a_toolPos(2) );

    // check if tool is located inside box
    double tx = (signX * a_toolPos(0) );
    double ty = (signY * a_toolPos(1) );
    double tz = (signZ * a_toolPos(2) );
    
    inside = false;
    if (cContains(tx, 0.0, m_hSizeX))
    {
        if (cContains(ty, 0.0, m_hSizeY))
        {
            if (cContains(tz, 0.0, m_hSizeZ))
            {
                inside = true;
            }
        }
    }

    if (inside)
    {
        // tool is located inside box, compute distance from tool to surface
        double m_distanceX = m_hSizeX - (signX * a_toolPos(0) );
        double m_distanceY = m_hSizeY - (signY * a_toolPos(1) );
        double m_distanceZ = m_hSizeZ - (signZ * a_toolPos(2) );

        // search nearest surface
        if (m_distanceX < m_distanceY)
        {
            if (m_distanceX < m_distanceZ)
            {
                projectedPoint(0)  = signX * m_hSizeX;
                projectedPoint(1)  = a_toolPos(1) ;
                projectedPoint(2)  = a_toolPos(2) ;
                normal.set(signX * 1.0, 0.0, 0.0);
            }
            else
            {
                projectedPoint(0)  = a_toolPos(0) ;
                projectedPoint(1)  = a_toolPos(1) ;
                projectedPoint(2)  = signZ * m_hSizeZ;
                normal.set(0.0, 0.0, signZ * 1.0);
            }
        }
        else
        {
            if (m_distanceY < m_distanceZ)
            {
                projectedPoint(0)  = a_toolPos(0) ;
                projectedPoint(1)  = signY * m_hSizeY;
                projectedPoint(2)  = a_toolPos(2) ;
                normal.set(0.0, signY * 1.0, 0.0);
            }
            else
            {
                projectedPoint(0)  = a_toolPos(0) ;
                projectedPoint(1)  = a_toolPos(1) ;
                projectedPoint(2)  = signZ * m_hSizeZ;
                normal.set(0.0, 0.0, signZ * 1.0);
            }
        }
    }
    else
    {
        projectedPoint(0)  = cClamp(a_toolPos(0) , -m_hSizeX, m_hSizeX);
        projectedPoint(1)  = cClamp(a_toolPos(1) , -m_hSizeY, m_hSizeY);
        projectedPoint(2)  = cClamp(a_toolPos(2) , -m_hSizeZ, m_hSizeZ);
    }

    // return results
    cVector3d n = a_toolPos - projectedPoint;
    if (n.lengthsq() > 0.0)
    {
        m_interactionNormal = n;
        m_interactionNormal.normalize();
    }

    m_interactionInside = inside;
}


//==============================================================================
/*!
    Update bounding box of current object.
*/
//==============================================================================
void cShapeBox::updateBoundaryBox()
{
    // compute half size lengths
    m_boundaryBoxMin.set(-m_hSizeX,-m_hSizeY,-m_hSizeZ);
    m_boundaryBoxMax.set( m_hSizeX, m_hSizeY, m_hSizeZ);
}


//==============================================================================
/*!
    Scale box with a uniform scale factor.

    \param  a_scaleFactor  Scale factor.
*/
//==============================================================================
void cShapeBox::scaleObject(const double& a_scaleFactor)
{
    // update dimensions
    m_hSizeX *= a_scaleFactor;
    m_hSizeY *= a_scaleFactor;
    m_hSizeZ *= a_scaleFactor;

    // update bounding box
    updateBoundaryBox();

    // invalidate display list
    invalidateDisplayList();
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
