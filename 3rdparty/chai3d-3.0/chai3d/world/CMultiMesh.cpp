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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 676 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CMultiMesh.h"
//------------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "collisions/CCollisionBrute.h"
#include "collisions/CCollisionAABB.h"
#include "files/CFileModel3DS.h"
#include "files/CFileModelOBJ.h"
#include "math/CMaths.h"
//------------------------------------------------------------------------------
#include <float.h>
#include <algorithm>
#include <set>
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cMultiMesh.
*/
//==============================================================================
cMultiMesh::cMultiMesh()
{
    // create array of mesh primitives
    m_meshes = new vector<cMesh*>;

    // initialize parent object of mesh. Not yet a child on an other object.
    m_parent = NULL;

    // should the frame (X-Y-Z) be displayed?
    m_showFrame = false;
}


//==============================================================================
/*!
    Destructor of cMultiMesh.
*/
//==============================================================================
cMultiMesh::~cMultiMesh()
{
    // clear vertex array
    m_meshes->clear();
}


//==============================================================================
/*!
    Create a copy of itself.

    \param      a_duplicateMaterialData  If __true__, material (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateTextureData  If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param      a_duplicateMeshData  If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param      a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.

    \return		Return new object.
*/
//==============================================================================
cMultiMesh* cMultiMesh::copy(const bool a_duplicateMaterialData,
                             const bool a_duplicateTextureData, 
                             const bool a_duplicateMeshData,
                             const bool a_buildCollisionDetector)
{
    // create multimesh object
    cMultiMesh* obj = new cMultiMesh();

    // copy generic properties
    copyGenericProperties(obj, a_duplicateMaterialData, a_duplicateTextureData);

    // create sub meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMesh* mesh = (*it)->copy(a_duplicateMaterialData,
                                  a_duplicateTextureData, 
                                  a_duplicateMeshData,
                                  a_buildCollisionDetector);
        obj->addMesh(mesh);
    }

    // return reult
    return (obj);
}


//==============================================================================
/*!
    Enable or Disable this object. When an object is disabled, haptic
    and graphic rendering are no longer performed through the scenegraph and the 
    object is simply ignored. Other operations however will still be active. 
    Enabling or disabling an object will not affect child objects, 
    unless explicitly specified.

    \param  a_enabled  Status.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setEnabled(bool a_enabled,
                            const bool a_affectChildren)
{
    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setEnabled(a_enabled, true);
    }

    // update current object and possibly children
    cGenericObject::setEnabled(a_enabled, a_affectChildren);
}


//==============================================================================
/*!
    Allow or disallow the object to be felt haptically. \n

    If \e a_affectChildren is set to __true__ then all children are updated
    with the new value.

    \param  a_hapticEnabled  If __true__ object can be felt when visible.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setHapticEnabled(const bool a_hapticEnabled, 
                                  const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setHapticEnabled(a_hapticEnabled, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setHapticEnabled(a_hapticEnabled, true);
    }
}


//==============================================================================
/*!
    Set the haptic stiffness for this object, possibly recursively 
    affecting children.

    \param  a_stiffness  The stiffness to apply to this object.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setStiffness(const double a_stiffness, 
                              const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setStiffness(a_stiffness, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setStiffness(a_stiffness, true);
    }
}


//==============================================================================
/*!
    Set the static and dynamic friction properties for this mesh, 
    possibly recursively affecting children.

    \param  a_staticFriction  The static friction to apply to this object.
    \param  a_dynamicFriction  The dynamic friction to apply to this object.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setFriction(double a_staticFriction, 
                             double a_dynamicFriction, 
                             const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setFriction(a_staticFriction, a_dynamicFriction, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setFriction(a_staticFriction, a_dynamicFriction, true);
    }
}


//==============================================================================
/*!
    Graphically show or hide this object, possibly recursively affecting children.

    \param  a_show  If __true__ object shape is visible.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setShowEnabled(const bool a_show, 
                                const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setShowEnabled(a_show, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowEnabled(a_show, true);
    }
}


//==============================================================================
/*!
    Specify whether transparency should be enabled. If you use transparency, 
    make sure that you enable multi-pass rendering. For more information, see
    class cCamera.

    \param  a_useTransparency  If __true__, transparency is enabled.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseTransparency(const bool a_useTransparency,
                                    const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseTransparency(a_useTransparency, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseTransparency(a_useTransparency, true);
    }
}


//==============================================================================
/*!
    Set the alpha value at each vertex, in all of my material colors,
    optionally propagating the operation to children. \n

    Using the 'apply to textures' option causes the actual texture
    alpha values to be over-written in my texture, if it exists. \n

    \param  a_level  Level of transparency ranging from 0.0 to 1.0.
    \param  a_applyToTextures  If __true__, then apply changes to texture.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setTransparencyLevel(const float a_level, 
                                      const bool a_applyToTextures,
                                      const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setTransparencyLevel(a_level,
                                         a_applyToTextures,
                                         a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setTransparencyLevel(a_level,
                                    a_applyToTextures,
                                    true);
    }
}


//==============================================================================
/*!
    Enable or disable wireframe rendering, optionally propagating the
    operation to children.

    \param  a_showWireMode  If __true__, wireframe mode is used.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setWireMode(const bool a_showWireMode, 
                             const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setWireMode(a_showWireMode, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setWireMode(a_showWireMode, true);
    }
}


//==============================================================================
/*!
    Enable or disable backface culling (rendering in GL is much faster
    with culling ON).

    \param  a_useCulling  If __true__, backfaces are culled.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseCulling(const bool a_useCulling, 
                               const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseCulling(a_useCulling, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseCulling(a_useCulling, true);
    }
}


//==============================================================================
/*!
    Enable or disable the use of per-vertex color information of when rendering
    the mesh.

    \param  a_useColors  If __true__, then vertex color information is applied.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseVertexColors(const bool a_useColors, 
                                    const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseVertexColors(a_useColors, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseVertexColors(a_useColors, true);
    }
}


//==============================================================================
/*!
    Backup color properties of this object, possibly recursively 
    affecting children.

    \param  a_material_properties_only  If \e true, then backup material properties only.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::backupColors(const bool a_material_properties_only, 
                              const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::backupColors(a_material_properties_only, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->backupColors(a_material_properties_only, true);
    }
}

 
//==============================================================================
/*!
    Restore color properties of this object, possibly recursively 
    affecting children.

    \param  a_material_properties_only  If \e true, then restore material properties only.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::restoreColors(const bool a_material_properties_only, 
                               const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::restoreColors(a_material_properties_only, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->restoreColors(a_material_properties_only, true);
    }
}


//==============================================================================
/*!
    This enables the use of display lists for mesh rendering.  This should
    significantly speed up rendering for large meshes, but it means that
    any changes you make to any cMesh options or any vertex positions
    will not take effect until you invalidate the existing display list
    (by calling updateDisplayList()).

    In general, if you aren't having problems with rendering performance,
    don't bother with this; you don't want to worry about having to
    invalidate display lists every time you change a tiny option.

    \param  a_useDisplayList  If __true__, this mesh will be rendered with a display list.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseDisplayList(const bool a_useDisplayList,
                                   const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseDisplayList(a_useDisplayList, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseDisplayList(a_useDisplayList, a_affectChildren);
    }
}


//==============================================================================
/*!
    Invalidate any existing display lists.  You should call this on if you're using
    display lists and you modify mesh options, vertex positions, etc.

    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::invalidateDisplayList(const bool a_affectChildren)
{
     // update current object and possibly children
    cGenericObject::invalidateDisplayList(a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->invalidateDisplayList(a_affectChildren);
    }
}


//==============================================================================
/*!
    Enable or disable the use of material properties.

    \param  a_useMaterial  If __true__, then material properties are used for rendering.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseMaterial(const bool a_useMaterial, 
                                const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseMaterial(a_useMaterial, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseMaterial(a_useMaterial, true);
    }
}


//==============================================================================
/*!
    Copy material properties defined in a_mat to the material structure
    of this object.\n

    Note that this does not affect whether material rendering is enabled;
    it sets the material that will be rendered _if_ material rendering is
    enabled.  Call useMaterial to enable / disable material rendering.

    \param  a_mat The material to apply to this object
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setMaterial(cMaterial* a_mat,
                             const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setMaterial(a_mat, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setMaterial(a_mat, true);
    }
}


//==============================================================================
/*!
    Copy material properties defined in a_mat to the material structure
    of this object.\n

    Note that this does not affect whether material rendering is enabled;
    it sets the material that will be rendered _if_ material rendering is
    enabled.  Call useMaterial to enable / disable material rendering.

    \param  a_mat  The material to apply to this object
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setMaterial(cMaterial& a_mat,
                             const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setMaterial(a_mat, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setMaterial(a_mat, true);
    }
}


//==============================================================================
/*!
    Enable or disable texture-mapping, possibly recursively affecting 
    children.

    \param  a_useTexture  If __true__, then texture mapping is used.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setUseTexture(const bool a_useTexture, 
                               const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setUseTexture(a_useTexture, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseTexture(a_useTexture, true);
    }
}


//==============================================================================
/*!
    Set the current texture for this mesh, possibly recursively affecting 
    children. \n

    Note that this does not affect whether texturing is enabled; it sets
    the texture that will be rendered _if_ texturing is enabled.  Call
    useTexture to enable / disable texturing.

    \param     a_texture  The texture to apply to this object.
    \param     a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setTexture(cTexture1d* a_texture,
                            const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setTexture(a_texture, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setTexture(a_texture, true);
    }
}


//==============================================================================
/*!
    Show or hide the graphic representation of the boundary box of 
    this object. \n

    If \e a_affectChildren is set to __true__ then all children are updated
    with the new value.

    \param  a_showBoundaryBox  If __true__, boundary box is displayed.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setShowBoundaryBox(const bool a_showBoundaryBox, 
                                    const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setShowBoundaryBox(a_showBoundaryBox, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowBoundaryBox(a_showBoundaryBox, true);
    }
}



//==============================================================================
/*!
    Update the boundary box of this object.
*/
//==============================================================================
void cMultiMesh::updateBoundaryBox()
{
    // initialization
    bool empty = true;
    cVector3d minBox, maxBox;
    minBox.set( C_LARGE, C_LARGE, C_LARGE);
    maxBox.set(-C_LARGE,-C_LARGE,-C_LARGE);


    // compute the bounding box of all my children
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cGenericObject* object = (*it);

        // compute boundary box of child
        object->computeBoundaryBox(true);

        // if boundary box is not empty, then include update
        if (!object->getBoundaryBoxEmpty())
        {
            // since child is not empty, parent is no longer empty too.
            empty = false;

            // retrieve position and orientation of child.
            cVector3d pos = object->getLocalPos();
            cMatrix3d rot = object->getLocalRot();

            // compute position of corners of child within parent reference frame
            double xMin = object->getBoundaryMin().x();
            double yMin = object->getBoundaryMin().y();
            double zMin = object->getBoundaryMin().z();
            double xMax = object->getBoundaryMax().x();
            double yMax = object->getBoundaryMax().y();
            double zMax = object->getBoundaryMax().z();

            cVector3d corners[8];
            corners[0] = pos + rot * cVector3d(xMin, yMin, zMin);
            corners[1] = pos + rot * cVector3d(xMin, yMin, zMax);
            corners[2] = pos + rot * cVector3d(xMin, yMax, zMin);
            corners[3] = pos + rot * cVector3d(xMin, yMax, zMax);
            corners[4] = pos + rot * cVector3d(xMax, yMin, zMin);
            corners[5] = pos + rot * cVector3d(xMax, yMin, zMax);
            corners[6] = pos + rot * cVector3d(xMax, yMax, zMin);
            corners[7] = pos + rot * cVector3d(xMax, yMax, zMax);

            // update boundary box by taking into account child boundary box
            for (int i=0; i<8; i++)
            {
                minBox(0) = cMin(corners[i](0),  minBox(0));
                minBox(1) = cMin(corners[i](1),  minBox(1));
                minBox(2) = cMin(corners[i](2),  minBox(2));
                maxBox(0) = cMax(corners[i](0),  maxBox(0));
                maxBox(1) = cMax(corners[i](1),  maxBox(1));
                maxBox(2) = cMax(corners[i](2),  maxBox(2));
            }
        }
    }

    if (empty)
    {
        m_boundaryBoxMin.zero();
        m_boundaryBoxMax.zero();
        m_boundaryBoxEmpty = true;
    }
    else
    {
        m_boundaryBoxMin = minBox;
        m_boundaryBoxMax = maxBox;
        m_boundaryBoxEmpty = false;
    }
}


//==============================================================================
/*!
    Delete any existing collision detector and set the current cd to null.
    It's fine for an object to have a null collision detector (that's the
    default for a new object, in fact), it just means that no collisions 
    will be found.

    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::deleteCollisionDetector(const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::deleteCollisionDetector(a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->deleteCollisionDetector(true);
    }
}


//==============================================================================
/*!
    Determine whether the given segment intersects a triangle in this object
    (or any of its descendants).  The segment is described by a start point
    \e a_segmentPointA and end point \e a_segmentPointB. Collision detection
    functions of all children of this object are called, which recursively
    call the collision detection functions for all of this object's descendants.
    If there is more than one collision, the one closest to a_segmentPointA is
    the one returned.

    \param  a_segmentPointA  Start point of segment.  Value may be changed if
                             returned collision is with a moving object.
    \param  a_segmentPointB  End point of segment.
    \param  a_recorder  Stores all collision events.
    \param  a_settings  Contains collision settings information.
*/
//==============================================================================
bool cMultiMesh::computeCollisionDetection(cVector3d& a_segmentPointA,
                                           cVector3d& a_segmentPointB,
                                           cCollisionRecorder& a_recorder,
                                           cCollisionSettings& a_settings)
{
    // check if node is a ghost. If yes, then ignore call
    if (m_ghostEnabled) { return (false); }

    // temp variable
    bool hit = false;

    // get the transpose of the local rotation matrix
    cMatrix3d transLocalRot;
    m_localRot.transr(transLocalRot);

    // convert first endpoint of the segment into local coordinate frame
    cVector3d localSegmentPointA = a_segmentPointA;
    localSegmentPointA.sub(m_localPos);
    transLocalRot.mul(localSegmentPointA);

    // convert second endpoint of the segment into local coordinate frame
    cVector3d localSegmentPointB = a_segmentPointB;
    localSegmentPointB.sub(m_localPos);
    transLocalRot.mul(localSegmentPointB);

    // check for a collision with this object if:
    // (1) it has a collision detector
    // (2) if other settings (visible and haptic enabled) are activated
    if ((m_collisionDetector != NULL) && (m_enabled) &&
        ((a_settings.m_checkVisibleObjects && m_showEnabled) ||
         (a_settings.m_checkHapticObjects && m_hapticEnabled)))
    {
        // adjust the first segment endpoint so that it is in the same position
        // relative to the moving object as it was at the previous haptic iteration
        cVector3d localSegmentPointAadjusted;
        if (a_settings.m_adjustObjectMotion)
        {
            adjustCollisionSegment(localSegmentPointA, localSegmentPointAadjusted);
        }
        else
        {
            localSegmentPointAadjusted = localSegmentPointA;
        }

        // call the collision detector's collision detection function
        if (m_collisionDetector->computeCollision(this,
                                                  localSegmentPointAadjusted,
                                                  localSegmentPointB,
                                                  a_recorder,
                                                  a_settings))
        {
            // record that there has been a collision
            hit = true;
        }
    }

    // compute any other collisions. This is a virtual function that can be extended for
    // classes that may contain other objects (sibbling) for wich collision detection may
    // need to be computed.
    hit = hit || computeOtherCollisionDetection(localSegmentPointA,
                                                localSegmentPointB,
                                                a_recorder,
                                                a_settings);

    // check for collisions with all meshes of this object
    for (unsigned int i=0; i<m_meshes->size(); i++)
    {
        // call this child's collision detection function to see if it (or any
        // of its descendants) are intersected by the segment
        bool hitMesh = m_meshes->at(i)->computeCollisionDetection(localSegmentPointA,
                                                                  localSegmentPointB,
                                                                  a_recorder,
                                                                  a_settings);

        // update if a hit ocured
        hit = hit | hitMesh;
    }


    // check for collisions with all children of this object
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        // call this child's collision detection function to see if it (or any
        // of its descendants) are intersected by the segment
        bool hitChild = m_children[i]->computeCollisionDetection(localSegmentPointA,
                                                                 localSegmentPointB,
                                                                 a_recorder,
                                                                 a_settings);

        // update if a hit occured
        hit = hit | hitChild;
    }


    // return whether there was a collision between the segment and this world
    return (hit);
}


//==============================================================================
/*!
    Show or hide the graphic representation of the collision tree at 
    this node. \n

    If \e a_affectChildren is set to __true__ then all children are updated
    with the new value.

    \param  a_showCollisionDetector If __true__, collision detector is displayed graphically.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setShowCollisionDetector(const bool a_showCollisionDetector, 
                                          const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setShowCollisionDetector(a_showCollisionDetector, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowCollisionDetector(a_showCollisionDetector, true);
    }
}


//==============================================================================
/*!
    Set the rendering properties for the graphic representation of collision 
    detection tree at this node. \n

    If \e a_affectChildren is set to __true__ then all children are updated
    with the new values.

    \param  a_color  Color used to render collision detector.
    \param  a_displayDepth  Indicated which depth of collision tree needs to be displayed
                            (see cGenericCollision).
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::setCollisionDetectorProperties(unsigned int a_displayDepth,
                                                cColorf& a_color, 
                                                const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::setCollisionDetectorProperties(a_displayDepth,
                                                   a_color, 
                                                   a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setCollisionDetectorProperties(a_displayDepth,
                                              a_color, 
                                              true);
    }
}


//==============================================================================
/*!
    Uniform scale, optionally include children.  Not necessarily
    implemented in all subclasses.  Does nothing at the cGenericObject
    level. Subclasses should scale themselves, then call the superclass
    method.

    \param  a_scaleFactor  Scale factor.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiMesh::scale(const double& a_scaleFactor, 
                       const bool a_affectChildren)
{
    // update current object and possibly children
    cGenericObject::scale(a_scaleFactor, a_affectChildren);

    // updated meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->scale(a_scaleFactor, true);
    }
}


//==============================================================================
/*!
    Scale this object by using different scale factors along X,Y and Z axes.

    \param  a_scaleX  Scale factor along X axis.
    \param  a_scaleY  Scale factor along Y axis.
    \param  a_scaleZ  Scale factor along Z axis.
*/
//==============================================================================
void cMultiMesh::scaleXYZ(const double a_scaleX, const double a_scaleY, const double a_scaleZ)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMatrix3d a_R_b = (*it)->getLocalRot();
        cMatrix3d b_R_a = cTranspose(a_R_b);

        // scale vertices
        vector<cVertex>::iterator it2 = (*it)->m_vertices->begin();
        for (it2 = (*it)->m_vertices->begin(); it2 < (*it)->m_vertices->end(); it2++)
        {
            cVector3d b_Vertex =  (*it2).m_localPos;
            cVector3d a_Vertex = a_R_b *  b_Vertex;
            a_Vertex = a_R_b * b_Vertex;
            a_Vertex.mul(a_scaleX, a_scaleY, a_scaleZ);
            b_Vertex = b_R_a * a_Vertex;
            (*it2).m_localPos = b_Vertex;
        }

        // scale position
        (*it)->m_localPos.mul(a_scaleX, a_scaleY, a_scaleZ);

        // update boundary box
        cVector3d b_BoxMin = (*it)->m_boundaryBoxMin;
        cVector3d a_BoxMin = a_R_b * b_BoxMin;
        a_BoxMin.mul(a_scaleX, a_scaleY, a_scaleZ);
        b_BoxMin = b_R_a * a_BoxMin;
        (*it)->m_boundaryBoxMin = b_BoxMin;

        cVector3d b_BoxMax = (*it)->m_boundaryBoxMax;
        cVector3d a_BoxMax = a_R_b * b_BoxMax;
        a_BoxMax.mul(a_scaleX, a_scaleY, a_scaleZ);
        b_BoxMax = b_R_a * a_BoxMax;
        (*it)->m_boundaryBoxMax = b_BoxMax;
    }

    // update boundary box
    m_boundaryBoxMax.mul(a_scaleX, a_scaleY, a_scaleZ);
    m_boundaryBoxMin.mul(a_scaleX, a_scaleY, a_scaleZ);
}


//==============================================================================
/*!
    Create a new mesh primitive and add it to the list of meshes.

    \return Return new mesh object.
*/
//==============================================================================
cMesh* cMultiMesh::newMesh()
{
    // create new mesh entity
    cMesh* obj = new cMesh();

    // set parent and owner
    obj->setParent(this);
    obj->setOwner(this);

    // add mesh to list
    m_meshes->push_back(obj);

    // resturn result
    return (obj);
}


//==============================================================================
/*!
    Add an existing mesh primitives to list of meshes.

    \return Return __true__ if operation succeeds.
*/
//==============================================================================
bool cMultiMesh::addMesh(cMesh* a_mesh)
{
    // sanity check
    if (a_mesh == NULL) { return (false); }
    if (a_mesh->getParent() != NULL) { return (false); }

    // set parent and owner
    a_mesh->setParent(this);
    a_mesh->setOwner(this);

    // add mesh to list
    m_meshes->push_back(a_mesh);

    // return success
    return (true);
}


//==============================================================================
/*!
    Remove an mesh primitive from list of meshes.

    \param  a_mesh  Mesh to remove.

    \return Return __true__ if operation succeeds.
*/
//==============================================================================
bool cMultiMesh::removeMesh(cMesh* a_mesh)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        if ((*it) == a_mesh)
        {
            // parent is set to NULL. Mesh becomes its own owwner.
            a_mesh->setParent(NULL);
            a_mesh->setOwner(a_mesh);

            // remove mesh from list of meshes
            m_meshes->erase(it);

            // return success
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    Remove all meshes from list.

    \return Return __true__ if operation succeeds.
*/
//==============================================================================
bool cMultiMesh::removeAllMesh()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        // parent is set to NULL. Mesh becomes its own owwner.
        (*it)->setParent(NULL);
        (*it)->setOwner(*it);
    }

    // clear list of meshes
    m_meshes->clear();

    // success
    return (true);
}


//==============================================================================
/*!
    Remove an mesh primitive from list of meshes.

    \param  a_mesh  Mesh to delete.

    \return Return __true__ if operation succeeds.
*/
//==============================================================================
bool cMultiMesh::deleteMesh(cMesh* a_mesh)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        if ((*it) == a_mesh)
        {
            // remove mesh from list of meshes
            m_meshes->erase(it);

            // delete mesh
            delete (*it);

            // return success
            return (true);
        }
    }

    // operation failed, mesh was not found
    return (false);
}


//==============================================================================
/*!
    Remove all meshes from list and delete them

    \return Return __true__ if operation succeeds.
*/
//==============================================================================
bool cMultiMesh::deleteAllMeshes()
{
    // delete all meshes
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMesh* nextMesh = (*it);
        delete nextMesh;
    }

    // clear all meshes from list
    m_meshes->clear();

    // success
    return (true);
}


//==============================================================================
/*!
    Retrieve the number of mesh primitives composing the multimesh.

    \return Return number of mesh primitives contained in current object.
*/
//==============================================================================
int cMultiMesh::getNumMeshes()
{
    unsigned int numMeshes = (unsigned int)(m_meshes->size());
    return (numMeshes);
}


//==============================================================================
/*!
    Get access to an individual mesh primitive by passing its index number.

    \param  a_index  Index number of mesh.

    \return Return pointer to requested mesh.
*/
//==============================================================================
cMesh* cMultiMesh::getMesh(unsigned  int a_index)
{
    if (a_index < m_meshes->size())
    {
        return (m_meshes->at(a_index));
    }
    else
    {
        return (NULL);
    }
}


//==============================================================================
/*!
    Returns the specified triangle.

    \param  a_index  The index of the requested triangle.
*/
//==============================================================================
cTriangle* cMultiMesh::getTriangle(unsigned int a_index)
{
    // sanity check
    if (a_index >= getNumTriangles()) return (NULL);

    // retrieve triangle
    unsigned int i, numMeshes;
    numMeshes = (unsigned int)(m_meshes->size());
    for (i=0; i<numMeshes; i++)
    {
        cMesh* nextMesh = m_meshes->at(i);
        if (nextMesh)
        {
             unsigned int numTriangles = nextMesh->getNumTriangles();

             if (a_index < numTriangles)
             {
                return (nextMesh->getTriangle(a_index));
             }
             else 
             {
                a_index -= numTriangles;
             }
        }
    }

    return (NULL);
}


//==============================================================================
/*!
    Returns the number of triangles contained in this multi-mesh.

    \return Return number of vertices.
*/
//==============================================================================
unsigned int cMultiMesh::getNumTriangles() const
{
    int numTriangles = 0;

    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        numTriangles = numTriangles + (*it)->getNumTriangles();
    }

    return (numTriangles);
}


//==============================================================================
/*!
    Returns the specified vertex.

    \param  a_index  Index of the requested vertex
*/
//==============================================================================
cVertex* cMultiMesh::getVertex(unsigned int a_index)
{
    // sanity check
    if (a_index >= getNumVertices()) return (NULL);

    // retrieve triangle
    unsigned int i, numMeshes;
    numMeshes = (unsigned int)(m_meshes->size());
    for (i=0; i<numMeshes; i++)
    {
        cMesh* nextMesh = m_meshes->at(i);
        if (nextMesh)
        {
             unsigned int numVertices = nextMesh->getNumVertices();

             if (a_index < numVertices)
             {
                return (nextMesh->getVertex(a_index));
             }
             else 
             {
                a_index -= numVertices;
             }
        }
    }

    return (NULL);
}


//==============================================================================
/*!
    Returns the number of vertices contained in this multi-mesh.

    \return Return number of vertices.
*/
//==============================================================================
unsigned int cMultiMesh::getNumVertices() const
{
    int numVertices = 0;

    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        numVertices = numVertices + (*it)->getNumVertices();
    }

    return (numVertices);
}


//==============================================================================
/*!
    This enables the use of vertex arrays for mesh rendering. This
    mode can be faster than the classical approach, however crashes
    sometime occur on certain types of graphic cards.

    In general, if you aren't having problems with rendering performance,
    don't bother with this.

    \param  a_useVertexArrays  If __true__, this mesh will be rendered using vertex array technique
*/
//==============================================================================
void cMultiMesh::setUseVertexArrays(const bool a_useVertexArrays)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setUseVertexArrays(a_useVertexArrays);
    }
}


//==============================================================================
/*!
    Clear all triangles and vertices.
*/
//==============================================================================
void cMultiMesh::clear()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->clear();
    }
}


//==============================================================================
/*!
    Load a 3D mesh file.  CHAI3D currently supports .obj and .3ds files.

    \param  a_filename  Filename of 3D model.
    \return Return __true__ is file loaded correctly. Otherwise return __false__.
*/
//==============================================================================
bool cMultiMesh::loadFromFile(string a_filename)
{ 
    // find extension
    string extension = cGetFileExtension(a_filename);

    // we need a file extension to figure out file type
    if (extension.length() == 0)
    {
        return (false);
    }

    // convert string to lower extension
    string fileType = cStrToLower(extension);

    // result for loading file
    bool result = false;

    //--------------------------------------------------------------------
    // .3DS FORMAT
    //--------------------------------------------------------------------
    if (fileType == "3ds")
    {
        result = cLoadFile3DS(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .OBJ FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "obj")
    {
        result = cLoadFileOBJ(this, a_filename);
    }

    return (result);
}


//==============================================================================
/*!
    Save a mesh object to file. CHAI3D currently supports .obj and .3ds files.

    \param  a_filename  Filename of 3D model.
    \return Return __true__ is file is save correctly. Otherwise return __false__.
*/
//==============================================================================
bool cMultiMesh::saveToFile(std::string a_filename)
{ 
    // find extension
    string extension = cGetFileExtension(a_filename);

    // we need a file extension to figure out file type
    if (extension.length() == 0)
    {
        return (false);
    }

    // convert string to lower extension
    string fileType = cStrToLower(extension);

    // result for loading file
    bool result = false;

    //--------------------------------------------------------------------
    // .3DS FORMAT
    //--------------------------------------------------------------------
    if (fileType == "3ds")
    {
        result = cSaveFile3DS(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .OBJ FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "obj")
    {
        result = cSaveFileOBJ(this, a_filename);
    }

    return (result);
}


//==============================================================================
/*!
    Compute surface normals for every vertex in the mesh, by averaging
    the face normals of the triangle that include each vertex.
*/
//==============================================================================
void cMultiMesh::computeAllNormals()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->computeAllNormals();
    }
}


//==============================================================================
/*!
    Compute the global position of all vertices

    \param  a_frameOnly  If __false__, the global position of all vertices
            is computed, otherwise this function does nothing.
*/
//==============================================================================
void cMultiMesh::updateGlobalPositions(const bool a_frameOnly)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->computeGlobalPositions(a_frameOnly, 
                                      m_globalPos, 
                                      m_globalRot);
    }
}


//==============================================================================
/*!
    Set color of each vertex.
*/
//==============================================================================
void cMultiMesh::setVertexColor(const cColorf& a_color)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setVertexColor(a_color);
    }
}


//==============================================================================
/*!
    Reverse the normal for every vertex on this model.  Useful for models
    that started with inverted faces and thus gave inward-pointing normals.
*/
//==============================================================================
void cMultiMesh::reverseAllNormals()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->reverseAllNormals();
    }
}


//==============================================================================
/*!
    Define the way normals are graphically rendered, optionally propagating
    the operation to my children.

    \param  a_length  Length of normals
    \param  a_color  Color of normals
*/
//==============================================================================
void cMultiMesh::setNormalsProperties(const double a_length, 
                                      const cColorf& a_color)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setNormalsProperties(a_length, a_color);
    }
}


//==============================================================================
/*!
    Enable or disable the graphic rendering of normal vectors at each vertex.

    \param  a_showNormals  If __true__, normal vectors are rendered graphically.
*/
//==============================================================================
void cMultiMesh::setShowNormals(const bool& a_showNormals)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowNormals(a_showNormals);
    }
}


//==============================================================================
/*!
    Enable or disable the rendering of edges.

    \param  a_showEdges  If __true__, edges are rendered.
*/
//==============================================================================
void cMultiMesh::setShowEdges(const bool a_showEdges)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setShowEdges(a_showEdges);
    }
}


//==============================================================================
/*!
    Set graphic properties for edge-rendering.

    \param  a_width  Width of edge lines.
    \param  a_color  Color of edge lines.
*/
//==============================================================================
void cMultiMesh::setEdgeProperties(const double a_width, 
                                   const cColorf& a_color)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->setEdgeProperties(a_width, a_color);
    }
}


//==============================================================================
/*!
    Create a list of edges by providing a threshold angle in degrees. All
    triangles for which the angle between their respective surface normals 
    are greater than the select angle threshold are added to the list of 
    edges.

    \param  a_angleThresholdDeg  Threshold angle in degrees.
*/
//==============================================================================
void cMultiMesh::computeAllEdges(double a_angleThresholdDeg)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->computeAllEdges(a_angleThresholdDeg);
    }
}


//==============================================================================
/*!
    Compute all edges.
*/
//==============================================================================
void cMultiMesh::clearAllEdges()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->clearAllEdges();
    }
}


//==============================================================================
/*!
    Set up a Brute Force collision detector for this object.
*/
//==============================================================================
void cMultiMesh::createBruteForceCollisionDetector()
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->createBruteForceCollisionDetector();
    }
}


//==============================================================================
/*!
    Set up an AABB collision detector for this object.

    \param  a_radius  Bounding radius.
*/
//==============================================================================
void cMultiMesh::createAABBCollisionDetector(const double a_radius)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->createAABBCollisionDetector(a_radius);
    }
}


//==============================================================================
/*!
    Render this mesh in OpenGL.  This method actually just prepares some
    OpenGL state, and uses renderMesh to actually do the rendering.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMultiMesh::render(cRenderOptions& a_options)
{
    vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        (*it)->renderSceneGraph(a_options);
    }
}


//==============================================================================
/*!
    Descend through child objects to compute interactions for all
    cGenericEffect classes defined for each object.

    \param  a_toolPos  Current position of tool.
    \param  a_toolVel  Current position of tool.
    \param  a_IDN  Identification number of the force algorithm.
    \param  a_interactions  List of recorded interactions.

    \return Return resulting interaction forces.
*/
//==============================================================================
cVector3d cMultiMesh::computeInteractions(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN,
                                          cInteractionRecorder& a_interactions)
{
    // compute inverse rotation
    cMatrix3d localRotTrans;
    m_localRot.transr(localRotTrans);

    // compute local position of tool and velocity vector
    cVector3d toolPosLocal = cMul(localRotTrans, cSub(a_toolPos, m_localPos));

    // compute interaction between tool and current object
    cVector3d toolVelLocal = cMul(localRotTrans, a_toolVel);

    // compute forces based on the effects programmed for this object
    cVector3d localForce(0,0,0);

    // process current object if enabled
    if (m_enabled)
    {
        // check if node is a ghost. If yes, then ignore call
        if (m_ghostEnabled) { return (cVector3d(0,0,0)); }

        // compute local interaction with current object
        computeLocalInteraction(toolPosLocal,
                                toolVelLocal,
                                a_IDN);

        if(m_hapticEnabled)
        {
            // compute each force effect
            bool interactionEvent = false;
            for (unsigned int i=0; i<m_effects.size(); i++)
            {
                cGenericEffect *nextEffect = m_effects[i];

                if (nextEffect->isEnabled())
                {
                    cVector3d force(0,0,0);

                    interactionEvent= interactionEvent |
                        nextEffect->computeForce(toolPosLocal,
                                                 toolVelLocal,
                                                 a_IDN,
                                                 force);
                    localForce.add(force);
                }
            }

            // report any interaction
            if (interactionEvent)
            {
                cInteractionEvent newInteractionEvent;
                newInteractionEvent.m_object = this;
                newInteractionEvent.m_isInside = m_interactionInside;
                newInteractionEvent.m_localPos = toolPosLocal;
                newInteractionEvent.m_localSurfacePos = m_interactionPoint;
                newInteractionEvent.m_localNormal = m_interactionNormal;
                newInteractionEvent.m_localForce = localForce;
                a_interactions.m_interactions.push_back(newInteractionEvent);
            }

            // compute any other force interactions
            cVector3d force = computeOtherInteractions(toolPosLocal,
                                                       toolVelLocal,
                                                       a_IDN,
                                                       a_interactions);

            localForce.add(force);
        }
    }

    // descend through the meshes
    {
        vector<cMesh*>::iterator it;
        for (it = m_meshes->begin(); it < m_meshes->end(); it++)
        {
            cVector3d force = (*it)->computeInteractions(toolPosLocal,
                                                         toolVelLocal,
                                                         a_IDN,
                                                         a_interactions);
            localForce.add(force);
        }
    }

    // descend through the children
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            cVector3d force = (*it)->computeInteractions(toolPosLocal,
                                                         toolVelLocal,
                                                         a_IDN,
                                                         a_interactions);
            localForce.add(force);
        }
    }

    // convert the reaction force into my parent coordinates
    cVector3d m_globalForce = cMul(m_localRot, localForce);

    // return resulting force
    return (m_globalForce);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
