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
    \author    Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1067 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "collisions/CGenericCollision.h"
#include "math/CMaths.h"
#include "graphics/CDraw3D.h"
#include "effects/CEffectMagnet.h"
#include "effects/CEffectStickSlip.h"
#include "effects/CEffectSurface.h"
#include "effects/CEffectVibration.h"
#include "effects/CEffectViscosity.h"
//------------------------------------------------------------------------------
#include <float.h>
#include <vector>
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
cMaterial cGenericObject::s_defaultMaterial;
cColorf cGenericObject::s_boundaryBoxColor(0.5, 0.5, 0.0);
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cGenericObject.
*/
//==============================================================================
cGenericObject::cGenericObject()
{
    // object is enabled
    m_enabled = true;

    // initialize local position and orientation
    m_localPos.zero();
    m_localRot.identity();

    // initialize global position and orientation
    m_globalPos.zero();
    m_globalRot.identity();
    m_prevGlobalPos.zero();
    m_prevGlobalRot.identity();

    // initialize openGL matrix with position vector and orientation matrix
    m_frameGL.set(m_globalPos, m_globalRot);

    // initialize name
    m_name = "";

    // custom user information
    m_userName = "";
    m_userTag  = 0;
    m_userData = NULL;
    m_userExternalObject = NULL;

    // should we use the material property?
    m_useMaterialProperty = true;

    // Are vertex colors used during rendering process?
    m_useVertexColors = false;

    // Should transparency be used?
    m_useTransparency = false;

    // Should texture mapping be used if a texture is defined?
    m_useTextureMapping = false;

    // How are triangles displayed; FILL or LINE ?
    m_triangleMode = GL_FILL;

    // initialize texture
    m_texture = NULL;

    // turn off culling on by default
    m_cullingEnabled = false;

    // disable ghost setting
    m_ghostEnabled = false;

    // no external parent defined
    m_owner = this;

    // no parent defined
    m_parent = NULL;

    // object is not interacting with any tool
    m_interactionInside = false;
    m_interactionPoint.zero();
    m_interactionNormal.set(1,0,0);

    // empty list of haptic effects
    m_effects.clear();

    // setup default material
    m_material = &s_defaultMaterial;

    // enable graphic rendering
    m_showEnabled = true;

    // enable haptic rendering
    m_hapticEnabled = true;

    // reference frame
    m_showFrame = false;
    m_frameSize = 1.0;
    m_frameThicknessScale = 2.0;

    // boundary box
    m_showBoundaryBox = false;
    m_boundaryBoxMin.set(0.0, 0.0, 0.0); 
    m_boundaryBoxMax.set(0.0, 0.0, 0.0);
    m_boundaryBoxEmpty = true;

    // collision detector
    m_collisionDetector = NULL; 
    m_showCollisionDetector = false;

}


//==============================================================================
/*!
    Destructor of cGenericObject.  This function deletes all children
    starting from this point in the scene graph, so if you have objects
    that shouldn't be deleted, be sure to remove them from the scene
    graph before deleting their parents.
*/
//==============================================================================
cGenericObject::~cGenericObject()
{
    // Each of my children will remove their own collision detector when
    // they get deleted, so I just delete my own right now...
    if (m_collisionDetector) 
    {
        deleteCollisionDetector(false);
    }

    // delete all children
    deleteAllChildren();
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
void cGenericObject::setEnabled(bool a_enabled, const bool a_affectChildren)
{
    m_enabled = a_enabled;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setEnabled(a_enabled, true);
        }
    }
}


//==============================================================================
/*!
    Translate this object by a specified offset.

    \param  a_translation  Translation offset.
*/
//==============================================================================
void cGenericObject::translate(const cVector3d& a_translation)
{
    cVector3d new_position = cAdd(m_localPos, a_translation);
    setLocalPos(new_position);
}


//==============================================================================
/*!
    Translate an object by a specified offset.

    \param  a_x  Translation component X.
    \param  a_y  Translation component Y.
    \param  a_z  Translation component Z.
*/
//==============================================================================
void cGenericObject::translate(const double a_x, 
    const double a_y, 
    const double a_z)
{
    translate(cVector3d(a_x,a_y,a_z));
}


//==============================================================================
/*!
    Rotate this object around a local axis

    \param  a_axis   Rotation axis. This vector must be normalized!
    \param  a_angleRad  Rotation angle in radians.
*/
//==============================================================================
void cGenericObject::rotateAboutLocalAxisRad(const cVector3d& a_axis, const double a_angleRad)
{
    m_localRot.rotateAboutLocalAxisRad(a_axis, a_angleRad);
}


//==============================================================================
/*!
    Rotate this object around a global axis

    \param  a_axis   Rotation axis. This vector must be normalized!
    \param  a_angleRad  Rotation angle in radians.
*/
//==============================================================================
void cGenericObject::rotateAboutGlobalAxisRad(const cVector3d& a_axis,  const double a_angleRad)
{
    m_localRot.rotateAboutGlobalAxisRad(a_axis, a_angleRad);
}


//==============================================================================
/*!
    Builds a rotation matrix from a set of Euler angles and fixed axes of rotations.

    \param  a_angleRad1  Angle in radians of the first rotation in the sequence.
    \param  a_angleRad2  Angle in radians of the second rotation in the sequence.
    \param  a_angleRad3  Angle in radians of the third rotation in the sequence.
    \param  a_eulerOrder  The order of the axes about which the rotations are to be applied
*/
//==============================================================================
void cGenericObject::rotateExtrinsicEulerAnglesRad(const double& a_angleRad1,
    const double& a_angleRad2,
    const double& a_angleRad3,
    const cEulerOrder a_eulerOrder)
{
    m_localRot.setExtrinsicEulerRotationRad(a_angleRad1,
                                            a_angleRad2,
                                            a_angleRad3,
                                            a_eulerOrder);
}


//==============================================================================
/*!
    Builds a rotation matrix from a set of Euler angles and co-moving axes of rotations.

    \param  a_angleRad1  Angle in radians of the first rotation in the sequence.
    \param  a_angleRad2  Angle in radians of the second rotation in the sequence.
    \param  a_angleRad3  Angle in radians of the third rotation in the sequence.
    \param  a_eulerOrder  The order of the axes about which the rotations are to be applied.
*/
//==============================================================================
void cGenericObject::rotateIntrinsicEulerAnglesRad(const double& a_angleRad1,
    const double& a_angleRad2,
    const double& a_angleRad3,
    const cEulerOrder a_eulerOrder)
{
    m_localRot.setIntrinsicEulerRotationRad(a_angleRad1,
                                            a_angleRad2,
                                            a_angleRad3,
                                            a_eulerOrder);
}


//==============================================================================
/*!
    Compute globalPos and globalRot given the localPos and localRot
    of this object and its parents. Optionally propagates to children. \n

    If \e a_frameOnly is set to __false__, additional global positions such as
    vertex positions are computed (which can be time-consuming). \n

    Make sure to call this method at the beginning of each haptic loop. 

    \param  a_frameOnly  If __true__ then only the global frame is computed
    \param  a_globalPos  Global position of my parent.
    \param  a_globalRot  Global rotation matrix of my parent.
*/
//==============================================================================
void cGenericObject::computeGlobalPositions(const bool a_frameOnly,
    const cVector3d& a_globalPos, 
    const cMatrix3d& a_globalRot)
{
    // check if node is a ghost. If yes, then ignore call
    if (m_ghostEnabled) { return; }

    // current values become previous values
    m_prevGlobalPos = m_globalPos;
    m_prevGlobalRot = m_globalRot;

    // update global position vector and global rotation matrix
    m_globalPos = cAdd(a_globalPos, cMul(a_globalRot, m_localPos));
    m_globalRot = cMul(a_globalRot, m_localRot);

    // update any positions within the current object that need to be
    // updated (e.g. vertex positions)
    updateGlobalPositions(a_frameOnly);

    // propagate this method to my children
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        (*it)->computeGlobalPositions(a_frameOnly, m_globalPos, m_globalRot);
    }
}


//==============================================================================
/*!
    Compute globalPos and globalRot for this object only, by recursively
    climbing up the scene graph tree until the root is reached.

    If \e a_frameOnly is set to __false__, additional global positions such as
    vertex positions are computed.

    \param  a_frameOnly  If __true__ then only the global frame is computed.
*/
//==============================================================================
void cGenericObject::computeGlobalPositionsFromRoot(const bool a_frameOnly)
{
    cMatrix3d globalRot;
    cVector3d globalPos;
    globalRot.identity();
    globalPos.zero();

    // get a pointer to current object
    cGenericObject *curObject = getParent();

    if (curObject != NULL)
    {
        // walk up the scene graph until we reach the root, updating
        // my global position and rotation at each step
        bool done = false;
        do 
        {
            curObject->getLocalRot().mul(globalPos);
            globalPos.add(curObject->getLocalPos());
            cMatrix3d rot;
            curObject->getLocalRot().mulr(globalRot, rot);
            rot.copyto(globalRot);
            curObject = curObject->getParent();
        } 
        while (curObject != NULL);
    }

    // compute global positions for current object and child
    computeGlobalPositions(a_frameOnly, globalPos, globalRot);
}


//==============================================================================
/*!
    Add a haptic effect to this object.

    \param  a_effect  Haptic effect to be added to list.
*/
//==============================================================================
bool cGenericObject::addEffect(cGenericEffect* a_effect)
{
    if (a_effect->m_parent != this)
    {
        return (false);
    }

    // update the effect object's parent pointer
    a_effect->m_parent = this;

    // add this child to my list of children
    m_effects.push_back(a_effect);

    // success
    return (true);
}


//==============================================================================
/*!
    Remove a haptic effect from this object.

    \param  a_effect  Haptic effect to be removed from list of effects.
*/
//==============================================================================
bool cGenericObject::removeEffect(cGenericEffect* a_effect)
{
    // find haptic effect and remove it from list
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if ((*it) == a_effect)
        {
            (*it)->m_parent = NULL;
            m_effects.erase(it);
            return (true);
        }
    }

    // operation failed
    return (false);
}


//==============================================================================
/*!
    Delete all haptic effects. Effect objects are deleted from memory.
*/
//==============================================================================
void cGenericObject::deleteAllEffects()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        delete (*it);
    }
    m_effects.clear();
}


//==============================================================================
/*!
    Create a magnetic haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectMagnetic()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectMagnet*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectMagnet(this);
        addEffect(effect);
        return (true);
    }
    return (false);
}


//==============================================================================
/*!
    Delete magnetic haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectMagnetic()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectMagnet*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    Create a stick-and-slip haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectStickSlip()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectStickSlip*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectStickSlip(this);
        addEffect(effect);
        return (true);
    }
    return (false);
}

//==============================================================================
/*!
    Delete stick-and-slip haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectStickSlip()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectStickSlip*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    Create a surface haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectSurface()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectSurface*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectSurface(this);
        addEffect(effect);
        return (true);
    }
    return (false);
}


//==============================================================================
/*!
    Delete surface haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectSurface()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectSurface*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    Create a vibration haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectVibration()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectVibration*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectVibration(this);
        addEffect(effect);
        return (true);
    }
    return (false);
}


//==============================================================================
/*!
    Delete current vibration haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectVibration()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectVibration*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
}


//==============================================================================
/*!
    Create a viscous haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::createEffectViscosity()
{
    bool flag = true;
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        if (dynamic_cast<cEffectViscosity*>(*it) != NULL)
        {
            flag = false;
        }
    }

    if (flag)
    {
        cGenericEffect* effect = new cEffectViscosity(this);
        addEffect(effect);
        return (true);
    }
    return (false);
}


//==============================================================================
/*!
    Delete current viscous haptic effect.

    \return Return __true__ if operation succeeds. Return __false__ otherwise.
*/
//==============================================================================
bool cGenericObject::deleteEffectViscosity()
{
    vector<cGenericEffect*>::iterator it;
    for (it = m_effects.begin(); it < m_effects.end(); it++)
    {
        cGenericEffect* effect = dynamic_cast<cEffectViscosity*>(*it);
        if (effect != NULL)
        {
            m_effects.erase(it);
            delete effect;
            return (true);
        }
    }
    return (false);
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
void cGenericObject::setHapticEnabled(const bool a_hapticEnabled, 
    const bool a_affectChildren)
{
    m_hapticEnabled = a_hapticEnabled;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setHapticEnabled(a_hapticEnabled, true);
        }
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
void cGenericObject::setStiffness(const double a_stiffness, 
    const bool a_affectChildren)
{
    m_material->setStiffness(a_stiffness);

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setStiffness(a_stiffness, true);
        }
    }
}


//==============================================================================
/*!
     Set the static and dynamic friction properties for this mesh, 
     possibly recursively affecting children.

     \param     a_staticFriction  The static friction to apply to this object.
     \param     a_dynamicFriction  The dynamic friction to apply to this object.
     \param     a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setFriction(double a_staticFriction, 
    double a_dynamicFriction, 
    const bool a_affectChildren)
{
    m_material->setStaticFriction(a_staticFriction);
    m_material->setDynamicFriction(a_dynamicFriction);

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setFriction(a_staticFriction, a_dynamicFriction, true);
        }
    }
}


//==============================================================================
/*!
    Graphically show or hide this object, possibly recursively affecting children.

    \param  a_show  If __true__ object shape is visible.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setShowEnabled(const bool a_show, const bool a_affectChildren)
{
    // update current object
    m_showEnabled = a_show;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShowEnabled(a_show, true);
        }
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
void cGenericObject::setUseTransparency(const bool a_useTransparency, const bool a_affectChildren)
{
    // update changes to object
    m_useTransparency = a_useTransparency;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseTransparency(a_useTransparency, true);
        }
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
void cGenericObject::setTransparencyLevel(const float a_level, 
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
    Enable or disable wireframe rendering, optionally propagating the
    operation to children.

    \param  a_showWireMode  If __true__, wireframe mode is used.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setWireMode(const bool a_showWireMode, 
                                 const bool a_affectChildren)
{
    if (a_showWireMode)  
    { 
        m_triangleMode = GL_LINE; 
    }
    else 
    { 
        m_triangleMode = GL_FILL; 
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setWireMode(a_showWireMode, true);
        }
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
void cGenericObject::setUseCulling(const bool a_useCulling, 
                                   const bool a_affectChildren)
{
    m_cullingEnabled = a_useCulling;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseCulling(a_useCulling, true);
        }
    }
}


//==============================================================================
/*!
    Enable or disable the use of per-vertex color information of when rendering
    the mesh.

    \param  a_useColors   If __true__, then vertex color information is applied.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setUseVertexColors(const bool a_useColors, 
                                        const bool a_affectChildren)
{
    m_useVertexColors = a_useColors;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseVertexColors(a_useColors, true);
        }
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
void cGenericObject::backupColors(const bool a_material_properties_only, 
                                  const bool a_affectChildren)
{
    if ((m_material != NULL) && (m_material != &s_defaultMaterial))
    {
        m_material->backupColors();
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->backupColors(a_material_properties_only, true);
        }
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
void cGenericObject::restoreColors(const bool a_material_properties_only, 
                                   const bool a_affectChildren)
{
    if ((m_material != NULL) && (m_material != &s_defaultMaterial))
    {
        m_material->restoreColors();
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->restoreColors(a_material_properties_only, true);
        }
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
void cGenericObject::setUseDisplayList(const bool a_useDisplayList,
                                       const bool a_affectChildren)
{
    // update changes to object
    m_useDisplayList = a_useDisplayList;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseDisplayList(a_useDisplayList, true);
        }
    }
}


//==============================================================================
/*!
    Invalidate any existing display lists.  You should call this on if you're using
    display lists and you modify mesh options, vertex positions, etc.

    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::invalidateDisplayList(const bool a_affectChildren)
{
    // invalidate display list
    m_displayList.invalidate();

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->invalidateDisplayList(true);
        }
    }
}


//==============================================================================
/*!
     Enable or disable the use of material properties.

     \param  a_useMaterial If __true__, then material properties are used for rendering.
     \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setUseMaterial(const bool a_useMaterial, 
                                    const bool a_affectChildren)
{
    // update changes to object
    m_useMaterialProperty = a_useMaterial;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseMaterial(a_useMaterial, true);
        }
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
void cGenericObject::setMaterial(cMaterial* a_mat,
                                 const bool a_affectChildren)
{
    // copy material properties
    m_material = a_mat;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setMaterial(a_mat, true);
        }
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
void cGenericObject::setMaterial(cMaterial& a_mat,
                                 const bool a_affectChildren)
{
    // copy material properties
    if ((m_material == NULL) || (m_material == &s_defaultMaterial))
    {
        m_material = new cMaterial();
        a_mat.copyTo(m_material);
    }
    else
    {
        a_mat.copyTo(m_material);
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setMaterial(a_mat, true);
        }
    }
}


//==============================================================================
/*!
    Enable or disable texture-mapping, possibly recursively affecting 
    children.

    \param  a_useTexture If __true__, then texture mapping is used.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setUseTexture(const bool a_useTexture, 
                                   const bool a_affectChildren)
{
    m_useTextureMapping = a_useTexture;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setUseTexture(a_useTexture, true);
        }
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
void cGenericObject::setTexture(cTexture1d* a_texture,
                                const bool a_affectChildren)
{
    m_texture = a_texture;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setTexture(a_texture, true);
        }
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
void cGenericObject::setShowBoundaryBox(const bool a_showBoundaryBox, 
                                        const bool a_affectChildren)
{
    // update current object
    m_showBoundaryBox = a_showBoundaryBox;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShowBoundaryBox(a_showBoundaryBox, true);
        }
    }
}


//==============================================================================
/*!
    Compute the bounding box of this object and (optionally) its children. \n

    If parameter \e a_includeChildren is set to __true__ then each object's
    bounding box covers its own volume and the volume of its children. \n

    Note that regardless of this parameter's value, this operation propagates
    down the scene graph.

    \param  a_includeChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::computeBoundaryBox(const bool a_includeChildren)
{
    // check if node is a ghost. If yes, then ignore call
    if (m_ghostEnabled) { return; }

    // compute the bounding box of this object
    updateBoundaryBox();

    // exit if children are not included
    if (a_includeChildren == false) 
    {
        return;
    }

    // compute the bounding box of all my children
    cVector3d minBox, maxBox;

    if (m_boundaryBoxEmpty)
    {
        minBox.set( C_LARGE, C_LARGE, C_LARGE);
        maxBox.set(-C_LARGE,-C_LARGE,-C_LARGE);
    }
    else
    {
        minBox = m_boundaryBoxMin;
        maxBox = m_boundaryBoxMax;
    }

    bool empty = true;
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        cGenericObject* object = (*it);

        // compute boundary box of child
        object->computeBoundaryBox(a_includeChildren);

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
                minBox(0) = cMin(corners[i](0),  m_boundaryBoxMin(0));
                minBox(1) = cMin(corners[i](1),  m_boundaryBoxMin(1));
                minBox(2) = cMin(corners[i](2),  m_boundaryBoxMin(2));
                maxBox(0) = cMin(corners[i](0),  m_boundaryBoxMax(0));
                maxBox(1) = cMin(corners[i](1),  m_boundaryBoxMax(1));
                maxBox(2) = cMin(corners[i](2),  m_boundaryBoxMax(2));
            }
        }
    }

    if (empty && m_boundaryBoxEmpty)
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
    Show or hide the set of arrows that represent this object's reference frame.

    If \e a_affectChildren is set to __true__ then all children are updated
    with the new value.

    \param  a_showFrame  If __true__ then frame is displayed.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setShowFrame(const bool a_showFrame, 
                                  const bool a_affectChildren)
{
    // update current object
    m_showFrame = a_showFrame;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShowFrame(a_showFrame, true);
        }
    }
}


//==============================================================================
/*!
    Set the display size of the arrows representing my reference frame.
    The size corresponds to the length of each displayed axis (X-Y-Z). \n

    If \e a_affectChildren is set to __true__ then all children are updated
    with the new value.

    \param  a_size            Length of graphical representation of frame.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setFrameSize(const double a_size, 
                                  const bool a_affectChildren)
{
    // sanity check
    m_frameSize = cAbs(a_size);
    m_frameThicknessScale = 1.7 * m_frameSize;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setFrameSize(a_size, true);
        }
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
void cGenericObject::deleteCollisionDetector(const bool a_affectChildren)
{

    if (m_collisionDetector)
    {
        delete m_collisionDetector;
        m_collisionDetector = 0;
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->deleteCollisionDetector(true);
        }
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
bool cGenericObject::computeCollisionDetection(cVector3d& a_segmentPointA,
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

    // check for collisions with all children of this object
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        // call this child's collision detection function to see if it (or any
        // of its descendants) are intersected by the segment
        bool hitChild = m_children[i]->computeCollisionDetection(localSegmentPointA,
                                                                 localSegmentPointB,
                                                                 a_recorder,
                                                                 a_settings);

        // update if a hit ocured
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

    \param  a_showCollisionDetector  If __true__, display collision detector graphically.
    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cGenericObject::setShowCollisionDetector(const bool a_showCollisionDetector, 
                                              const bool a_affectChildren)
{
    m_showCollisionDetector = a_showCollisionDetector;

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setShowCollisionDetector(a_showCollisionDetector, true);
        }
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
void cGenericObject::setCollisionDetectorProperties(unsigned int a_displayDepth,
                                                    cColorf& a_color, 
                                                    const bool a_affectChildren)
{
    if (m_collisionDetector != NULL)
    {
        m_collisionDetector->m_color = a_color;
        m_collisionDetector->setDisplayDepth(a_displayDepth);
    }

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->setCollisionDetectorProperties(a_displayDepth,
                                                  a_color, 
                                                  true);
        }
    }
}


//==============================================================================
/*!
    Add an object to the scene graph below this object. Not that an object
    can only be a child of \b one single object, unless ghosting is enabled.

    \param  a_object  Object to be added to children list.
    \return Return __true__ if operation succeeded.
*/
//==============================================================================
bool cGenericObject::addChild(cGenericObject* a_object)
{
    // sanity check
    if ((a_object == NULL) || (a_object == this)) { return (false); }

    // the object does not have any parent yet, so we can add it as a child
    // to current object.
    if (a_object->m_parent == NULL)
    {
        m_children.push_back(a_object);
        a_object->m_parent = this;
        return (true);
    }

    // the object already has a parent, however since ghosting is enabled we add it
    // as a child without changing its m_parent member
    else if (m_ghostEnabled)
    {
        m_children.push_back(a_object);
        return (true);		
    }

    // operation fails
    return (false);
}


//==============================================================================
/*!
    Removes an object from my list of children, without deleting the
    child object from memory.

    This method assigns the child object's parent point to null, so
    if you're moving someone around in your scene graph, make sure you
    call this function _before_ you add the child to another node in
    the scene graph.

    \param  a_object  Object to be removed from my list of children.
    \return Returns __true__ if the specified object was found on my list 
            of children
*/
//==============================================================================
bool cGenericObject::removeChild(cGenericObject* a_object)
{
    // sanity check
    if (a_object == NULL) { return (false); }

    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        if ((*it) == a_object)
        {
            // he doesn't have a parent any more
            a_object->m_parent = NULL;

            // remove this object from my list of children
            m_children.erase(it);

            // return success
            return (true);
        }
    }

    // operation failed
    return (false);
}


//==============================================================================
/*!
    Remove this object from my parent's list of children.
*/
//==============================================================================
bool cGenericObject::removeFromGraph()
{
    if (m_parent) 
    {
        return (m_parent->removeChild(this));
    }
    else 
    {
        return (false);
    }
}


//==============================================================================
/*!
    Does this object have the specified object as a child?

    \param  a_object  Object to search for.
    \param  a_includeChildren  Should we also search this object's descendants?
*/
//==============================================================================
bool cGenericObject::containsChild(cGenericObject* a_object, 
                                   bool a_includeChildren)
{
    // sanity check
    if (a_object == NULL) { return (false); }

    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        if ((*it) == a_object)
        {
            return (true);
        }

        if (a_includeChildren)
        {
            bool result = (*it)->containsChild(a_object, true);
            if (result)
            {
                return (true);
            }
        }
    }
    return (false);
}


//==============================================================================
/*!
    Removes an object from my list of children, and deletes the
    child object from memory.

    \param  a_object  Object to be removed from my list of children 
                      and deleted.

    \return Returns __true__ if the specified object was found on my list 
            of children.
*/
//==============================================================================
bool cGenericObject::deleteChild(cGenericObject* a_object)
{
    // sanity check
    if (a_object == NULL) { return (false); }

    // remove object from list
    bool result = removeChild(a_object);

    // if operation succeeds, delete the object
    if (result)
    {
        delete (a_object);
    }

    // return result
    return result;
}


//==============================================================================
/*!
    Clear all objects from my list of children, without deleting them.
*/
//==============================================================================
void cGenericObject::clearAllChildren()
{
    // clear parent member for all children
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        (*it)->m_parent = NULL;
    }

    // clear children list
    m_children.clear();
}


//==============================================================================
/*!
    Delete and clear all objects from my list of children.
*/
//==============================================================================
void cGenericObject::deleteAllChildren()
{
    // delete all children
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        cGenericObject* nextObject = (*it);
        delete (nextObject);
    }

    // clear my list of children
    m_children.clear();
}


//==============================================================================
/*!
    Return my total number of descendants, optionally including this object.

    \param  a_includeCurrentObject  Should I include myself in the count?

    \return Returns the number of descendants found.
*/
//==============================================================================
unsigned int cGenericObject::getNumDescendants(bool a_includeCurrentObject)
{
    unsigned int numDescendants = a_includeCurrentObject?1:0;

    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        numDescendants += (*it)->getNumDescendants(true);
    }

    return (numDescendants);
}


//==============================================================================
/*!
    Fill this list with all of my descendants.  The current object is optionally
    included in this list.  Does not clear the list before appending to it.

    \param  a_childList  The list to write our enumerated results to.
    \param  a_includeCurrentObject  Should I include myself on the list?
*/
//==============================================================================
void cGenericObject::enumerateChildren(std::list<cGenericObject*>& a_childList,
                                       bool a_includeCurrentObject)
{
    if (a_includeCurrentObject) a_childList.push_back(this);

    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        (*it)->enumerateChildren(a_childList, true);
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
void cGenericObject::scale(const double& a_scaleFactor, 
                           const bool a_affectChildren)
{
    // scale object
    scaleObject(a_scaleFactor);

    // apply change to children
    if (a_affectChildren)
    {
        vector<cGenericObject*>::iterator it;
        for (it = m_children.begin(); it < m_children.end(); it++)
        {
            (*it)->m_localPos.mul(a_scaleFactor);
            (*it)->scale(a_scaleFactor, true);
        }
    }
}


//==============================================================================
/*!
    Render this object.  Subclasses will generally override this method.
    This is called from renderSceneGraph, which subclasses generally do
    not need to override. \n\n

    A word on OpenGL conventions: \n

    CHAI3D does not re-initialize the OpenGL state at every rendering
    pass.  The only OpenGL state variables that CHAI3D sets explicitly in a typical
    rendering pass are: \n\n

    * lighting is enabled (cWorld) \n
    * depth-testing is enabled (cWorld) \n
    * glColorMaterial is enabled and set to GL_AMBIENT_AND_DIFFUSE/GL_FRONT_AND_BACK (cWorld) \n
    * a perspective projection matrix is set up (cCamera) \n\n

    This adherence to the defaults is nice because it lets an application change an important
    piece of state globally and not worry about it getting changed by CHAI3D objects. \n\n

    It is expected that objects will "clean up after themselves" if they change
    any rendering state besides:\n\n

    * color (glColor) \n
    * material properties (glMaterial) \n
    * normals (glNormal) \n\n

    For example, if my object changes the rendering color, I don't need to set it back
    before returning, but if my object turns on vertex buffering, I should turn it
    off before returning.  Consequently if I care about the current color, I should
    set it up in my own render() function, because I shouldn't count on it being
    meaningful when my render() function is called. \n\n

    Necessary exceptions to these conventions include: \n\n

    * cLight will change the lighting state for his assigned GL_LIGHT \n
    * cCamera sets up relevant transformation matrices \n\n

    \param  a_options  Rendering options.
*/
//==============================================================================
void cGenericObject::render(cRenderOptions& a_options) 
{ 
}


//==============================================================================
/*!
    From the position of the tool, search for the nearest point located
    at the surface of the current object. Decide if the point is located inside
    or outside of the object.

    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//==============================================================================
void cGenericObject::computeLocalInteraction(const cVector3d& a_toolPos,
                                             const cVector3d& a_toolVel,
                                             const unsigned int a_IDN)
{
}


//==============================================================================
/*!
    Copy all material and texture properties from current generic object to 
    another. This function is typically called by copy() method 
    which duplicates an instance of a subclass of cGenericObject.


    \param  a_obj  Object to which properties are copied to
    \param  a_duplicateMaterialData  If __true__, then material data is duplicated, otherwise it it shared.
    \param  a_duplicateTextureData  If __true__, then texture data is duplicated, otherwise it shared.
*/
//==============================================================================
void cGenericObject::copyGenericProperties(cGenericObject* a_obj, 
                                           bool a_duplicateMaterialData,
                                           bool a_duplicateTextureData)
{
    // sanity check
    if (a_obj == NULL) { return; }

    // copy material
    if ((a_duplicateMaterialData) && ((m_material != NULL) || (m_material != &s_defaultMaterial)))
    {
        a_obj->m_material = m_material->copy();    
    }
    else
    {
        a_obj->m_material = m_material;
    }

    // copy texture
    if ((a_duplicateTextureData) && (m_texture != NULL))
    {
        a_obj->m_texture = m_texture->copy();    
    }
    else
    {
        a_obj->m_material = m_material;
    }

    // copy general properties and settings
    a_obj->m_useTextureMapping    = m_useTextureMapping;
    a_obj->m_useMaterialProperty  = m_useMaterialProperty;
    a_obj->m_useVertexColors      = m_useVertexColors;
    a_obj->m_useTransparency      = m_useTransparency;
    a_obj->m_cullingEnabled       = m_cullingEnabled;
}


//==============================================================================
/*!
    Render the scene graph starting at this object. This method is called
    for each object and optionally render the object itself, its reference frame
    and the collision and/or scenegraph trees. \n

    The object itself is rendered by calling render(), which should be defined
    for each subclass that has a graphical representation.  renderSceneGraph
    does not generally need to be over-ridden in subclasses. \n

    The a_options parameter is used to allow multiple rendering passes. 
    See CRenderOptionh.h for more information.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cGenericObject::renderSceneGraph(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    /////////////////////////////////////////////////////////////////////////
    // Initialize rendering
    /////////////////////////////////////////////////////////////////////////

    // convert the position and orientation of the object into a 4x4 matrix 
    // that is supported by openGL. This task only needs to be performed during the first
    // rendering pass
    if (a_options.m_storeObjectPositions)
    {
        m_frameGL.set(m_localPos, m_localRot);
    }

    // push object position/orientation on stack
    glPushMatrix();
    glMultMatrixd( (const double *)m_frameGL.getData() );

    // render if object is enabled
    if (m_enabled)
    {
        //--------------------------------------------------------------------------
        // Request for RESET
        //-----------------------------------------------------------------------
        if(a_options.m_resetDisplay)
        {
            // invalidate display list 
            invalidateDisplayList();

            // invalidate texture
            if (m_texture != NULL)
            {
                m_texture->markForUpdate();
            }
        }

        //-----------------------------------------------------------------------
        // Init
        //-----------------------------------------------------------------------

        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);

        //-----------------------------------------------------------------------
        // Render bounding box, frame, collision detector. (opaque components)
        //-----------------------------------------------------------------------
        if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options) && (!a_options.m_rendering_shadow))
        {
            // disable lighting
            glDisable(GL_LIGHTING);

            // render boundary box
            if (m_showBoundaryBox)
            {
                // set size on lines
                glLineWidth(1.0);

                // set color of boundary box
                glColor4fv(s_boundaryBoxColor.pColor());

                // draw box line
                cDrawWireBox(m_boundaryBoxMin(0) , m_boundaryBoxMax(0) ,
                             m_boundaryBoxMin(1) , m_boundaryBoxMax(1) ,
                             m_boundaryBoxMin(2) , m_boundaryBoxMax(2) );
            }

            // render collision tree
            if (m_showCollisionDetector && (m_collisionDetector != NULL))
            {
                m_collisionDetector->render(a_options);
            }

            // enable lighting
            glEnable(GL_LIGHTING);

            // render frame
            if (m_showFrame)
            {
                // set rendering properties
                glPolygonMode(GL_FRONT, GL_FILL);

                // draw frame
                cDrawFrame(m_frameSize, m_frameThicknessScale, true);
            }
        }

        //-----------------------------------------------------------------------
        // Render graphical representation of object
        //-----------------------------------------------------------------------
        if (m_showEnabled)
        {
            // set polygon and face mode
            glPolygonMode(GL_FRONT_AND_BACK, m_triangleMode);


            /////////////////////////////////////////////////////////////////////
            // CREATING SHADOW DEPTH MAP
            /////////////////////////////////////////////////////////////////////
            if (a_options.m_creating_shadow_map)
            {
                glEnable(GL_CULL_FACE);
                glCullFace(GL_FRONT);

                // render object
                render(a_options);
                glDisable(GL_CULL_FACE);
            }


            /////////////////////////////////////////////////////////////////////
            // SINGLE PASS RENDERING
            /////////////////////////////////////////////////////////////////////
            else if (a_options.m_single_pass_only)
            {
                if (m_cullingEnabled)
                {
                    glEnable(GL_CULL_FACE);
                    glCullFace(GL_BACK);
                }
                else
                {
                    glDisable(GL_CULL_FACE);
                }

                if (m_useTransparency)
                {
                    glEnable(GL_BLEND);
                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                    glDepthMask(GL_FALSE);
                }
                else
                {
                    glDisable(GL_BLEND);
                    glDepthMask(GL_TRUE);
                }

                // render object
                render(a_options);	

                // disable blending
                glDisable(GL_BLEND);
                glDepthMask(GL_TRUE);
            }


            /////////////////////////////////////////////////////////////////////
            // MULTI PASS RENDERING
            /////////////////////////////////////////////////////////////////////
            else
            {
                // opaque objects
                if (a_options.m_render_opaque_objects_only)
                {
                    if (m_cullingEnabled)
                    {
                        glEnable(GL_CULL_FACE);
                        glCullFace(GL_BACK);
                    }
                    else
                    {
                        glDisable(GL_CULL_FACE);
                    }

                    render(a_options);
                }

                // render transparent back triangles
                if (a_options.m_render_transparent_back_faces_only)
                {
                    if (m_useTransparency)
                    {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDepthMask(GL_FALSE);
                    }
                    else
                    {
                        glDisable(GL_BLEND);
                        glDepthMask(GL_TRUE);
                    }

                    glEnable(GL_CULL_FACE);
                    glCullFace(GL_FRONT);
                    
                    render(a_options);

                    // disable blending
                    glDisable(GL_BLEND);
                    glDepthMask(GL_TRUE);
                }

                // render transparent front triangles
                if (a_options.m_render_transparent_front_faces_only)
                {
                    if (m_useTransparency)
                    {
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        glDepthMask(GL_FALSE);
                    }
                    else
                    {
                        glDisable(GL_BLEND);
                        glDepthMask(GL_TRUE);
                    }

                    glEnable(GL_CULL_FACE);
                    glCullFace(GL_BACK);

                    render(a_options);
    
                    // disable blending
                    glDisable(GL_BLEND);
                    glDepthMask(GL_TRUE);
                }
            }
        }
    }

    // render children
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        m_children[i]->renderSceneGraph(a_options);
    }

    // pop current matrix
    glPopMatrix();

#endif
}


//==============================================================================
/*!
    Adjust the given segment such that it tests for intersection of the ray with
    objects at their previous positions at the last haptic loop so that collision
    detection will work in a dynamic environment.

    \param  a_segmentPointA  Start point of segment.
    \param  a_segmentPointAadjusted  Same segment, adjusted to be in local space.
*/
//==============================================================================
void cGenericObject::adjustCollisionSegment(cVector3d& a_segmentPointA,
                                       cVector3d& a_segmentPointAadjusted)
{
    // convert point from local to global coordinates by using
    // the previous object position and orientation
    cVector3d point = cAdd(m_globalPos, cMul(m_globalRot, a_segmentPointA));

    // compute the new position of the point based on
    // the new object position and orientation
    a_segmentPointAadjusted = cMul( cTranspose(m_prevGlobalRot), cSub(point, m_prevGlobalPos));
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
cVector3d cGenericObject::computeInteractions(const cVector3d& a_toolPos,
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

    // check if node is a ghost. If yes, then ignore call
    if (m_ghostEnabled) { return (cVector3d(0,0,0)); }

    // process current object if enabled
    if (m_enabled)
    {
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

                    interactionEvent = interactionEvent |
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

    // descend through the children
    vector<cGenericObject*>::iterator it;
    for (it = m_children.begin(); it < m_children.end(); it++)
    {
        cVector3d force = (*it)->computeInteractions(toolPosLocal,
                                                     toolVelLocal,
                                                     a_IDN,
                                                     a_interactions);
        localForce.add(force);
    }

    // convert the reaction force into my parent coordinates
    cVector3d m_globalForce = cMul(m_localRot, localForce);

    // return resulting force
    return (m_globalForce);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
