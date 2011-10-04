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
    \author    Dan Morris
    \version   2.0.0 $Rev: 270 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
#include "collisions/CGenericCollision.h"
#include <float.h>
//---------------------------------------------------------------------------
#include <vector>
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cGenericObject.

    \fn     cGenericObject::cGenericObject()
*/
//===========================================================================
cGenericObject::cGenericObject() : m_parent(NULL),
m_localPos(0.0, 0.0, 0.0), m_globalPos(0.0, 0.0, 0.0),
m_show(true), m_showFrame(false), m_frameSize(1.0), m_frameThicknessScale(2.0),
m_boundaryBoxMin(0.0, 0.0, 0.0), m_boundaryBoxMax(0.0, 0.0, 0.0),
m_showBox(false), m_boundaryBoxColor(0.5, 0.5, 0.0),
m_showTree(false), m_treeColor(0.5, 0.0, 0.0),
m_collisionDetector(NULL), m_showCollisionTree(false),
m_hapticEnabled(true),
m_tag(-1), m_userData(0)
{
    // initialize local position and orientation
    m_localRot.identity();

    // initialize global position and orientation
    m_globalRot.identity();

    // initialize openGL matrix with position vector and orientation matrix
    m_frameGL.set(m_globalPos, m_globalRot);

    // custom user information
    m_objectName[0] = '\0';

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

    // turn culling on by default
    m_cullingEnabled = true;

    // by default, if transparency is enabled, use the multi-pass approach
    m_useMultipassTransparency = true;

	// disable ghost setting
	m_ghostStatus = false;

    // no external parent defined
    m_externalParent = NULL;

    // by default, the object is the super parent of itself
    m_superParent = this;
}


//===========================================================================
/*!
    Destructor of cGenericObject.  This function deletes all children
    starting from this point in the scene graph, so if you have objects
    that shouldn't be deleted, be sure to remove them from the scene
    graph before deleting their parents.

    \fn       cGenericObject::~cGenericObject()
*/
//===========================================================================
cGenericObject::~cGenericObject()
{
    // Each of my children will remove their own collision detectors when
    // they get deleted, so I just delete my own right now...
    if (m_collisionDetector) deleteCollisionDetector(0);

    // delete all children
    deleteAllChildren();
}


//===========================================================================
/*!
    Sets a pointer to an external parent of the current object. Optionally
    propagating the change to children.
    A pointer to an external parent located outside of the scenegraph.
    This parameter can typically be used if you want to attach an
    generic object to some other object outside of CHAI3D of to an external
    representation such as a dynamics engine model. See the ODE examples
    to understand how a generic object can be attached to an ODE object.

    \fn     void cGenericObject::setExternalParent(cGenericType* a_externalParent,
                                                   const bool a_affectChildren)
    \param  a_externalParent  External parent.
    \param  a_affectChildren  If \b true, this message is passed to children.
*/
//===========================================================================
void cGenericObject::setExternalParent(cGenericType* a_externalParent, const bool a_affectChildren)
{
    // set external parent
    m_externalParent = a_externalParent;

    // apply change to children
    if (a_affectChildren == false) return;
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        cGenericObject* nextObject = m_children[i];
        nextObject->setExternalParent(a_externalParent, true);
    }
}


//===========================================================================
/*!
    Sets the super parent of the current object. Optionally propagating the
    change to children. A super parent points to another object generally
    located higher up in the scene graph. When a mesh is created,
    the super parent of its children will generally point towards
    the root of the mesh. This parameter is automatically set by the 3D object
    file loader.

    \fn     void cGenericObject::setSuperParent(cGenericObject* a_superParent,
                                                const bool a_affectChildren)
    \param  a_superParent  Super parent.
    \param  a_affectChildren  If \b true, this message is passed to children.
*/
//===========================================================================
void cGenericObject::setSuperParent(cGenericObject* a_superParent, const bool a_affectChildren)
{
    // set super parent
    m_superParent = a_superParent;

    // apply change to children
    if (a_affectChildren == false) return;
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        cGenericObject* nextObject = m_children[i];
        nextObject->setSuperParent(a_superParent, true);
    }
}


//===========================================================================
/*!
    It can be sometimes useful to add an object twice in a scenegraph to
	create for instance a reflexion of a model over a ground. In these
	cases there only one model for which all the interactions should be computed.
	The second one is only there for display purposes.
	By enabling the ghost status, we disable all collision detection, force
	interaction further down the scene graph. An example of this feature
	can be found in the example "ODE-cube".

    \fn       void cGenericObject::setAsGhost(bool a_ghostStatus)
*/
//===========================================================================
void cGenericObject::setAsGhost(bool a_ghostStatus)
{
	m_ghostStatus = a_ghostStatus;
}


//===========================================================================
/*!
    Non-uniform scale, optionally include children.  Not necessarily
    implemented in all subclasses.  Does nothing at the cGenericObject
    level; subclasses should scale themselves, then call the superclass
    method.

    \fn     void cGenericObject::scale(const cVector3d& a_scaleFactors,
            const bool a_includeChildren)
    \param    a_scaleFactors  Possibly non-uniform scale factors.
    \param    a_includeChildren  If \b true, this message is passed to children.
*/
//===========================================================================
void cGenericObject::scale(const cVector3d& a_scaleFactors, const bool a_includeChildren)
{
    // scale current object
    scaleObject(a_scaleFactors);

    // scale children
    if (a_includeChildren == false) return;
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        cGenericObject* nextObject = m_children[i];

        // scale the position of this child
        nextObject->m_localPos.elementMul(a_scaleFactors);
        nextObject->scale(a_scaleFactors, true);
    }
}


//===========================================================================
/*!
    Uniform scale, optionally include children.  Not necessarily
    implemented in all subclasses.  Does nothing at the cGenericObject
    level. Subclasses should scale themselves, then call the superclass
    method.

    \fn     void cGenericObject::scale(const double& a_scaleFactor,
            const bool a_includeChildren)
    \param    a_scaleFactor  Scale factor.
    \param    a_includeChildren  If \b true, this message is passed to children.
*/
//===========================================================================
void cGenericObject::scale(const double& a_scaleFactor, const bool a_includeChildren)
{
    scale(cVector3d(a_scaleFactor, a_scaleFactor, a_scaleFactor), a_includeChildren);
}


//===========================================================================
/*!
    Adds an object to the scene graph below this object.

    \fn       void cGenericObject::addChild(cGenericObject* a_object)
    \param    a_object  Object to be added to children list.
*/
//===========================================================================
void cGenericObject::addChild(cGenericObject* a_object)
{
    // update the child object's parent pointer
    a_object->m_parent = this;

    // add this child to my list of children
    m_children.push_back(a_object);
}


//===========================================================================
/*!
    Does this object have the specified object as a child?

    \fn       bool cGenericObject::containsChild(cGenericObject* a_object,
              bool a_includeChildren=false)
    \param    a_object  Object to search for.
    \param    a_includeChildren  Should we also search this object's descendants?
*/
//===========================================================================
bool cGenericObject::containsChild(cGenericObject* a_object, bool a_includeChildren)
{
	for (unsigned int i=0; i<m_children.size(); i++)
	{
		cGenericObject* nextObject = m_children[i];
		if (nextObject == a_object)
		{
			return (true);
		}

		if (a_includeChildren)
		{
			bool result = nextObject->containsChild(a_object,true);
			if (result)
			{
				return (true);
			}
		}
	}
	return (false);
}


//===========================================================================
/*!
    Removes an object from my list of children, without deleting the
    child object from memory.

    This method assigns the child object's parent point to null, so
    if you're moving someone around in your scene graph, make sure you
    call this function _before_ you add the child to another node in
    the scene graph.

    \fn       bool cGenericObject::removeChild(cGenericObject* a_object)
    \param    a_object  Object to be removed from my list of children.
    \return   Returns \b true if the specified object was found on my list 
              of children
*/
//===========================================================================
bool cGenericObject::removeChild(cGenericObject* a_object)
{
    std::vector<cGenericObject*>::iterator nextObject;

    for(nextObject = m_children.begin();
        nextObject != m_children.end();
        nextObject++ )
	{
        // Did we find the object we're trying to delete?
        if ((*nextObject) == a_object)
        {
            // he doesn't have a parent any more
            a_object->m_parent = NULL;

            // remove this object from my list of children
            m_children.erase(nextObject);

            // return success
            return (true);
        }

    }

    // operation failed
    return (false);
}


//===========================================================================
/*!
    Removes an object from my list of children, and deletes the
    child object from memory.

    \fn       bool cGenericObject::deleteChild(cGenericObject* a_object)
    \param    a_object  Object to be removed from my list of children 
                        and deleted.
    \return   Returns \b true if the specified object was found on my list 
              of children.
*/
//===========================================================================
bool cGenericObject::deleteChild(cGenericObject* a_object)
{
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


//===========================================================================
/*!
    Clear all objects from my list of children, without deleting them.

    \fn     void cGenericObject::clearAllChildren()
*/
//===========================================================================
void cGenericObject::clearAllChildren()
{
    // clear children list
    m_children.clear();
}


//===========================================================================
/*!
    Delete and clear all objects from my list of children.

    \fn     void cGenericObject::deleteAllChildren()
*/
//===========================================================================
void cGenericObject::deleteAllChildren()
{
    // delete all children
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        cGenericObject* nextObject = m_children[i];
        delete (nextObject);
    }

    // clear my list of children
    m_children.clear();
}


//===========================================================================
/*!
	Return my total number of descendants, optionally including this object.
	\fn     unsigned int cGenericObject::getNumDescendants(bool a_includeCurrentObject);
	\param  a_includeCurrentObject  Should I include myself in the count?
    \return Returns the number of descendants found.
*/
//===========================================================================
unsigned int cGenericObject::getNumDescendants(bool a_includeCurrentObject)
{
	unsigned int numDescendants = a_includeCurrentObject?1:0;

	for (unsigned int i=0; i<m_children.size(); i++)
	{
		cGenericObject* nextObject = m_children[i];
		numDescendants += nextObject->getNumDescendants(true);
	}

	return numDescendants;
}


//===========================================================================
/*!
    Fill this list with all of my descendants.  The current object is optionally
    included in this list.  Does not clear the list before appending to it.

    \fn     void cGenericObject::enumerateChildren(std::list<cGenericObject*>& a_childList,
            bool a_includeCurrentObject=true);
    \param  a_childList  The list to write our enumerated results to.
    \param  a_includeCurrentObject  Should I include myself on the list?
*/
//===========================================================================
void cGenericObject::enumerateChildren(std::list<cGenericObject*>& a_childList,
                                       bool a_includeCurrentObject)
{
    if (a_includeCurrentObject) a_childList.push_back(this);

    for (unsigned int i=0; i<m_children.size(); i++)
    {
        cGenericObject* nextObject = m_children[i];
        nextObject->enumerateChildren(a_childList,true);
    }
}


//===========================================================================
/*!
    Adds a haptic effect to the current object

    \fn       void cGenericObject::addEffect(cGenericEffect* a_effect)
    \param    a_effect  Effect to be added to effects list.
*/
//===========================================================================
void cGenericObject::addEffect(cGenericEffect* a_effect)
{
    // update the effect object's parent pointer
    a_effect->m_parent = this;

    // add this child to my list of children
    m_effects.push_back(a_effect);
}


//===========================================================================
/*!
    Descend through child objects to compute interactions for all
    cGenericEffect classes defined for each object.

    \fn       cVector3d cGenericObject::computeInteractions(const cVector3d& a_toolPos,
                                              const cVector3d& a_toolVel,
                                              const unsigned int a_IDN,
                                              cInteractionRecorder& a_interactions,
                                              cInteractionSettings& a_interactionSettings)
    \param    a_toolPos  Current position of tool.
    \param    a_toolVel  Current position of tool.
    \param    a_IDN  Identification number of the force algorithm.
    \param    a_interactions  List of recorded interactions.
    \param    a_interactionSettings  Settings of the interaction recorder.
    \return   Return resulting interaction forces.

*/
//===========================================================================
cVector3d cGenericObject::computeInteractions(const cVector3d& a_toolPos,
                                              const cVector3d& a_toolVel,
                                              const unsigned int a_IDN,
                                              cInteractionRecorder& a_interactions,
                                              cInteractionSettings& a_interactionSettings)
{
	// check if node is a ghost. If yes, then ignore call
	if (m_ghostStatus) { return (cVector3d(0,0,0)); }

    cMatrix3d localRotTrans;
    m_localRot.transr(localRotTrans);

    // compute local position of tool and velocity vector
    cVector3d toolPosLocal = cMul(localRotTrans, cSub(a_toolPos, m_localPos));

    // compute interaction between tool and current object
    cVector3d toolVelLocal = cMul(localRotTrans, a_toolVel);

    // compute local interaction with current object
    computeLocalInteraction(toolPosLocal,
                            toolVelLocal,
                            a_IDN);

    // compute forces based on the effects programmed for this object
    cVector3d localForce(0,0,0);

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
            newInteractionEvent.m_localSurfacePos = m_interactionProjectedPoint;
            newInteractionEvent.m_localForce = localForce;
            a_interactions.m_interactions.push_back(newInteractionEvent);
        }

        // compute any other force interactions
        cVector3d force = computeOtherInteractions(toolPosLocal,
                                                   toolVelLocal,
                                                   a_IDN,
                                                   a_interactions,
                                                   a_interactionSettings);

        localForce.add(force);
    }

    // descend through the children
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        cGenericObject *nextObject = m_children[i];

        cVector3d force = nextObject->computeInteractions(toolPosLocal,
                                                          toolVelLocal,
                                                          a_IDN,
                                                          a_interactions,
                                                          a_interactionSettings);
        localForce.add(force);
    }

    // convert the reaction force into my parent coordinates
    cVector3d m_globalForce = cMul(m_localRot, localForce);

    // return resulting force
    return (m_globalForce);
}


//===========================================================================
/*!
    From the position of the tool, search for the nearest point located
    at the surface of the current object. Decide if the point is located inside
    or outside of the object.

    \fn     void cGenericObject::computeLocalInteraction(const cVector3d& a_toolPos,
                                                         const cVector3d& a_toolVel,
                                                         const unsigned int a_IDN)
    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void cGenericObject::computeLocalInteraction(const cVector3d& a_toolPos,
                                             const cVector3d& a_toolVel,
                                             const unsigned int a_IDN)
{
    // no surface limits defined, so we simply return the same position of the tool
    m_interactionProjectedPoint.copyfrom(a_toolPos);

    // no surface limits, so we consider that we are inside the object
    m_interactionInside = true;
}


//===========================================================================
/*!
    Translate this object by a specified offset.

    \fn     void cGenericObject::translate(const cVector3d& a_translation)
    \param  a_translation  Translation offset.
*/
//===========================================================================
void cGenericObject::translate(const cVector3d& a_translation)
{
    // apply the translation to this object
    cVector3d new_position = cAdd(m_localPos,a_translation);
    setPos(new_position);
}


//===========================================================================
/*!
    Translate an object by a specified offset.

    \fn     void cGenericObject::translate(const double a_x, const double a_y,
            const double a_z)
    \param  a_x  Translation component X.
    \param  a_y  Translation component Y.
    \param  a_z  Translation component Z.
*/
//===========================================================================
void cGenericObject::translate(const double a_x, const double a_y, const double a_z)
{
    translate(cVector3d(a_x,a_y,a_z));
}


//===========================================================================
/*!
    Rotate this object by multiplying with a specified rotation matrix.

    \fn     void cGenericObject::rotate(const cMatrix3d& a_rotation)
    \param  a_rotation  Rotation matrix.
*/
//===========================================================================
void cGenericObject::rotate(const cMatrix3d& a_rotation)
{
    cMatrix3d new_rot = m_localRot;
    new_rot.mul(a_rotation);
    setRot(new_rot);
}


//===========================================================================
/*!
    Rotate this object around axis a_axis by angle a_angle (radians).  a_axis
    is not normalized, so unless you know what you're doing, normalize your
    axis before supplying it to this function.

    \fn     void cGenericObject::rotate(const cVector3d& a_axis,
            const double a_angle)
    \param  a_axis   Rotation axis.
    \param  a_angle  Rotation angle in radians.
*/
//===========================================================================
void cGenericObject::rotate(const cVector3d& a_axis, const double a_angle)
{
    cMatrix3d new_rot = m_localRot;
    new_rot.rotate(a_axis, a_angle);
    setRot(new_rot);

}


//===========================================================================
/*!
    Compute globalPos and globalRot given the localPos and localRot
    of this object and its parents.  Optionally propagates to children. \n

    If \e a_frameOnly is set to \b false, additional global positions such as
    vertex positions are computed (which can be time-consuming). \n

    Call this method any time you've moved an object and will need to access
    to globalPos and globalRot in this object or its children.  For performance
    reasons, these values are not kept up-to-date by default, since almost
    all operations use local positions and rotations.? 

    \fn     void cGenericObject::computeGlobalPositions(const bool a_frameOnly,
            const cVector3d& a_globalPos, const cMatrix3d& a_globalRot)
    \param  a_frameOnly  If \b true then only the global frame is computed
    \param  a_globalPos  Global position of my parent.
    \param  a_globalRot  Global rotation matrix of my parent.
*/
//===========================================================================
void cGenericObject::computeGlobalPositions(const bool a_frameOnly,
     const cVector3d& a_globalPos, const cMatrix3d& a_globalRot)
{
	// check if node is a ghost. If yes, then ignore call
	if (m_ghostStatus) { return; }

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
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        m_children[i]->computeGlobalPositions(a_frameOnly, m_globalPos, m_globalRot);
    }
}


//===========================================================================
/*!
    Compute globalPos and globalRot for this object only, by recursively
    climbing up the scene graph tree until the root is reached.

    If \e a_frameOnly is set to \b false, additional global positions such as
    vertex positions are computed.

    \fn     void cGenericObject::computeGlobalCurrentObjectOnly(
            const bool a_frameOnly)
    \param  a_frameOnly  If \b true then only the global frame is computed.
*/
//===========================================================================
void cGenericObject::computeGlobalCurrentObjectOnly(const bool a_frameOnly)
{
    cMatrix3d globalRot;
    cVector3d globalPos;
    globalRot.identity();
    globalPos.zero();

    // get a pointer to current object
    cGenericObject *curObject = this;

    // walk up the scene graph until we reach the root, updating
    // my global position and rotation at each step
    do {
        curObject->getRot().mul(globalPos);
        globalPos.add(curObject->getPos());
        cMatrix3d rot;
        curObject->getRot().mulr(globalRot, rot);
        rot.copyto(globalRot);
        curObject = curObject->getParent();
    } while (curObject != NULL);

    // update values
    m_globalPos = globalPos;
    m_globalRot = globalRot;

    // update any positions within the current object that need to be
    // updated (e.g. vertex positions)
    updateGlobalPositions(a_frameOnly);
}


//===========================================================================
/*!
    Set the tag for this object and - optionally - for my children.

    \fn     void cGenericObject::setTag(const int a_tag, 
                                        const bool a_affectChildren=0)
    \param  a_tag   The tag we'll assign to this object.
    \param  a_affectChildren  If \b true, the operation propagates through 
                              the scene graph.
*/
//===========================================================================
void cGenericObject::setTag(const int a_tag, const bool a_affectChildren)
{
    m_tag = a_tag;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setTag(a_tag,true);
        }
    }
}


//===========================================================================
/*!
    Set the name for this object and - optionally - for my children.

    \fn     void cGenericObject::setName(const char* a_name, 
                                         const bool a_affectChildren=0)
    \param  a_name   The name we'll assign to this object.
    \param  a_affectChildren  If \b true, the operation propagates through 
                              the scene graph.
*/
//===========================================================================
void cGenericObject::setName(const char* a_name, const bool a_affectChildren)
{
    strncpy(m_objectName, a_name, CHAI_SIZE_NAME);
    m_objectName[CHAI_SIZE_NAME-1]='\0';

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
        m_children[i]->setName(a_name,true);
        }
    }
}


//===========================================================================
/*!
    Set the m_userData pointer for this object and - optionally - for my children.

    \fn     void cGenericObject::setUserData(void* a_data, 
                                             const bool a_affectChildren=0)
    \param  a_data   The pointer to which we will set m_userData
    \param  a_affectChildren  If \b true, the operation propagates.
                              through the scene graph.
*/
//===========================================================================
void cGenericObject::setUserData(void* a_data, const bool a_affectChildren)
{
    m_userData = a_data;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setUserData(a_data,true);
        }
    }
}

//===========================================================================
/*!
    Users should call this function when it's necessary to re-initialize the OpenGL
    context; e.g. re-initialize textures and display lists.  Subclasses should
    perform whatever re-initialization they need to do. \n

    Note that this is not an event CHAI can easily detect, so it's up to
    the application-writer to call this function on the root of the scenegraph.

    \fn     void cGenericObject::onDisplayReset(const bool a_affectChildren)
    \param  a_affectChildren  If \b true, the operation propagates through 
                              the scene graph.
*/
//===========================================================================
void cGenericObject::onDisplayReset(const bool a_affectChildren)
{
    // Since I don't have any display context to update, I don't do anything here...

    // We _don't_ call this method on the current object, which allows subclasses
    // to do their business in this method, then call the cGenericObject version
    // to propagate the call through the scene graph

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->onDisplayReset(true);
        }
    }
}


//===========================================================================
/*!
    This call tells an object that you're not going to modify him any more.
    For example, a mesh-like object might optimize his vertex arrangement
    when he gets this call.  Always optional; just for performance...

    \fn     void cGenericObject::finalize(const bool a_affectChildren)
    \param  a_affectChildren  If \b true, the operation propagates through 
                              the scene graph.
*/
//===========================================================================
void cGenericObject::finalize(const bool a_affectChildren)
{
    // We _don't_ call this method on the current object, which allows subclasses
    // to do their business in this method, then call the cGenericObject version
    // to propagate the call through the scene graph

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->finalize(true);
        }
    }
}


//===========================================================================
/*!
    This call tells an object that you may modify his contents.  See
    finalize() for more information.

    \fn     void cGenericObject::unfinalize(const bool a_affectChildren)
    \param  a_affectChildren  If \b true, the operation propagates through 
                              the scene graph.
*/
//===========================================================================
void cGenericObject::unfinalize(const bool a_affectChildren)
{
    // We _don't_ call this method on the current object, which allows subclasses
    // to do their business in this method, then call the cGenericObject version
    // to propagate the call through the scene graph

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->unfinalize(true);
        }
    }
}

//===========================================================================
/*!
    Show or hide this object. \n

    If \e a_affectChildren is set to \b true then all children are updated
    with the new value.

    \fn     void cGenericObject::setShowEnabled(const bool a_show,
                                                const bool a_affectChildren)
    \param  a_show  If \b true object shape is visible.
    \param  a_affectChildren  If \b true all children are updated.
*/
//===========================================================================
void cGenericObject::setShowEnabled(const bool a_show, const bool a_affectChildren)
{
	// update current object
    m_show = a_show;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setShowEnabled(a_show, true);
        }
    }
}


//===========================================================================
/*!
    Allow or disallow the object to be felt (when visible). \n

    If \e a_affectChildren is set to \b true then all children are updated
    with the new value.

    \fn     void cGenericObject::setHapticEnabled(const bool a_hapticEnabled,
            const bool a_affectChildren)
    \param  a_hapticEnabled  If \b true object can be felt when visible.
    \param  a_affectChildren  If \b true all children are updated.
*/
//===========================================================================
void cGenericObject::setHapticEnabled(const bool a_hapticEnabled, const bool a_affectChildren)
{
    // update current object
    m_hapticEnabled = a_hapticEnabled;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setHapticEnabled(a_hapticEnabled, true);
        }
    }
}


//===========================================================================
/*!
    Show or hide the set of arrows that represent this object's reference frame.

    If \e a_affectChildren is set to \b true then all children are updated
    with the new value.

    \fn     void cGenericObject::setShowFrame(const bool a_showFrame,
            const bool a_affectChildren)
    \param  a_showFrame  If \b true then frame is displayed.
    \param  a_affectChildren  If \b true all children are updated.
*/
//===========================================================================
void cGenericObject::setShowFrame(const bool a_showFrame, const bool a_affectChildren)
{
    // update current object
    m_showFrame = a_showFrame;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setShowFrame(a_showFrame, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
    Set the display size of the arrows representing my reference frame.
    The size corresponds to the length of each displayed axis (X-Y-Z). \n

    If \e a_affectChildren is set to \b true then all children are updated
    with the new value.


    \fn     bool cGenericObject::setFrameSize(const double a_size,
            const double a_thickness, const bool a_affectChildren)
    \param  a_size            Length of graphical representation of frame.
    \param  a_thickness       Thickness of graphical representation of frame.
    \param  a_affectChildren  If \b true all children are updated.
*/
//===========================================================================
bool cGenericObject::setFrameSize(const double a_size, const double a_thickness,
                                  const bool a_affectChildren)
{
    // check value of size
    if (a_size <= 0) { return (false); }

    // update current object
    m_frameSize = a_size;
    m_frameThicknessScale = a_thickness;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setFrameSize(a_size, a_thickness, a_affectChildren);
        }
    }

    // operation succeeded
    return (true);
}


//===========================================================================
/*!
    Show or hide the graphic representation of the scene graph at this
    node. \n

    If \e a_affectChildren is set to \b true then all children are updated
    with the new value.

    \fn     void cGenericObject::setShowTree(const bool a_showTree,
            const bool a_affectChildren)
    \param  a_showTree  If \b true then tree is displayed.
    \param  a_affectChildren  If \b true all children are updated.
*/
//===========================================================================
void cGenericObject::setShowTree(const bool a_showTree, const bool a_affectChildren)
{
    // update current object
    m_showTree = a_showTree;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setShowTree(a_showTree, true);
        }
    }
}


//===========================================================================
/*!
    Set the color of the graphic representation of the scene graph at
    this node. \n

    If \e a_affectChildren is set to \b true then all children are updated
    with the new value.

    \fn     void cGenericObject::setTreeColor(const cColorf& a_treeColor, 
                                              const bool a_affectChildren)
    \param  a_treeColor  Color of tree.
    \param  a_affectChildren  If \b true all children are updated.
*/
//===========================================================================
void cGenericObject::setTreeColor(const cColorf& a_treeColor, const bool a_affectChildren)
{
    // update current object
    m_treeColor = a_treeColor;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setTreeColor(a_treeColor, true);
        }
    }
}


//===========================================================================
/*!
    Show or hide the graphic representation of the boundary box of 
    this object. \n

    If \e a_affectChildren is set to \b true then all children are updated
    with the new value.

    \fn     void cGenericObject::setShowBox(const bool a_showBox,
            const bool a_affectChildren)
    \param  a_showBox  If \b true, boundary box is displayed.
    \param  a_affectChildren  If \b true all children are updated.
*/
//===========================================================================
void cGenericObject::setShowBox(const bool a_showBox, const bool a_affectChildren)
{
    // update current object
    m_showBox = a_showBox;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setShowBox(a_showBox, true);
        }
    }
}


//===========================================================================
/*!
    Set the color of the graphic representation of the boundary boundary 
    box of this object.

    \fn     void cGenericObject::setBoxColor(const cColorf& a_boxColor, 
                                             const bool a_affectChildren)
    \param  a_boxColor  Color of boundary box.
    \param  a_affectChildren  If \b true all children are updated.
*/
//===========================================================================
void cGenericObject::setBoxColor(const cColorf& a_boxColor, const bool a_affectChildren)
{
    // update current object
    m_boundaryBoxColor = a_boxColor;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setBoxColor(a_boxColor, true);
        }
    }
}


//===========================================================================
/*!
    Show or hide the graphic representation of the collision tree at 
    this node. \n

    If \e a_affectChildren is set to \b true then all children are updated
    with the new value.

    \fn     void cGenericObject::setShowCollisionTree(const bool a_showCollisionTree,
            const bool a_affectChildren)
    \param  a_showCollisionTree If \b true, small normals are rendered graphicaly.
    \param  a_affectChildren  If \b true all children are updated.
*/
//===========================================================================
void cGenericObject::setShowCollisionTree(const bool a_showCollisionTree, const bool a_affectChildren)
{
    // update current object
    m_showCollisionTree = a_showCollisionTree;

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setShowCollisionTree(a_showCollisionTree, true);
        }
    }
}


//===========================================================================
/*!
     Enables or disables backface culling (rendering in GL is much faster
     with culling on).

     \fn       void cGenericObject::setUseCulling(const bool a_useCulling, 
                                                  const bool a_affectChildren)
     \param    a_useCulling  If \b true, backfaces are culled.
     \param    a_affectChildren  If \b true, this operation is propagated 
                                 to my children.
*/
//===========================================================================
void cGenericObject::setUseCulling(const bool a_useCulling, const bool a_affectChildren)
{
    // apply changes to this object
    m_cullingEnabled = a_useCulling;

    // propagate changes to children
    if (a_affectChildren)
    {
        unsigned int i, numItems;
        numItems = m_children.size();
        for (i=0; i<numItems; i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setUseCulling(a_useCulling, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
     Enable or disable wireframe rendering, optionally propagating the
     operation to my children.

     \fn        void cGenericObject::setWireMode(const bool a_showWireMode,
                                                 const bool a_affectChildren)
     \param     a_showWireMode  If \b true, wireframe mode is used.
     \param     a_affectChildren  If \b true, then children are also updated.
*/
//===========================================================================
void cGenericObject::setWireMode(const bool a_showWireMode, const bool a_affectChildren)
{
    // update changes to object
    if (a_showWireMode)  { m_triangleMode = GL_LINE; }
    else { m_triangleMode = GL_FILL; }

    // update changes to children
    if (a_affectChildren)
    {
        unsigned int i, numItems;
        numItems = m_children.size();
        for (i=0; i<numItems; i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setWireMode(a_showWireMode, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
     Set the alpha value at each vertex, in all of my material colors,
     optionally propagating the operation to my children. \n

     Using the 'apply to textures' option causes the actual texture
     alpha values to be over-written in my texture, if it exists. \n

     [Editor's note: the 'apply to textures' option is not currently
     enabled, since (a) it's a silly way to control transparency
     and (b) not all textures have an alpha channel.]

     \fn        void cGenericObject::setTransparencyLevel(const float a_level,
                const bool a_applyToTextures, const bool a_affectChildren)
     \param     a_level  Level of transparency ranging from 0.0 to 1.0.
     \param     a_applyToTextures  If \b true, then apply changes to texture.
     \param     a_affectChildren  If \b true, then children also modified.
*/
//===========================================================================
void cGenericObject::setTransparencyLevel(const float a_level, const bool a_applyToTextures,
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
    m_material.setTransparencyLevel(a_level);

    // apply changes to texture if required
    /*
    GLubyte level = GLubyte(255.0f * a_level);
    if (a_applyToTextures && (m_texture != NULL))
    {
       // m_texture->setTransparency(a_level);
    }
    */

    // propagate the operation to my children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setTransparencyLevel(a_level,
                                             a_applyToTextures,
                                             a_affectChildren);
        }
    }
}


//===========================================================================
/*!
     Enable or disable the use of per-vertex color information of when rendering
     the mesh.

     \fn       void cGenericObject::setUseVertexColors(const bool a_useColors, 
                                                       const bool a_affectChildren)
     \param    a_useColors   If \b true, then vertex color information is applied.
     \param    a_affectChildren  If \b true, then children are also modified.
*/
//===========================================================================
void cGenericObject::setUseVertexColors(const bool a_useColors, const bool a_affectChildren)
{
    // update changes to object
    m_useVertexColors = a_useColors;

    // update changes to children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setUseVertexColors(a_useColors, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
     Specify whether transparency should be enabled.  Note that this
     does not affect the transparency _mode_, which controls the use
     of simple transparency vs. multipass transparency.

     \fn       void cGenericObject::setUseTransparency(const bool a_useTransparency,
                                                       const bool a_affectChildren)
     \param    a_useTransparency   If \b true, transparency is enabled.
     \param    a_affectChildren    If \b true, then children are also modified.
*/
//===========================================================================
void cGenericObject::setUseTransparency(const bool a_useTransparency,
                                        const bool a_affectChildren)
{
    // update changes to object
    m_useTransparency = a_useTransparency;

    // propagate changes to my children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setUseTransparency(a_useTransparency, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
     Specify whether transparent rendering should use two passes (looks better)
     or one pass (runs faster).  See cCamera for more information about
     multipass transparency.

     \fn       void cGenericObject::setTransparencyRenderMode(const bool a_useMultipassTransparency,
                                                              const bool a_affectChildren)
     \param    a_useMultipassTransparency   If \b true, this mesh uses multipass rendering.
     \param    a_affectChildren  If \b true, then children are also modified.
*/
//===========================================================================
void cGenericObject::setTransparencyRenderMode(const bool a_useMultipassTransparency,
                                           const bool a_affectChildren)
{
    // update changes to object
    m_useMultipassTransparency = a_useMultipassTransparency;

    // propagate changes to my children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setTransparencyRenderMode(a_useMultipassTransparency, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
     Enable or disable the use of material properties.

     \fn        void cGenericObject::setUseMaterial(const bool a_useMaterial,
                                                    const bool a_affectChildren)
     \param     a_useMaterial If \b true, then material properties are used for rendering.
     \param     a_affectChildren  If \b true, then children are also modified.
*/
//===========================================================================
void cGenericObject::setUseMaterial(const bool a_useMaterial, const bool a_affectChildren)
{
    // update changes to object
    m_useMaterialProperty = a_useMaterial;

    // propagate changes to my children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setUseMaterial(a_useMaterial, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
     Set the static and dynamic friction for this mesh, possibly recursively 
     affecting children.

     \fn        void cGenericObject::setFriction(double a_staticFriction, 
                                                 double a_dynamicFriction, 
                                                 const bool a_affectChildren=0)
     \param     a_staticFriction  The static friction to apply to this object.
     \param     a_dynamicFriction  The dynamic friction to apply to this object.
     \param     a_affectChildren  If \b true, then children are also modified.
*/
//===========================================================================
void cGenericObject::setFriction(double a_staticFriction, double a_dynamicFriction, const bool a_affectChildren)
{
    m_material.setStaticFriction(a_staticFriction);
    m_material.setDynamicFriction(a_dynamicFriction);

    // propagate changes to children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setFriction(a_staticFriction,a_dynamicFriction,a_affectChildren);
        }
    }
}


//===========================================================================
/*!
     Set the haptic stiffness for this mesh, possibly recursively 
     affecting children.

     \fn        void cGenericObject::setStiffness(double a_stiffness, 
                                                  const bool a_affectChildren=0);
     \param     a_stiffness  The stiffness to apply to this object.
     \param     a_affectChildren  If \b true, then children are also modified.

*/
//===========================================================================
void cGenericObject::setStiffness(double a_stiffness, const bool a_affectChildren)
{
    m_material.setStiffness(a_stiffness);

    // propagate changes to children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setStiffness(a_stiffness, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
    Set the current texture for this mesh, possibly recursively affecting 
    children. \n

    Note that this does not affect whether texturing is enabled; it sets
    the texture that will be rendered _if_ texturing is enabled.  Call
    useTexture to enable / disable texturing.

    \fn        void cGenericObject::setTexture(cTexture2D* a_texture,
                                               const bool a_affectChildren)
    \param     a_texture  The texture to apply to this object.
    \param     a_affectChildren  If \b true, then children are also modified.
*/
//===========================================================================
void cGenericObject::setTexture(cTexture2D* a_texture, const bool a_affectChildren)
{
    m_texture = a_texture;

    // propagate changes to children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setTexture(a_texture, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
    Set the current material for this mesh, possibly recursively affecting children.
    If parameter a_applyPhysicalParmetersOnly is set to \b true then only
    the physical properties are modified. This is extremely useful if the user
    loads a 3d object file into a mesh and then applies some haptic properties
    which are also defined in the cMaterial class. \n

    Note that this does not affect whether material rendering is enabled;
    it sets the maetrial that will be rendered _if_ material rendering is
    enabled.  Call useMaterial to enable / disable material rendering.

    \fn     void cGenericObject::setMaterial(cMaterial& a_mat,
                                             const bool a_affectChildren,
                                             const bool a_applyPhysicalParmetersOnly)
    \param  a_mat The material to apply to this object
    \param  a_affectChildren  If \b true, then children are also modified.
    \param  a_applyPhysicalParmetersOnly  If \b true, then only physical properties 
                                          are applied
*/
//===========================================================================
void cGenericObject::setMaterial(cMaterial& a_mat,
                                 const bool a_affectChildren,
                                 const bool a_applyPhysicalParmetersOnly)
{
    if (a_applyPhysicalParmetersOnly)
    {
        m_material.setDynamicFriction(a_mat.getDynamicFriction());
        m_material.setMagnetMaxDistance(a_mat.getMagnetMaxDistance());
        m_material.setMagnetMaxForce(a_mat.getMagnetMaxForce());
        m_material.setStaticFriction(a_mat.getStaticFriction());
        m_material.setStickSlipForceMax(a_mat.getStickSlipForceMax());
        m_material.setStickSlipStiffness(a_mat.getStickSlipStiffness());
        m_material.setVibrationAmplitude(a_mat.getVibrationAmplitude());
        m_material.setVibrationFrequency(a_mat.getVibrationFrequency());
        m_material.setViscosity(a_mat.getViscosity());
    }
    else
    {
        m_material = a_mat;
    }

    // propagate changes to children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setMaterial(a_mat, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
    Enable or disable texture-mapping, possibly recursively affecting 
    children.

    \fn     void cGenericObject::setUseTexture(const bool a_useTexture,
                                               const bool a_affectChildren)
    \param  a_useTexture If \b true, then texture mapping is used.
    \param  a_affectChildren  If \b true, then children are also modified.
*/
//===========================================================================
void cGenericObject::setUseTexture(const bool a_useTexture, const bool a_affectChildren)
{
    m_useTextureMapping = a_useTexture;

    // propagate changes to children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            cGenericObject *nextObject = m_children[i];
            nextObject->setUseTexture(a_useTexture, a_affectChildren);
        }
    }
}


//===========================================================================
/*!
    Delete any existing collision detector and set the current cd to null.
    It's fine for an object to have a null collision detector (that's the
    default for a new object, in fact), it just means that no collisions 
    will be found.

    \fn     void cGenericObject::deleteCollisionDetector(const bool a_affectChildren)
    \param  a_affectChildren  If true, all my children's cd's are also deleted.
*/
//===========================================================================
void cGenericObject::deleteCollisionDetector(const bool a_affectChildren)
{

    if (m_collisionDetector)
    {
        delete m_collisionDetector;
        m_collisionDetector = 0;
    }

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->deleteCollisionDetector(true);
        }
    }
}


//===========================================================================
/*!
    Set the rendering properties for the graphic representation of collision 
    detection tree at this node. \n

    If \e a_affectChildren is set to \b true then all children are updated
    with the new values.

    \fn     void cGenericObject::setCollisionDetectorProperties(
            unsigned int a_displayDepth, cColorf& a_color, 
            const bool a_affectChildren=false)
    \param  a_color  Color used to render collision detector.
    \param  a_displayDepth  Indicated which depth of collision tree needs to be displayed
                            (see cGenericCollision).
    \param  a_affectChildren  If \b true, all children are updated.
*/
//===========================================================================
void cGenericObject::setCollisionDetectorProperties(unsigned int a_displayDepth,
     cColorf& a_color, const bool a_affectChildren)
{
    // update current collision detector
    if (m_collisionDetector != NULL)
    {
        m_collisionDetector->m_material.m_ambient.set(a_color.getR(), a_color.getG(),
                                                      a_color.getB(), a_color.getA());
        m_collisionDetector->setDisplayDepth(a_displayDepth);
    }

    // update children
    if (a_affectChildren)
    {
        for (unsigned int i=0; i<m_children.size(); i++)
        {
            m_children[i]->setCollisionDetectorProperties(a_displayDepth,
            a_color, a_affectChildren);
        }
    }
}


/*!
    We need a constant to determine if an object has already been assigned
    a 'real' bounding box
*/
#define BOUNDARY_BOX_EPSILON 1e-15

//===========================================================================
/*!
    Compute the bounding box of this object and (optionally) its children. \n

    If parameter \e a_includeChildren is set to \b true then each object's
    bounding box covers its own volume and the volume of its children. \n

    Note that regardless of this parameter's value, this operation propagates
    down the scene graph.

    \fn     void cGenericObject::computeBoundaryBox(const bool a_includeChildren=true)
    \param  a_includeChildren  If \b true, then children are included.
*/
//===========================================================================
void cGenericObject::computeBoundaryBox(const bool a_includeChildren)
{
	// check if node is a ghost. If yes, then ignore call
	if (m_ghostStatus) { return; }

    // compute the bounding box of this object
    updateBoundaryBox();

    if (a_includeChildren == false) return;

    unsigned int n = m_children.size();

    // compute the bounding box of all my children
    for (unsigned int i=0; i<n; i++)
    {
        m_children[i]->computeBoundaryBox(a_includeChildren);

        // see if this child has a _valid_ boundary box
        bool child_box_valid = (
          fabs(cDistance(m_children[i]->getBoundaryMax(),
                          m_children[i]->getBoundaryMin())) >
                        BOUNDARY_BOX_EPSILON
          );

        // don't include invalid boxes in my bounding box
        if (child_box_valid == 0) continue;

        // get position and rotation of child frame
        cMatrix3d rot = m_children[i]->getRot();
        cVector3d pos = m_children[i]->getPos();
        cVector3d V;

        // enumerate each corner of the child's bounding box
        int xshifts[8] = {0,0,0,0,1,1,1,1};
        int yshifts[8] = {0,0,1,1,0,0,1,1};
        int zshifts[8] = {0,1,0,1,0,1,0,1};

        cVector3d childBoxMin, childBoxMax;
        childBoxMin.set( DBL_MAX,  DBL_MAX,  DBL_MAX);
        childBoxMax.set(-DBL_MAX, -DBL_MAX, -DBL_MAX);

        cVector3d localmin = m_children[i]->getBoundaryMin();
        cVector3d localmax = m_children[i]->getBoundaryMax();

        // for each corner...
        for(int corner=0; corner<8; corner++)
        {
            // grab this corner of the child's bounding box
            cVector3d cornerLocation_child;
            cornerLocation_child.x = (xshifts[corner]) ? localmin.x : localmax.x;
            cornerLocation_child.y = (yshifts[corner]) ? localmin.y : localmax.y;
            cornerLocation_child.z = (zshifts[corner]) ? localmin.z : localmax.z;

            // convert this point into the parent reference frame
            cVector3d cornerLocation_parent;
            rot.mulr(cornerLocation_child, cornerLocation_parent);
            cornerLocation_parent.add(pos);

            // is this a max or min on any axis?
            for(int k=0; k<3; k++)
            {
                if (cornerLocation_parent[k] < childBoxMin[k]) childBoxMin[k] = cornerLocation_parent[k];
                if (cornerLocation_parent[k] > childBoxMax[k]) childBoxMax[k] = cornerLocation_parent[k];
            }
        }

        /*
        // convert value into parent frame
        cVector3d childBoxMin = m_children[i]->getBoundaryMin();
        rot.mulr(childBoxMin, V);
        V.addr(pos, childBoxMin);

        // convert value into parent frame
        cVector3d childBoxMax = m_children[i]->getBoundaryMax();
        rot.mulr(childBoxMax, V);
        V.addr(pos, childBoxMax);
        */

        // see if _I_ have a valid boundary box
        bool current_box_valid = (
          fabs(cDistance(m_boundaryBoxMax,m_boundaryBoxMin)) > BOUNDARY_BOX_EPSILON
          );

        // if I don't, take my child's boundary box, which is valid...
        if (current_box_valid == 0) 
        {
            m_boundaryBoxMin = childBoxMin;
            m_boundaryBoxMax = childBoxMax;
        }

        else 
        {
            // compute new boundary
            m_boundaryBoxMin.x = cMin(m_boundaryBoxMin.x, childBoxMin.x);
            m_boundaryBoxMin.y = cMin(m_boundaryBoxMin.y, childBoxMin.y);
            m_boundaryBoxMin.z = cMin(m_boundaryBoxMin.z, childBoxMin.z);

            // compute new boundary
            m_boundaryBoxMax.x = cMax(m_boundaryBoxMax.x, childBoxMax.x);
            m_boundaryBoxMax.y = cMax(m_boundaryBoxMax.y, childBoxMax.y);
            m_boundaryBoxMax.z = cMax(m_boundaryBoxMax.z, childBoxMax.z);
        }
    }
}


//===========================================================================
/*!
    Determine whether the given segment intersects a triangle in this object
    (or any of its descendants).  The segment is described by a start point
    \e a_segmentPointA and end point \e a_segmentPointB. Collision detection
    functions of all children of this object are called, which recursively
    call the collision detection functions for all of this object's descendants.
    If there is more than one collision, the one closest to a_segmentPointA is
    the one returned.

	\fn		bool cGenericObject::computeCollisionDetection(cVector3d& a_segmentPointA,
											   cVector3d& a_segmentPointB,
                                               cCollisionRecorder& a_recorder,
                                               cCollisionSettings& a_settings)
    \param  a_segmentPointA  Start point of segment.  Value may be changed if
                             returned collision is with a moving object.
    \param  a_segmentPointB  End point of segment.
    \param  a_recorder  Stores all collision events.
    \param  a_settings  Contains collision settings information.
*/
//===========================================================================
bool cGenericObject::computeCollisionDetection(cVector3d& a_segmentPointA,
											   cVector3d& a_segmentPointB,
                                               cCollisionRecorder& a_recorder,
                                               cCollisionSettings& a_settings)
{
	// check if node is a ghost. If yes, then ignore call
	if (m_ghostStatus) { return (false); }

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
    if ((m_collisionDetector != NULL) &&
        (!a_settings.m_checkVisibleObjectsOnly || m_show) &&
        (!a_settings.m_checkHapticObjectsOnly || m_hapticEnabled))
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
        if (m_collisionDetector->computeCollision(localSegmentPointAadjusted,
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


//===========================================================================
/*!
    Adjust the given segment such that it tests for intersection of the ray with
    objects at their previous positions at the last haptic loop so that collision
    detection will work in a dynamic environment.

    \fn     void cGenericObject::adjustCollisionSegment(cVector3d& a_segmentPointA,
                                                        cVector3d& a_segmentPointAadjusted)
    \param  a_segmentPointA  Start point of segment.
    \param  a_segmentPointAadjusted  Same segment, adjusted to be in local space.
*/
//===========================================================================
void cGenericObject::adjustCollisionSegment(cVector3d& a_segmentPointA,
                                       cVector3d& a_segmentPointAadjusted)
{
    // convert point from local to global coordinates by using
    // the previous object position and orientation
    cVector3d point = cAdd(m_globalPos, cMul(m_globalRot, a_segmentPointA));

    // compute the new position of the point based on
    // the new object position and orientation
    a_segmentPointAadjusted = cMul( cTrans(m_prevGlobalRot), cSub(point, m_prevGlobalPos));
}


//===========================================================================
/*!
    Render the scene graph starting at this object. This method is called
    for each object and optionally renders the object itself, its reference frame
    and the collision and/or scenegraph trees. \n

    The object itself is rendered by calling render(), which should be defined
    for each subclass that has a graphical representation.  renderSceneGraph
    does not generally need to be over-ridden in subclasses. \n

    The a_renderMode parameter is used to allow multiple rendering passes,
    and takes one of the following values: \n

    CHAI_RENDER_MODE_NON_TRANSPARENT_ONLY=0, \n
    CHAI_RENDER_MODE_TRANSPARENT_BACK_ONLY,  \n    
    CHAI_RENDER_MODE_TRANSPARENT_FRONT_ONLY, \n
    CHAI_RENDER_MODE_RENDER_ALL              \n

    If you have multipass transparency disabled (see cCamera), your objects will
    only be rendered once per frame, with a_renderMode set to CHAI_RENDER_MODE_RENDER_ALL.
    This is the default, and unless you enable multipass transparency, you don't
    ever need to care about a_renderMode.

    \fn     void cGenericObject::renderSceneGraph(const int a_renderMode)
    \param  a_renderMode  Rendering mode.
*/
//===========================================================================
void cGenericObject::renderSceneGraph(const int a_renderMode)
{
    //-----------------------------------------------------------------------
    // Initialize rendering
    //-----------------------------------------------------------------------

    // rotate the current reference frame to match this object's
    // reference frame
    m_frameGL.set(m_localPos, m_localRot);
    m_frameGL.glMatrixPushMultiply();

    // Handle rendering meta-object components, e.g. collision trees,
    // bounding boxes, scenegraph tree, etc.
    // set up useful rendering state
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);

    //-----------------------------------------------------------------------
    // Render non transparent components of cGenericObject
    //-----------------------------------------------------------------------

    if (a_renderMode == CHAI_RENDER_MODE_NON_TRANSPARENT_ONLY ||
        a_renderMode == CHAI_RENDER_MODE_RENDER_ALL)
    {
        // disable lighting
        glDisable(GL_LIGHTING);

        // render tree
        if (m_showTree)
        {
            // set size on lines
            glLineWidth(1.0);

            // set color of tree
            glColor4fv(m_treeColor.pColor());

            // render tree
            for (unsigned int i=0; i<m_children.size(); i++)
            {
                cGenericObject *nextChild = m_children[i];

                // draw a line from origin of current frame to origin  of child frame
                glBegin(GL_LINES);
                    glVertex3d(0.0, 0.0, 0.0);
                    glVertex3dv((const double *)&nextChild->m_localPos);
                glEnd();
            }
        }

        // render boundary box
        if (m_showBox)
        {
            // set size on lines
            glLineWidth(1.0);

            // set color of boundary box
            glColor4fv(m_boundaryBoxColor.pColor());

            // draw box line
            cDrawWireBox(m_boundaryBoxMin.x, m_boundaryBoxMax.x,
                         m_boundaryBoxMin.y, m_boundaryBoxMax.y,
                         m_boundaryBoxMin.z, m_boundaryBoxMax.z);
        }

        // render collision tree
        if (m_showCollisionTree && (m_collisionDetector != NULL))
        {
            m_collisionDetector->render();
        }

        // enable lighting
        glEnable(GL_LIGHTING);

        // render frame
        if (m_showFrame)
        {
            // set rendering properties
            glPolygonMode(GL_FRONT, GL_FILL);

            // draw frame
            cDrawFrame(m_frameSize,m_frameThicknessScale,true);
        }
    }

    //-----------------------------------------------------------------------
    // Render graphical representation of object
    //-----------------------------------------------------------------------
    if (m_show)
    {
        // set polygon and face mode
        glPolygonMode(GL_FRONT_AND_BACK, m_triangleMode);


        /////////////////////////////////////////////////////////////////////
        // SINGLE PASS RENDERING
        /////////////////////////////////////////////////////////////////////

        if (a_renderMode == CHAI_RENDER_MODE_RENDER_ALL)
        {
            if (this->m_cullingEnabled)
            {
                glEnable(GL_CULL_FACE);
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
            render(a_renderMode);
        }


        /////////////////////////////////////////////////////////////////////
        // MULTI PASS RENDERING
        /////////////////////////////////////////////////////////////////////

        // opaque objects
        else if (a_renderMode == CHAI_RENDER_MODE_NON_TRANSPARENT_ONLY)
        {
            if (this->m_cullingEnabled)
            {
                glEnable(GL_CULL_FACE);
            }
            else
            {
                glDisable(GL_CULL_FACE);
            }

            render(a_renderMode);
        }

        // render transparent front triangles
        else if (a_renderMode == CHAI_RENDER_MODE_TRANSPARENT_FRONT_ONLY)
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
            render(a_renderMode);
        }

        else if (a_renderMode == CHAI_RENDER_MODE_TRANSPARENT_BACK_ONLY)
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
            render(a_renderMode);
        }
    }

    // render children
    for (unsigned int i=0; i<m_children.size(); i++)
    {
        m_children[i]->renderSceneGraph(a_renderMode);
    }

    // pop current matrix
    m_frameGL.glMatrixPop();

    // restore settings
    glDisable(GL_CULL_FACE);
}


//===========================================================================
/*!
    Render this object.  Subclasses will generally override this method.
    This is called from renderSceneGraph, which subclasses generally do
    not need to override. \n

    The a_renderMode parameter is used to allow multiple rendering passes,
    and takes one of the following values: \n

    CHAI_RENDER_MODE_NON_TRANSPARENT_ONLY=0, \n
    CHAI_RENDER_MODE_TRANSPARENT_BACK_ONLY,  \n    
    CHAI_RENDER_MODE_TRANSPARENT_FRONT_ONLY, \n
    CHAI_RENDER_MODE_RENDER_ALL              \n

    If you have multipass transparency disabled (see cCamera), your objects will
    only be rendered once per frame, with a_renderMode set to CHAI_RENDER_MODE_RENDER_ALL.
    This is the default, and unless you enable multipass transparency, you don't
    ever need to care about a_renderMode. \n

    A word on OpenGL conventions: \n

    CHAI does not re-initialize the OpenGL state at every rendering
    pass.  The only OpenGL state variables that CHAI sets explicitly in a typical
    rendering pass are: \n

    * lighting is enabled (cWorld) \n
    * depth-testing is enabled (cWorld) \n
    * glColorMaterial is enabled and set to GL_AMBIENT_AND_DIFFUSE/GL_FRONT_AND_BACK (cWorld) \n
    * a perspective projection matrix is set up (cCamera) \n

    This adherence to the defaults is nice because it lets an application change an important
    piece of state globally and not worry about it getting changed by CHAI objects. \n

    It is expected that objects will "clean up after themselves" if they change
    any rendering state besides:

    * color (glColor) \n
    * material properties (glMaterial) \n
    * normals (glNormal) \n

    For example, if my object changes the rendering color, I don't need to set it back
    before returning, but if my object turns on vertex buffering, I should turn it
    off before returning.  Consequently if I care about the current color, I should
    set it up in my own render() function, because I shouldn't count on it being
    meaningful when my render() function is called. \n

    Necessary exceptions to these conventions include: \n

    * cLight will change the lighting state for his assigned GL_LIGHT \n
    * cCamera sets up relevant transformation matrices \n

    \fn     void cGenericObject::render(const int a_renderMode)
    \param  a_renderMode  Rendering mode; see above
*/
//===========================================================================
void cGenericObject::render(const int a_renderMode) { }
