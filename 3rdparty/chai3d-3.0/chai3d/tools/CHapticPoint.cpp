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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 322 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "tools/CHapticPoint.h"
//---------------------------------------------------------------------------
#include "tools/CGenericTool.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cHapticPoint

    \fn       cAlgorithmPotentialField::cAlgorithmPotentialField()
*/
//===========================================================================
cHapticPoint::cHapticPoint(cGenericTool* a_parentTool)
{
    // set parent tool
    m_parentTool = a_parentTool;

    // initialize variables
    m_radiusDisplay = 0.0;
    m_radiusContact = 0.0;
    m_lastComputedGlobalForce.zero();
    m_meshProxyContacts[0] = NULL;
    m_meshProxyContacts[1] = NULL;
    m_meshProxyContacts[2] = NULL;

    // create finger-proxy algorithm used for modelling contacts with 
    // cMesh objects.
    m_algorithmFingerProxy = new cAlgorithmFingerProxy();

    // create potential field algorithm used for modelling interaction 
    // forces with haptic effects.
    m_algorithmPotentialField = new cAlgorithmPotentialField();

    // create sphere object used for rendering the Proxy position. 
    m_sphereProxy = new cShapeSphere(0.0);

    // create sphere object used for rendering the Goal position.
    m_sphereGoal = new cShapeSphere(0.0);

    // Set default radius of proxy and goal spheres
    setRadius(0.01, 0.01);

    // define a color for the proxy sphere
    m_sphereProxy->m_material->setGrayDim();

    // define a color for the goal sphere
    m_sphereGoal->m_material->setGrayLight();

    // Initialize force rendering algorithms
    initialize(cVector3d(0.0, 0.0, 0.0));
}


//===========================================================================
/*!
    Destructor of cHapticPoint.

    \fn     cHapticPoint::~cHapticPoint() {};
*/
//===========================================================================
cHapticPoint::~cHapticPoint() 
{
    // delete force rendering algorithms
    delete m_algorithmFingerProxy;
    delete m_algorithmPotentialField;

    // delete graphical spheres (proxy and goal)
    delete m_sphereProxy;
    delete m_sphereGoal;
}


//===========================================================================
/*!
    Read the current desired goal position of the contact point in world
    global coordinates.

    \fn     cVector3d cHapticPoint::getGlobalPosGoal()
    \return Return the goal position.  
*/
//===========================================================================
cVector3d cHapticPoint::getGlobalPosGoal()
{
    return(m_algorithmFingerProxy->getDeviceGlobalPosition());
}


//===========================================================================
/*!
     Read the current proxy position of the contact point in world 
     global coordinates.

    \fn     cVector3d cHapticPoint::getGlobalPosProxy()
    \return Return the proxy position. 
*/
//===========================================================================
cVector3d cHapticPoint::getGlobalPosProxy()
{
    return(m_algorithmFingerProxy->getProxyGlobalPosition());
}


//===========================================================================
/*!
    Read the current desired goal position of the contact point in local
    tool coordinates.

    \fn     cVector3d cHapticPoint::getLocalPosGoal()
    \return Return the goal position.  
*/
//===========================================================================
cVector3d cHapticPoint::getLocalPosGoal()
{
    cMatrix3d rot;
    cVector3d newpos;
    cVector3d toolGlobalPos = m_parentTool->getGlobalPos();
    cMatrix3d toolGlobalRot = m_parentTool->getGlobalRot();
    cVector3d pos = m_algorithmFingerProxy->getDeviceGlobalPosition();
	toolGlobalRot.transr(rot);
    pos.sub(toolGlobalPos);
    rot.mulr(pos, newpos);
    return(newpos);
}


//===========================================================================
/*!
     Read the current proxy position of the contact point in local 
     tool coordinates.

    \fn     cVector3d cHapticPoint::getLocalPosProxy()
    \return Return the proxy position. 
*/
//===========================================================================
cVector3d cHapticPoint::getLocalPosProxy()
{
    cMatrix3d rot;
    cVector3d newpos;
    cVector3d toolGlobalPos = m_parentTool->getGlobalPos();
    cMatrix3d toolGlobalRot = m_parentTool->getGlobalRot();
    cVector3d pos = m_algorithmFingerProxy->getProxyGlobalPosition();
    toolGlobalRot.transr(rot);
    pos.sub(toolGlobalPos);
    rot.mulr(pos, newpos);
    return(newpos);
}


//===========================================================================
/*!
    Reset the position of the proxy to be at the desired goal position.

    \fn     void cHapticPoint::initialize()
*/
//===========================================================================
void cHapticPoint::initialize()
{
    // reset finger proxy algorithm by placing the proxy and the same position 
    // as the goal. 
    m_algorithmFingerProxy->reset();

    // update position of proxy and goal spheres in tool coordinates
    updateSpherePositions();
}


//===========================================================================
/*!
    Initialize the position of the goal and proxy at a new given position.

    \fn     void cHapticPoint::initialize(cVector3d a_globalPos)
    \param  a_globalPos  New position to apply to goal and proxy.
*/
//===========================================================================
void cHapticPoint::initialize(cVector3d a_globalPos)
{
    // initialize proxy algorithm.
    m_algorithmFingerProxy->initialize(m_parentTool->getParentWorld(), a_globalPos);
    m_algorithmPotentialField->initialize(m_parentTool->getParentWorld(), a_globalPos);

    // update position of proxy and goal spheres in tool coordinates
    updateSpherePositions();
}


//===========================================================================
/*!
    Set radius size of contact point. The radius affects the physical 
    radius of the proxy and spheres used to render the goal and proxy 
    positions.

    \fn     void cHapticPoint::setRadius(double a_radius)
    \param  a_radius  New radius for display and contact computation.
*/
//===========================================================================
void cHapticPoint::setRadius(double a_radius)
{
    m_radiusContact = a_radius;
    m_radiusDisplay = a_radius;

    // set the radius of the contact model of the proxy
    m_algorithmFingerProxy->setProxyRadius(m_radiusContact);

    // set radius of proxy and goal. goal radius is slighlty smaller to avoid graphical
    // artifacts when both sphere are located at the same position.
    m_sphereProxy->setRadius(m_radiusDisplay);
    m_sphereGoal->setRadius(0.95 * m_radiusDisplay);
}


//===========================================================================
/*!
   Set radius size of the display spheres (goal and proxy) and physical contact 
   sphere (proxy) used to compute the contact forces. Setting the a_radiusContact
   parameter to zero will generally accelerate the force rendering algorithm.

    \fn     void cHapticPoint::setRadius(double a_radiusDisplay, 
                                              double a_radiusContact)
    \param  a_radiusDisplay  New radius for display of spheres (proxy and goal).
    \param  a_radiusContact  New radius for contact computation (proxy).
*/
//===========================================================================
void cHapticPoint::setRadius(double a_radiusDisplay, double a_radiusContact)
{
    m_radiusContact = a_radiusContact;
    m_radiusDisplay = a_radiusDisplay;

    // set the radius of the contact model of the proxy
    m_algorithmFingerProxy->setProxyRadius(m_radiusContact);

    // set radius of proxy and goal. goal radius is slighlty smaller to avoid graphical
    // artifacts when both sphere are located at the same position.
    m_sphereProxy->setRadius(m_radiusDisplay);
    m_sphereGoal->setRadius(0.95 * m_radiusDisplay);
}


//===========================================================================
/*!
    Set radius size of the physical proxy.

    \fn     void cHapticPoint::setRadiusContact(double a_radiusContact)
    \param  a_radiusContact  New radius for contact computation (proxy).
*/
//===========================================================================
void cHapticPoint::setRadiusContact(double a_radiusContact)
{
    m_radiusContact = a_radiusContact;
    
    // set the radius of the contact model of the proxy
    m_algorithmFingerProxy->setProxyRadius(m_radiusContact);
}


//===========================================================================
/*!
    Set radius size of the sphere used to display the proxy and goal position.

    \fn     void cHapticPoint::setRadiusDisplay(double a_radiusDisplay)
    \param  a_radiusDisplay  New radius for display of spheres (proxy and goal).
*/
//===========================================================================
void cHapticPoint::setRadiusDisplay(double a_radiusDisplay)
{
    m_radiusDisplay = a_radiusDisplay;

    // set radius of proxy and goal. goal radius is slighlty smaller to avoid graphical
    // artifacts when both sphere are located at the same position.
    m_sphereProxy->setRadius(m_radiusDisplay);
    m_sphereGoal->setRadius(0.99 * m_radiusDisplay);
}


//===========================================================================
/*!
    Set display options of the goal and proxy spheres. If both spheres are 
    enabled, a small line is drawn between both spheres.

    \fn     void cHapticPoint::setShow(bool a_showProxy, 
                                bool a_showGoal, 
                                cColorf a_colorLine)

    \param  a_showProxy If \b true, then the proxy sphere is displayed.
    \param  a_showGoal If \b true, then the goal sphere is displayed.
    \param  a_colorLine  Color of line connecting proxy to goal spheres.
*/
//===========================================================================
void cHapticPoint::setShow(bool a_showProxy, 
                           bool a_showGoal, 
                           cColorf a_colorLine)
{
    // update display properties of both spheres
    m_sphereProxy->setShowEnabled(a_showProxy);
    m_sphereGoal->setShowEnabled(a_showGoal);
    m_colorLine = a_colorLine;
}


//===========================================================================
/*!
    Compute all interaction forces between the tool contact points and the 
    virtual environment.

    \fn     cVector3d cHapticPoint::computeInteractionForces()
    \param  a_globalPos  New desired goal position.
    \param  a_globalRot  New desired goal rotation.
    \param  a_globalLinVel  Linear velocity of tool.
    \param  a_globalAngVel  Angular velocity of tool.

    \return Return computed interaction force in world coordinates.
*/
//===========================================================================
cVector3d cHapticPoint::computeInteractionForces(cVector3d& a_globalPos, 
                                                 cMatrix3d& a_globalRot,
                                                 cVector3d& a_globalLinVel,
                                                 cVector3d& a_globalAngVel)
{
    ///////////////////////////////////////////////////////////////////////////
    // ALGORITHM FINGER PROXY
    ///////////////////////////////////////////////////////////////////////////

    // we first consider all object the proxy may have been in contact with and
    // mark their interaction as no longer active. 
    for (int i=0; i<3; i++)
    {
        if (m_meshProxyContacts[i] != NULL)
	    {
		    m_meshProxyContacts[i]->m_interactionInside = false;
		    m_meshProxyContacts[i] = NULL;
	    }
    }

    // we now update the new position of a goal point and update the proxy position.
    // As a result, the force contribution from the proxy is now calculated.
    cVector3d force0 = m_algorithmFingerProxy->computeForces(a_globalPos, a_globalLinVel);

    // we now flag each mesh for which the proxy may be interactinbg with. This information is 
	// necessary for haptic effects that may be associated with these mesh objects.
    for (int i=0; i<3; i++)
    {
        if (m_algorithmFingerProxy->m_collisionEvents[i]->m_object != NULL)
	    {
		    m_meshProxyContacts[i] = m_algorithmFingerProxy->m_collisionEvents[i]->m_object;
		    m_meshProxyContacts[i]->m_interactionInside = true;
		    m_meshProxyContacts[i]->m_interactionProjectedPoint = m_algorithmFingerProxy->m_collisionEvents[i]->m_localPos;
	    }
    }

	
    ///////////////////////////////////////////////////////////////////////////
    // ALGORITHM POTENTIAL FIELD
    ///////////////////////////////////////////////////////////////////////////

    // compute interaction forces (haptic effects) in world coordinates between tool and all
	// objects for which haptic effects have been programmed
    cVector3d force1 = m_algorithmPotentialField->computeForces(a_globalPos, a_globalLinVel);


    ///////////////////////////////////////////////////////////////////////////
    // FINALIZATION
    ///////////////////////////////////////////////////////////////////////////
    
    // update position of proxy and goal spheres in tool coordinates
    updateSpherePositions();

    // finally return the contribution fr om both force models.
    force0.addr(force1, m_lastComputedGlobalForce);
    return (m_lastComputedGlobalForce);
}


//===========================================================================
/*!
    Check if the tool is touching a particular object.

    \fn     bool cHapticPoint::isInContact(cGenericObject* a_object)
    \param  a_object  Object to checked for possible contact.
    \return Return \b true if a_object is in contact with tool, 
            \b false otherwise.
*/
//===========================================================================
bool cHapticPoint::isInContact(cGenericObject* a_object)
{
    /////////////////////////////////////////////////////////////////////
    // verify finger-proxy algorithm 
    /////////////////////////////////////////////////////////////////////

    // contact 0
    if (m_algorithmFingerProxy->m_collisionEvents[0]->m_object == a_object)
    {
        return (true);
    }

    // contact 1
    if ((m_algorithmFingerProxy->m_collisionEvents[0]->m_object != NULL) &&
        (m_algorithmFingerProxy->m_collisionEvents[1]->m_object == a_object))
    {
        return (true);
    }

    // contact 2
    if ((m_algorithmFingerProxy->m_collisionEvents[0]->m_object != NULL) &&
        (m_algorithmFingerProxy->m_collisionEvents[1]->m_object != NULL) &&
        (m_algorithmFingerProxy->m_collisionEvents[2]->m_object == a_object))
    {
        return (true);
    }

    /////////////////////////////////////////////////////////////////////
    // verify potential-field algorithm
    /////////////////////////////////////////////////////////////////////
    unsigned int num = (int)(m_algorithmPotentialField->m_interactionRecorder.m_interactions.size());
    unsigned int i = 0;
    while (i < num)
    {
        // check next interaction
        if (m_algorithmPotentialField->m_interactionRecorder.m_interactions[i].m_object == a_object)
        {
            return (true);
        }

        // increment counter
        i++;
    }

    // no object in contact
    return (false);
}


//===========================================================================
/*!
    Render the tool graphicaly in OpenGL.

    \fn     void cHapticPoint::render(cRenderOptions& a_options)
    \param  a_options  Rendering options.
*/
//===========================================================================
void cHapticPoint::render(cRenderOptions& a_options)
{
	/////////////////////////////////////////////////////////////////////////
	// Render parts that are always opaque
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
	{
        // render line only if both spheres are enabled for display
        if (m_sphereProxy->getShowEnabled() && m_sphereGoal->getShowEnabled())
        {
		    // disable lighting
		    glDisable(GL_LIGHTING);

            // points describe line extremities
            cVector3d posA = m_sphereProxy->getLocalPos();
            cVector3d posB = m_sphereGoal->getLocalPos(); 

		    // draw line
		    glBegin(GL_LINES);
			    m_colorLine.render();
			    glVertex3dv(&posA(0) );
			    glVertex3dv(&posB(0) );
		    glEnd();

		    // restore lighting to default value
		    glEnable(GL_LIGHTING);
        }
	}

 	/////////////////////////////////////////////////////////////////////////
	// Render other objects
	/////////////////////////////////////////////////////////////////////////

    // render proxy sphere
    m_sphereProxy->renderSceneGraph(a_options);

    // render goal sphere
    m_sphereGoal->renderSceneGraph(a_options);

    // render proxy algorithm (debug purposes)
    //m_algorithmFingerProxy->render(a_options);
}


//===========================================================================
/*!
    Update the position of the spheres in tool coordinates. The position of
    the actual proxy and goal spheres need to be expressed in the tool's 
    local coordinate system. This comes from the fact that the contact points 
    are rendered at the same time as the tool, respectively when the
    OpenGL model view matrix corresponds to the one of the parent tool.

    \fn     bool cHapticPoint::isInContact(cGenericObject* a_object)
    \param  a_object  Object to checked for possible contact.
    \return Return \b true if a_object is in contact with tool, 
            \b false otherwise.
*/
//===========================================================================
void cHapticPoint::updateSpherePositions()
{
    // sanity check
    if (m_parentTool == NULL) { return; }

    // position and orientation of tool in world global coordinates
    cVector3d toolGlobalPos = m_parentTool->getGlobalPos();
    cMatrix3d toolGlobalRot = m_parentTool->getGlobalRot();

    // temp variables
    cVector3d pos;
    cMatrix3d rot;
	toolGlobalRot.transr(rot);
    
    // update position of proxy sphere
    pos = getLocalPosProxy();
    m_sphereProxy->setLocalPos(pos);

    // update position of goal sphere
    pos = getLocalPosGoal();
    m_sphereGoal->setLocalPos(pos);
}
