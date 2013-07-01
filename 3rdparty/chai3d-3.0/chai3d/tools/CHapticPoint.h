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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 365 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef cHapticPointH
#define cHapticPointH
//------------------------------------------------------------------------------
#include "forces/CAlgorithmFingerProxy.h"
#include "forces/CAlgorithmPotentialField.h"
#include "world/CGenericObject.h"
#include "world/CShapeSphere.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cGenericTool;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CHapticPointH.h

    \brief  
    <b> Haptic Tools </b> \n 
    Haptic interaction point.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cHapticPoint
    \ingroup    tools  

    \brief      
    cHapticPoint describes a haptic interaction point associated with a tool.
    A tool may combine one or more interaction points depending of its type.
    For instance a cursor type tool will typically contain a single point contact,
    while a gripper type tool will contain at least two contact points 
    (one for the finger and a second for the thumb).

    An interaction point combines a goal position and a proxy position. The goal
    sphere is moved by the haptic device while the proxy sphere follows the
    desired goal while respecting constraints (triangles) located on the path.
*/
//==============================================================================
class cHapticPoint
{
  public:
    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cHapticPoint.
    cHapticPoint(cGenericTool* a_parentTool);

    //! Destructor of cHapticPoint.
    virtual ~cHapticPoint();


    //--------------------------------------------------------------------------
    // METHODS - POSITION COMMANDS
    //--------------------------------------------------------------------------

    //! Read the current desired goal position of the contact point in world coordinates.
    cVector3d getGlobalPosGoal();

    //! Read the current proxy position of the contact point in world coordinates.
    cVector3d getGlobalPosProxy();

    //! Read the current desired goal position of the contact point in local tool coordinates.
    cVector3d getLocalPosGoal();

    //! Read the current proxy position of the contact point in local tool coordinates.
    cVector3d getLocalPosProxy();

    //! Reset the position of the proxy to be at the desired goal position.
    void initialize();

    //! Initialize the position of the goal and proxy at a new given position.
    void initialize(cVector3d a_globalPos);

    //! Get parent tool.
    inline cGenericTool* getParentTool() { return (m_parentTool); }


    //--------------------------------------------------------------------------
    // METHODS - GENERAL SETTINGS
    //--------------------------------------------------------------------------

    //! Set radius size of contact point.
    void setRadius(double a_radius);

    //! Set radius size of contact point.
    void setRadius(double a_radiusDisplay, 
                   double a_radiusContact);

    //! Set radius size of the physical proxy.
    void setRadiusContact(double a_radiusContact);

    //! Set radius size of the sphere used to display the proxy and goal position
    void setRadiusDisplay(double a_radiusDisplay);

    //! Get display radius size.
    double getRadiusDisplay() { return (m_radiusDisplay); }

    //! Get contact radius size.
    double getRadiusContact() { return (m_radiusContact); }

    //! Set display options of the goal and proxy positions
    void setShow(bool a_showProxy = true, 
                 bool a_showGoal = false,
                 cColorf a_colorLine = cColorf(0.5, 0.5, 0.5));


    //--------------------------------------------------------------------------
    // METHODS - COLLISION EVENTS (MESH OBJECTS)
    //--------------------------------------------------------------------------

    // Get the number of collision events between this interaction point and the environment.
    inline int getNumCollisionEvents() { return (m_algorithmFingerProxy->getNumCollisionEvents()); }

    // Get the ith collision events for this interaction point.
    inline cCollisionEvent* getCollisionEvent(const int a_index) { return (m_algorithmFingerProxy->m_collisionEvents[a_index]); }


    //--------------------------------------------------------------------------
    // METHODS - INTERACTION EVENTS (POTENTIAL FIELDS)
    //--------------------------------------------------------------------------

    // Get the number of interaction events between this interaction point and the environment.
    inline int getNumInteractionEvents() { return (int)(m_algorithmPotentialField->m_interactionRecorder.m_interactions.size()); }

    // Get the n'th interaction events for this interaction point.
    inline cInteractionEvent* getInteractionEvent(const int a_index) { return (&(m_algorithmPotentialField->m_interactionRecorder.m_interactions[a_index])); }


    //--------------------------------------------------------------------------
    // METHODS - FORCE COMPUTATIONS
    //--------------------------------------------------------------------------

    //! Compute all interaction forces between the contact point and the virtual environment.
    cVector3d computeInteractionForces(cVector3d& a_globalPos, 
                                       cMatrix3d& a_globalRot,
                                       cVector3d& a_globalLinVel,
                                       cVector3d& a_globalAngVel);

    //! Read the last computed force in global world coordinates.
    cVector3d getLastComputedForce() { return (m_lastComputedGlobalForce); }

    //! Check if the tool is touching a particular object.
    bool isInContact(cGenericObject* a_object);


    //--------------------------------------------------------------------------
    // METHODS - GRAPHICS
    //--------------------------------------------------------------------------

    //! Render the tool graphicaly in OpenGL.
    void render(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // MEMBERS - FORCE RENDERING ALGORITHMS
    //--------------------------------------------------------------------------

    //! Finger-proxy algorithm used for modelling contacts with cMesh objects.
    cAlgorithmFingerProxy* m_algorithmFingerProxy;

    //! Potential fields algorithm used for modelling interaction forces with haptic effects.
    cAlgorithmPotentialField* m_algorithmPotentialField;


    //--------------------------------------------------------------------------
    // METHODS - GRAPHICS
    //--------------------------------------------------------------------------

    //! Sphere object used for rendering the Proxy position. 
    cShapeSphere* m_sphereProxy;

    //! Sphere object used for rendering the Goal position.
    cShapeSphere* m_sphereGoal;

    //! Color of connecting both spheres.
    cColorf m_colorLine;


  protected:

    //! Parent tool
    cGenericTool* m_parentTool;

    //! Last computed interaction force [N] in world global coordinates. 
    cVector3d m_lastComputedGlobalForce;

    //! Radius used for rendering the proxy and goal spheres.
    double m_radiusDisplay;

    //! Radius used to model the physical radius of the proxy.
    double m_radiusContact;
    
	//! Stores a pointer to a possible mesh objects for which the proxy may be in contact with.
	cGenericObject* m_meshProxyContacts[3];

    //! Update position of spheres in tool coordinates
    void updateSpherePositions();
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

