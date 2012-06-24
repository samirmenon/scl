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
#ifndef CAlgorithmFingerProxyH
#define CAlgorithmFingerProxyH
//---------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "collisions/CGenericCollision.h"
#include "forces/CGenericForceAlgorithm.h"
#include <map>
//---------------------------------------------------------------------------
class cWorld;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CAlgorithmFingerProxy.h

    \brief 
    <b> Force Rendering Algorithms </b> \n 
    Finger-Proxy Model.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cAlgorithmFingerProxy
    \ingroup    forces 
    
    \brief    
    Implements the finger-proxy algorithm for computing interaction forces 
    between a point force device and meshes.
*/
//===========================================================================
class cAlgorithmFingerProxy : public cGenericForceAlgorithm
{
  public:
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cAlgorithmFingerProxy.
    cAlgorithmFingerProxy();

    //! Destructor of cAlgorithmFingerProxy.
    virtual ~cAlgorithmFingerProxy() {}


    //-----------------------------------------------------------------------
    // METHODS - GRAPHICS
    //-----------------------------------------------------------------------

    //! Render force algorithms graphicaly in OpenGL. (For debug purposes)
    virtual void render(cRenderOptions& a_options);


    //----------------------------------------------------------------------
    // METHODS - BASIC PROXY:
    //----------------------------------------------------------------------

    //! Initialize the algorithm.
    void initialize(cWorld* a_world, const cVector3d& a_initialGlobalPosition);

    //! Reset the algorithm. Set proxy position to device position.
    void reset();

    //! Calculate interaction forces between device and meshes.
    virtual cVector3d computeForces(const cVector3d& a_toolPos, const cVector3d& a_toolVel);


    //----------------------------------------------------------------------
    // METHODS - GETTER AND SETTER FUNCTIONS:
    //----------------------------------------------------------------------

    //! Set radius of proxy.
    void setProxyRadius(const double& a_radius) { m_radius = a_radius; m_collisionSettings.m_collisionRadius = m_radius;}

    //! Read radius of proxy.
    inline double getProxyRadius() const { return (m_radius); }

    //! Get last computed position of proxy in world coordinates.
    inline cVector3d getProxyGlobalPosition() const { return (m_proxyGlobalPos); }

    //! Set position of proxy in world coordinates.
    inline void setProxyGlobalPosition(const cVector3d& a_position)  { m_proxyGlobalPos = a_position; }

    //! Get last specified position of device in world coordinates.
    inline cVector3d getDeviceGlobalPosition() const { return (m_deviceGlobalPos); }

    //! Get last computed force vector in world coordinates.
    inline cVector3d getForce() { return (m_lastGlobalForce); }

    //! Return most recently calculated normal force.
    inline cVector3d getNormalForce() { return (m_normalForce); }

    //! Return most recently calculated tangential force.
    inline cVector3d getTangentialForce() { return (m_tangentialForce); }


    //----------------------------------------------------------------------
    // METHODS - COLLISION INFORMATION BETWEEN PROXY AND WORLD
    //----------------------------------------------------------------------
    
    //! Return the number of collision events (0, 1, 2 or 3):
    int getNumCollisionEvents() { return (m_numCollisionEvents); }

    //! Table of collision events (0-3). Call getNumCollisionEvents() to see how many are actually active.
    cCollisionEvent* m_collisionEvents[3];


    //----------------------------------------------------------------------
    // METHODS - FORCE MODELS
    //----------------------------------------------------------------------

    //! Use the dynamic proxy algorithm to deal with mobile objects?
    bool m_useDynamicProxy;

    /*!
        Dynamic friction hysteresis multiplier
        In CHAI's proxy, the angle computed from the coefficient is multiplied
        by this constant to avoid rapidly oscillating between slipping and sticking
        without having to turn the dynamic friction level way down.
    */
    double m_frictionDynHysteresisMultiplier;

    //! Maximum force shading angle (radians) threshold between normals of triangle.
    double m_forceShadingAngleThreshold;

    //! Collision cettings
    cCollisionSettings m_collisionSettings;

    //----------------------------------------------------------------------
    // METHODS - RESOLUTION / ERRORS
    //----------------------------------------------------------------------

    //! Set epsilon base value.
    void setEpsilonBaseValue(double a_value);

    //! Read current epsilon value.
    double getEpsilonBaseValue() { return (m_epsilonBaseValue); }


  protected:

    //! Test whether the proxy has reached the goal point.
    virtual bool goalAchieved(const cVector3d& a_proxy, const cVector3d& a_goal) const;

    //! Compute the next goal position of the proxy.
    virtual void computeNextBestProxyPosition(const cVector3d& a_goal);

    //! Attempt to move the proxy, subject to friction constraints.
    virtual void testFrictionAndMoveProxy(const cVector3d& a_goal, const cVector3d& a_proxy, cVector3d& a_normal, cGenericObject* a_parent);

    //! Compute force to apply to device.
    virtual void updateForce();


    //----------------------------------------------------------------------
    // MEMBERS - PROXY, DEVICE AND FORCE INFORMATION:
    //----------------------------------------------------------------------

    //! Global position of the proxy.
    cVector3d m_proxyGlobalPos;

    //! Global position of device.
    cVector3d m_deviceGlobalPos;

    //! Last computed force (in global coordinate frame).
    cVector3d m_lastGlobalForce;

    //! Next best position for the proxy (in global coordinate frame).
    cVector3d m_nextBestProxyGlobalPos;

    //! Are we currently in a "slip friction" state?
    bool m_slipping;

    //! Normal force.
    cVector3d m_normalForce;

    //! Tangential force.
    cVector3d m_tangentialForce;

    //! Number of collision events between proxy and triangles (0, 1, 2 or 3).
    unsigned int m_numCollisionEvents;

    //! Radius of the proxy.
    double m_radius;


    //----------------------------------------------------------------------
    // MEMBERS - TOOLS REQUIRED FOR THE PROXY ALGORITHM
    //----------------------------------------------------------------------

    //! Collision detection recorder for searching first constraint.
    cCollisionRecorder m_collisionRecorderConstraint0;

    //! Collision detection recorder for searching second constraint.
    cCollisionRecorder m_collisionRecorderConstraint1;

    //! Collision detection recorder for searching third constraint.
    cCollisionRecorder m_collisionRecorderConstraint2;

    /*!
        To address numerical errors during geometric computation,
        several epsilon values are computed and used.
    */

    //! epsilon value - used for handling numerical limits.
    double m_epsilonInitialValue;

    //! epsilon value - used for handling numerical limits.
    double m_epsilon;

    //! epsilon value - used for handling numerical limits.
    double m_epsilonCollisionDetection;

    //! epsilon value - used for handling numerical limits.
    double m_epsilonBaseValue;

    //! epsilon value - used for handling numerical limits.
    double m_epsilonMinimalValue;

    //! Value of state machine.
    unsigned int m_algoCounter;

    //! Implementation of the proxy algorithm - constraint 0.
    bool computeNextProxyPositionWithContraints0(const cVector3d& a_goalGlobalPos);

	//! Implementation of the proxy algorithm - constraint 1.
    bool computeNextProxyPositionWithContraints1(const cVector3d& a_goalGlobalPos);

	//! Implementation of the proxy algorithm - constraint 2.
    bool computeNextProxyPositionWithContraints2(const cVector3d& a_goalGlobalPos);

    cVector3d computeShadedSurfaceNormal(cCollisionEvent* a_contactPoint);



    //----------------------------------------------------------------------
    // DEBUG PURPOSES
    //----------------------------------------------------------------------
    cVector3d surfaceNormal;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

