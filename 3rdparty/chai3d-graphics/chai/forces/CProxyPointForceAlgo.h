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
#ifndef CProxyPointForceAlgoH
#define CProxyPointForceAlgoH
//---------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "collisions/CGenericCollision.h"
#include "forces/CGenericPointForceAlgo.h"
#include <map>
//---------------------------------------------------------------------------
class cWorld;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CProxyPointForceAlgo.h

    \brief 
    <b> Force Rendering Algorithms </b> \n 
    Finger-Proxy Model.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cProxyPointForceAlgo
    \ingroup    forces 
    
    \brief    
    Implements the finger-proxy algorithm for computing interaction forces 
    between a point force device and meshes.
*/
//===========================================================================
class cProxyPointForceAlgo : public cGenericPointForceAlgo
{
  public:
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cProxyPointForceAlgo.
    cProxyPointForceAlgo();

    //! Destructor of cProxyPointForceAlgo.
    virtual ~cProxyPointForceAlgo() {}


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
    //! Return the number of contacts (0, 1, 2 or 3):
    int getNumContacts() { return (m_numContacts); }

    //! Information about collision point 0. Call getNumContacts() to see if valid.
    cCollisionEvent* m_contactPoint0;

    //! Information about collision point 1. Call getNumContacts() to see if valid.
    cCollisionEvent* m_contactPoint1;

    //! Information about collision point 2. Call getNumContacts() to see if valid.
    cCollisionEvent* m_contactPoint2;


    //----------------------------------------------------------------------
    // METHODS - FORCE MODELS
    //----------------------------------------------------------------------

    //! Use any friction algorithm?
    bool m_useFriction;

    //! Use the dynamic proxy algorithm to deal with mobile objects?
    bool m_useDynamicProxy;

    //! Use force shading.
    bool m_useForceShading;

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

    //! Number of contacts between proxy and triangles (0, 1, 2 or 3).
    unsigned int m_numContacts;

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
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

