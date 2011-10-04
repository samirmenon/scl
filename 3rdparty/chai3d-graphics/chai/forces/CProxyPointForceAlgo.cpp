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
    \version   2.0.0 $Rev: 245 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "forces/CProxyPointForceAlgo.h"
#include "scenegraph/CWorld.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cProxyPointForceAlgo.

    \fn       cProxyPointForceAlgo::cProxyPointForceAlgo()
*/
//===========================================================================
cProxyPointForceAlgo::cProxyPointForceAlgo()
{
    // initialize world pointer
    m_world = NULL;

    // no contacts yet between proxy and environment
    m_numContacts = 0;

    // set epsilon base value
    setEpsilonBaseValue(0.00001);
	
    m_epsilonBaseValue = 0.00001;
    m_epsilonMinimalValue = 0.01 * m_epsilonBaseValue;
    m_epsilon = m_epsilonBaseValue;
    m_epsilonCollisionDetection = 1.0 * m_epsilon;

    // force model settings
    m_frictionDynHysteresisMultiplier = 0.6;
    m_forceShadingAngleThreshold = 0.75; 

    // initialize device and proxy positions
    m_deviceGlobalPos.zero();
    m_proxyGlobalPos.zero();
    m_lastGlobalForce.zero();

    // this will generally be over-written by the calling pointer
    m_radius = 0.01f;

    // by default, we do not use dynamic proxy (which handles moving objects)
    m_useDynamicProxy = true;

    // initialize dynamic proxy members
    m_collisionRecorderConstraint0.m_nearestCollision.clear();
    m_collisionRecorderConstraint1.m_nearestCollision.clear();
    m_collisionRecorderConstraint2.m_nearestCollision.clear();

    // friction properties
    m_slipping = true;
    m_useFriction = true;

    // use force shading
    m_useForceShading = false;

    // setup collision detector seetings
    m_collisionSettings.m_checkForNearestCollisionOnly  = true;
    m_collisionSettings.m_returnMinimalCollisionData    = false;
    m_collisionSettings.m_checkVisibleObjectsOnly       = true;
    m_collisionSettings.m_checkHapticObjectsOnly        = true;
    m_collisionSettings.m_checkBothSidesOfTriangles     = true;
    m_collisionSettings.m_adjustObjectMotion            = m_useDynamicProxy;

    // setup pointers to collision recoders so that user can access
    // collision information about each contact point.
    m_contactPoint0 = &(m_collisionRecorderConstraint0.m_nearestCollision);
    m_contactPoint1 = &(m_collisionRecorderConstraint1.m_nearestCollision);
    m_contactPoint2 = &(m_collisionRecorderConstraint2.m_nearestCollision);

    // initialize algorithm variables
    m_algoCounter = 0;
}


//===========================================================================
/*!
    Initialize the algorithm, including setting the pointer to the world
    in which the algorithm is to operate, and setting the initial position
    of the device.

    \fn       void cProxyPointForceAlgo::initialize(cWorld* a_world, 
					const cVector3d& a_initialGlobalPosition)
    \param    a_world  Pointer to world in which force algorithm is operating.
    \param    a_initialGlobalPosition  Initial position of the device.
*/
//===========================================================================
void cProxyPointForceAlgo::initialize(cWorld* a_world, const cVector3d& a_initialGlobalPosition)
{
    // reset some variables
    m_lastGlobalForce.zero();

    // no contacts yet between proxy and environment
    m_numContacts = 0;

    // the proxy can slip along surfaces
    m_slipping = true;

    // initialize device and proxy positions
    m_deviceGlobalPos = a_initialGlobalPosition;
    m_proxyGlobalPos  = a_initialGlobalPosition;

    // set pointer to world in which force algorithm operates
    m_world = a_world;
}


//===========================================================================
/*!
    Reset the algorithm. Set the proxy position to the device position.

    \fn       void cProxyPointForceAlgo::reset()
*/
//===========================================================================
void cProxyPointForceAlgo::reset()
{
    // reset force
    m_lastGlobalForce.zero();

    // set proxy position to be equal to the device position
    m_proxyGlobalPos = m_deviceGlobalPos;

    // no contacts yet between proxy and environment
    m_numContacts = 0;
}


//===========================================================================
/*!
    Set the epsilon value which is used during geometry computation of the
    proxy model.

    \fn       void cProxyPointForceAlgo::setEpsilonBaseValue(double a_value)
*/
//===========================================================================
void cProxyPointForceAlgo::setEpsilonBaseValue(double a_value)
{
    m_epsilonBaseValue = a_value;
    m_epsilonMinimalValue = 0.01 * m_epsilonBaseValue;
    m_epsilon = m_epsilonBaseValue;
    m_epsilonCollisionDetection = 1.0 * m_epsilon;
}


//===========================================================================
/*!
    This method computes the force to add to the device due to any collisions
    with meshes by calling computeNextBestProxyPosition() to update the
    proxy location and then computeForce() to calculate a force vector based
    on the proxy location.

    \fn       cVector3d cProxyPointForceAlgo::computeForces(
                    const cVector3d& a_toolPos,
                    const cVector3d& a_toolVel)
    \param    a_toolPos  New position of tool
    \param    a_toolVel  New velocity of tool
    \return   Return the force to add to the device due to any collisions
              with meshes.
*/
//===========================================================================
cVector3d cProxyPointForceAlgo::computeForces(const cVector3d& a_toolPos,
                                              const cVector3d& a_toolVel)
{
    // update device position
    m_deviceGlobalPos = a_toolPos;

    // check if world has been defined; if so, compute forces
    if (m_world != NULL)
    {
        // compute next best position of proxy
        computeNextBestProxyPosition(m_deviceGlobalPos);

        // update proxy to next best position
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;

        // compute force vector applied to device
        updateForce();

        // return result
        return (m_lastGlobalForce);
    }

    // if no world has been defined in which algorithm operates, there is no force
    else
    {
        return (cVector3d(0.0, 0.0, 0.0));
    }
}


//===========================================================================
/*!
    Given the new position of the device and considering the current
    position of the proxy, this function attempts to move the proxy towards
    the device position (the goal).  If its path is blocked by an obstacle
    (e.g., a triangle in a mesh), the proxy is moved to this intersection
    point and a new goal is calculated as the closest point to the original
    goal in the half-plane above the intersection triangle.
    The process is repeated if necessary, bringing the proxy to its
    final location.

    \fn		void cProxyPointForceAlgo::computeNextBestProxyPosition(const cVector3d& a_goal)
	\param  a_goal  The goal towards which to move the proxy, subject to constraints
*/
//===========================================================================
void cProxyPointForceAlgo::computeNextBestProxyPosition(const cVector3d& a_goal)
{
    if (m_useDynamicProxy)
    {
        bool hit0, hit1, hit2;
        hit0 = computeNextProxyPositionWithContraints0(a_goal);
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;
        if (!hit0) { return; }

        hit1 = computeNextProxyPositionWithContraints1(a_goal);
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;
        if (!hit1) { return; }

        hit2 = computeNextProxyPositionWithContraints2(a_goal);
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;
    }
    else
    {
        // In order to keep the finger-proxy algorithm running fast, we only
        // compute collision with one constraint at the time. The next time
        // the algorithm is called, it searches for the second or
        // third obstacle.

        switch(m_algoCounter)
        {
            case 0:
                computeNextProxyPositionWithContraints0(a_goal);
                break;

            case 1:
                computeNextProxyPositionWithContraints1(a_goal);
                break;

            case 2:
                computeNextProxyPositionWithContraints2(a_goal);
                break;
        }
    }
}

//---------------------------------------------------------------------------

bool cProxyPointForceAlgo::computeNextProxyPositionWithContraints0(const cVector3d& a_goalGlobalPos)
{
    // We define the goal position of the proxy.
    cVector3d goalGlobalPos = a_goalGlobalPos;

    // To address numerical errors of the computer, we make sure to keep the proxy
    // slightly above any triangle and not directly on it. If we are using a radius of
    // zero, we need to define a default small value for epsilon
    m_epsilonInitialValue = cAbs(0.0001 * m_radius);
    if (m_epsilonInitialValue < m_epsilonBaseValue)
    {
        m_epsilonInitialValue = m_epsilonBaseValue;
    }

    // The epsilon value is dynamic (can be reduced). We set it to its initial
    // value if the proxy is not touching any triangle.
    if (m_numContacts == 0)
    {
        m_epsilon = m_epsilonInitialValue;
        m_slipping = true;
    }

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (!m_useDynamicProxy)
    {
        if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
        {
            m_nextBestProxyGlobalPos = m_proxyGlobalPos;
            m_algoCounter = 0;
            return (false);
        }
    }

    // compute the normalized form of the vector going from the
    // current proxy position to the desired goal position

    // compute the distance between the proxy and the goal positions
    double distanceProxyGoal = cDistance(m_proxyGlobalPos, goalGlobalPos);

    // A vector from the proxy to the goal
    cVector3d vProxyToGoal;
    cVector3d vProxyToGoalNormalized;
    bool proxyAndDeviceEqual;

    if (distanceProxyGoal > m_epsilon)
    {
        goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);
        vProxyToGoal.normalizer(vProxyToGoalNormalized);
        proxyAndDeviceEqual = false;
    }
    else
    {
        vProxyToGoal.zero();
        vProxyToGoalNormalized.zero();
        proxyAndDeviceEqual = true;
    }

    // Test whether the path from the proxy to the goal is obstructed.
    // For this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos;
    if (m_useDynamicProxy)
    {
        targetPos = goalGlobalPos;
    }
    else
    {
        targetPos = goalGlobalPos +
                    cMul(m_epsilonCollisionDetection, vProxyToGoalNormalized);
    }

    // setup collision detector
    m_collisionSettings.m_collisionRadius = m_radius;

    // Search for a collision between the first segment (proxy-device)
    // and the environment.
    m_collisionSettings.m_adjustObjectMotion = m_useDynamicProxy;
    m_collisionRecorderConstraint0.clear();
    bool hit = m_world->computeCollisionDetection(m_proxyGlobalPos,
                                                  targetPos,
                                                  m_collisionRecorderConstraint0,
                                                  m_collisionSettings);


    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(m_collisionRecorderConstraint0.m_nearestCollision.m_squareDistance);
        if (m_useDynamicProxy)
        {
            // retrieve new position of proxy
            cVector3d posLocal = m_collisionRecorderConstraint0.m_nearestCollision.m_adjustedSegmentAPoint;
            cGenericObject* obj = m_collisionRecorderConstraint0.m_nearestCollision.m_object;
            cVector3d posGlobal = cAdd(obj->getGlobalPos(), cMul( obj->getGlobalRot(), posLocal ));
            m_proxyGlobalPos = posGlobal;

            distanceProxyGoal = cDistance(m_proxyGlobalPos, goalGlobalPos);
            goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);
            vProxyToGoal.normalizer(vProxyToGoalNormalized);
        }


        if (collisionDistance > (distanceProxyGoal + CHAI_SMALL))
        {
            hit = false;
        }


        if (hit)
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < m_epsilon)
            {
                m_epsilon = collisionDistance;
                if (m_epsilon < m_epsilonMinimalValue)
                {
                    m_epsilon = m_epsilonMinimalValue;
                }
            }
        }
    }

    // If no collision occurs, then we move the proxy to its goal, and we're done
    if (!hit)
    {
        m_numContacts = 0;
        m_algoCounter = 0;
        m_slipping = true;
        m_nextBestProxyGlobalPos = goalGlobalPos;
        return (false);
    }

    // a first collision has occurred
    m_algoCounter = 1;

    //-----------------------------------------------------------------------
    // FIRST COLLISION OCCURES:
    //-----------------------------------------------------------------------

    // We want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle.  We want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.
    //
    // So we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal);

    // Now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // If only ASCII art were a little more expressive...
    double distanceTriangleProxy = m_epsilon / cAbs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, m_epsilon); }

    // We compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // A vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    m_proxyGlobalPos.subr(m_contactPoint0->m_globalPos, vCollisionToProxy);

    // Move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_contactPoint0->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(goalGlobalPos, nextProxyPos))
    {
        m_numContacts = 1;
        m_algoCounter = 0;
        return (true);
    }

    return (true);
}

//---------------------------------------------------------------------------

bool cProxyPointForceAlgo::computeNextProxyPositionWithContraints1(const cVector3d& a_goalGlobalPos)
{
    // The proxy is now constrained on a plane; we now calculate the nearest
    // point to the original goal (device position) on this plane; this point
    // is computed by projecting the ideal goal onto the plane defined by the
    // intersected triangle
    cVector3d goalGlobalPos = cProjectPointOnPlane(a_goalGlobalPos,
              m_proxyGlobalPos,
              m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal);

    // A vector from the proxy to the goal
    cVector3d vProxyToGoal;
    goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numContacts = 1;
        return (false);
    }

    // compute the normalized form of the vector going from the
    // current proxy position to the desired goal position
    cVector3d vProxyToGoalNormalized;
    vProxyToGoal.normalizer(vProxyToGoalNormalized);

    // Test whether the path from the proxy to the goal is obstructed.
    // For this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos = goalGlobalPos +
                          cMul(m_epsilonCollisionDetection, vProxyToGoalNormalized);

    // setup collision detector
    m_collisionSettings.m_collisionRadius = m_radius;

    // search for collision
    m_collisionSettings.m_adjustObjectMotion = false;
    m_collisionRecorderConstraint1.clear();
    bool hit = m_world->computeCollisionDetection( m_proxyGlobalPos,
                                                   targetPos,
                                                   m_collisionRecorderConstraint1,
                                                   m_collisionSettings);

    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(m_collisionRecorderConstraint1.m_nearestCollision.m_squareDistance);
        if (collisionDistance > (cDistance(m_proxyGlobalPos, goalGlobalPos) + CHAI_SMALL))
        {
            hit = false;
        }
        else
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < m_epsilon)
            {
                m_epsilon = collisionDistance;
                if (m_epsilon < m_epsilonMinimalValue)
                {
                    m_epsilon = m_epsilonMinimalValue;
                }
            }
        }
    }

    // If no collision occurs, we move the proxy to its goal, unless
    // friction prevents us from doing so.
    if (!hit)
    {
        testFrictionAndMoveProxy(goalGlobalPos,
                                 m_proxyGlobalPos,
                                 m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal,
                                 m_collisionRecorderConstraint0.m_nearestCollision.m_object);

        m_numContacts = 1;
        m_algoCounter = 0;

        return (false);
    }

    // a second collision has occurred
    m_algoCounter = 2;

    //-----------------------------------------------------------------------
    // SECOND COLLISION OCCURES:
    //-----------------------------------------------------------------------
    // We want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle.  We want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.
    //
    // So we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(m_collisionRecorderConstraint1.m_nearestCollision.m_globalNormal);

    // Now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // If only ASCII art were a little more expressive...
    double distanceTriangleProxy = m_epsilon / cAbs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, m_epsilon); }

    // We compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // A vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    m_proxyGlobalPos.subr(m_contactPoint1->m_globalPos, vCollisionToProxy);

    // Move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_contactPoint1->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(goalGlobalPos, nextProxyPos))
    {
        m_numContacts = 2;
        m_algoCounter = 0;
        return (true);
    }

    return (true);
}

//---------------------------------------------------------------------------

bool cProxyPointForceAlgo::computeNextProxyPositionWithContraints2(const cVector3d& a_goalGlobalPos)
{
    // The proxy is now constrained by two triangles and can only move along
    // a virtual line; we now calculate the nearest point to the original
    // goal (device position) along this line by projecting the ideal
    // goal onto the line.
    //
    // The line is expressed by the cross product of both surface normals,
    // which have both been oriented to point away from the device
    cVector3d line;
    m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal.crossr(m_collisionRecorderConstraint1.m_nearestCollision.m_globalNormal, line);

    // check result.
    if (line.equals(cVector3d(0,0,0)))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numContacts = 2;
        return (false);
    }

    line.normalize();

    // Compute the projection of the device position (goal) onto the line; this
    // gives us the new goal position.
    cVector3d goalGlobalPos = cProjectPointOnLine(a_goalGlobalPos, m_proxyGlobalPos, line);

    // A vector from the proxy to the goal
    cVector3d vProxyToGoal;
    goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numContacts = 2;
        return (false);
    }

    // compute the normalized form of the vector going from the
    // current proxy position to the desired goal position
    cVector3d vProxyToGoalNormalized;
    vProxyToGoal.normalizer(vProxyToGoalNormalized);

    // Test whether the path from the proxy to the goal is obstructed.
    // For this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos = goalGlobalPos +
                          cMul(m_epsilonCollisionDetection, vProxyToGoalNormalized);

    // setup collision detector
    m_collisionSettings.m_collisionRadius = m_radius;

    // search for collision
    m_collisionSettings.m_adjustObjectMotion = false;
    m_collisionRecorderConstraint2.clear();
    bool hit = m_world->computeCollisionDetection( m_proxyGlobalPos,
                                                   targetPos,
                                                   m_collisionRecorderConstraint2,
                                                   m_collisionSettings);

    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(m_collisionRecorderConstraint2.m_nearestCollision.m_squareDistance);
        if (collisionDistance > (cDistance(m_proxyGlobalPos, goalGlobalPos) + CHAI_SMALL))
        {
            hit = false;
        }
        else
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < m_epsilon)
            {
                m_epsilon = collisionDistance;
                if (m_epsilon < m_epsilonMinimalValue)
                {
                    m_epsilon = m_epsilonMinimalValue;
                }
            }
        }
    }

    // If no collision occurs, we move the proxy to its goal, unless
    // friction prevents us from doing so
    if (!hit)
    {
		cVector3d normal = cMul(0.5,cAdd(m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal,
										 m_collisionRecorderConstraint1.m_nearestCollision.m_globalNormal));
						 
        testFrictionAndMoveProxy(goalGlobalPos, 
								 m_proxyGlobalPos,
								 normal,
								m_collisionRecorderConstraint1.m_nearestCollision.m_triangle->getParent());
        m_numContacts = 2;
        m_algoCounter = 0;

        return (false);
    }

    //-----------------------------------------------------------------------
    // THIRD COLLISION OCCURES:
    //-----------------------------------------------------------------------
   // We want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle.  We want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.
    //
    // So we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(m_collisionRecorderConstraint2.m_nearestCollision.m_globalNormal);

    // Now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // If only ASCII art were a little more expressive...
    double distanceTriangleProxy = m_epsilon / cAbs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, m_epsilon); }

    // We compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // A vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    m_proxyGlobalPos.subr(m_contactPoint2->m_globalPos, vCollisionToProxy);

    // Move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_contactPoint2->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;
    m_algoCounter = 0;
    m_numContacts = 3;

    // TODO: There actually should be a third friction test to see if we
    // can make it to our new goal position, but this is generally such a
    // small movement in one iteration that it's irrelevant...

    return (true);
}


//===========================================================================
/*!
    Test whether the proxy has reached the goal point, allowing for subclass-
    specific approximations.

    \fn   virtual bool cProxyPointForceAlgo::goalAchieved(const cVector3d& a_proxy, const cVector3d& a_goal) const;
    \param    a_goal        The location to which we'd like to move the proxy
    \param    a_proxy       The current position of the proxy
    \return   true is the proxy has effectively reached the goal
*/
//===========================================================================
bool cProxyPointForceAlgo::goalAchieved(const cVector3d& a_proxy, const cVector3d& a_goal) const
{
    if (m_useDynamicProxy)
    {
        return (!(a_proxy.distance(a_goal) > 0.0));
    }
    else
    {
        return (a_proxy.distance(a_goal) < (m_epsilonBaseValue));
    }
}


//===========================================================================
/*!
    Attempt to move the proxy, subject to friction constraints.  This is called
    from computeNextBestProxyPosition when the proxy is ready to move along a
    known surface.

    \fn   void cProxyPointForceAlgo::testFrictionAndMoveProxy(const cVector3d& a_goal, 
												    const cVector3d& a_proxy,
												    cVector3d& a_normal, 
												    cGenericObject* a_parent)
    \param    a_goal        The location to which we'd like to move the proxy
    \param    a_proxy       The current position of the proxy
    \param    a_normal      The surface normal at the obstructing surface
    \param    a_parent      The surface along which we're moving
*/
//===========================================================================
void cProxyPointForceAlgo::testFrictionAndMoveProxy(const cVector3d& a_goal, 
													const cVector3d& a_proxy,
													cVector3d& a_normal, 
													cGenericObject* a_parent)
{
    // check if friction is enabled
    if (m_useFriction == false)
    {
        m_nextBestProxyGlobalPos = a_goal;
        return;
    }

    // Compute penetration depth; how far is the device "behind" the
    // plane of the obstructing surface
    cVector3d projectedGoal = cProjectPointOnPlane(m_deviceGlobalPos, a_proxy, a_normal);
    double penetrationDepth = cSub(m_deviceGlobalPos,projectedGoal).length();

    // Find the appropriate friction coefficient

    // Our dynamic and static coefficients...
    cMesh* parent_mesh = dynamic_cast<cMesh*>(a_parent);

    // Right now we can only work with cMesh's
    if (parent_mesh == NULL)
    {
        m_nextBestProxyGlobalPos = a_goal;
        return;
    }

    double mud = parent_mesh->m_material.getDynamicFriction();
    double mus = parent_mesh->m_material.getStaticFriction();

    // No friction; don't try to compute friction cones
    if ((mud == 0) && (mus == 0))
    {
        m_nextBestProxyGlobalPos = a_goal;
        return;
    }

    // The corresponding friction cone radii
    double atmd = atan(mud);
    double atms = atan(mus);

    // Compute a vector from the device to the proxy, for computing
    // the angle of the friction cone
    cVector3d vDeviceProxy = cSub(a_proxy, m_deviceGlobalPos);
    vDeviceProxy.normalize();

    // Now compute the angle of the friction cone...
    double theta = acos(vDeviceProxy.dot(a_normal));

    // Manage the "slip-friction" state machine

    // If the dynamic friction radius is for some reason larger than the
    // static friction radius, always slip
    if (mud > mus)
    {
        m_slipping = true;
    }

    // If we're slipping...
    else if (m_slipping)
    {
        if (theta < (atmd * m_frictionDynHysteresisMultiplier))
        {
            m_slipping = false;
        }
        else
        {
            m_slipping = true;
        }
    }

    // If we're not slipping...
    else
    {
        if (theta > atms)
        {
            m_slipping = true;
        }
        else
        {
            m_slipping = false;
        }
    }

    // The friction coefficient we're going to use...
    double mu;
    if (m_slipping) mu = mud;
    else mu = mus;

    // Calculate the friction radius as the absolute value of the penetration
    // depth times the coefficient of friction
    double frictionRadius = fabs(penetrationDepth * mu);

    // Calculate the distance between the proxy position and the current
    // goal position.
    double r = a_proxy.distance(a_goal);

    // If this distance is smaller than CHAI_SMALL, we consider the proxy
    // to be at the same position as the goal, and we're done...
    if (r < CHAI_SMALL)
    {
        m_nextBestProxyGlobalPos = a_proxy;
    }

    // If the proxy is outside the friction cone, update its position to
    // be on the perimeter of the friction cone...
    else if (r > frictionRadius)
    {
        m_nextBestProxyGlobalPos = cAdd(a_goal, cMul(frictionRadius/r, cSub(a_proxy, a_goal)));
    }

    // Otherwise, if the proxy is inside the friction cone, the proxy
    // should not be moved (set next best position to current position)
    else
    {
        m_nextBestProxyGlobalPos = a_proxy;
    }

    // We're done; record the fact that we're still touching an object...
    return;
}

//===========================================================================
/*!
    This method uses the information computed earlier in
    computeNextProxyPosition() to determine the force to apply to the device.
    The function computes a force proportional to the distance between the
    positions of the proxy and the device and scaled by the average
    stiffness of each contact triangle.

    \fn       void cProxyPointForceAlgo::updateForce()
*/
//===========================================================================
void cProxyPointForceAlgo::updateForce()
{
    // initialize variables
    double stiffness = 0.0;
    cVector3d normal;
    normal.zero();

    // if there are no contacts between proxy and environment, no force is applied
    if (m_numContacts == 0)
    {
        m_lastGlobalForce.zero();
        return;
    }

    //---------------------------------------------------------------------
    // stiffness and surface normal estimation
    //---------------------------------------------------------------------
    else if (m_numContacts == 1)
    {
        // compute stiffness
        stiffness = ( m_contactPoint0->m_triangle->getParent()->m_material.getStiffness() );

        // compute surface normal
        normal.add(m_contactPoint0->m_globalNormal);
    }

    // if there are two contact points, the stiffness is the average of the
    // stiffnesses of the two intersected triangles' meshes
    else if (m_numContacts == 2)
    {
        // compute stiffness
        stiffness = ( m_contactPoint0->m_triangle->getParent()->m_material.getStiffness() +
                      m_contactPoint1->m_triangle->getParent()->m_material.getStiffness() ) / 2.0;

        // compute surface normal
        normal.add(m_contactPoint0->m_globalNormal);
        normal.add(m_contactPoint1->m_globalNormal);
        normal.mul(1.0/2.0);
    }

    // if there are three contact points, the stiffness is the average of the
    // stiffnesses of the three intersected triangles' meshes
    else if (m_numContacts == 3)
    {
        // compute stiffness
        stiffness = ( m_contactPoint0->m_triangle->getParent()->m_material.getStiffness() +
                      m_contactPoint1->m_triangle->getParent()->m_material.getStiffness() +
                      m_contactPoint2->m_triangle->getParent()->m_material.getStiffness() ) / 3.0;

        // compute surface normal
        normal.add(m_contactPoint0->m_globalNormal);
        normal.add(m_contactPoint1->m_globalNormal);
        normal.add(m_contactPoint2->m_globalNormal);
        normal.mul(1.0/3.0);
    }

    //---------------------------------------------------------------------
    // computing a force (Hooke's law)
    //---------------------------------------------------------------------

    // compute the force by modeling a spring between the proxy and the device
    cVector3d force;
    m_proxyGlobalPos.subr(m_deviceGlobalPos, force);
    force.mul(stiffness);
    m_lastGlobalForce = force;

    // compute tangential and normal forces
    if ((force.lengthsq() > 0) && (m_numContacts > 0))
    {
        m_normalForce = cProject(force, normal);
        force.subr(m_normalForce, m_tangentialForce);
    }
    else
    {
        m_tangentialForce.zero();
        m_normalForce = force;
    }


    //---------------------------------------------------------------------
    // force shading (optional)
    //---------------------------------------------------------------------

    if ((m_useForceShading) && (m_numContacts == 1))
    {
        // get vertices and normals related to contact triangle
        cVector3d vertex0 = cAdd(m_contactPoint0->m_object->getGlobalPos(), cMul(m_contactPoint0->m_object->getGlobalRot(), m_contactPoint0->m_triangle->getVertex0()->getPos()));
        cVector3d vertex1 = cAdd(m_contactPoint0->m_object->getGlobalPos(), cMul(m_contactPoint0->m_object->getGlobalRot(), m_contactPoint0->m_triangle->getVertex1()->getPos()));
        cVector3d vertex2 = cAdd(m_contactPoint0->m_object->getGlobalPos(), cMul(m_contactPoint0->m_object->getGlobalRot(), m_contactPoint0->m_triangle->getVertex2()->getPos()));
        cVector3d normal0 = cMul(m_contactPoint0->m_object->getGlobalRot(), m_contactPoint0->m_triangle->getVertex0()->getNormal());
        cVector3d normal1 = cMul(m_contactPoint0->m_object->getGlobalRot(), m_contactPoint0->m_triangle->getVertex1()->getNormal());
        cVector3d normal2 = cMul(m_contactPoint0->m_object->getGlobalRot(), m_contactPoint0->m_triangle->getVertex2()->getNormal());

        // compute angles between normals. If the angles are very different, then do not apply shading.
        double angle01 = cAngle(normal0, normal1);
        double angle02 = cAngle(normal0, normal2);
        double angle12 = cAngle(normal1, normal2);

        if ((angle01 < m_forceShadingAngleThreshold) || (angle02 < m_forceShadingAngleThreshold) || (angle12 < m_forceShadingAngleThreshold))
        {
            double a0 = 0; 
						double a1 = 0;
            cProjectPointOnPlane(m_contactPoint0->m_globalPos, vertex0, vertex1, vertex2, a0, a1);

            cVector3d normalShaded = cAdd(
                       cMul(0.5, cAdd(cMul(a0, normal1), cMul((1-a0), normal0))),
                       cMul(0.5, cAdd(cMul(a1, normal2), cMul((1-a1), normal0)))
                       );
            normalShaded.normalize();

            if (cAngle(normalShaded, normal) > 1.0)
            {
                normalShaded.negate();
            }

            if (cAngle(normal, normalShaded) < m_forceShadingAngleThreshold)
            {
                double forceMagnitude = m_normalForce.length();
                force = cAdd( cMul(forceMagnitude, normalShaded), m_tangentialForce);
                m_lastGlobalForce = force;
                normal = normalShaded;

                // update tangential and normal forces again
                if ((force.lengthsq() > 0) && (m_numContacts > 0))
                {
                    m_normalForce = cProject(force, normal);
                    force.subr(m_normalForce, m_tangentialForce);
                }
                else
                {
                    m_tangentialForce.zero();
                    m_normalForce = force;
                }
            }
        }
    }
}


