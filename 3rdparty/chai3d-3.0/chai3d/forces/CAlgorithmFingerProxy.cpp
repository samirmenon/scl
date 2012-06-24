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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 349 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "forces/CAlgorithmFingerProxy.h"
#include "world/CWorld.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cAlgorithmFingerProxy.

    \fn       cAlgorithmFingerProxy::cAlgorithmFingerProxy()
*/
//===========================================================================
cAlgorithmFingerProxy::cAlgorithmFingerProxy()
{
    // initialize world pointer
    m_world = NULL;

    // no contacts yet between proxy and environment
    m_numCollisionEvents = 0;

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

    // by default, we do not use dynamic proxy (which handles dynamic objects)
    m_useDynamicProxy = false;

    // initialize dynamic proxy members
    m_collisionRecorderConstraint0.m_nearestCollision.clear();
    m_collisionRecorderConstraint1.m_nearestCollision.clear();
    m_collisionRecorderConstraint2.m_nearestCollision.clear();

    // friction properties
    m_slipping = true;

    // setup collision detector seetings
    m_collisionSettings.m_checkForNearestCollisionOnly  = true;
    m_collisionSettings.m_returnMinimalCollisionData    = false;
    m_collisionSettings.m_checkVisibleObjects           = false;
    m_collisionSettings.m_checkHapticObjects            = true;
    m_collisionSettings.m_adjustObjectMotion            = m_useDynamicProxy;

    // setup pointers to collision recoders so that user can access
    // collision information about each contact point.
    m_collisionEvents[0] = &(m_collisionRecorderConstraint0.m_nearestCollision);
    m_collisionEvents[1] = &(m_collisionRecorderConstraint1.m_nearestCollision);
    m_collisionEvents[2] = &(m_collisionRecorderConstraint2.m_nearestCollision);

    // initialize algorithm variables
    m_algoCounter = 0;

    // render settings (for debug purposes)
    m_showEnabled = true;
}


//===========================================================================
/*!
    Initialize the algorithm, including setting the pointer to the world
    in which the algorithm is to operate, and setting the initial position
    of the device.

    \fn       void cAlgorithmFingerProxy::initialize(cWorld* a_world, 
					const cVector3d& a_initialGlobalPosition)
    \param    a_world  Pointer to world in which force algorithm is operating.
    \param    a_initialGlobalPosition  Initial position of the device.
*/
//===========================================================================
void cAlgorithmFingerProxy::initialize(cWorld* a_world, const cVector3d& a_initialGlobalPosition)
{
    // reset some variables
    m_lastGlobalForce.zero();

    // no contacts yet between proxy and environment
    m_numCollisionEvents = 0;

    // the proxy can slip along surfaces
    m_slipping = true;

    // initialize counter
    m_algoCounter = 0;

    // initialize device and proxy positions
    m_deviceGlobalPos = a_initialGlobalPosition;
    m_proxyGlobalPos  = a_initialGlobalPosition;

    // set pointer to world in which force algorithm operates
    m_world = a_world;
}


//===========================================================================
/*!
    Reset the algorithm. Set the proxy position to the device position.

    \fn       void cAlgorithmFingerProxy::reset()
*/
//===========================================================================
void cAlgorithmFingerProxy::reset()
{
    // reset some variables
    m_lastGlobalForce.zero();

    // no contacts yet between proxy and environment
    m_numCollisionEvents = 0;

    // the proxy can slip along surfaces
    m_slipping = true;

    // initialize counter
    m_algoCounter = 0;

    // set proxy position to be equal to the device position
    m_proxyGlobalPos = m_deviceGlobalPos;
}


//===========================================================================
/*!
    Set the epsilon value which is used during geometry computation of the
    proxy model.

    \fn       void cAlgorithmFingerProxy::setEpsilonBaseValue(double a_value)
*/
//===========================================================================
void cAlgorithmFingerProxy::setEpsilonBaseValue(double a_value)
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

    \fn       cVector3d cAlgorithmFingerProxy::computeForces(
                    const cVector3d& a_toolPos,
                    const cVector3d& a_toolVel)
    \param    a_toolPos  New position of tool
    \param    a_toolVel  New velocity of tool
    \return   Return the force to add to the device due to any collisions
              with meshes.
*/
//===========================================================================
cVector3d cAlgorithmFingerProxy::computeForces(const cVector3d& a_toolPos,
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

    \fn		void cAlgorithmFingerProxy::computeNextBestProxyPosition(const cVector3d& a_goal)
	\param  a_goal  The goal towards which to move the proxy, subject to constraints
*/
//===========================================================================
void cAlgorithmFingerProxy::computeNextBestProxyPosition(const cVector3d& a_goal)
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

bool cAlgorithmFingerProxy::computeNextProxyPositionWithContraints0(const cVector3d& a_goalGlobalPos)
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
    if (m_numCollisionEvents == 0)
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


        if (collisionDistance > (distanceProxyGoal + C_SMALL))
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
        m_numCollisionEvents = 0;
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
    m_proxyGlobalPos.subr(m_collisionEvents[0]->m_globalPos, vCollisionToProxy);

    // Move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_collisionEvents[0]->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(goalGlobalPos, nextProxyPos))
    {
        m_numCollisionEvents = 1;
        m_algoCounter = 0;
        return (true);
    }

    return (true);
}

//---------------------------------------------------------------------------

bool cAlgorithmFingerProxy::computeNextProxyPositionWithContraints1(const cVector3d& a_goalGlobalPos)
{
	cVector3d goalGlobalPos;

    if (m_collisionRecorderConstraint0.m_nearestCollision.m_object->m_material->getUseHapticShading())
	{
		// computed adjusted surface normal for modelling haptic shading and texture rendering
		cVector3d normal = computeShadedSurfaceNormal(&m_collisionRecorderConstraint0.m_nearestCollision);

		// The proxy is now constrained on the shaded plane; we now calculate the nearest
		// point to the original goal (device position) on this shaded plane; this point
		// is computed by projecting the desired goal onto the shaded plane.
		cVector3d goalGlobalPosOnForceShadingPlane = cProjectPointOnPlane(a_goalGlobalPos,
				  m_proxyGlobalPos,
				  normal);

		// We now project the point from the shaded plane back onto the original triangle
		// (See publication: The Haptic Display of Complex Graphical Environments from
		//  Ruspini and al. for more information about the shading algorithm)
		goalGlobalPos = cProjectPointOnPlane(goalGlobalPosOnForceShadingPlane,
											 m_proxyGlobalPos,
											 m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal);
	}
	else
	{
		// The proxy is now constrained on a plane; we now calculate the nearest
		// point to the original goal (device position) on this plane; this point
		// is computed by projecting the desired goal onto the plane defined by the
		// intersected triangle.
		goalGlobalPos = cProjectPointOnPlane(a_goalGlobalPos,
											 m_proxyGlobalPos,
											 m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal);
	}

    // A vector from the proxy to the goal
    cVector3d vProxyToGoal;
    goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numCollisionEvents = 1;
        return (false);
    }

    // compute the normalized vector going from the
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
        if (collisionDistance > (cDistance(m_proxyGlobalPos, goalGlobalPos) + C_SMALL))
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

        m_numCollisionEvents = 1;
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
    m_proxyGlobalPos.subr(m_collisionEvents[1]->m_globalPos, vCollisionToProxy);

    // Move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_collisionEvents[1]->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(goalGlobalPos, nextProxyPos))
    {
        m_numCollisionEvents = 2;
        m_algoCounter = 0;
        return (true);
    }

    return (true);
}

//---------------------------------------------------------------------------

bool cAlgorithmFingerProxy::computeNextProxyPositionWithContraints2(const cVector3d& a_goalGlobalPos)
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
        m_numCollisionEvents = 2;
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
        m_numCollisionEvents = 2;
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
        if (collisionDistance > (cDistance(m_proxyGlobalPos, goalGlobalPos) + C_SMALL))
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
                                 m_collisionRecorderConstraint1.m_nearestCollision.m_object);
        m_numCollisionEvents = 2;
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
    m_proxyGlobalPos.subr(m_collisionEvents[2]->m_globalPos, vCollisionToProxy);

    // Move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_collisionEvents[2]->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;
    m_algoCounter = 0;
    m_numCollisionEvents = 3;

    // TODO: There actually should be a third friction test to see if we
    // can make it to our new goal position, but this is generally such a
    // small movement in one iteration that it's irrelevant...

    return (true);
}


//===========================================================================
/*!
    Test whether the proxy has reached the goal point, allowing for subclass-
    specific approximations.

    \fn   virtual bool cAlgorithmFingerProxy::goalAchieved(const cVector3d& a_proxy, const cVector3d& a_goal) const;
    \param    a_goal        The location to which we'd like to move the proxy
    \param    a_proxy       The current position of the proxy
    \return   true is the proxy has effectively reached the goal
*/
//===========================================================================
bool cAlgorithmFingerProxy::goalAchieved(const cVector3d& a_proxy, const cVector3d& a_goal) const
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

    \fn   void cAlgorithmFingerProxy::testFrictionAndMoveProxy(const cVector3d& a_goal, 
												    const cVector3d& a_proxy,
												    cVector3d& a_normal, 
												    cGenericObject* a_parent)
    \param    a_goal        The location to which we'd like to move the proxy
    \param    a_proxy       The current position of the proxy
    \param    a_normal      The surface normal at the obstructing surface
    \param    a_parent      The surface along which we're moving
*/
//===========================================================================
void cAlgorithmFingerProxy::testFrictionAndMoveProxy(const cVector3d& a_goal, 
													const cVector3d& a_proxy,
													cVector3d& a_normal, 
													cGenericObject* a_parent)
{
    // check if friction is enabled
    if (!a_parent->m_material->getUseHapticFriction())
    {
        m_nextBestProxyGlobalPos = a_goal;
        return;
    }

    // Compute penetration depth; how far is the device "behind" the
    // plane of the obstructing surface
    cVector3d projectedGoal = cProjectPointOnPlane(m_deviceGlobalPos, a_proxy, a_normal);
    double penetrationDepth = cSub(m_deviceGlobalPos,projectedGoal).length();

    // Find the appropriate friction coefficient
    double mud = a_parent->m_material->getDynamicFriction();
    double mus = a_parent->m_material->getStaticFriction();

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

    // If this distance is smaller than C_SMALL, we consider the proxy
    // to be at the same position as the goal, and we're done...
    if (r < C_SMALL)
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

    \fn       void cAlgorithmFingerProxy::updateForce()
*/
//===========================================================================
void cAlgorithmFingerProxy::updateForce()
{
    // initialize variables
    double stiffness;
    cVector3d averagedSurfaceNormal;

    //---------------------------------------------------------------------
    // verify for contacts
    //---------------------------------------------------------------------

    // if proxy is not touching any object, then the force is simply set to zero.
    if (m_numCollisionEvents == 0)
    {
        m_tangentialForce.zero();
        m_normalForce.zero();
        m_lastGlobalForce.zero();
        return;
    }

    //---------------------------------------------------------------------
    // compute stiffness and surface normal estimation
    //---------------------------------------------------------------------

    // initialize surface normal and stiffness values
    averagedSurfaceNormal.zero();
    stiffness = 0.0;

    // compute the average surface normal and stiffness
    for (unsigned int i=0; i<m_numCollisionEvents; i++)
    {
        // compute stiffness
        stiffness += ( m_collisionEvents[i]->m_object->m_material->getStiffness() );

        // compute surface normal
        averagedSurfaceNormal.add(m_collisionEvents[i]->m_globalNormal);
    }

    if (m_numCollisionEvents > 0)
    {
        double scale = 1.0/(double)m_numCollisionEvents;
        stiffness *= scale;
        averagedSurfaceNormal.mul(scale);
    }


    //---------------------------------------------------------------------
    // computing a force using Hooke's law
    //---------------------------------------------------------------------

    // compute the force by modeling a spring between the proxy and the device
    cVector3d force;
    m_proxyGlobalPos.subr(m_deviceGlobalPos, force);
    force.mul(stiffness);

    if (force.lengthsq() == 0)
    {
        m_normalForce.zero();
        m_tangentialForce.zero();
        m_lastGlobalForce.zero();
        return;
    }


    //---------------------------------------------------------------------
    // force texture rendering
    //---------------------------------------------------------------------

    // is haptic texture rendering enabled?
    bool useTexture = m_collisionEvents[0]->m_object->m_material->getUseHapticTexture();
    if (useTexture)
    {
        for (unsigned int i=0; i<1; i++)
        {
            if (m_collisionEvents[i] != NULL)
            {
                // get triangle physical normal
                cVector3d normal = m_collisionEvents[i]->m_globalNormal;

                // compute normal and tangential components of the force for current triangle
                cVector3d tangentialForce, normalForce;

                // compute tangential and normal forces for current force
                normalForce = cProject(force, normal);
                force.subr(normalForce, tangentialForce);
                
                // get vertices of contact triangles
                cVector3d vertex0 = cAdd(m_collisionEvents[i]->m_object->getGlobalPos(), cMul(m_collisionEvents[i]->m_object->getGlobalRot(), m_collisionEvents[i]->m_triangle->getVertex0()->getLocalPos()));
                cVector3d vertex1 = cAdd(m_collisionEvents[i]->m_object->getGlobalPos(), cMul(m_collisionEvents[i]->m_object->getGlobalRot(), m_collisionEvents[i]->m_triangle->getVertex1()->getLocalPos()));
                cVector3d vertex2 = cAdd(m_collisionEvents[i]->m_object->getGlobalPos(), cMul(m_collisionEvents[i]->m_object->getGlobalRot(), m_collisionEvents[i]->m_triangle->getVertex2()->getLocalPos()));

				// get contact information
                double v01 = m_collisionEvents[i]->m_trianglePosV01;
                double v02 = m_collisionEvents[i]->m_trianglePosV02;

                //--------------------------------------------------------------------------------------
                // TEXTURE RENDERING
                //--------------------------------------------------------------------------------------

				// check texture coordinates
				bool textureAvailable = false;		
				if (useTexture && m_collisionEvents[i]->m_object->getUseTexture())
				{		
					// retrieve pointer to texture texture object
					cTexture1d* texture = m_collisionEvents[i]->m_object->m_texture;
					
					// sanity check
					if (texture != NULL)
					{
						// retrieve the texture coordinate for each triangle vertex
						cVector3d texCoord0 = m_collisionEvents[i]->m_triangle->getVertex0()->getTexCoord();
						cVector3d texCoord1 = m_collisionEvents[i]->m_triangle->getVertex1()->getTexCoord();
						cVector3d texCoord2 = m_collisionEvents[i]->m_triangle->getVertex2()->getTexCoord();

                        // compute texture variations
                        cVector3d vTexCoord01 = texCoord1 - texCoord0;
                        cVector3d vTexCoord02 = texCoord2 - texCoord0;

						// compute the exact texture coordinate at the contact point
						cVector3d texCoord = texCoord0 + v01 * vTexCoord01 + v02 * vTexCoord02; 

						// define three neigbour pixels along axis X with pixelXb being centered on the contact point.
						// pixelXa is located just before pixelXb, and pixelXc is located just after pixelXb
						// we will use these pixels to compute the image gradient along the X axi. 
						// we apply the same strategy Y axis.
						int pixelXa, pixelXb, pixelXc;
						int pixelYa, pixelYb, pixelYc;

						// compute nearest pixels along the X axis.
						double px = (double)(texture->m_image->getWidth()) * texCoord(0);
						pixelXb = (int)(floor(px));
						pixelXc = pixelXb + 1;
						pixelXa = pixelXb - 1;

						// compute nearest pixels along the Y axis.
						double py = (double)(texture->m_image->getHeight()) * texCoord(1);
						pixelYb = (int)(floor(py));
						pixelYc = pixelYb + 1;
						pixelYa = pixelYb - 1;

						// given the neighbour pixels positions, we now compute the color intensity or
						// gray scale value. We will use these values as a height map.
						cColorb colorXaYb, colorXbYb, colorXcYb, colorXbYa, colorXbYc;

						texture->m_image->getPixelColor(pixelXa, pixelYb, colorXaYb);
						texture->m_image->getPixelColor(pixelXb, pixelYb, colorXbYb);
						texture->m_image->getPixelColor(pixelXc, pixelYb, colorXcYb);
						texture->m_image->getPixelColor(pixelXb, pixelYa, colorXbYa);
						texture->m_image->getPixelColor(pixelXb, pixelYc, colorXbYc);
	                    
						const double CONSTANT = (1.0/255.0);
						double levelXaYb = CONSTANT * (double)(colorXaYb.getLuminance());
						double levelXbYb = CONSTANT * (double)(colorXbYb.getLuminance());
						double levelXcYb = CONSTANT * (double)(colorXcYb.getLuminance());
						double levelXbYa = CONSTANT * (double)(colorXbYa.getLuminance());
						double levelXbYc = CONSTANT * (double)(colorXbYc.getLuminance());
	                   
						// from the computed pixel values we define the gradient at the contact point in texture space.
						// this operation is performed by using the derivative of a polynmial of degree 2 centered on 
						// point (pixelXb, pixelYb).
						double ppx = px - (double)pixelXb - 0.5;
						double ppy = py - (double)pixelYb - 0.5;
						double deltaX = (levelXaYb + levelXcYb - 2 * levelXbYb) * ppx  + 0.5 * (levelXcYb - levelXaYb);
						double deltaY = (levelXbYa + levelXbYc - 2 * levelXbYb) * ppx  + 0.5 * (levelXbYc - levelXbYa);
						cVector3d gradientTexture;
						gradientTexture.set(deltaX, deltaY, 0.0);

						// given the gradient defined on the texture image, we now project this gradient on the triangle (or surface model)
						cVector3d gradientSurface(0,0,0);

						// boolean used to inform us if the projection suceeds
						bool success = true;

						// setup projection matrix
                        double length_vTexCoord01 = vTexCoord01.length();
                        double length_vTexCoord02 = vTexCoord02.length();

                        if ((length_vTexCoord01 > 0.0) && (length_vTexCoord02 > 0.0))
                        {
                            // normalize vectore in texture coordinate space
                            vTexCoord01.div(length_vTexCoord01);
                            vTexCoord02.div(length_vTexCoord02);

                            double m00 = vTexCoord01(0);
						    double m01 = vTexCoord02(0);
						    double m10 = vTexCoord01(1);
						    double m11 = vTexCoord02(1);

						    // compute projected gradient
						    double det = m00 * m11 - m10 * m01;
						    if (det == 0.0)
						    {
							    success = false;
						    }
						    else
						    {
							    double dtx = gradientTexture(0) ;
							    double dty = gradientTexture(1) ;

							    double f = 1.0 / det;
							    double c01 = f * ( m11 * dtx - m01 * dty);
							    double c02 = f * (-m10 * dtx + m00 * dty);

                                cVector3d v01 = vertex1 - vertex0;
                                cVector3d v02 = vertex2 - vertex0;
                                double v01_length = v01.length();
                                double v02_length = v02.length();
                                
                                if ((v01_length > 0.0) && (v02_length > 0.0))
                                {
                                    gradientSurface = (c01 / cSqr(v01_length)) * v01 + (c02 / cSqr(v02_length)) * v02;
                                }
                                else
                                {
                                    gradientSurface.zero();
                                }							
						    }
                        }
		
						// if the operation succeeds, we compute a new surface normal that will
						// be used to reoriente the previously computed reaction force
						if (success)
						{
                            cVector3d newNormal = normal + gradientSurface * m_collisionEvents[i]->m_object->m_material->getTextureLevel();
							double length = newNormal.length(); 
							if (length > 0.0)
							{
								newNormal.div(length);
								if (cAngle(newNormal, normal) > 1.57) 
								{ 
									newNormal.negate(); 
								}
								normal = newNormal;
							}
						}

						// we now compute the magnitude of the current surface normal and create a new force vector in the 
						// direction of the gradient normal.
						double forceMagnitude = normalForce.length();
						force = cAdd(tangentialForce, cMul(forceMagnitude, normal));
                        surfaceNormal = normal;
					}
				}          
            }
        }
    }


    //---------------------------------------------------------------------
    // return result
    //---------------------------------------------------------------------
    
    // compute tangential and normal forces for current triangle
    m_normalForce = cProject(force, averagedSurfaceNormal);
    force.subr(m_normalForce, m_tangentialForce);

    // return computed force
    m_lastGlobalForce = force;
}


//===========================================================================
/*!
    TBC

    \fn       cVector3d cAlgorithmFingerProxy::computeAdjustedSurfaceNormal(cCollisionEvent* a_contactPoint
*/
//===========================================================================
cVector3d cAlgorithmFingerProxy::computeShadedSurfaceNormal(cCollisionEvent* a_contactPoint)
{
    // get triangle physical normal
    cVector3d normal = a_contactPoint->m_globalNormal;

    // is force shading enabled for that object?
    bool useForceShading = a_contactPoint->m_object->m_material->getUseHapticShading();

    if (useForceShading)
    {
        if (a_contactPoint != NULL)
        {            
            // get vertices of contact triangles
            cVector3d vertex0 = cAdd(a_contactPoint->m_object->getGlobalPos(), cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangle->getVertex0()->getLocalPos()));
            cVector3d vertex1 = cAdd(a_contactPoint->m_object->getGlobalPos(), cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangle->getVertex1()->getLocalPos()));
            cVector3d vertex2 = cAdd(a_contactPoint->m_object->getGlobalPos(), cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangle->getVertex2()->getLocalPos()));

            // get vertex normals of contact triangle
            cVector3d normal0 = cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangle->getVertex0()->getNormal());
            cVector3d normal1 = cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangle->getVertex1()->getNormal());
            cVector3d normal2 = cMul(a_contactPoint->m_object->getGlobalRot(), a_contactPoint->m_triangle->getVertex2()->getNormal());

			// project the current contact point on triangle
			double a0 = 0; 
			double a1 = 0;
			cProjectPointOnPlane(a_contactPoint->m_globalPos, vertex0, vertex1, vertex2, a0, a1);
			cVector3d vertex = cAdd(vertex0, cMul(a0, cSub(vertex1, vertex0)), cMul(a1, cSub(vertex2, vertex0)));

			// compute area of triangle
            double area  = cTriangleArea(vertex0, vertex1, vertex2);

            // compute areas of three sub-triangles formed by the three vertices of the triangle and the contact point.
            double area0 = cTriangleArea(vertex, vertex1, vertex2);
            double area1 = cTriangleArea(vertex, vertex0, vertex2);
            double area2 = cTriangleArea(vertex, vertex0, vertex1);
			
			// compute weights based on position of contact point
			double c0, c1, c2;
			if (area > 0.0)
            {
				c0 = area0/area;
				c1 = area1/area;
				c2 = area2/area;
			}
			else
			{
				c0 = c1 = c2 = (1.0/3.0);
			}


            //--------------------------------------------------------------------------------------
            // FORCE SHADING
            //--------------------------------------------------------------------------------------
            if (useForceShading)
            {
                // compute angles between normals
                double angle01 = cAngle(normal0, normal1);
                double angle02 = cAngle(normal0, normal2);
                double angle12 = cAngle(normal1, normal2);

                // if angles are withing a certain threshold, perform shading
                if ((angle01 < m_forceShadingAngleThreshold) || (angle02 < m_forceShadingAngleThreshold) || (angle12 < m_forceShadingAngleThreshold))
                {
                    // just like with Fong sshading in computer graphics, we compute the surface normal at the actual contact point 
                    // by combining the normals defined at each vertex on the triangle
                    cVector3d normalShaded = normal;
                   
                    // compute new interpolated normal 
                    normalShaded = cAdd( cMul(c0, normal0), cMul(c1, normal1), cMul(c2, normal2));

                    // sanity check and normalization
                    double length = normalShaded.length();
                    if (length > 0.0)
                    {
                        normalShaded.div(length);
                    }
                    else
                    {
                        normalShaded = normal;
                    }

					normal = normalShaded;
                }
            }
        }
    }


    //---------------------------------------------------------------------
    // return result
    //---------------------------------------------------------------------
    return (normal);
}

//===========================================================================
/*!
    Render the force algorithm graphicaly in OpenGL.

    \fn     void cAlgorithmFingerProxy::render(cRenderOptions& a_options)
    \param  a_options  Rendering options.
*/
//===========================================================================
void cAlgorithmFingerProxy::render(cRenderOptions& a_options)
{
	if (!m_showEnabled) { return; }

	/////////////////////////////////////////////////////////////////////////
	// Render parts that are always opaque
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
	{
	    // disable lighting
	    glDisable(GL_LIGHTING);

		glLineWidth(1.0);

        cVector3d posA = m_proxyGlobalPos;
        cVector3d posB = m_proxyGlobalPos + 0.1 * surfaceNormal;

	    // draw line
	    glBegin(GL_LINES);
		    glColor3f(1.0, 0.0, 0.0);
		    glVertex3dv(&posA(0) );
		    glVertex3dv(&posB(0) );
	    glEnd();

	    // restore lighting to default value
	    glEnable(GL_LIGHTING);
	}
}

