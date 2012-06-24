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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 717 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CGELWorld.h"
//---------------------------------------------------------------------------

//==========================================================================
/*!
    Extends cGELWorld to support collision detection.

    \fn     bool cGELWorldCollision::computeCollision(cVector3d& a_segmentPointA,
                                        cVector3d& a_segmentPointB,
                                        cCollisionRecorder& a_recorder,
                                        cCollisionSettings& a_settings)
    \param    a_segmentPointA  Start point of segment.
    \param    a_segmentPointB  End point of segment.
    \param    a_recorder  Stores all collision events
    \param    a_settings  Contains collision settings information.
    \return   Return \b true if a collision has occurred.
*/
//===========================================================================
bool cGELWorldCollision::computeCollision(cVector3d& a_segmentPointA,
                                          cVector3d& a_segmentPointB,
                                          cCollisionRecorder& a_recorder,
                                          cCollisionSettings& a_settings)
{
    list<cGELMesh*>::iterator i;
    bool result = false;
    for(i = m_gelWorld->m_gelMeshes.begin(); i != m_gelWorld->m_gelMeshes.end(); ++i)
    {
        cGELMesh *nextItem = *i;
        bool collide = nextItem->computeCollisionDetection(a_segmentPointA, a_segmentPointB, a_recorder,
                        a_settings);
        if (collide) { result = true; }
    }
    return (result);
}


//==========================================================================
/*!
    Constructor of cDefWorld.

    \fn       cGELWorld::cGELWorld()
    \return   Return pointer to new cGELWorld instance.
*/
//===========================================================================
cGELWorld::cGELWorld()
{
    // reset simulation time.
    m_simulationTime = 0.0;

    // set a default value for the integration time step [s].
    m_integrationTime = 1.0f / 400.0f;

    // create a collision detector for world
    m_collisionDetector = new cGELWorldCollision(this);
}


//===========================================================================
/*!
    Destructor of cGELWorld.

    \fn       cGELWorld::~cGELWorld()
*/
//===========================================================================
cGELWorld::~cGELWorld()
{
    m_gelMeshes.clear();
}


//===========================================================================
/*!
    Render world in OpenGL.

    \fn       void cGELWorld::render(cRenderOptions& a_options)
    \param    a_options  Rendering options (see cGenericObject)
*/
//===========================================================================
void cGELWorld::render(cRenderOptions& a_options)
{
    list<cGELMesh*>::iterator i;

    for(i = m_gelMeshes.begin(); i != m_gelMeshes.end(); ++i)
    {
        cGELMesh* nextItem = *i;
        nextItem->renderSceneGraph(a_options);
    }
}

//===========================================================================
/*!
    Clear external forces on all deformable objects in scene.

    \fn       void cGELWorld::clearExternalForces()
*/
//===========================================================================
void cGELWorld::clearExternalForces()
{
    list<cGELMesh*>::iterator i;

    for(i = m_gelMeshes.begin(); i != m_gelMeshes.end(); ++i)
    {
        cGELMesh *nextItem = *i;
        nextItem->clearExternalForces();
    }
}


//===========================================================================
/*!
    Compute simulation for a_time time interval.

    \fn       void cGELWorld::updateDynamics(double a_time)
*/
//===========================================================================
void cGELWorld::updateDynamics(double a_time)
{
    list<cGELMesh*>::iterator i;

    double nextTime = m_simulationTime + a_time;

    double integrationTime = cMin(m_integrationTime, a_time);

    while (m_simulationTime < nextTime)
    {
        // clear all internal forces of each model
        for(i = m_gelMeshes.begin(); i != m_gelMeshes.end(); ++i)
        {
            cGELMesh *nextItem = *i;
            nextItem->clearForces();
        }

        // compute all internal forces for ach model
        for(i = m_gelMeshes.begin(); i != m_gelMeshes.end(); ++i)
        {
            cGELMesh *nextItem = *i;
            nextItem->computeForces();
        }

        // compute next pose of model
        for(i = m_gelMeshes.begin(); i != m_gelMeshes.end(); ++i)
        {
            cGELMesh *nextItem = *i;
            nextItem->computeNextPose(integrationTime);
        }

        // apply next pose
        for(i = m_gelMeshes.begin(); i != m_gelMeshes.end(); ++i)
        {
            cGELMesh *nextItem = *i;
            nextItem->applyNextPose();
        }

        // update simulation time
        m_simulationTime = m_simulationTime + m_integrationTime;
    }
}

//===========================================================================
/*!
    Update vertices of all objects.

    \fn       void cGELWorld::updateDynamics(double a_time)
*/
//===========================================================================
void cGELWorld::updateSkins()
{
    // update surface mesh to latest skeleton configuration
    list<cGELMesh*>::iterator i;
    for(i = m_gelMeshes.begin(); i != m_gelMeshes.end(); ++i)
    {
        cGELMesh *nextItem = *i;
        nextItem->updateVertexPosition();
    }
}

