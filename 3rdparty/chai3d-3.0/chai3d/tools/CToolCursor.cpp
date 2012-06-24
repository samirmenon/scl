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
    \author    Federico Barbagli
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 365 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "tools/CToolCursor.h"
#include "graphics/CTriangle.h"
//---------------------------------------------------------------------------

//==========================================================================
/*!
    Constructor of cToolCursor.

    \fn       cToolCursor::cToolCursor(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
    \param    a_world  World in which the tool will operate.
*/
//===========================================================================
cToolCursor::cToolCursor(cWorld* a_parentWorld):cGenericTool(a_parentWorld)
{
    // create a single point contact
    m_hapticPoint = new cHapticPoint(this);

    // add point to list
    m_hapticPoints.push_back(m_hapticPoint);

    // show proxy spheres only
    setShowContactPoints(true, false);
}


//==========================================================================
/*!
    Destructor of cToolCursor.

    \fn       cToolCursor::~cToolCursor()
*/
//===========================================================================
cToolCursor::~cToolCursor()
{
    delete m_hapticPoint;
}


//===========================================================================
/*!
    Update the position and orientation of the tool image.

    \fn       void cToolCursor::computeInteractionForces()
*/
//===========================================================================
void cToolCursor::updateToolImagePosition()
{
    // set the position and orientation of the tool image to be equal to the 
    // one of the contact point proxy.
    cVector3d pos = m_hapticPoint->getLocalPosProxy();
    m_image->setLocalPos(pos);
    m_image->setLocalRot(m_deviceLocalRot);
}


//===========================================================================
/*!
    Compute the interaction forces between the cursor and the virtual
    environment.

    \fn       void cToolCursor::computeInteractionForces()
*/
//===========================================================================
void cToolCursor::computeInteractionForces()
{
    // compute force interaction forces at contact point
    m_lastComputedGlobalForce = m_hapticPoint->computeInteractionForces(m_deviceGlobalPos,
                                                                        m_deviceGlobalRot,
                                                                        m_deviceGlobalLinVel,
                                                                        m_deviceGlobalAngVel);
    m_lastComputedGlobalTorque.zero();
    m_lastComputedGripperForce = 0.0;
}


//==========================================================================
/*!
    Render the current tool in OpenGL.

    \fn       void cToolCursor::render(cRenderOptions& a_options)
    \param    a_options  Rendering options.
*/
//===========================================================================
void cToolCursor::render(cRenderOptions& a_options)
{
    ///////////////////////////////////////////////////////////////////////
    // render contact points
    ///////////////////////////////////////////////////////////////////////
    int numContactPoint = (int)(m_hapticPoints.size());
    for (int i=0; i<numContactPoint; i++)
    {
        // get next contact point
        cHapticPoint* nextContactPoint = m_hapticPoints[i];

        // render tool
        nextContactPoint->render(a_options);
    }

    ///////////////////////////////////////////////////////////////////////
    // render mesh image
    ///////////////////////////////////////////////////////////////////////
    if (m_image != NULL)
    {
        m_image->renderSceneGraph(a_options);    
    }
}




