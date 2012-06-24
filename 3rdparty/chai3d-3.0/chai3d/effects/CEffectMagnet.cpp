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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 707 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "effects/CEffectMagnet.h"
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
//---------------------------------------------------------------------------


//===========================================================================
/*!
    Constructor of cEffectMagnet.

    \fn  cEffectMagnet::cEffectMagnet(cGenericObject* a_parent):cGenericEffect(a_parent)
    \param  a_parent Parent object.
*/
//===========================================================================
cEffectMagnet::cEffectMagnet(cGenericObject* a_parent):cGenericEffect(a_parent)
{
}


//===========================================================================
/*!
    Compute the resulting force effect.

    \fn  bool cEffectMagnet::computeForce(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int& a_toolID,
                                          cVector3d& a_reactionForce);
    \param  a_toolPos Position of tool.
    \param  a_toolVel Velocity of tool.
    \param  a_toolID  Identification number of force algorithm.
    \param  a_reactionForce  Return the computed force here.
    \return  Return false if no interaction force is computed.
*/
//===========================================================================
bool cEffectMagnet::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    // compute distance from object to tool
    double distance = cDistance(a_toolPos, m_parent->m_interactionProjectedPoint);

    // get parameters of magnet
    double magnetMaxForce = m_parent->m_material->getMagnetMaxForce();
    double magnetMaxDistance = m_parent->m_material->getMagnetMaxDistance();
    double stiffness = m_parent->m_material->getStiffness();
    double forceMagnitude = 0;

    if ((distance > 0) && (distance < magnetMaxDistance) && (stiffness > 0))
    {
        double limitLinearModel = magnetMaxForce / stiffness;
        cClamp(limitLinearModel, 0.0, 0.5 * distance);

        if (distance < limitLinearModel)
        {
            // apply local linear model near magnet
            forceMagnitude = stiffness * distance;
        }
        else
        {
            // compute quadratic model
            cMatrix3d sys;
            sys(0,0) = limitLinearModel * limitLinearModel;
            sys(0,1) = limitLinearModel;
            sys(0,2) = 1.0;
            sys(1,0) = magnetMaxDistance * magnetMaxDistance;
            sys(1,1) = magnetMaxDistance;
            sys(1,2) = 1.0;
            sys(2,0) = 2.0 * limitLinearModel;
            sys(2,1) = 1.0;
            sys(2,2) = 0.0;
            sys.invert();

            cVector3d param;
            sys.mulr(cVector3d(magnetMaxForce, 0.0, -1.0), param);

            // apply quadratic model
            double val = distance - limitLinearModel;
            forceMagnitude = param(0)  * val * val + param(1)  * val + param(2) ;
        }

        // compute magnetic force
        a_reactionForce = cMul(forceMagnitude, cNormalize(cSub(m_parent->m_interactionProjectedPoint, a_toolPos)));

        // add damping component
        double viscosity = m_parent->m_material->getViscosity();
        cVector3d viscousForce = cMul(-viscosity, a_toolVel);
        a_reactionForce.add(viscousForce);

        return (true);
    }
    else
    {
        // the tool is located outside the magnet zone
        a_reactionForce.zero();
        return (false);
    }
}


