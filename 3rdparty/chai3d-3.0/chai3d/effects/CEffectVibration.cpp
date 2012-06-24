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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 708 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "effects/CEffectVibration.h"
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
//---------------------------------------------------------------------------
#include "math/CMaths.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cEffectVibration.

    \fn  cEffectVibration::cEffectVibration(cGenericObject* a_parent):cGenericEffect(a_parent)
    \param  a_parent Parent object.
*/
//===========================================================================
cEffectVibration::cEffectVibration(cGenericObject* a_parent):cGenericEffect(a_parent)
{
    // reset and initialize clock
    clock.start(true);
}


//===========================================================================
/*!
    Compute the resulting force effect.

    \fn  bool cEffectVibration::computeForce(const cVector3d& a_toolPos,
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
bool cEffectVibration::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    if (m_parent->m_interactionInside)
    {
        // read vibration parameters
        double vibrationFrequency = m_parent->m_material->getVibrationFrequency();
        double vibrationAmplitude = m_parent->m_material->getVibrationAmplitude();

        // read time
        double time = clock.getCurrentTimeSeconds();

        // compute force magnitude
        double forceMag = vibrationAmplitude * sin(2.0 * C_PI *vibrationFrequency * time);

        a_reactionForce = cMul(forceMag, cVector3d(1, 0, 0));
        return (true);
    }
    else
    {
        // the tool is located outside the object, so zero reaction force
        a_reactionForce.zero();
        return (false);
    }
}


