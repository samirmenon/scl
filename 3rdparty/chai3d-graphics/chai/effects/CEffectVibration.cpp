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
    \version   2.0.0 $Rev: 244 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "effects/CEffectVibration.h"
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
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
        double vibrationFrequency = m_parent->m_material.getVibrationFrequency();
        double vibrationAmplitude = m_parent->m_material.getVibrationAmplitude();

        // read time
        double time = clock.getCurrentTimeSeconds();

        // compute force magnitude
        double forceMag = vibrationAmplitude * sin(2.0 * CHAI_PI *vibrationFrequency * time);

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


