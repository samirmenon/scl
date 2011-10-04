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
#include "effects/CEffectStickSlip.h"
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
//---------------------------------------------------------------------------
#include "math/CMaths.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cEffectStickSlip.

    \fn     cEffectStickSlip::cEffectStickSlip(cGenericObject* a_parent):cGenericEffect(a_parent)
    \param  a_parent Parent object.
*/
//===========================================================================
cEffectStickSlip::cEffectStickSlip(cGenericObject* a_parent):cGenericEffect(a_parent)
{
    for (int i=0; i<CHAI_EFFECT_MAX_IDN; i++)
    {
        m_history[i].m_currentStickPosition.zero();
        m_history[i].m_valid = false;
    }
}


//===========================================================================
/*!
    Compute the resulting force effect.

    \fn  bool cEffectStickSlip::computeForce(const cVector3d& a_toolPos,
                                           const cVector3d& a_toolVel,
                                           const unsigned int& a_toolID,
                                           cVector3d& a_reactionForce);
    \param  a_toolPos Position of tool.
    \param  a_toolVel Velocity of tool.
    \param  a_toolID  Identification number of force algorithm
    \param  a_reactionForce  Return the computed force here.
    \return  Return false if no interaction force is computed
*/
//===========================================================================
bool cEffectStickSlip::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    // check if history for this IDN exists
    if (a_toolID < CHAI_EFFECT_MAX_IDN)
    {
        if (m_parent->m_interactionInside)
        {
            // check if a recent valid point has been stored previously
            if (!m_history[a_toolID].m_valid)
            {
                m_history[a_toolID].m_currentStickPosition = a_toolPos;
                m_history[a_toolID].m_valid = true;
            }

            // read parameters for stick and slip model
            double stiffness = m_parent->m_material.getStickSlipStiffness();
            double forceMax = m_parent->m_material.getStickSlipForceMax();

            // compute current force between last stick position and current tool position
            double distance = cDistance(a_toolPos, m_history[a_toolID].m_currentStickPosition);
            double forceMag = distance * stiffness;

            if (forceMag > 0)
            {
                // if force above threshold, slip...
                if (forceMag > forceMax)
                {
                    m_history[a_toolID].m_currentStickPosition = a_toolPos;
                    a_reactionForce.zero();
                }
                // ...otherwise stick
                else
                {
                    a_reactionForce = (forceMag / distance) * cSub(m_history[a_toolID].m_currentStickPosition, a_toolPos);
                }
            }
            else
            {
                a_reactionForce.zero();
            }

            return (true);
        }
        else
        {
            // the tool is located outside the object, so zero reaction force
            m_history[a_toolID].m_valid = false;
            a_reactionForce.zero();
            return (false);
        }
    }
    else
    {
        a_reactionForce.zero();
        return (false);
    }
}


