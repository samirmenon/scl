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
#include "effects/CEffectSurface.h"
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
//---------------------------------------------------------------------------


//===========================================================================
/*!
    Constructor of cEffectSurface.

    \fn  cEffectSurface::cEffectSurface(cGenericObject* a_parent):cGenericEffect(a_parent)
    \param  a_parent Parent object.
*/
//===========================================================================
cEffectSurface::cEffectSurface(cGenericObject* a_parent):cGenericEffect(a_parent)
{
}


//===========================================================================
/*!
    Compute the resulting force effect.

    \fn  bool cEffectSurface::computeForce(const cVector3d& a_toolPos,
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
bool cEffectSurface::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
{
    if (m_parent->m_interactionInside)
    {
        // the tool is located inside the object,
        // we compute a reaction force using Hooke's law
        double stiffness = m_parent->m_material.getStiffness();
        a_reactionForce = cMul(stiffness, cSub(m_parent->m_interactionProjectedPoint, a_toolPos));
        return (true);
    }
    else
    {
        // the tool is located outside the object, so zero reaction force
        a_reactionForce.zero();
        return (false);
    }
}


