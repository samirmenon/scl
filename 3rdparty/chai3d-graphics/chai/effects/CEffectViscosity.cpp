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
#include "effects/CEffectViscosity.h"
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
//---------------------------------------------------------------------------


//===========================================================================
/*!
    Constructor of cEffectViscosity.

    \fn  cEffectViscosity::cEffectViscosity(cGenericObject* a_parent):cGenericEffect(a_parent)
    \param  a_parent Parent object.
*/
//===========================================================================
cEffectViscosity::cEffectViscosity(cGenericObject* a_parent):cGenericEffect(a_parent)
{
}


//===========================================================================
/*!
    Compute the resulting force effect.

    \fn  bool cEffectViscosity::computeForce(const cVector3d& a_toolPos,
                                             const cVector3d& a_toolVel,
                                             const unsigned int& a_toolID,
                                             cVector3d& a_reactionForce)
    \param  a_toolPos Position of tool.
    \param  a_toolVel Velocity of tool.
    \param  a_toolID  Identification number of the force algorithm stored in the tool.
    \param  a_reactionForce  Return the computed force here.
    \return  Return \b false if no interaction force is computed.
*/
//===========================================================================
bool cEffectViscosity::computeForce(const cVector3d& a_toolPos,
                                    const cVector3d& a_toolVel,
                                    const unsigned int& a_toolID,
                                    cVector3d& a_reactionForce)
{
    if (m_parent->m_interactionInside)
    {
        // the tool is located inside the object.
        double viscosity = m_parent->m_material.getViscosity();
        a_reactionForce = cMul(-viscosity, a_toolVel);
        return (true);
    }
    else
    {
        // the tool is located outside the object, so zero reaction force.
        a_reactionForce.zero();
        return (false);
    }
}


