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
#include "effects/CEffectMagnet.h"
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
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
    double magnetMaxForce = m_parent->m_material.getMagnetMaxForce();
    double magnetMaxDistance = m_parent->m_material.getMagnetMaxDistance();
    double stiffness = m_parent->m_material.getStiffness();
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
            sys.m[0][0] = limitLinearModel * limitLinearModel;
            sys.m[0][1] = limitLinearModel;
            sys.m[0][2] = 1.0;
            sys.m[1][0] = magnetMaxDistance * magnetMaxDistance;
            sys.m[1][1] = magnetMaxDistance;
            sys.m[1][2] = 1.0;
            sys.m[2][0] = 2.0 * limitLinearModel;
            sys.m[2][1] = 1.0;
            sys.m[2][2] = 0.0;
            sys.invert();

            cVector3d param;
            sys.mulr(cVector3d(magnetMaxForce, 0.0, -1.0), param);

            // apply quadratic model
            double val = distance - limitLinearModel;
            forceMagnitude = param.x * val * val + param.y * val + param.z;
        }

        // compute magnetic force
        a_reactionForce = cMul(forceMagnitude, cNormalize(cSub(m_parent->m_interactionProjectedPoint, a_toolPos)));

        // add damping component
        double viscosity = m_parent->m_material.getViscosity();
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


