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
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericEffectH
#define CGenericEffectH
//---------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "math/CVector3d.h"
//---------------------------------------------------------------------------
class cGenericObject;


//===========================================================================
/*!
    \file CGenericEffect.h
    \brief
    <b> Haptic Effects </b> \n 
    Base Class.
*/
//===========================================================================

//---------------------------------------------------------------------------
/*!
    Maximum number of force models supported the CHAI simulation. Some
    force effects require a history of the tool, such as its  previous position
    for instance. Effects which need such information store it in a local table
    and retrieve the information when they calculate the reaction force.
    See \e CEffectStickSlip class as an example
*/
//---------------------------------------------------------------------------
const int CHAI_EFFECT_MAX_IDN = 16;


//===========================================================================
/*!
    \class      cGenericEffect
    \ingroup    effects

    \brief    
    cGenericEffect provides a base class to program haptic
    effects (force models) when a virtual tool interacts with objects
    of a virtual environment.
*/
//===========================================================================
class cGenericEffect
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
      
    //! Constructor of CGenericEffect.
    cGenericEffect(cGenericObject* a_parent); 

    //! Destructor of CGenericEffect.
    virtual ~cGenericEffect() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Compute a resulting force.
    virtual bool computeForce(const cVector3d& a_toolPos,
                              const cVector3d& a_toolVel,
                              const unsigned int& a_toolID,
                              cVector3d& a_reactionForce)
                              {
                                  a_reactionForce.zero();
                                  return (false);
                              }

    //! Read last computed force.
    cVector3d getLastComputedForce() { return (m_lastComputedForce); }

    //! object to which the force effects applies.
    cGenericObject* m_parent;

    //! Enable or disable current effect.
    inline void enable(bool a_status) { m_enabled = a_status; }

    //! Is the current effect enabled.
    inline bool isEnabled() { return (m_enabled); }

  protected:

    //! last computed force.
    cVector3d m_lastComputedForce;

    //! initialize effect model.
    virtual void initialize() { return; }

    //! is the current effect enabled.
    bool m_enabled;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
