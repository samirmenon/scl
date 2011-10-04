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
#ifndef CEffectVibrationH
#define CEffectVibrationH
//---------------------------------------------------------------------------
#include "effects/CGenericEffect.h"
#include "timers/CPrecisionClock.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CEffectVibration.h

    \brief
    <b> Haptic Effects </b> \n 
    Vibration Model.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cEffectVibration
    \ingroup    effects  
    
    \brief    
    cEffectVibration models force vibrations by defining a frequency and 
    amplitude.
*/
//===========================================================================
class cEffectVibration : public cGenericEffect
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cEffectVibration.
    cEffectVibration(cGenericObject* a_parent);

    //! Destructor of cEffectVibration.
    virtual ~cEffectVibration() {};

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Compute resulting force.
    bool computeForce(const cVector3d& a_toolPos,
                      const cVector3d& a_toolVel,
                      const unsigned int& a_toolID,
                      cVector3d& a_reactionForce);


  protected:

    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Vibration signal clock.
    cPrecisionClock clock;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
