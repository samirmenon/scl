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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 793 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CEffectStickSlipH
#define CEffectStickSlipH
//---------------------------------------------------------------------------
#include "effects/CGenericEffect.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CEffectStickSlip.h

    \brief
    <b> Haptic Effects </b> \n 
    Stick'n Slip Model.
*/
//===========================================================================

//---------------------------------------------------------------------------
/*!
    Provides some history information to class cEffectStickSlip for a given
    haptic tool.
*/
//---------------------------------------------------------------------------
struct cStickSlipStatus
{
    //! current stick position
    cVector3d m_currentStickPosition;

    //! is the stick position valid?
    bool m_valid;
};


//===========================================================================
/*!
    \class      cEffectStickSlip
    \ingroup    effects  
    
    \brief    
    cEffectStickSlip models a stick and slip effect.
*/
//===========================================================================
class cEffectStickSlip : public cGenericEffect
{
  public:
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
      
    //! Constructor of cEffectStickSlip.
    cEffectStickSlip(cGenericObject* a_parent);

    //! Destructor of cEffectStickSlip.
    virtual ~cEffectStickSlip() {};


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

    //! store the algorithm history for each IDN calling this effect.
    cStickSlipStatus m_history[C_EFFECT_MAX_IDN];
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
