//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2013, CHAI3D.
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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 322 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CAlgorithmPotentialFieldH
#define CAlgorithmPotentialFieldH
//------------------------------------------------------------------------------
#include "forces/CGenericForceAlgorithm.h"
#include "forces/CInteractionBasics.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CAlgorithmPotentialField.h

    \brief 
    <b> Force Rendering Algorithms </b> \n 
    Potential Field.
*/
//==============================================================================

//==============================================================================
/*! 
    \class      cAlgorithmPotentialField
    \ingroup    forces 
    
    \brief    
    cAlgorithmPotentialField is an abstract class for algorithms that 
    compute single point force contacts.
*/
//==============================================================================
class cAlgorithmPotentialField : public cGenericForceAlgorithm
{
  public:

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
    
      //! Constructor of cAlgorithmPotentialField.
    cAlgorithmPotentialField();

    //! Destructor of cAlgorithmPotentialField.
    virtual ~cAlgorithmPotentialField() {};


    //--------------------------------------------------------------------------
    // METHODS - GRAPHICS
    //--------------------------------------------------------------------------

    //! Render force algorithms graphicaly in OpenGL. (For debug purposes)
    virtual void render(cRenderOptions& a_options) {}


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    //! Initialize the algorithm by passing the initial position of the device.
    void initialize(cWorld* a_world, const cVector3d& a_initialPos) { m_world = a_world; };

    //! Compute the next force given the updated position of the device.
    virtual cVector3d computeForces(const cVector3d& a_toolPos, const cVector3d& a_toolVel);

    //! Interactions recorder.
    cInteractionRecorder m_interactionRecorder;


  private:

    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

    //! Identification number for this force algorithm.
    unsigned int m_IDN;

    //! IDN counter for all.
    static unsigned int m_IDNcounter;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
