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
    \author    Federico Barbagli
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 365 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CToolGripperH
#define CToolGripperH
//---------------------------------------------------------------------------
#include "tools/CGenericTool.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CToolGripper.h
    
    \brief  
    <b> Haptic Tools </b> \n 
    Gripper Tool.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cToolGripper
    \ingroup    tools  
    
    \brief      
    cToolGripper represents a haptic tool that can apply forces in 
    three degrees of freedom and maintains three or six degrees of 
    device pose. \n
*/
//===========================================================================
class cToolGripper : public cGenericTool
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

	public:

    //! Constructor of cToolGripper.
    cToolGripper(cWorld* a_parentWorld);

    //! Destructor of cToolGripper.
    virtual ~cToolGripper();


    //-----------------------------------------------------------------------
    // MEMBERS
    //-----------------------------------------------------------------------
    
	public:

    // Haptic interaction point modelling the thumb.
    cHapticPoint* m_hapticPointThumb;

    // Haptic interaction point modelling the index.
    cHapticPoint* m_hapticPointFinger;


    //-----------------------------------------------------------------------
    // METHODS
    //-----------------------------------------------------------------------

	public:

    //! Compute interaction forces between cursor's contact point and environment.
    virtual void computeInteractionForces();

    //! Render the object in OpenGL.
    virtual void render(cRenderOptions& a_options);

	//! Set a workspace scale factor for the gripper.
	virtual void setGripperWorkspaceScale(double a_gripperWorkspaceScale) { m_gripperWorkspaceScale = cAbs(a_gripperWorkspaceScale); }

	//! Get  workspace scale factor of the gripper.
	virtual double getGripperWorskpaceScale() { return (m_gripperWorkspaceScale); }

    //-----------------------------------------------------------------------
    // MEMBERS
    //-----------------------------------------------------------------------
    
	private:

	// workspace of force gripper [m] when .
	double m_gripperWorkspaceScale;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

