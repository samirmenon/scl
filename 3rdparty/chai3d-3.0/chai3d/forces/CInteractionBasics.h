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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 995 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CInteractionBasicsH
#define CInteractionBasicsH
//------------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "graphics/CVertex.h"
#include "materials/CMaterial.h"
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cGenericObject;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CInteractionBasics.h

    \brief 
    <b> Force Rendering Algorithms </b> \n 
    Interaction Events.
*/
//==============================================================================

//==============================================================================
/*!  
    \struct     cInteractionEvent
    \ingroup    forces 
    
    \brief    
    cInteractionEvent stores all information related to the intersection 
    between a point and an object.
*/
//==============================================================================
struct cInteractionEvent
{
    //! Pointer to the interaction object.
    cGenericObject* m_object;

    //! Did the interaction event occur inside the object.
    bool m_isInside;

    //! Position of the interaction point in reference to the object's coordinate frame (local coordinates).
    cVector3d m_localPos;

    //! Surface normal at the interaction point in reference to the object's coordinate frame (local coordinates).
    cVector3d m_localNormal;

    //! Resulting force in object's local coordinate frame.
    cVector3d m_localForce;

    //! Nearest point to the object's surface in local coordinates
    cVector3d m_localSurfacePos;

    //! Initialize all data contained in current event.
    void clear()
    {
        m_object   = NULL;
        m_isInside = false;
        m_localPos.zero();
        m_localNormal.set(1,0,0);
        m_localForce.zero();
        m_localSurfacePos.zero();
    }
};


//==============================================================================
/*!
    \class      cInteractionRecorder
    \ingroup    forces 

    \brief    
    cInteractionRecorder stores a list of interaction events.
*/
//==============================================================================
class cInteractionRecorder
{
  public:
    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor of cCollisionRecorder.
    cInteractionRecorder() { clear(); }

    //! Destructor of cCollisionRecorder.
    virtual ~cInteractionRecorder() {};


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    //! Clear all interaction event records.
    void clear()
    {
        m_interactions.clear();
    }


    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

    //! List of interaction events stored in recorder.
    std::vector<cInteractionEvent> m_interactions;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
