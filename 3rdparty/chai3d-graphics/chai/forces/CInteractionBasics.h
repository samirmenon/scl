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
#ifndef CInteractionBasicsH
#define CInteractionBasicsH
//---------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "graphics/CVertex.h"
#include "graphics/CMaterial.h"
#include <vector>
//---------------------------------------------------------------------------
using std::vector;
class cGenericObject;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CInteractionBasics.h

    \brief 
    <b> Force Rendering Algorithms </b> \n 
    Interaction Events.
*/
//===========================================================================

//===========================================================================
/*!  
    \struct     cInteractionEvent
    \ingroup    forces 
    
    \brief    
    cInteractionEvent stores all information related to the intersection 
    between a point and an object.
*/
//===========================================================================
struct cInteractionEvent
{
    //! Pointer to the interaction object.
    cGenericObject* m_object;

    //! Is the pointer located inside the object.
    bool m_isInside;

    //! Position of the interaction point in reference to the objects coordinate frame (local coordinates).
    cVector3d m_localPos;

    //! Resulting force in local coordinates.
    cVector3d m_localForce;

    //! Nearest point to the object surface in local coordinates
    cVector3d m_localSurfacePos;

    //! Initialize all data contained in current event.
    void clear()
    {
        m_object    = NULL;
        m_isInside    = false;
        m_localPos.zero();
        m_localForce.zero();
        m_localSurfacePos.zero();
    }
};

//===========================================================================
/*!
    \class      cInteractionSettings
    \ingroup    forces 
    
    \brief    
    This structure contains a list of settings which are passed
    to the interaction detector when checking for an interaction.
*/
//===========================================================================
struct cInteractionSettings
{
    //! If \b true, then collision detector shall check for collisions on visible objects only.
    bool m_checkVisibleObjectsOnly;

    //! If \b true, then collision detector shall check for collisions on haptic enabled objects only.
    bool m_checkHapticObjectsOnly;
};


//===========================================================================
/*!
    \class      cInteractionRecorder
    \ingroup    forces 

    \brief    
    cInteractionRecorder stores a list of interaction events.
*/
//===========================================================================
class cInteractionRecorder
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCollisionRecorder.
    cInteractionRecorder() { clear(); }

    //! Destructor of cCollisionRecorder.
    virtual ~cInteractionRecorder() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Clear all interaction event records.
    void clear()
    {
        m_interactions.clear();
    }


    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! List of interaction events stored in recorder.
    vector<cInteractionEvent> m_interactions;
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

