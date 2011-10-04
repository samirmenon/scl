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
    \version   2.0.0 $Rev: 250 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CCollisionBasicsH
#define CCollisionBasicsH
//---------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "graphics/CVertex.h"
#include "graphics/CMaterial.h"
#include <vector>
//---------------------------------------------------------------------------
using std::vector;
class cGenericObject;
class cTriangle;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CCollisionBasics.h
    
    \brief  
    <b> Collision Detection </b> \n
    Collision Events & Recording.
*/
//===========================================================================

//===========================================================================
/*!
    \struct     cCollisionEvent
    \ingroup    collisions

    \brief    
    cCollisionEvent stores all information related to the
    intersection (or collision) between an segment and an object.
*/
//===========================================================================
struct cCollisionEvent
{
    //! Pointer to the collided object.
    cGenericObject* m_object;

    //! Pointer to collided triangle. This pointer may be NULL for collisions with non triangle based objects.
    cTriangle* m_triangle;

    //! Position of the collision point in reference to the objects coordinate frame (local coordinates).
    cVector3d m_localPos;

    //! Position of the collision point in world coordinates (global coordinates).
    cVector3d m_globalPos;

    //! Surface normal at collision point in reference to the objects coordinate frame (local coordinates)
    cVector3d m_localNormal;

    //! Surface normal at collision point in world coordinates (global coordinates).
    cVector3d m_globalNormal;

    //! Square distance between ray origin and collision point.
    double m_squareDistance;

    //! If the position of segment A is modified to take into account motion
    //! (see m_adjustObjectMotion in cCollisionSettings), the value is stored hare.
    cVector3d m_adjustedSegmentAPoint;

    //! initialize all data
    void clear()
    {
        m_object    = NULL;
        m_triangle  = NULL;
        m_localPos.zero();
        m_globalPos.zero();
        m_localNormal.zero();
        m_globalNormal.zero();
        m_squareDistance = CHAI_DBL_MAX;
    }
};


//===========================================================================
/*!
    \class      cCollisionRecorder
    \ingroup    collisions
    
    \brief    
    cCollisionRecorder stores a list of collision events.
*/
//===========================================================================
class cCollisionRecorder
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCollisionRecorder
    cCollisionRecorder() { clear(); }

    //! Destructor of cCollisionRecorder
    virtual ~cCollisionRecorder() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Clear all records.
    void clear()
    {
        m_nearestCollision.clear();
        m_collisions.clear();
    }


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! nearest collision from start point of segment.
    cCollisionEvent m_nearestCollision;

    //! List of collisions.
    vector<cCollisionEvent> m_collisions;
};


//===========================================================================
/*!
    \struct     cCollisionSettings
    \ingroup    collisions
    
    \brief    
    This structure contains a list of settings which are passed to the 
    collision detector when checking for a collision.
*/
//===========================================================================
struct cCollisionSettings
{
    //! If \b true, only return the nearest collision collision event.
    bool m_checkForNearestCollisionOnly;

    //! If \b true, return minimal amount of the collision.
    bool m_returnMinimalCollisionData;

    //! If \b true, then collision detector shall check for collisions on visible objects only.
    bool m_checkVisibleObjectsOnly;

    //! If \b true, then collision detector shall check for collisions on haptic enabled objects only.
    bool m_checkHapticObjectsOnly;

    //! If \b true, then collision can occur on both sides of triangles.
    bool m_checkBothSidesOfTriangles;

    //! If \b true, then adjust for object motion. (dynamic proxy model).
    bool m_adjustObjectMotion;

    //! Radius of the virtual tool or cursor.
    double m_collisionRadius;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

