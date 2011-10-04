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
    \author    Chris Sewell
    \author    Francois Conti
    \version   2.0.0 $Rev: 250 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CCollisionBruteH
#define CCollisionBruteH
//---------------------------------------------------------------------------
#include "math/CMaths.h"
#include "graphics/CTriangle.h"
#include "graphics/CVertex.h"
#include "collisions/CGenericCollision.h"
#include <vector>
//---------------------------------------------------------------------------
using std::vector;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CCollisionBrute.h
    
    \brief  
    <b> Collision Detection </b> \n
    Brute Force Model.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cCollisionBrute
    \ingroup    collisions
    
    \brief    
    cCollisionBrute provides methods to check for the intersection
    of a line segment with a mesh by checking all triangles in the mesh.
*/
//===========================================================================
class cCollisionBrute : public cGenericCollision
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCollisionBrute.
    cCollisionBrute(vector<cTriangle> *a_triangles) : m_triangles(a_triangles) {}

    //! Destructor of cCollisionBrute.
    virtual ~cCollisionBrute() { }


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! No initialization is necessary for the brute force method.
    virtual void initialize(double a_radius = 0) {};

    //! There isn't really a useful "visualization" of "check all triangles".
    virtual void render() {};

    //! Return the triangles intersected by the given segment, if any.
    bool computeCollision(cVector3d& a_segmentPointA,
                          cVector3d& a_segmentPointB,
                          cCollisionRecorder& a_recorder,
                          cCollisionSettings& a_settings);

  protected:

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Pointer to the list of triangles in the mesh.
    vector<cTriangle> *m_triangles;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

