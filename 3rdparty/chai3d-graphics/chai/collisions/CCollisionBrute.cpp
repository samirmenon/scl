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
    \version   2.0.0 $Rev: 201 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "collisions/CCollisionBrute.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Check if the given line segment intersects any triangle of the mesh. This
    method is called "brute force" because all triangles are checked by
    invoking their collision-detection methods.  This method is simple but very
    inefficient.

    \fn       bool cCollisionBrute::computeCollision(cVector3d& a_segmentPointA,
                                       cVector3d& a_segmentPointB,
                                       cCollisionRecorder& a_recorder,
                                       cCollisionSettings& a_settings)
    \param    a_segmentPointA  Initial point of segment.
    \param    a_segmentPointB  End point of segment.
    \param    a_recorder  Returns pointer to nearest collided object.
    \param    a_settings  Structure which contains some rules about how the
                          collision detection should be performed.
    \return   Return true if the line segment intersects one or more triangles.
*/
//===========================================================================
bool cCollisionBrute::computeCollision(cVector3d& a_segmentPointA,
                                       cVector3d& a_segmentPointB,
                                       cCollisionRecorder& a_recorder,
                                       cCollisionSettings& a_settings)
{
    // temp variables
    bool hit = false;

    // check all triangles for collision
    unsigned int numTriangles = m_triangles->size();
    for (unsigned int i=0; i<numTriangles; i++)
    {

        // check for a collision between this triangle and the segment by
        // calling the triangle's collision detection method
        if ((*m_triangles)[i].computeCollision(
            a_segmentPointA, a_segmentPointB, a_recorder, a_settings))
        {
            hit = true;
        }
    }

    // return result
    return (hit);
}



