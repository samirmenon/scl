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
    \version   2.0.0 $Rev: 227 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "collisions/CCollisionAABB.h"
#include <iostream>
using namespace std;
//---------------------------------------------------------------------------
//! Pointer to first free location in array of AABB tree nodes.
cCollisionAABBInternal* g_nextFreeNode;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cCollisionAABB.

    \fn       cCollisionAABB::cCollisionAABB(vector<cTriangle> *a_triangles, bool a_useNeighbors)
    \param    a_triangles     Pointer to array of triangles.
    \param    a_useNeighbors  Use neighbor lists to speed up collision detection?
*/
//===========================================================================
cCollisionAABB::cCollisionAABB(vector<cTriangle> *a_triangles, bool a_useNeighbors)
{
    // list of triangles used when building the tree
    m_triangles = a_triangles;

    // initialize members
    m_internalNodes = NULL;
    m_root          = NULL;
    m_leaves        = NULL;
    m_numTriangles  = 0;
    m_useNeighbors  = a_useNeighbors;
}


//===========================================================================
/*!
    Destructor of cCollisionAABB.

    \fn       cCollisionAABB::~cCollisionAABB()
*/
//===========================================================================
cCollisionAABB::~cCollisionAABB()
{

    // clear collision tree
    if (m_root != NULL)
    {
        delete [] m_internalNodes;
        m_root = NULL;
    }

    // Delete the allocated array of leaf nodes
    //
    // If there's only one triangle, m_root = m_leaves
    // and we've already deleted the leaves...
    if (m_numTriangles > 1)
    {
        if (m_leaves)
        {
            delete [] m_leaves;
            m_leaves = NULL;
        }
    }
}


//===========================================================================
/*!
    Build the Axis-Aligned Bounding Box collision-detection tree.  Each
    leaf is associated with one triangle and with a bounding box of minimal
    dimensions such that it fully encloses the triangle and is aligned with
    the coordinate axes (no rotations).  Each internal node is associated
    with a bounding box of minimal dimensions such that it fully encloses
    the bounding boxes of its two children and is aligned with the axes.

    \fn       void cCollisionAABB::initialize(double a_radius)
    \param    a_radius radius to add around the triangles.
*/
//===========================================================================
void cCollisionAABB::initialize(double a_radius)
{
    unsigned int i;
    m_lastCollision = NULL;

    // if a previous tree was created, delete it
    if (m_root != NULL)
    {
        delete[] m_root;
        m_root = NULL;
    }

    // reset triangle counter
    m_numTriangles = 0;

    // count the number of allocated triangles that will be used to create
    // the tree.
    for (i = 0; i < m_triangles->size(); ++i)
    {
        cTriangle* nextTriangle = &(*m_triangles)[i];
        if (nextTriangle->allocated())
        {
            m_numTriangles++;
        }
    }

    // check if the number of triangles is equal to zero
    if (m_numTriangles == 0)
    {
        m_root = NULL;
        return;
    }

    // create a leaf node for each triangle
    m_leaves = new cCollisionAABBLeaf[m_numTriangles];
    int j=0;
    for (i = 0; i < m_triangles->size(); ++i)
    {
        cTriangle* nextTriangle = &(*m_triangles)[i];
        if (nextTriangle->allocated())
        {
            m_leaves[j].initialize(nextTriangle, a_radius);
            j++;
        }
    }

    // allocate an array to hold all internal nodes of the binary tree
    if (m_numTriangles >= 2)
    {
        g_nextFreeNode = new cCollisionAABBInternal[m_numTriangles];
        m_internalNodes = g_nextFreeNode;
        m_root = g_nextFreeNode;
        g_nextFreeNode->initialize(m_numTriangles, m_leaves, 0);
    }

    // there is only one triangle, so the tree consists of just one leaf
    else
    {
        m_root = &m_leaves[0];
    }

    // assign parent relationships in the tree
    m_root->setParent(0,1);
}


//===========================================================================
/*!
    Check if the given line segment intersects any triangle of the mesh.  If so,
    return true, as well as (through the output parameters) pointers to the
    intersected triangle, the mesh of which this triangle is a part, the point
    of intersection, and the distance from the origin of the segment to the
    collision point.  If more than one triangle is intersected, return the one
    closest to the origin of the segment.  The method uses the pre-computed
    AABB boxes, starting at the root and recursing through the tree, breaking
    the recursion along any path in which the bounding box of the line segment
    does not intersect the bounding box of the node.  At the leafs,
    triangle-segment intersection testing is called.

    \fn       bool cCollisionAABB::computeCollision(cVector3d& a_segmentPointA,
              cVector3d& a_segmentPointB, cCollisionRecorder& a_recorder,
              cCollisionSettings& a_settings)
    \param    a_segmentPointA  Initial point of segment.
    \param    a_segmentPointB  End point of segment.
    \param    a_recorder  Stores all collision events
    \param    a_settings  Contains collision settings information.
    \return   Return true if a collision event has occurred.
*/
//===========================================================================
bool cCollisionAABB::computeCollision(cVector3d& a_segmentPointA, cVector3d& a_segmentPointB,
         cCollisionRecorder& a_recorder, cCollisionSettings& a_settings)
{
    // if the root is null, the tree is empty, so there can be no collision
    if (m_root == NULL)
    {
        return (false);
    }

    // create an axis-aligned bounding box for the line
    cCollisionAABBBox lineBox;
    lineBox.setEmpty();
    lineBox.enclose(a_segmentPointA);
    lineBox.enclose(a_segmentPointB);

    // test for intersection between the line segment and the root of the
    // collision tree; the root will recursively call children down the tree
    bool result = m_root->computeCollision(a_segmentPointA, 
                                           a_segmentPointB, 
                                           lineBox,
                                           a_recorder, a_settings);

    // return whether there was an intersection
    return result;
}


//===========================================================================
/*!
    Render the bounding boxes of the collision tree in OpenGL.

    \fn       void cCollisionAABB::render()
*/
//===========================================================================
void cCollisionAABB::render()
{
    if (m_root != NULL)
    {
        // set rendering settings
        glDisable(GL_LIGHTING);
        glLineWidth(1.0);
        glColor4fv(m_material.m_ambient.pColor());

        // render tree by calling the root, which recursively calls the children
        m_root->render(m_displayDepth);

        // restore lighting settings
        glEnable(GL_LIGHTING);
    }
}



