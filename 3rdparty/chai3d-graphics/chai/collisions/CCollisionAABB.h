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
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CCollisionAABBH
#define CCollisionAABBH
//---------------------------------------------------------------------------
#include "graphics/CTriangle.h"
#include "graphics/CVertex.h"
#include "math/CMaths.h"
#include "collisions/CGenericCollision.h"
#include "collisions/CCollisionAABBBox.h"
#include "collisions/CCollisionAABBTree.h"
#include <vector>
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CCollisionAABB.h

    \brief    
    <b> Collision Detection </b> \n 
    Axis-Aligned Bounding Box Tree (AABB) - Main Interface.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cCollisionAABB
    \ingroup    collisions

    \brief    
    cCollisionAABB provides methods to create an Axis-Aligned Bounding Box 
    collision detection tree, and to use this tree to check for the 
    intersection of a line segment with a mesh.
*/
//===========================================================================
class cCollisionAABB : public cGenericCollision
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cAABBTree.
    cCollisionAABB(vector<cTriangle>* a_triangles, bool a_useNeighbors);

    //! Destructor of cAABBTree.
    virtual ~cCollisionAABB();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Build the AABB Tree for the first time.
    void initialize(double a_radius = 0);

    //! Draw the bounding boxes in OpenGL.
    void render();

    //! Return the nearest triangle intersected by the given segment, if any.
    bool computeCollision(cVector3d& a_segmentPointA, cVector3d& a_segmentPointB,
         cCollisionRecorder& a_recorder, cCollisionSettings& a_settings);

    //! Return the root node of the collision tree.
    cCollisionAABBNode* getRoot() { return (m_root); }


  protected:

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Pointer to the list of triangles in the mesh.
    vector<cTriangle> *m_triangles;

    //! Pointer to an array of leaf nodes for the AABB Tree.
    cCollisionAABBLeaf *m_leaves;

    //! Pointer to array of internal nodes.
    cCollisionAABBInternal *m_internalNodes;

    //! Pointer to the root of the AABB Tree.
    cCollisionAABBNode *m_root;

    //! The number of triangles in the mesh.
    unsigned int m_numTriangles;

    //! Triangle returned by last successful collision test.
    cTriangle* m_lastCollision;

    //! Use list of triangles' neighbors to speed up collision detection?
    bool m_useNeighbors;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
