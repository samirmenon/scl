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
    \author    Chris Sewell
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 733 $
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
    cCollisionAABB(vector<cTriangle>* a_triangles);

    //! Destructor of cAABBTree.
    virtual ~cCollisionAABB();


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Build the AABB Tree for the first time.
    void initialize(const double a_radius = 0.0);

    //! Draw the bounding boxes in OpenGL.
    void render(cRenderOptions& a_options);

    //! Return the nearest triangle intersected by the given segment, if any.
    bool computeCollision(cGenericObject* a_object,
                          cVector3d& a_segmentPointA, 
                          cVector3d& a_segmentPointB,
                          cCollisionRecorder& a_recorder, 
                          cCollisionSettings& a_settings);

    //! Return the root node of the collision tree.
    cCollisionAABBNode* getRoot() const { return (m_root); }


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
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
