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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 727 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CCollisionAABBTreeH
#define CCollisionAABBTreeH
//---------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
#include "collisions/CCollisionAABBBox.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CCollisionAABBTree.h

    \brief    
    <b> Collision Detection </b> \n
    Axis-Aligned Bounding Box Tree (AABB) - Implementation.
*/
//===========================================================================

//---------------------------------------------------------------------------
//! Internal AABB Node Types.
typedef enum
{
  AABB_NODE_INTERNAL=0,
  AABB_NODE_LEAF,
  AABB_NODE_GENERIC
} aabb_node_types;

//---------------------------------------------------------------------------

//===========================================================================
/*!
    \class      cCollisionAABBNode
    \ingroup    collisions

    \brief    
    cCollisionAABBNode is an abstract class that contains methods
    to set up internal and leaf nodes of an AABB tree and to use them to 
    detect for collision with a line.
*/
//===========================================================================
class cCollisionAABBNode
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCollisionAABBNode.
    cCollisionAABBNode() : m_parent(0), m_depth(0), m_nodeType(AABB_NODE_GENERIC) {}

    //! Constructor of cCollisionAABBNode.
    cCollisionAABBNode(aabb_node_types a_nodeType, int a_depth) : m_parent(0),
        m_depth(a_depth), m_nodeType(a_nodeType) {}

    //! Destructor of cCollisionAABBNode.
    virtual ~cCollisionAABBNode() {}
	
    
    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Create a bounding box for the portion of the model at or below the node.
    virtual void fitBBox(double a_radius = 0) {}

    //! Draw the edges of the bounding box for this node, if at the given depth.
    virtual void render(int a_depth = -1) = 0;

    //! Determine whether line intersects mesh bounded by subtree rooted at node.
    virtual bool computeCollision(cGenericObject* a_owner,
                                  cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentDirection, 
                                  cCollisionAABBBox &a_lineBox,
                                  cCollisionRecorder& a_recorder, 
                                  cCollisionSettings& a_settings) = 0;

    //! Return true if this node contains the specified triangle tag.
    virtual bool contains_triangle(int a_tag) = 0;

    //! Set the parent of this node.
    virtual void setParent(cCollisionAABBNode* a_parent, int a_recusive) = 0;


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! The bounding box for this node.
    cCollisionAABBBox m_bbox;

    //! Parent node of this node.
    cCollisionAABBNode* m_parent;

    //! The depth of this node in the collision tree.
    int m_depth;

    //! The node type, used only for proper deletion right now.
    int m_nodeType;
};


//===========================================================================
/*!
    \class      cCollisionAABBLeaf
    \ingroup    collisions

    \brief    
    cCollisionAABBLeaf contains methods to set up leaf nodes of an AABB 
    tree and to use them to detect for collision with a line.
*/
//===========================================================================
class cCollisionAABBLeaf : public cCollisionAABBNode
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Default constructor of cCollisionAABBLeaf.
    cCollisionAABBLeaf() : cCollisionAABBNode(AABB_NODE_LEAF, 0) {}

    //! Destructor of cCollisionAABBLeaf.
    virtual ~cCollisionAABBLeaf() {}


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Initialize internal node.
    void initialize(cTriangle *a_triangle, double a_radius) { m_triangle = a_triangle; fitBBox(a_radius); }

    //! Create a bounding box to enclose triangle belonging to this leaf node.
    void fitBBox(double a_radius = 0);

    //! Draw the edges of the bounding box for this leaf if it is at depth a_depth.
    void render(int a_depth = -1);

    //! Determine whether the given line intersects this leaf's triangle.
    bool computeCollision(cGenericObject* a_owner,
                          cVector3d& a_segmentPointA,
                          cVector3d& a_segmentPointB,
                          cCollisionAABBBox& a_lineBox,
                          cCollisionRecorder& a_recorder,
                          cCollisionSettings& a_settings);

    //! Return true if this node contains the specified triangle tag.
    virtual bool contains_triangle(int a_tag)
        { return (m_triangle != 0 && m_triangle->m_tag == a_tag); }

    //! Return parent of this node.
    virtual void setParent(cCollisionAABBNode* a_parent, int a_recusive)
        { m_parent = a_parent; }


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! The triangle bounded by the leaf.
    cTriangle *m_triangle;
};


//===========================================================================
/*!
    \class      cCollisionAABBInternal
    \ingroup    collisions
    
    \brief    
    cCollisionAABBInternal contains methods to set up internal nodes of 
    an AABB tree and to use them to detect for collision with a line.
*/
//===========================================================================
class cCollisionAABBInternal : public cCollisionAABBNode
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Default constructor of cCollisionAABBInternal.
    cCollisionAABBInternal() : cCollisionAABBNode(AABB_NODE_INTERNAL, 0) { }

    //! Destructor of cCollisionAABBInternal.
    virtual ~cCollisionAABBInternal() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Initialize internal node.
    void initialize(unsigned int a_numLeaves, cCollisionAABBLeaf *a_leaves,
            unsigned int a_depth = -1);

    //! Size the bounding box for this node to enclose its children.
    void fitBBox(double a_radius = 0) {m_bbox.enclose(m_leftSubTree->m_bbox, m_rightSubTree->m_bbox);}

    //! Draw the edges of the bounding box for this node if it is at depth a_depth.
    void render(int a_depth = -1);

    //! Determine whether given line intersects the tree rooted at this node.
    bool computeCollision(cGenericObject* a_owner,
                          cVector3d& a_segmentPointA,
                          cVector3d& a_segmentPointB,
                          cCollisionAABBBox &a_lineBox,
                          cCollisionRecorder& a_recorder,
                          cCollisionSettings& a_settings);

    //! Return true if this node contains the specified triangle tag.
    virtual bool contains_triangle(int a_tag);

    //! Return parent node and optionally propagate the assignment to children.
    virtual void setParent(cCollisionAABBNode* a_parent, int a_recursive);


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! The root of this node's left subtree.
    cCollisionAABBNode *m_leftSubTree;

    //! The root of this node's right subtree.
    cCollisionAABBNode *m_rightSubTree;

    //! Test box for collision with ray itself (as compared to line's box)?
    bool m_testLineBox;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

