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
    \author    Chris Sewell
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 995 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "collisions/CCollisionAABBTree.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//! Pointer for creating new AABB tree nodes, declared in CCollisionAABB.cpp.
extern cCollisionAABBInternal* g_nextFreeNode;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Determine whether the two given boxes intersect each other.

    \fn       bool intersect(const cCollisionAABBBox& a_0, const cCollisionAABBBox& a_1)
    \param    a_0   First box; may intersect with second box.
    \param    a_1   Second box; may intersect with first box.
    \return   Return whether there is any overlap of the two boxes.
*/
//==============================================================================
inline bool intersect(const cCollisionAABBBox& a_0, const cCollisionAABBBox& a_1)
{
    // check for overlap along each axis
    if (a_0.getLowerX() > a_1.getUpperX()) return false;
    if (a_0.getLowerY() > a_1.getUpperY()) return false;
    if (a_0.getLowerZ() > a_1.getUpperZ()) return false;
    if (a_1.getLowerX() > a_0.getUpperX()) return false;
    if (a_1.getLowerY() > a_0.getUpperY()) return false;
    if (a_1.getLowerZ() > a_0.getUpperZ()) return false;

    // if the boxes are not separated along any axis, a collision has occurred
    return true;
}


//==============================================================================
/*!
    Render bounding box of leaf node if it is at level a_depth in the tree.

    \fn       void cCollisionAABBLeaf::render(int a_depth)
    \param    a_depth  Only draw nodes at this depth in the tree.
                       a_depth < 0 render _up to_ abs(a_depth).
*/
//==============================================================================
void cCollisionAABBLeaf::render(int a_depth)
{
#ifdef C_USE_OPENGL
    if ( ( (a_depth < 0) && (abs(a_depth) >= m_depth) ) || a_depth == m_depth)
    {
        if (a_depth < 0)
        {
            cColorf c(1.0, 0.0, 0.0, 1.0);
            glColor4fv(c.pColor());
        }
        m_bbox.render();
    }
#endif
}


//==============================================================================
/*!
      Create a bounding box to enclose the three vertices of the triangle
      belonging to the leaf node.

      \fn       void cCollisionAABBLeaf::fitBBox(double a_radius)
      \param    a_radius Radius around the triangle
*/
//==============================================================================
void cCollisionAABBLeaf::fitBBox(double a_radius)
{
    // empty box
    m_bbox.setEmpty();

    // enclose all three vertices of triangle
    if (m_triangle != NULL)
    {
        a_radius = 2*a_radius;
        m_bbox.enclose(m_triangle->getVertex0()->getLocalPos());
        m_bbox.enclose(m_triangle->getVertex1()->getLocalPos());
        m_bbox.enclose(m_triangle->getVertex2()->getLocalPos());
        cVector3d min = m_bbox.m_min;
        cVector3d max = m_bbox.m_max;
        min.sub(a_radius, a_radius, a_radius);
        max.add(a_radius, a_radius, a_radius);
        m_bbox.setValue(min, max);
    }
}


//==============================================================================
/*!
    Determine whether the given line intersects the triangle belonging to
    this leaf node by calling the triangle's collision detection method.

    \fn       bool cCollisionAABBLeaf::computeCollision(cGenericObject* a_owner,
                                          cVector3d& a_segmentPointA,
                                          cVector3d& a_segmentPointB,
                                          cCollisionAABBBox& a_lineBox,
                                          cCollisionRecorder& a_recorder,
                                          cCollisionSettings& a_settings)

    \param    a_owner  Pointer to object which owns collision detector.
    \param    a_segmentPointA  Initial point of segment.
    \param    a_segmentPointB  End point of segment.
    \param    a_lineBox  A bounding box for the incoming segment, for quick
                         discarding of collision tests.
    \param    a_recorder  Stores all collision events.
    \param    a_settings  Contains collision settings information.

    \return   Return __true__ if the line segment intersects the leaf's triangle.
*/
//==============================================================================
bool cCollisionAABBLeaf::computeCollision(cGenericObject* a_owner,
                                          cVector3d& a_segmentPointA,
                                          cVector3d& a_segmentPointB,
                                          cCollisionAABBBox& a_lineBox,
                                          cCollisionRecorder& a_recorder,
                                          cCollisionSettings& a_settings)
{
    // check for a collision between this leaf's triangle and the segment by
    // calling the triangle's collision detection method; it will only
    // return true if the distance between the segment origin and this
    // triangle is less than the current closest intersecting triangle
    // (whose distance squared is kept in colSquareDistance)
    bool result = m_triangle->computeCollision(a_owner,
                                               a_segmentPointA,
                                               a_segmentPointB,
                                               a_recorder,
                                               a_settings);

    // return result
    return (result);
}


//==============================================================================
/*!
    Draw the edges of the bounding box for an internal tree node if it is
    at depth a_depth in the tree, and call the draw function for its children.

    \fn       void cCollisionAABBInternal::render(int a_depth)
    \param    a_depth   Only draw nodes at this level in the tree.
                        a_depth < 0 render _up to_ this level.
*/
//==============================================================================
void cCollisionAABBInternal::render(int a_depth)
{
#ifdef C_USE_OPENGL
    // render current node
    if ( ( (a_depth < 0) && (abs(a_depth) >= m_depth) ) || a_depth == m_depth)
    {
        if (a_depth < 0)
        {
            cColorf c(1.0, 0.0, 0.0, 1.0);
            glColor4fv(c.pColor());
        }
        m_bbox.render();
    }

    // render left sub tree
    m_leftSubTree->render(a_depth);

    // render right sub tree
    m_rightSubTree->render(a_depth);
#endif
}


//==============================================================================
/*!
    Initialize an internal AABB tree node.

    \fn       void cCollisionAABBInternal::initialize(unsigned int a_numLeaves,
                                        cCollisionAABBLeaf *a_leaves,
                                        unsigned int a_depth)
    \param    a_numLeaves  Number of leaves in subtree rooted at this node.
    \param    a_leaves  Pointer to the location in the array of leafs for the
                        first leaf under this internal node.
    \param    a_depth  Depth of this node in the collision tree.
*/
//==============================================================================
void cCollisionAABBInternal::initialize(unsigned int a_numLeaves,
                                        cCollisionAABBLeaf *a_leaves,
                                        unsigned int a_depth)
{
    // increment free node counter
    g_nextFreeNode++;

    // set depth of this node and initialize left and right subtree pointers
    m_depth = a_depth;
    m_leftSubTree = NULL;
    m_rightSubTree = NULL;
    m_testLineBox = true;

    // create a box to enclose all the leafs below this internal node
    m_bbox.setEmpty();
    for (unsigned int j = 0; j < a_numLeaves; ++j)
    {
        m_bbox.enclose(a_leaves[j].m_bbox);
    }

    // move leafs with smaller coordinates (on the longest axis) towards the
    // beginning of the array and leaves with larger coordinates towards the
    // end of the array
    int axis = m_bbox.longestAxis();
    unsigned int i = 0;
    unsigned int mid = a_numLeaves;
    while (i < mid)
    {
        if (a_leaves[i].m_bbox.getCenter().get(axis) < m_bbox.getCenter().get(axis))
        {
            ++i;
        }
        else
        {
            mid--;
            //std::swap(a_leaves[i], a_leaves[mid]);

            cTriangle *t_triangle           = a_leaves[i].m_triangle;
            cCollisionAABBBox t_bbox        = a_leaves[i].m_bbox;
            int t_depth                     = a_leaves[i].m_depth;
            cCollisionAABBNode* t_parent    = a_leaves[i].m_parent;
            int t_nodeType                  = a_leaves[i].m_nodeType;

            a_leaves[i].m_triangle = a_leaves[mid].m_triangle;
            a_leaves[i].m_bbox     = a_leaves[mid].m_bbox;
            a_leaves[i].m_depth    = a_leaves[mid].m_depth;
            a_leaves[i].m_parent   = a_leaves[mid].m_parent;
            a_leaves[i].m_nodeType = a_leaves[mid].m_nodeType;

            a_leaves[mid].m_triangle = t_triangle;
            a_leaves[mid].m_bbox     = t_bbox;
            a_leaves[mid].m_depth    = t_depth;
            a_leaves[mid].m_parent   = t_parent;
            a_leaves[mid].m_nodeType = t_nodeType;
        }
    }

    // we expect mid, used as the right iterator in the "insertion sort" style
    // rearrangement above, to have moved roughly to the middle of the array;
    // however, if it never moved left or moved all the way left, set it to
    // the middle of the array so that neither the left nor right subtree will
    // be empty
    if (mid == 0 || mid == a_numLeaves)
    {
        mid = a_numLeaves / 2;
    }

    // if the right subtree contains multiple triangles, create new internal node
    if (mid >= 2)
    {
        m_rightSubTree = g_nextFreeNode;
        g_nextFreeNode->initialize(mid, &a_leaves[0], m_depth + 1);
        //new(g_nextFreeNode++) cCollisionAABBInternal(mid, &a_leaves[0], m_depth + 1);
    }

    // if there is only one triangle in the right subtree, the right subtree
    // pointer should just point to the leaf node
    else
    {
        m_rightSubTree = &a_leaves[0];
        if (m_rightSubTree != NULL) m_rightSubTree->m_depth = m_depth + 1;
    }

    // if the left subtree contains multiple triangles, create new internal node
    if (a_numLeaves - mid >= 2)
    {
        m_leftSubTree = g_nextFreeNode;
        g_nextFreeNode->initialize(a_numLeaves - mid, &a_leaves[mid], m_depth + 1);
        // new(g_nextFreeNode++) cCollisionAABBInternal(a_numLeaves - mid, &a_leaves[mid], m_depth + 1);
    }

    // if there is only one triangle in the left subtree, the left subtree
    // pointer should just point to the leaf node
    else
    {
        m_leftSubTree = &a_leaves[mid];
        if (m_leftSubTree) m_leftSubTree->m_depth = m_depth + 1;
    }
}


//==============================================================================
/*!
    Determine whether the given ray intersects the bounding box.  Based on code
    by Andrew Woo from "Graphics Gems", Academic Press, 1990.

    \fn       bool hitBoundingBox(double a_minB[3], double a_maxB[3],
                                  double a_origin[3], double a_dir[3])
    \param    a_minB[3]   Minimum coordinates (along each axis) of bounding box.
    \param    a_maxB[3]   Maximum coordinates (along each axis) of bounding box.
    \param    a_origin[3] Origin of the ray.
    \param    a_dir[3]    Direction of the ray.
    \return   Return true if line segment intersects the bounding box.
*/
//==============================================================================
bool hitBoundingBox(const double a_minB[3], const double a_maxB[3], const double a_origin[3], const double a_end[3])
{
    const int RIGHT	= 0;
    const int LEFT = 1;
    const int MIDDLE = 2;

    double coord[3];
    char inside = true;
    char quadrant[3];
    register int i;
    int whichPlane;
    double maxT[3];
    double candidatePlane[3];
    double dir[3];
    dir[0] = a_end[0] - a_origin[0];
    dir[1] = a_end[1] - a_origin[1];
    dir[2] = a_end[2] - a_origin[2];

    // Find candidate planes; this loop can be avoided if
    // rays cast all from the eye (assume perspective view)
    for (i=0; i<3; i++)
    {
        if(a_origin[i] < a_minB[i])
        {
            quadrant[i] = LEFT;
            candidatePlane[i] = a_minB[i];
            inside = false;
        }
        else if (a_origin[i] > a_maxB[i])
        {
            quadrant[i] = RIGHT;
            candidatePlane[i] = a_maxB[i];
            inside = false;
        }
        else
        {
           quadrant[i] = MIDDLE;
        }
    }

    // Ray origin inside bounding box
    if (inside)
    {
        //coord = origin;
        return (true);
    }

    // Calculate T distances to candidate planes
    for (i = 0; i < 3; i++)
    {
        if (quadrant[i] != MIDDLE && dir[i] !=0.)
            maxT[i] = (candidatePlane[i]-a_origin[i]) / dir[i];
        else
            maxT[i] = -1.;
    }

    // Get largest of the maxT's for final choice of intersection
    whichPlane = 0;
    for (i = 1; i < 3; i++)
        if (maxT[whichPlane] < maxT[i])
            whichPlane = i;

    // Check final candidate actually inside box
    if (maxT[whichPlane] < 0.) return (false);
    for (i = 0; i < 3; i++)
    {
        if (whichPlane != i)
        {
            coord[i] = a_origin[i] + maxT[whichPlane] * dir[i];
            if (coord[i] < a_minB[i] || coord[i] > a_maxB[i])
                return (false);
        }
        else
        {
            coord[i] = candidatePlane[i];
        }
    }

    // Ray hits box...
    return (true);
}


//==============================================================================
/*!
    Determine whether the given line intersects the mesh covered by the
    AABB Tree rooted at this internal node.  If so, return (in the output
    parameters) information about the intersected triangle of the mesh closest
    to the segment origin.

    \fn      bool cCollisionAABBInternal::computeCollision(cGenericObject* a_owner
                                              cVector3d& a_segmentPointA,
                                              cVector3d& a_segmentPointB,
                                              cCollisionAABBBox &a_lineBox,
                                              cCollisionRecorder& a_recorder,
                                              cCollisionSettings& a_settings)

    \param    a_owner  Pointer to object which owns collision detector.
    \param    a_segmentPointA  Initial point of segment.
    \param    a_segmentPointB  End point of segment.
    \param    a_lineBox  A bounding box for the incoming segment, for quick
                         discarding of collision tests.
    \param    a_recorder  Stores all collision events.
    \param    a_settings  Contains collision settings information.

    \return   Return __true__ if line segment intersects a triangle in the subtree.
*/
//==============================================================================
bool cCollisionAABBInternal::computeCollision(cGenericObject* a_owner,
                                              cVector3d& a_segmentPointA,
                                              cVector3d& a_segmentPointB,
                                              cCollisionAABBBox &a_lineBox,
                                              cCollisionRecorder& a_recorder,
                                              cCollisionSettings& a_settings)
{
    // if a line's bounding box does not intersect the node's bounding box,
    // there can be no intersection
    if (!intersect(m_bbox, a_lineBox))
    {
        return (false);
    }

    if (m_testLineBox)
    {
      if (!hitBoundingBox(
        (const double*)(&m_bbox.m_min),
        (const double*)(&m_bbox.m_max),
        (const double*)(&a_segmentPointA),
        (const double*)(&a_segmentPointB)))
        return (false);
    }

    // initialize objects for calls to left and right subtrees
    cVector3d l_colPoint, r_colPoint;
    bool l_result = false;
    bool r_result = false;

    // check collision between line and left subtree node; it will only
    // return true if the distance between the segment origin and this
    // triangle is less than the current closest intersecting triangle
    // (whose distance squared is in l_colSquareDistance)
    if ( m_leftSubTree && m_leftSubTree->computeCollision(a_owner, 
                                                          a_segmentPointA, 
                                                          a_segmentPointB, 
                                                          a_lineBox, 
                                                          a_recorder, 
                                                          a_settings))
    {
        l_result = true;
    }

    // check collision between line and right subtree node; it will only
    // return true if the distance between the segment origin and this
    // triangle is less than the current closest intersecting triangle
    // (whose distance squared is in r_colSquareDistance)
    if ( m_rightSubTree && m_rightSubTree->computeCollision(a_owner, 
                                                            a_segmentPointA, 
                                                            a_segmentPointB, 
                                                            a_lineBox, 
                                                            a_recorder, 
                                                            a_settings))
    {
        r_result = true;
    }

    // return result
    return (l_result || r_result);
}


//==============================================================================
/*!
    Return whether this node contains the specified triangle tag.

    \fn       void cCollisionAABBInternal::contains_triangle(int tag)
    \param    tag  Tag to inquire about
*/
//==============================================================================
bool cCollisionAABBInternal::contains_triangle(int a_tag)
{
    return (m_leftSubTree->contains_triangle(a_tag) ||
            m_rightSubTree->contains_triangle(a_tag));
}


//==============================================================================
/*!
    Sets this node's parent pointer and optionally propagate
    assignments to its children (setting their parent pointers to this node).

    \fn       void cCollisionAABBInternal::setParent(cCollisionAABBNode* a_parent,
              int a_recursive);
    \param    a_parent     Pointer to this node's parent.
    \param    a_recursive  Propagate assignment down the tree?
*/
//==============================================================================
void cCollisionAABBInternal::setParent(cCollisionAABBNode* a_parent, int a_recursive)
{
    m_parent = a_parent;
    if (m_leftSubTree && a_recursive)  m_leftSubTree->setParent(this,1);
    if (m_rightSubTree && a_recursive) m_rightSubTree->setParent(this,1);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
