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
    \version   2.0.0 $Rev: 244 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "collisions/CCollisionSpheres.h"
#include <algorithm>
//---------------------------------------------------------------------------
//! Pointer to first free location in array of sphere tree internal nodes.
cCollisionSpheresNode* g_nextInternalNode;

//! Pointer to first free location in array of sphere tree leaf nodes.
cCollisionSpheresLeaf* g_nextLeafNode;

//! A "sufficiently small" number; zero within tolerated precision.
const double LITTLE = 1e-10;

//! A "sufficiently large" number; effectively infinity.
const double LARGE = 1e10;

//cTriangle* secret2;
//---------------------------------------------------------------------------


//===========================================================================
/*!
    Constructor of cCollisionSpheres.

    \fn       cCollisionSpheres::cCollisionSpheres(vector<cTriangle> *a_triangles,
              bool a_useNeighbors)
    \param    a_triangles     Pointer to array of triangles.
    \param    a_useNeighbors  Use neighbor lists to speed up collision detection?
*/
//===========================================================================
cCollisionSpheres::cCollisionSpheres(vector<cTriangle> *a_triangles,
                                     bool a_useNeighbors)
{
    m_trigs = a_triangles;
    m_useNeighbors = a_useNeighbors;
    m_root = NULL;
    m_firstLeaf = 0;

    // set material properties
    m_material.m_ambient.set(0.1, 0.3, 0.1, 0.3);
    m_material.m_diffuse.set(0.1, 0.8, 0.1, 0.3);
    m_material.m_specular.set(0.1, 1.0, 0.1, 0.3);
    m_material.setShininess(100);
}


//===========================================================================
/*!
    Destructor of cCollisionSpheres.

    \fn       cCollisionSpheres::~cCollisionSpheres()
*/
//===========================================================================
cCollisionSpheres::~cCollisionSpheres()
{
    // delete array of internal nodes
    if (m_root != NULL)
        delete [] m_root;

    // delete array of leaf nodes
    // if ((m_trigs) && (m_trigs->size() > 1) && (m_firstLeaf))
    if (m_firstLeaf)
    {
        delete [] m_firstLeaf;
        m_firstLeaf = 0;
    }
}


//===========================================================================
/*!
    Build the Sphere Tree collision-detection tree.  Each leaf is associated
    with one triangle and with a bounding sphere of minimal radius such that
    it fully encloses the triangle.  Each internal node is associated
    with a bounding sphere of minimal radius such that it fully encloses
    the bounding spheres of its two children.

    \fn       void cCollisionSpheres::initialize(double a_radius)
    \param    a_radius radius to add around the triangles.
*/
//===========================================================================
void cCollisionSpheres::initialize(double a_radius)
{
	secret = NULL;

    // initialize number of triangles, root pointer, and last intersected triangle
    int numTriangles = m_trigs->size();

    m_root = NULL;
    m_lastCollision = NULL;

    // if there are triangles, build the tree
    if (numTriangles > 0)
    {

        // allocate array for leaf nodes
        g_nextLeafNode = new cCollisionSpheresLeaf[numTriangles];
        m_firstLeaf = g_nextLeafNode;

        // if there is more than one triangle, allocate internal nodes
        if (numTriangles > 1)
        {
            g_nextInternalNode = new cCollisionSpheresNode[numTriangles-1];
            m_root = g_nextInternalNode;
            new(g_nextInternalNode++) cCollisionSpheresNode(m_trigs, NULL, a_radius);
        }

        // if there is only one triangle, just allocate one leaf node and
        // set the root to point to it
        else
        {
            new(&m_firstLeaf[0]) cCollisionSpheresLeaf(&((*m_trigs)[0]));
            m_root = g_nextLeafNode;
        }
    }

    // if there are no triangles, just set the root to null
    else
    {
        m_root = 0;
    }
}


//===========================================================================
/*!
    Draw the collision spheres at the given level.

    \fn       void cCollisionSpheres::render()
*/

//===========================================================================
void cCollisionSpheres::render()
{
    if (m_root == NULL) return;

    bool transparency = m_material.isTransparent();

    // set up transparency if we need it...
    if (transparency)
    {
        glEnable(GL_BLEND);
        glDepthMask(GL_FALSE);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    // set rendering settings
    glEnable(GL_LIGHTING);
    glLineWidth(1.0);
    m_material.render();

    // render tree
    m_root->draw(m_displayDepth);

    // turn off transparency if we used it...
    if (transparency)
    {
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
}


//===========================================================================
/*!
    Check if the given line segment intersects any triangle of the mesh.  If so,
    return true, as well as (through the output parameters) pointers to the
    intersected triangle, the mesh of which this triangle is a part, the point
    of intersection, and the distance from the origin of the segment to the
    collision point.  If more than one triangle is intersected, return the one
    closest to the origin of the segment.  The method uses the pre-computed
    sphere tree, starting at the root and recursing through the tree, breaking
    the recursion along any path in which the sphere bounding the line segment
    does not intersect the sphere of the node.  At the leafs, triangle-segment
    intersection testing is called.

    \fn       bool cCollisionSpheres::computeCollision(cVector3d& a_segmentPointA,
                                         cVector3d& a_segmentPointB,
                                         cCollisionRecorder& a_recorder,
                                         cCollisionSettings& a_settings)
    \param    a_segmentPointA  Initial point of segment.
    \param    a_segmentPointB  End point of segment.
    \param    a_recorder  Stores all collision events
    \param    a_settings  Contains collision settings information.
    \return   Return \b true if a collision event has occurred.
*/
//===========================================================================
bool cCollisionSpheres::computeCollision(cVector3d& a_segmentPointA,
                                         cVector3d& a_segmentPointB,
                                         cCollisionRecorder& a_recorder,
                                         cCollisionSettings& a_settings)
{
    // if this is a subsequent call from the proxy algorithm after detecting
    // an initial collision, and if the flag to use neighbor checking is set,
    // only neighbors of the triangle from the first collision detection
    // need to be checked
    if ((m_useNeighbors) && (m_root != NULL) &&
        (m_lastCollision != NULL) && (m_lastCollision->m_neighbors != NULL))
    {
        // check each neighbor, and find the closest for which there is a
        // collision, if any
        for (unsigned int i=0; i<m_lastCollision->m_neighbors->size(); i++)
        {
            ((*(m_lastCollision->m_neighbors))[i])->computeCollision(
                    a_segmentPointA, a_segmentPointB, a_recorder, a_settings);
        }

        // if at least one neighbor triangle was intersected, return true
        if (a_recorder.m_collisions.size() > 0) return true;

        // otherwise there was no collision; return false
        m_lastCollision = NULL;
        return false;
    }

    // otherwise, if this is the first call in an iteration of the proxy
    // algorithm (or a call from any other algorithm), check the sphere tree

    // if the root is null, the tree is empty, so there can be no collision
    if (m_root == 0)
    {
        m_lastCollision = 0;
        return 0;
    }

    // create a cCollisionSpheresLine object and enclose it in a sphere leaf
    cCollisionSpheresLine curLine(a_segmentPointA,a_segmentPointB);
    cCollisionSpheresLeaf lineSphere(&curLine);

    // test for intersection between the line segment and the root of the
    // collision tree; the root will recursively call children down the tree
    bool result = cCollisionSpheresSphere::computeCollision(m_root,
                                                            &lineSphere,
                                                            a_recorder,
                                                            a_settings);

    // This prevents the destructor from deleting a stack-allocated SpheresLine
    // object
    lineSphere.m_prim = 0;

    // return whether there was an intersection
    return result;
}


//===========================================================================
/*!
    Constructor of cCollisionSpheresSphere.

    \fn         cCollisionSpheresSphere::cCollisionSpheresSphere(cCollisionSpheresSphere *a_parent)
    \param      a_parent     Pointer to parent of this node in the sphere tree.
    \return     Return a pointer to new cCollisionSpheresSphere instance.
*/
//===========================================================================
cCollisionSpheresSphere::cCollisionSpheresSphere(cCollisionSpheresSphere *a_parent) :
    m_parent(a_parent), m_center(0,0,0), m_depth(0)
{
    // set the depth of this node to be one below its parent
    if (m_parent)
    {
        m_depth = m_parent->m_depth + 1;
    }
}


//===========================================================================
/*!
    Determine whether there is any intersection between the primitives
    (line and triangles) in the collision subtrees rooted at the two given
    collision spheres.

    \fn       bool cCollisionSpheresSphere::computeCollision(cCollisionSpheresSphere *a_sa,
              cCollisionSpheresSphere *a_sb, cCollisionRecorder& a_recorder,
              cCollisionSettings& a_settings)
    \param    a_sa  Node 0.
    \param    a_sb  Node 1.
    \param    a_recorder  Stores all collision events
    \param    a_settings  Contains collision settings information.
    \return   Return true if a collision event has occurred.
*/
//===========================================================================
bool cCollisionSpheresSphere::computeCollision(cCollisionSpheresSphere *a_sa,
                                               cCollisionSpheresSphere *a_sb,
                                               cCollisionRecorder& a_recorder,
                                               cCollisionSettings& a_settings)
{
    // if first sphere is an internal node, call internal node collision function
    if (!a_sa->isLeaf())
    {
        return cCollisionSpheresNode::computeCollision((cCollisionSpheresNode*) a_sa,
                a_sb, a_recorder, a_settings);
    }

    // if second sphere is an internal node, call internal node collision function
    if (!a_sb->isLeaf())
    {
        return cCollisionSpheresNode::computeCollision((cCollisionSpheresNode*) a_sb,
                a_sa, a_recorder, a_settings);
    }

    // if both spheres are leaves, call leaf collision function
    return cCollisionSpheresLeaf::computeCollision((cCollisionSpheresLeaf*) a_sa,
            (cCollisionSpheresLeaf*) a_sb, a_recorder, a_settings);
}


//===========================================================================
/*!
    Constructor of cCollisionSpheresNode.

    \fn         cCollisionSpheresNode::cCollisionSpheresNode(Plist &a_primList,
                cCollisionSpheresSphere *a_parent)
    \param      a_primList  List of shape primitives to be enclosed in the
                            subtree rooted at this internal node.
    \param      a_parent  Pointer to the parent of this node in the tree.
*/
//===========================================================================
cCollisionSpheresNode::cCollisionSpheresNode(Plist &a_primList,
                                             cCollisionSpheresSphere *a_parent) :
                                             cCollisionSpheresSphere(a_parent)
{
    // set this node's parent pointer
    m_parent = a_parent;

    // create the left and right subtrees of this node
    ConstructChildren(a_primList);
}


//===========================================================================
/*!
    Constructor of cCollisionSpheresNode.

    \fn       cCollisionSpheresNode::cCollisionSpheresNode(std::vector<cTriangle>* a_tris,
                 cCollisionSpheresSphere *a_parent,
                 double a_extendedRadius) :
                 cCollisionSpheresSphere(a_parent)
    \param    a_tris  Pointer to vector of triangles to use for collision detection.
    \param    a_parent  Pointer to the parent of this node in sphere tree.
    \param    a_extendedRadius  Bounding radius.
*/
//===========================================================================
cCollisionSpheresNode::cCollisionSpheresNode(std::vector<cTriangle>* a_tris,
                                             cCollisionSpheresSphere *a_parent,
                                             double a_extendedRadius) :
                                             cCollisionSpheresSphere(a_parent)
{
    // create cCollisionSpheresTri primitive object for each cTriangle object
    Plist primList;
    for (unsigned int i=0; i<a_tris->size(); i++)
    {
        // create cCollisionSpheresPoint primitive object for first point
        cVector3d vpos1 =  (*a_tris)[i].getVertex0()->getPos();
        cVector3d vpos2 =  (*a_tris)[i].getVertex1()->getPos();
        cVector3d vpos3 =  (*a_tris)[i].getVertex2()->getPos();

        cCollisionSpheresTri* t = new cCollisionSpheresTri(vpos1,
                                                           vpos2,
                                                           vpos3,
                                                           a_extendedRadius);

        t->setOriginal(&(*a_tris)[i]);

        // insert new object in primitives list
        primList.insert(primList.end(), t);
    }

    // set parent pointer
    m_parent = a_parent;

    // create left and right subtrees of this node
    ConstructChildren(primList);
}


//===========================================================================
/*!
    Create subtrees by splitting primitives into left and right lists.

    \fn       void cCollisionSpheresNode::ConstructChildren(Plist &a_primList)
    \param    a_primList  List of shape primitives to be split into left
                          and right subtrees.
*/
//===========================================================================
void cCollisionSpheresNode::ConstructChildren(Plist &a_primList)
{
    // ensure that there are at least two primitives so that it makes sense
    // to split them into left and right subtrees
    Plist::iterator primIter;
    assert(a_primList.size() >= 2);

    // declare and initialize local variables for splitting primitives
    Plist leftList, rightList;
    double min[3] = {LARGE, LARGE, LARGE};
    double max[3] = {-LARGE, -LARGE, -LARGE};

    // find minimum and maximum values for each coordinate of primitves' centers
    for (primIter = a_primList.begin(); primIter != a_primList.end(); primIter++)
    {
        cCollisionSpheresGenericShape *cur = *primIter;
        const cVector3d &center = cur->getCenter();
        for (int i = 0; i < 3; i++)
        {
          if (center.get(i) < min[i])
              min[i] = center.get(i);
          if (center.get(i) > max[i])
              max[i] = center.get(i);
        }
    }

    // find the coordinate index with the largest range (max to min)
    int split = 0;
    if ((max[1] - min[1]) > (max[split] - min[split]))
        split = 1;
    if ((max[2] - min[2]) > (max[split] - min[split]))
        split = 2;

    // sort the primitives according to the coordinate with largest range
    cCollisionSpheresGenericShape::m_split = split;
    std::sort(a_primList.begin(), a_primList.end());

    // put first half in left subtree and second half in right subtree
    unsigned int s;
    for (s=0; s<a_primList.size()/2; s++)
        leftList.insert(leftList.end(), a_primList[s]);
    for (s=a_primList.size()/2; s<a_primList.size(); s++)
        rightList.insert(rightList.end(), a_primList[s]);

    // if the left subtree is empty, transfer one from the right list
    if (leftList.size() == 0)
    {
        leftList.insert(leftList.begin(), *rightList.begin());
        rightList.erase(rightList.begin());
    }

    // create new internal nodes as roots for left and right subtree lists, or
    // a leaf node if the subtree list has only one primitive
    if (leftList.size() == 1)
        m_left = new(g_nextLeafNode++) cCollisionSpheresLeaf(*(leftList.begin()), this);
    else
        m_left = new(g_nextInternalNode++) cCollisionSpheresNode(leftList, this);
    if (rightList.size() == 1)
        m_right = new(g_nextLeafNode++) cCollisionSpheresLeaf(*(rightList.begin()), this);
    else
        m_right = new(g_nextInternalNode++) cCollisionSpheresNode(rightList, this);

    // get centers and radii of left and right children
    const cVector3d &lc = m_left->m_center;
    const cVector3d &rc = m_right->m_center;
    double lr = m_left->getRadius();
    double rr = m_right->getRadius();

    // compute new radius as one-half the sum of the distance between the two
    // childrens' centers and the two childrens' radii
    double dist = lc.distance(rc);
    m_radius = (dist + lr + rr) / 2.0;

    // compute new center along line between childrens' centers
    if (dist != 0)
    {
        double lambda = (m_radius - lr) / dist;
        m_center.x = lc.x + lambda*(rc.x - lc.x);
        m_center.y = lc.y + lambda*(rc.y - lc.y);
        m_center.z = lc.z + lambda*(rc.z - lc.z);
    }

    // if the left and right children have the same center, use this as the
    // new center
    else
    {
        m_center = lc;
    }

    // if one sphere is entirely contained within the other, set this sphere's
    // new center and radius equal to those of the larger one
    if (lr > dist + rr)
    {
        m_center = lc;
        m_radius = lr;
    }
    if (rr > dist + lr)
    {
        m_center = rc;
        m_radius = rr;
    }

	m_radius*=1.001;
}


//===========================================================================
/*!
    Draw the collision sphere if at the given depth.

    \fn       void cCollisionSpheresNode::draw(int a_depth)
    \param    a_depth  Only draw nodes at this depth in the tree.
                       a_depth = -1 renders the complete tree.
*/
//===========================================================================
void cCollisionSpheresNode::draw(int a_depth)
{
    // only render the sphere if this node is at the given depth
    if ( ( (a_depth < 0) && (abs(a_depth) >= m_depth) ) || a_depth == m_depth)
    {
      glPushMatrix();
      glTranslated(m_center.x, m_center.y, m_center.z);
      cDrawSphere(m_radius, 16, 16);
      glPopMatrix();
    }

    // do not go any further if the target depth has been reached
    if (a_depth >= 0 && a_depth == m_depth) { return; }

    // recursively call left and right subtrees
    if (m_left) m_left->draw(a_depth);
    if (m_right) m_right->draw(a_depth);
}


//===========================================================================
/*!
    Exchange the two given pointers.

    \fn       void cCollisionSpheresNode::swapptr(void **a_a, void **a_b)
    \param    a_a     First pointer to be swapped.
    \param    a_b     Second pointer to be swapped.
*/
//===========================================================================
void cCollisionSpheresNode::swapptr(void **a_a, void **a_b)
{
    void *temp;
    temp = *a_a;
    *a_a = *a_b;
    *a_b = temp;
}


//===========================================================================
/*!
    Determine whether there is any intersection between the primitives
    (line and triangles) in the collision subtrees rooted at the two given
    collision spheres.  If so, return (in the output parameters) information
    about the intersected triangle of the mesh closest to the segment origin.

    \fn       bool cCollisionSpheresNode::computeCollision(cCollisionSpheresNode *a_sa,
                                             cCollisionSpheresSphere *a_sb,
                                             cCollisionRecorder& a_recorder,
                                             cCollisionSettings& a_settings)
    \param    a_sa  Root of one sphere tree to check for collision.
    \param    a_sb  Root of other sphere tree to check for collision.
    \param    a_recorder  Stores all collision events
    \param    a_settings  Contains collision settings information.
    \return   Return whether any primitives within the two sphere trees collide.
*/
//===========================================================================
bool cCollisionSpheresNode::computeCollision(cCollisionSpheresNode *a_sa,
                                             cCollisionSpheresSphere *a_sb,
                                             cCollisionRecorder& a_recorder,
                                             cCollisionSettings& a_settings)
{
    // if both nodes are internal nodes, arrange that the larger one is first
    if (!a_sb->isLeaf() && (a_sa->m_radius < a_sb->m_radius))
      cCollisionSpheresNode::swapptr((void **)&a_sa, (void **)&a_sb);

    // if spheres don't overlap, there can be no collision
    double minSep = (a_sa->m_center).distance(a_sb->m_center);
    if ((minSep - (a_sa->m_radius + a_sb->m_radius)) >= LITTLE)
      return false;

    // check for overlap of larger sphere's left subtree with smaller sphere
    bool l_result = cCollisionSpheresSphere::computeCollision(a_sa->m_left, a_sb, a_recorder, a_settings);

    // check for overlap of larger sphere's right subtree with smaller sphere
    bool r_result = cCollisionSpheresSphere::computeCollision(a_sa->m_right, a_sb, a_recorder, a_settings);

    // return result
    return (l_result || r_result);
}


//===========================================================================
/*!
    Constructor of cCollisionSpheresLeaf.

    \fn       cCollisionSpheresLeaf::cCollisionSpheresLeaf(
              cCollisionSpheresGenericShape *a_prim,
              cCollisionSpheresSphere *a_parent)
    \param    a_prim  Pointer to a shape primitive to be enclosed by
                      the new sphere leaf node.
    \param    a_parent  Pointer to the parent of this node in sphere tree.
*/
//===========================================================================
cCollisionSpheresLeaf::cCollisionSpheresLeaf(cCollisionSpheresGenericShape *a_prim,
                                             cCollisionSpheresSphere *a_parent) :
                                             cCollisionSpheresSphere(a_parent)
{
    // set this node's primitive pointer to the received shape primitive
    assert(a_prim);
    m_prim = a_prim;

    // set the primitive's sphere pointer to this node
    m_prim->setSphere(this);

    // set the center and radius of the bounding sphere to enclose the primitive
    m_radius = m_prim->getRadius();
    m_center = m_prim->getCenter();

    // set this node's parent pointer
    m_parent = a_parent;
}


//===========================================================================
/*!
    Constructor of cCollisionSpheresLeaf.

    \fn       cCollisionSpheresLeaf::cCollisionSpheresLeaf(cTriangle* a_tri,
                                             cCollisionSpheresSphere *a_parent,
                                             double a_extendedRadius)
    \param    a_tri  Pointer to triangle to be enclosed by new sphere leaf.
    \param    a_parent  Pointer to the parent of this node in the sphere tree.
    \param    a_extendedRadius  Bounding radius.
    \return   Return a pointer to new cCollisionSpheresLeaf instance.
*/
//===========================================================================
cCollisionSpheresLeaf::cCollisionSpheresLeaf(cTriangle* a_tri,
                                             cCollisionSpheresSphere *a_parent,
                                             double a_extendedRadius) :
                                             cCollisionSpheresSphere(a_parent)
{
    // create cCollisionSpheresPoint primitive object for first point
    cVector3d vpos0 = a_tri->getVertex0()->getPos();
    cVector3d vpos1 = a_tri->getVertex1()->getPos();
    cVector3d vpos2 = a_tri->getVertex2()->getPos();

    cCollisionSpheresTri* t = new cCollisionSpheresTri(vpos0,
                                                       vpos1,
                                                       vpos2,
                                                       a_extendedRadius);

    // set pointers
    t->setOriginal(a_tri);
    m_prim = t;
    m_prim->setSphere(this);
    m_parent = a_parent;

    // set the center and radius of the bounding sphere to enclose the primitive
    m_radius = m_prim->getRadius();
    m_center = m_prim->getCenter();
}


//===========================================================================
/*!
    Draw the collision sphere if at the given depth.

    \fn       void cCollisionSpheresLeaf::draw(int a_depth)
    \param    a_depth   Only draw nodes at this depth in the tree.
                        a_depth = -1 renders the complete tree.
*/
//===========================================================================
void cCollisionSpheresLeaf::draw(int a_depth)
{
    // only render the sphere if this node is at the given depth
    if ( ( (a_depth < 0) && (abs(a_depth) >= m_depth) ) || a_depth == m_depth)
    {      
      glPushMatrix();
      glTranslated(m_center.x, m_center.y, m_center.z);
      cDrawSphere(m_radius, 16, 16);
      glPopMatrix();
    }
}


//===========================================================================
/*!
    Determine whether there is any intersection between the primitives
    (line and triangle) of the two given collision spheres by calling the
    primitive's collision detection method.

    \fn       bool cCollisionSpheresLeaf::computeCollision(cCollisionSpheresLeaf *a_sa,
                                             cCollisionSpheresLeaf *a_sb,
                                             cCollisionRecorder& a_recorder,
                                             cCollisionSettings& a_settings)

    \param    a_sa  One sphere tree leaf to check for collision.
    \param    a_sb  Other sphere tree leaf to check for collision.
    \param    a_recorder  Stores all collision events
    \param    a_settings  Contains collision settings information.
    \return   Return whether primitives of the two leaves collide.
*/
//===========================================================================
bool cCollisionSpheresLeaf::computeCollision(cCollisionSpheresLeaf *a_sa,
                                             cCollisionSpheresLeaf *a_sb,
                                             cCollisionRecorder& a_recorder,
                                             cCollisionSettings& a_settings)
{
    // check for a collision between the primitives (one a triangle and the
    // other a line segment, we assume) of these two leafs; it will only
    // return true if the distance between the segment origin and the
    // triangle is less than the current closest intersecting triangle
    // (whose distance squared is kept in colSquareDistance)
    return a_sa->m_prim->computeCollision(a_sb->m_prim,
                                          a_recorder,
                                          a_settings);
}

