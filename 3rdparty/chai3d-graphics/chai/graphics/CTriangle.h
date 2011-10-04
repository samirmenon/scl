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
    \version   2.0.0 $Rev: 270 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CTriangleH
#define CTriangleH
//---------------------------------------------------------------------------
#include "math/CMaths.h"
#include "graphics/CVertex.h"
#include "scenegraph/CMesh.h"
#include "collisions/CCollisionBasics.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file   CTriangle.h
    
    \brief  
    <b> Graphics </b> \n 
    3D triangle.
*/
//===========================================================================

//===========================================================================
/*!
    \struct     cTriangle
    \ingroup    graphics

    \brief    
    cTriangle defines a triangle, typically bound to a mesh for graphic 
    rendering.
*/
//===========================================================================
class cTriangle
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Constructor of cTriangle.

        \param      a_parent  Parent mesh.
        \param      a_indexVertex0  index position of vertex 0.
        \param      a_indexVertex1  index position of vertex 1.
        \param      a_indexVertex2  index position of vertex 2.
    */
    //-----------------------------------------------------------------------
    cTriangle(cMesh* a_parent, const unsigned int a_indexVertex0,
        const unsigned int a_indexVertex1, const unsigned int a_indexVertex2) :
        m_indexVertex0(a_indexVertex0), m_indexVertex1(a_indexVertex1),
        m_indexVertex2(a_indexVertex2),
        m_index(0), m_parent(a_parent), m_allocated(false), m_tag(0), m_neighbors(0)
    { }

    //-----------------------------------------------------------------------
    /*!
        Default constructor of cTriangle.
    */
    //-----------------------------------------------------------------------
    cTriangle() : m_indexVertex0(0), m_indexVertex1(0), m_indexVertex2(0),
        m_index(0), m_parent(0), m_allocated(false), m_tag(0), m_neighbors(0)
    { }


    //-----------------------------------------------------------------------
    /*!
        Destructor of cTriangle.
    */
    //-----------------------------------------------------------------------
    ~cTriangle() {
      if (m_neighbors) {
        m_neighbors->clear();
        //delete m_neighbors;
        //m_neighbors = 0;
      }
    }


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Set the vertices of the triangle by passing the index numbers of
        the corresponding vertices.

        \param      a_indexVertex0  index position of vertex 0.
        \param      a_indexVertex1  index position of vertex 1.
        \param      a_indexVertex2  index position of vertex 2.
    */
    //-----------------------------------------------------------------------
    inline void setVertices(const unsigned int a_indexVertex0,
							const unsigned int a_indexVertex1, 
							const unsigned int a_indexVertex2)
    {
        m_indexVertex0 = a_indexVertex0;
        m_indexVertex1 = a_indexVertex1;
        m_indexVertex2 = a_indexVertex2;
    }


    //-----------------------------------------------------------------------
    /*!
        Read pointer to vertex 0 of triangle.

        \return     Return pointer to vertex 0.
    */
    //-----------------------------------------------------------------------
    inline cVertex* getVertex0() const
    {
        // Where does the vertex array live?
        vector<cVertex>* vertex_vector = m_parent->pVertices();
        cVertex* vertex_array = (cVertex*) &((*vertex_vector)[0]);
        return vertex_array+m_indexVertex0;
    };


    //-----------------------------------------------------------------------
    /*!
        Read pointer to vertex 1 of triangle.

        \return     Return pointer to vertex 1.
    */
    //-----------------------------------------------------------------------
    inline cVertex* getVertex1() const
    {
        // Where does the vertex array live?
        vector<cVertex>* vertex_vector = m_parent->pVertices();
        cVertex* vertex_array = (cVertex*) &((*vertex_vector)[0]);
        return vertex_array+m_indexVertex1;
    };


    //-----------------------------------------------------------------------
    /*!
        Read pointer to vertex 2 of triangle.

        \return     Return pointer to vertex 2.
    */
    //-----------------------------------------------------------------------
    inline cVertex* getVertex2() const
    {
        // Where does the vertex array live?
        vector<cVertex>* vertex_vector = m_parent->pVertices();
        cVertex* vertex_array = (cVertex*) &((*vertex_vector)[0]);
        return vertex_array+m_indexVertex2;
    };


    //-----------------------------------------------------------------------
    /*!
        Access a pointer to the specified vertex of this triangle.

        \param      vi  The triangle (0, 1, or 2) to access.
        \return     Returns a pointer to the requested triangle, or 0 for an
                    illegal index
    */
    //-----------------------------------------------------------------------
    inline cVertex* getVertex(int vi) const
    {

        // Where does the vertex array live?
        vector<cVertex>* vertex_vector = m_parent->pVertices();
        cVertex* vertex_array = (cVertex*) &((*vertex_vector)[0]);

        switch (vi)
        {
          case 0 : return vertex_array+m_indexVertex0;
          case 1 : return vertex_array+m_indexVertex1;
          case 2 : return vertex_array+m_indexVertex2;
        }
        return NULL;
    }


    //-----------------------------------------------------------------------
    /*!
        Access the index of the specified vertex of this triangle.

        \param      vi  The triangle (0, 1, or 2) to access.
        \return     Returns the index of the specified triangle.
    */
    //-----------------------------------------------------------------------
    inline unsigned int getVertexIndex(int vi) const
    {
		switch (vi)
		{
			case 0 : return m_indexVertex0;
			case 1 : return m_indexVertex1;
			case 2 : return m_indexVertex2;
		}
		return 0;
    }

    //-----------------------------------------------------------------------
    /*!
        Read index number of vertex 0 (defines a location in my owning
        mesh's vertex array).

        \return     Return index number.
    */
    //-----------------------------------------------------------------------
    inline unsigned int getIndexVertex0() const
    {
        return (m_indexVertex0);
    };


    //-----------------------------------------------------------------------
    /*!
        Read index number of vertex 1 (defines a location in my owning
        mesh's vertex array).

        \return     Return index number.
    */
    //-----------------------------------------------------------------------
    inline unsigned int getIndexVertex1() const
    {
        return (m_indexVertex1);
    };


    //-----------------------------------------------------------------------
    /*!
        Read index number of vertex 2 (defines a location in my owning
        mesh's vertex array).

        \return     Return index number.
    */
    //-----------------------------------------------------------------------
    inline unsigned int getIndexVertex2() const
    {
        return (m_indexVertex2);
    };


    //-----------------------------------------------------------------------
    /*!
        Read the index of this triangle (defines a location in my owning
        mesh's triangle array).

        \return     Return index number.
    */
    //-----------------------------------------------------------------------
    inline unsigned int getIndex() const
    {
        return (m_index);
    };


    //-----------------------------------------------------------------------
    /*!
        Retrieve a pointer to the mesh that owns this triangle.

        \return     Return pointer to parent mesh.
    */
    //-----------------------------------------------------------------------
    inline cMesh* getParent() const
    {
        return (m_parent);
    };

    //-----------------------------------------------------------------------
    /*!
        Set pointer to mesh parent of triangle.
    */
    //-----------------------------------------------------------------------
    inline void setParent(cMesh* parent)
    {
        m_parent = parent;
    };


    //-----------------------------------------------------------------------
    /*!
        Is this triangle allocated to an existing mesh?

        \return     Return \b true if triangle is allocated to an existing
                    mesh, otherwise return \b false.
    */
    //-----------------------------------------------------------------------
    inline bool allocated() const
    {
        return (m_allocated);
    };



    //-----------------------------------------------------------------------
    /*!
        Check if a ray intersects this triangle. The ray is described
        by its origin (\e a_origin) and its direction (\e a_direction). \n

        If a collision occurs, this information is stored in the collision
        recorder \e a_recorder. \n

        This is one of the most performance-critical routines in CHAI,
        so we have code here for a couple different approaches that may
        become useful in different scenarios.

        \param   a_segmentPointA  Point from where collision ray starts (in local frame).
        \param   a_segmentPointB  Direction vector of collision ray (in local frame).
        \param   a_recorder  Stores collision events
        \param   a_settings  Settings related to collision detection process.
        \return  Returns \b true if a collision occured, otherwise \b false.    

    */
    //-----------------------------------------------------------------------
    inline bool computeCollision(cVector3d& a_segmentPointA,
                                 cVector3d& a_segmentPointB,
                                 cCollisionRecorder& a_recorder,
                                 cCollisionSettings& a_settings) const
    {
        // temp variables
        bool hit = false;
        cVector3d collisionPoint;
        cVector3d collisionNormal;
        double collisionDistanceSq = CHAI_LARGE;

        // Get the position of the triangle's vertices
        vector<cVertex>* vertex_vector = m_parent->pVertices();
        cVertex* vertex_array = (cVertex*) &((*vertex_vector)[0]);
        cVector3d vertex0 = vertex_array[m_indexVertex0].getPos();
        cVector3d vertex1 = vertex_array[m_indexVertex1].getPos();
        cVector3d vertex2 = vertex_array[m_indexVertex2].getPos();

        // If m_collisionRadius == 0, we search for a possible intersection between
        // the segment AB and the triangle defined by its three vertices V0, V1, V2.
        if (a_settings.m_collisionRadius == 0)
        {
            // check for collision between segment and triangle only
            if (cIntersectionSegmentTriangle(a_segmentPointA,
                                            a_segmentPointB,
                                            vertex0,
                                            vertex1,
                                            vertex2,
                                            collisionPoint,
                                            collisionNormal))
            {
                hit = true;
                collisionDistanceSq = cDistanceSq(a_segmentPointA, collisionPoint);
            }
        }


        // If m_collisionRadius > 0, we search for a possible intersection between
        // the segment AB and the shell of the current triangle which is described
        // by its three vertices and m_collisionRadius.
        else
        {
            cVector3d t_collisionPoint, t_collisionNormal;
            double t_collisionDistanceSq;
            cVector3d normal = cComputeSurfaceNormal(vertex0, vertex1, vertex2);
            cVector3d offset; normal.mulr(a_settings.m_collisionRadius, offset);
            cVector3d t_vertex0, t_vertex1, t_vertex2;

            // check for collision between segment and triangle upper shell
            vertex0.addr(offset, t_vertex0);
            vertex1.addr(offset, t_vertex1);
            vertex2.addr(offset, t_vertex2);
            if (cIntersectionSegmentTriangle(a_segmentPointA,
                                            a_segmentPointB,
                                            t_vertex0,
                                            t_vertex1,
                                            t_vertex2,
                                            collisionPoint,
                                            collisionNormal))
            {
                hit = true;
                collisionDistanceSq = cDistanceSq(a_segmentPointA, collisionPoint);
            }

            // check for collision between segment and triangle lower shell
            vertex0.subr(offset, t_vertex0);
            vertex1.subr(offset, t_vertex1);
            vertex2.subr(offset, t_vertex2);
            if (cIntersectionSegmentTriangle(a_segmentPointA,
                                            a_segmentPointB,
                                            t_vertex0,
                                            t_vertex1,
                                            t_vertex2,
                                            t_collisionPoint,
                                            t_collisionNormal))
            {
                hit = true;
                t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
                if (t_collisionDistanceSq < collisionDistanceSq)
                {
                    collisionPoint = t_collisionPoint;
                    collisionNormal = t_collisionNormal;
                    collisionDistanceSq = t_collisionDistanceSq;
                }
            }

            // check for collision between sphere located at vertex 0
            cVector3d t_p, t_n;
            if (cIntersectionSegmentSphere(a_segmentPointA,
                                           a_segmentPointB,
                                           vertex0,
                                           a_settings.m_collisionRadius,
                                           t_collisionPoint,
                                           t_collisionNormal,
                                           t_p,
                                           t_n) > 0)
            {
                hit = true;
                t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
                if (t_collisionDistanceSq < collisionDistanceSq)
                {
                    collisionPoint = t_collisionPoint;
                    collisionNormal = t_collisionNormal;
                    collisionDistanceSq = t_collisionDistanceSq;
                }
            }

            // check for collision between sphere located at vertex 1
            if (cIntersectionSegmentSphere(a_segmentPointA,
                                           a_segmentPointB,
                                           vertex1,
                                           a_settings.m_collisionRadius,
                                           t_collisionPoint,
                                           t_collisionNormal,
                                           t_p,
                                           t_n) > 0)
            {
                hit = true;
                t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
                if (t_collisionDistanceSq < collisionDistanceSq)
                {
                    collisionPoint = t_collisionPoint;
                    collisionNormal = t_collisionNormal;
                    collisionDistanceSq = t_collisionDistanceSq;
                }
            }

            // check for collision between sphere located at vertex 2
            if (cIntersectionSegmentSphere(a_segmentPointA,
                                           a_segmentPointB,
                                           vertex2,
                                           a_settings.m_collisionRadius,
                                           t_collisionPoint,
                                           t_collisionNormal,
                                           t_p,
                                           t_n) > 0)
            {
                hit = true;
                t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
                if (t_collisionDistanceSq < collisionDistanceSq)
                {
                    collisionPoint = t_collisionPoint;
                    collisionNormal = t_collisionNormal;
                    collisionDistanceSq = t_collisionDistanceSq;
                }
            }

            // check for collision between segment and triangle edge01 shell
            if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
                                           a_segmentPointB,
                                           vertex0,
                                           vertex1,
                                           a_settings.m_collisionRadius,
                                           t_collisionPoint,
                                           t_collisionNormal,
                                           t_p,
                                           t_n) > 0)
            {
                hit = true;
                t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
                if (t_collisionDistanceSq < collisionDistanceSq)
                {
                    collisionPoint = t_collisionPoint;
                    collisionNormal = t_collisionNormal;
                    collisionDistanceSq = t_collisionDistanceSq;
                }
            }

            // check for collision between segment and triangle edge02 shell
            if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
                                           a_segmentPointB,
                                           vertex0,
                                           vertex2,
                                           a_settings.m_collisionRadius,
                                           t_collisionPoint,
                                           t_collisionNormal,
                                           t_p,
                                           t_n) > 0)
            {
                hit = true;
                t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
                if (t_collisionDistanceSq < collisionDistanceSq)
                {
                    collisionPoint = t_collisionPoint;
                    collisionNormal = t_collisionNormal;
                    collisionDistanceSq = t_collisionDistanceSq;
                }
            }

            // check for collision between segment and triangle edge12 shell
            if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
                                           a_segmentPointB,
                                           vertex1,
                                           vertex2,
                                           a_settings.m_collisionRadius,
                                           t_collisionPoint,
                                           t_collisionNormal,
                                           t_p,
                                           t_n) > 0)
            {
                hit = true;
                t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
                if (t_collisionDistanceSq < collisionDistanceSq)
                {
                    collisionPoint = t_collisionPoint;
                    collisionNormal = t_collisionNormal;
                    collisionDistanceSq = t_collisionDistanceSq;
                }
            }
        }

        // report collision
        if (hit)
        {
            // before reporting the new collision, we need to check if
            // the collision settings require us to verify the side of the
            // triangle which has been hit.
            bool hit_confirmed;
            if (a_settings.m_checkBothSidesOfTriangles)
            {
                // settings specify that a collision can occur on both sides
                // of the triangle, so the new collision is reported.
                hit_confirmed = true;
            }
            else
            {
                // we need check on which side of the triangle the collision occurred
                // and see it needs to be reported.
                cVector3d segmentAB;
                a_segmentPointB.subr(a_segmentPointA, segmentAB);

                cVector3d v01, v02, triangleNormal;
                vertex2.subr(vertex0, v02);
                vertex1.subr(vertex0, v01);
                v01.crossr(v02, triangleNormal);

                double value = cCosAngle(segmentAB, triangleNormal);
                if (value <= 0.0)
                {
                    hit_confirmed = true;
                }
                else
                {
                    hit_confirmed = false;
                }
            }


            // here we finally report the new collision to the collision event handler.
            if (hit_confirmed)
            {
                // we verify if anew collision needs to be created or if we simply
                // need to update the nearest collision.
                if (a_settings.m_checkForNearestCollisionOnly)
                {
                    // no new collision event is create. We just check if we need
                    // to update the nearest collision
                    if(collisionDistanceSq < a_recorder.m_nearestCollision.m_squareDistance)
                    {
                        // report basic collision data
                        a_recorder.m_nearestCollision.m_object = m_parent;
                        a_recorder.m_nearestCollision.m_triangle = (cTriangle*)this;
                        a_recorder.m_nearestCollision.m_localPos = collisionPoint;
                        a_recorder.m_nearestCollision.m_localNormal = collisionNormal;
                        a_recorder.m_nearestCollision.m_squareDistance = collisionDistanceSq;
                        a_recorder.m_nearestCollision.m_adjustedSegmentAPoint = a_segmentPointA;

                        // report advanced collision data
                        if (!a_settings.m_returnMinimalCollisionData)
                        {
                            a_recorder.m_nearestCollision.m_globalPos = cAdd(m_parent->getGlobalPos(),
                                                                        cMul(m_parent->getGlobalRot(),
                                                                        a_recorder.m_nearestCollision.m_localPos));
                            a_recorder.m_nearestCollision.m_globalNormal = cMul(m_parent->getGlobalRot(),
                            a_recorder.m_nearestCollision.m_localNormal);
                        }

                    }
                }
                else
                {
                    cCollisionEvent newCollisionEvent;

                    // report basic collision data
                    newCollisionEvent.m_object = m_parent;
                    newCollisionEvent.m_triangle = (cTriangle*)this;
                    newCollisionEvent.m_localPos = collisionPoint;
                    newCollisionEvent.m_localNormal = collisionNormal;
                    newCollisionEvent.m_squareDistance = collisionDistanceSq;
                    newCollisionEvent.m_adjustedSegmentAPoint = a_segmentPointA;

                    // report advanced collision data
                    if (!a_settings.m_returnMinimalCollisionData)
                    {
                        newCollisionEvent.m_globalPos = cAdd(m_parent->getGlobalPos(),
                                                        cMul(m_parent->getGlobalRot(),
                                                        newCollisionEvent.m_localPos));
                        newCollisionEvent.m_globalNormal = cMul(m_parent->getGlobalRot(),
                        newCollisionEvent.m_localNormal);
                    }

                    // add new collision even to collision list
                    a_recorder.m_collisions.push_back(newCollisionEvent);

                    // check if this new collision is a candidate for "nearest one"
                    if(collisionDistanceSq < a_recorder.m_nearestCollision.m_squareDistance)
                    {
                        a_recorder.m_nearestCollision = newCollisionEvent;
                    }
                }
            }

            // return result
            return (hit_confirmed);
        }
        else
        {
            return (false);
        }
    }


    //-----------------------------------------------------------------------
    /*!
        Compute and return the area of this triangle.

        \return     Returns the area of this triangle.
    */
    //-----------------------------------------------------------------------
    double computeArea()
    {
        // A = 0.5 * | u x v |
        cVector3d u = cSub(getVertex(1)->getPos(),getVertex(0)->getPos());
        cVector3d v = cSub(getVertex(2)->getPos(),getVertex(0)->getPos());
        return (0.5 * (cCross(u,v).length()));
    }


  public:
    
    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------
      
    //! Index number of vertex 0 (defines a location in my owning mesh's vertex array).
    unsigned int m_indexVertex0;

    //! Index number of vertex 1 (defines a location in my owning mesh's vertex array).
    unsigned int m_indexVertex1;

    //! Index number of vertex 2 (defines a location in my owning mesh's vertex array).
    unsigned int m_indexVertex2;

    //! Index number of this triangle (defines a location in my owning mesh's triangle array).
    unsigned int m_index;

    //! The mesh that owns me.
    cMesh* m_parent;

    //! Is this triangle still active?
    bool m_allocated;

    //! For custom use. No specific purpose.
    int m_tag;

    //! A mesh can be organized into a network of neighboring triangles, which are stored here...
    std::vector<cTriangle*>* m_neighbors;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------


