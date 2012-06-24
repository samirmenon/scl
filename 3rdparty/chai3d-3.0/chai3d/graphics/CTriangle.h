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
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 831 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CTriangleH
#define CTriangleH
//---------------------------------------------------------------------------
#include "math/CMaths.h"
#include "graphics/CVertex.h"
#include "world/CMesh.h"
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
    cTriangle(cMesh* a_owner, 
              const unsigned int a_indexVertex0,
              const unsigned int a_indexVertex1, 
              const unsigned int a_indexVertex2) :
        m_indexVertex0(a_indexVertex0), m_indexVertex1(a_indexVertex1),
        m_indexVertex2(a_indexVertex2), m_index(0), m_owner(a_owner),
        m_allocated(false), m_tag(0)
    { }

    //-----------------------------------------------------------------------
    /*!
        Default constructor of cTriangle.
    */
    //-----------------------------------------------------------------------
    cTriangle() : m_indexVertex0(0), m_indexVertex1(0), m_indexVertex2(0),
        m_index(0), m_owner(0), m_allocated(false), m_tag(0)
    { }


    //-----------------------------------------------------------------------
    /*!
        Destructor of cTriangle.
    */
    //-----------------------------------------------------------------------
    ~cTriangle() 
	{ }


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
        return (m_owner->getVertex(m_indexVertex0));
    };


    //-----------------------------------------------------------------------
    /*!
        Read pointer to vertex 1 of triangle.

        \return     Return pointer to vertex 1.
    */
    //-----------------------------------------------------------------------
    inline cVertex* getVertex1() const
    {
        return (m_owner->getVertex(m_indexVertex1));
    };


    //-----------------------------------------------------------------------
    /*!
        Read pointer to vertex 2 of triangle.

        \return     Return pointer to vertex 2.
    */
    //-----------------------------------------------------------------------
    inline cVertex* getVertex2() const
    {
        return (m_owner->getVertex(m_indexVertex2));
    };


    //-----------------------------------------------------------------------
    /*!
        Access a pointer to the specified vertex of this triangle.

        \param      a_index  The triangle (0, 1, or 2) to access.
        \return     Returns a pointer to the requested vertex.
    */
    //-----------------------------------------------------------------------
    inline cVertex* getVertex(const unsigned int a_index) const
    {
		switch (a_index)
		{
			case 0 : return (m_owner->getVertex(m_indexVertex0));
			case 1 : return (m_owner->getVertex(m_indexVertex1));
			case 2 : return (m_owner->getVertex(m_indexVertex2));
		}
		return (NULL);
		return (m_owner->getVertex(m_indexVertex0 + a_index));
    }


    //-----------------------------------------------------------------------
    /*!
        Access the index of the specified vertex of this triangle.

        \param      a_index  The triangle (0, 1, or 2) to access.
        \return     Returns the index of the specified triangle.
    */
    //-----------------------------------------------------------------------
    inline unsigned int getVertexIndex(const unsigned int a_index) const
    {
		switch (a_index)
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
    inline cMesh* getOwner() const
    {
        return (m_owner);
    };

    //-----------------------------------------------------------------------
    /*!
        Set pointer to mesh owner of triangle.
    */
    //-----------------------------------------------------------------------
    inline void setOwner(cMesh* a_owner)
    {
        m_owner = a_owner;
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
        Get pixel coordinates of texture image by passing relative vertex
        position V01 and V02.

        \param      a_v01  Position of point in respect to vertices segment V01.
        \param      a_v02  Position of point in respect to vertices segment V02.
        \param      a_pixelX  Pixel coordinate X.
        \param      a_pixelY  Pixel coordinate Y.
        \return     Return \b true if pixel found, otherwise return \b false.
    */
    //-----------------------------------------------------------------------
    inline bool getPixel(double& a_v01, double a_v02, int& a_pixelX, int& a_pixelY) const
    {
        // sanity check
        if (m_owner == NULL) { return (false); }
        if (m_owner->m_texture == NULL) { return (false); }
        if (m_owner->m_texture->m_image == NULL) { return (false); }

		cVector3d texCoord0 = getVertex0()->getTexCoord();
		cVector3d texCoord1 = getVertex1()->getTexCoord();
		cVector3d texCoord2 = getVertex2()->getTexCoord();

		cVector3d texCoord = texCoord0 + a_v01 * (texCoord1 - texCoord0) + a_v02 * (texCoord2 - texCoord0);

        int w = m_owner->m_texture->m_image->getWidth();
        int h = m_owner->m_texture->m_image->getHeight();

		double px = (double)(w) * texCoord(0);
		double py = (double)(h) * texCoord(1);

        a_pixelX = cClamp((int)(px), 0, w);
        a_pixelY = cClamp((int)(py), 0, h);

        return (true);
    };


    //-----------------------------------------------------------------------
    /*!
        Get pixel coordinates of texture image by passing a point expressed in
        the local coordinate system of the mesh owning the triangle.

        \param      a_point  Point in local coordinates.
        \param      a_pixelX  Pixel coordinate X.
        \param      a_pixelY  Pixel coordinate Y.

        \return     Return \b true if pixel found, otherwise return \b false.
    */
    //-----------------------------------------------------------------------
    inline bool getPixelFromLocalPoint(const cVector3d a_point, 
                                       int& a_pixelX, 
                                       int& a_pixelY) const
    {
        // sanity check
        if (m_owner == NULL) { return (false); }
        if (m_owner->m_texture == NULL) { return (false); }
        if (m_owner->m_texture->m_image == NULL) { return (false); }

        // get vertices of contact triangles
        cVector3d vertex0 = getVertex0()->getLocalPos();
        cVector3d vertex1 = getVertex1()->getLocalPos();
        cVector3d vertex2 = getVertex2()->getLocalPos();

         // project desired point on triangle
		double a0 = 0; 
		double a1 = 0;
		cProjectPointOnPlane(a_point, vertex0, vertex1, vertex2, a0, a1);
		cVector3d vertex = cAdd(vertex0, cMul(a0, cSub(vertex1, vertex0)), cMul(a1, cSub(vertex2, vertex0)));

		// compute area of triangle
        double area  = cTriangleArea(vertex0, vertex1, vertex2);

        // compute areas of three sub-triangles formed by the three vertices of the triangle and the contact point.
        double area0 = cTriangleArea(vertex, vertex1, vertex2);
        double area1 = cTriangleArea(vertex, vertex0, vertex2);
        double area2 = cTriangleArea(vertex, vertex0, vertex1);
				
		// compute weights based on position of contact point
		double c0, c1, c2;
		if (area > 0.0)
        {
			c0 = area0/area;
			c1 = area1/area;
			c2 = area2/area;
		}
		else
		{
			c0 = c1 = c2 = (1.0/3.0);
		}

        // retrieve the texture coordinate for each triangle vertex
		cVector3d texCoord0 = getVertex0()->getTexCoord();
		cVector3d texCoord1 = getVertex1()->getTexCoord();
		cVector3d texCoord2 = getVertex2()->getTexCoord();

        // compute the exact texture coordinate at the contact point
		cVector3d texCoord = cAdd( cMul(c0, texCoord0), cMul(c1, texCoord1), cMul(c2, texCoord2));

        // compute pixel coordinate
        int w = m_owner->m_texture->m_image->getWidth();
        int h = m_owner->m_texture->m_image->getHeight();

		double px = (double)(w) * texCoord(0);
		double py = (double)(h) * texCoord(1);

        a_pixelX = cClamp((int)(px), 0, w);
        a_pixelY = cClamp((int)(py), 0, h);

        return (true);
    };


    //-----------------------------------------------------------------------
    /*!
        Get pixel coordinates of texture image by passing a point expressed in
        the local coordinate system of the mesh owning the triangle.

        \param      a_point  Point in local coordinates.
        \param      a_pixelX  Pixel coordinate X.
        \param      a_pixelY  Pixel coordinate Y.

        \return     Return \b true if pixel found, otherwise return \b false.
    */
    //-----------------------------------------------------------------------
    inline bool getPixelFromGlobalPoint(const cVector3d a_point, 
                                        int& a_pixelX, 
                                        int& a_pixelY) const
    {
        // sanity check
        if (m_owner == NULL) { return (false); }

        // compute point in local coordinates of object
        cVector3d point = cTranspose(m_owner->getGlobalRot()) * (a_point - m_owner->getGlobalPos());

        // get pixel
        return (getPixelFromLocalPoint(point, a_pixelX, a_pixelY));
    }


    //-----------------------------------------------------------------------
    /*!
        Check if a ray intersects this triangle. The ray is described
        by its origin (\e a_origin) and its direction (\e a_direction). \n

        If a collision occurs, this information is stored in the collision
        recorder \e a_recorder. \n

        This is one of the most performance-critical routines in CHAI,
        so we have code here for a couple different approaches that may
        become useful in different scenarios.

        \param   a_mesh  Pointer to the object on which collision detection is being performed.
        \param   a_segmentPointA  Point from where collision ray starts (in local frame).
        \param   a_segmentPointB  Direction vector of collision ray (in local frame).
        \param   a_recorder  Stores collision events
        \param   a_settings  Settings related to collision detection process.
        \return  Returns \b true if a collision occured, otherwise \b false.    

    */
    //-----------------------------------------------------------------------
    inline bool computeCollision(cGenericObject* a_object,
                                 cVector3d& a_segmentPointA,
                                 cVector3d& a_segmentPointB,
                                 cCollisionRecorder& a_recorder,
                                 cCollisionSettings& a_settings) const
    {
		// verify that triangle is active
		if (!m_allocated) { return (false); }

        // temp variables
        bool hit = false;
        cVector3d collisionPoint;
        cVector3d collisionNormal;
        double collisionDistanceSq = C_LARGE;
        double collisionPointV01 = 0.0;
        double collisionPointV02 = 0.0;

        // retrieve information about which side of the triangles need to be checked
        cMaterial* material = a_object->m_material;
        bool checkFrontSide = material->getRenderFrontSideOfTriangles();
        bool checkBackSide = material->getRenderBackSideOfTriangles();

        // Get the position of the triangle's vertices
		cVector3d vertex0 = m_owner->m_vertices->at(m_indexVertex0).getLocalPos();
        cVector3d vertex1 = m_owner->m_vertices->at(m_indexVertex1).getLocalPos();
        cVector3d vertex2 = m_owner->m_vertices->at(m_indexVertex2).getLocalPos();

        // If m_collisionRadius == 0, we search for a possible intersection between
        // the segment AB and the triangle defined by its three vertices V0, V1, V2.
        if (a_settings.m_collisionRadius == 0.0)
        {
            // check for collision between segment and triangle only
            if (cIntersectionSegmentTriangle(a_segmentPointA,
                                             a_segmentPointB,
                                             vertex0,
                                             vertex1,
                                             vertex2,
                                             checkFrontSide,
                                             checkBackSide,
                                             collisionPoint,
                                             collisionNormal,
                                             collisionPointV01,
                                             collisionPointV02))
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
            double t_collisionPointV01, t_collisionPointV02, t_collisionPointV12;

            // check for collision between segment and triangle upper shell
            vertex0.addr(offset, t_vertex0);
            vertex1.addr(offset, t_vertex1);
            vertex2.addr(offset, t_vertex2);
            if (cIntersectionSegmentTriangle(a_segmentPointA,
                                            a_segmentPointB,
                                            t_vertex0,
                                            t_vertex1,
                                            t_vertex2,
                                            checkFrontSide,
                                            false,
                                            collisionPoint,
                                            collisionNormal,
                                            collisionPointV01,
                                            collisionPointV02))
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
                                            false,
                                            checkBackSide,
                                            t_collisionPoint,
                                            t_collisionNormal,
                                            t_collisionPointV01,
                                            t_collisionPointV02))
            {
                hit = true;
                t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
                if (t_collisionDistanceSq < collisionDistanceSq)
                {
                    collisionPoint = t_collisionPoint;
                    collisionNormal = t_collisionNormal;
                    collisionDistanceSq = t_collisionDistanceSq;
                    collisionPointV01 = t_collisionPointV01;
                    collisionPointV02 = t_collisionPointV02;
                }
            }

            // check for collision between sphere located at vertex 0.
			// if the starting point (a_segmentPointA) is located inside
			// the sphere, we ignore the collision to avoid remaining
			// stuck inside the triangle.
            cVector3d t_p, t_n;
            double t_c;
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
					collisionPointV01 = 0.0;
					collisionPointV02 = 0.0;
				}
			}


            // check for collision between sphere located at vertex 1.
			// if the starting point (a_segmentPointA) is located inside
			// the sphere, we ignore the collision to avoid remaining
			// stuck inside the triangle.
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
					collisionPointV01 = 1.0;
					collisionPointV02 = 0.0;
				}
			}
	

            // check for collision between sphere located at vertex 2.
			// if the starting point (a_segmentPointA) is located inside
			// the sphere, we ignore the collision to avoid remaining
			// stuck inside the triangle.
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
					collisionPointV01 = 0.0;
					collisionPointV02 = 1.0;
				}
			}

            // check for collision between segment and triangle edge01 shell.
			// if the starting point (a_segmentPointA) is located inside
			// the cylinder, we ignore the collision to avoid remaining
			// stuck inside the triangle.
			if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
													a_segmentPointB,
													vertex0,
													vertex1,
													a_settings.m_collisionRadius,
													t_collisionPoint,
													t_collisionNormal,
													t_collisionPointV01,
													t_p,
													t_n,
													t_c) > 0)
			{
				hit = true;
				t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
				if (t_collisionDistanceSq < collisionDistanceSq)
				{
					collisionPoint = t_collisionPoint;
					collisionNormal = t_collisionNormal;
					collisionDistanceSq = t_collisionDistanceSq;
					collisionPointV01 = t_collisionPointV01;
					collisionPointV02 = 0.0;
				}
			}


            // check for collision between segment and triangle edge02 shell.
			// if the starting point (a_segmentPointA) is located inside
			// the cylinder, we ignore the collision to avoid remaining
			// stuck inside the triangle.
			if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
													a_segmentPointB,
													vertex0,
													vertex2,
													a_settings.m_collisionRadius,
													t_collisionPoint,
													t_collisionNormal,
													t_collisionPointV02,
													t_p,
													t_n,
													t_c) > 0)
			{
				hit = true;
				t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
				if (t_collisionDistanceSq < collisionDistanceSq)
				{
					collisionPoint = t_collisionPoint;
					collisionNormal = t_collisionNormal;
					collisionDistanceSq = t_collisionDistanceSq;
					collisionPointV01 = 0.0;
					collisionPointV02 = t_collisionPointV02;
				}
			}


            // check for collision between segment and triangle edge12 shell.
			// if the starting point (a_segmentPointA) is located inside
			// the cylinder, we ignore the collision to avoid remaining
			// stuck inside the triangle.
			if (cIntersectionSegmentToplessCylinder(a_segmentPointA,
													a_segmentPointB,
													vertex1,
													vertex2,
													a_settings.m_collisionRadius,
													t_collisionPoint,
													t_collisionNormal,
													t_collisionPointV12,
													t_p,
													t_n,
													t_c) > 0)
			{
				hit = true;
				t_collisionDistanceSq = cDistanceSq(a_segmentPointA, t_collisionPoint);
				if (t_collisionDistanceSq < collisionDistanceSq)
				{
					collisionPoint = t_collisionPoint;
					collisionNormal = t_collisionNormal;
					collisionDistanceSq = t_collisionDistanceSq;
					collisionPointV01 = 1.0 - t_collisionPointV12;
					collisionPointV02 = t_collisionPointV12;
				}
			}  
        }

        // report collision
        if (hit)
        {
            // before reporting the new collision, we need to check if
            // the collision settings require us to verify the side of the
            // triangle which has been hit.
            bool hit_confirmed = false;

            if (checkFrontSide && checkBackSide)
            {
                // settings specify that a collision can occur on both sides
                // of the triangle, so the new collision is reported.
                hit_confirmed = true;
            }
            else
            {
                // we need check on which side of the triangle the collision occurred
                // and see if it needs to be reported.
                cVector3d segmentAB;
                a_segmentPointB.subr(a_segmentPointA, segmentAB);

                cVector3d v01, v02, triangleNormal;
                vertex2.subr(vertex0, v02);
                vertex1.subr(vertex0, v01);
                v01.crossr(v02, triangleNormal);

                double value = cCosAngle(segmentAB, triangleNormal);
                if (value <= 0.0)
                {
                    if (checkFrontSide)
                        hit_confirmed = true;
                }
                else
                {
                    if (checkBackSide)
                        hit_confirmed = true;
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
                        a_recorder.m_nearestCollision.m_object = a_object;
                        a_recorder.m_nearestCollision.m_triangle = (cTriangle*)this;
                        a_recorder.m_nearestCollision.m_localPos = collisionPoint;
                        a_recorder.m_nearestCollision.m_localNormal = collisionNormal;
                        a_recorder.m_nearestCollision.m_squareDistance = collisionDistanceSq;
                        a_recorder.m_nearestCollision.m_adjustedSegmentAPoint = a_segmentPointA;
                        a_recorder.m_nearestCollision.m_trianglePosV01 = collisionPointV01;
                        a_recorder.m_nearestCollision.m_trianglePosV02 = collisionPointV02;

                        // report advanced collision data
                        if (!a_settings.m_returnMinimalCollisionData)
                        {
                            a_recorder.m_nearestCollision.m_globalPos = cAdd(a_object->getGlobalPos(),
                                                                        cMul(a_object->getGlobalRot(),
                                                                        a_recorder.m_nearestCollision.m_localPos));
                            a_recorder.m_nearestCollision.m_globalNormal = cMul(a_object->getGlobalRot(),
                            a_recorder.m_nearestCollision.m_localNormal);
                        }

                    }
                }
                else
                {
                    cCollisionEvent newCollisionEvent;

                    // report basic collision data
                    newCollisionEvent.m_object = a_object;
                    newCollisionEvent.m_triangle = (cTriangle*)this;
                    newCollisionEvent.m_localPos = collisionPoint;
                    newCollisionEvent.m_localNormal = collisionNormal;
                    newCollisionEvent.m_squareDistance = collisionDistanceSq;
                    newCollisionEvent.m_adjustedSegmentAPoint = a_segmentPointA;
                    newCollisionEvent.m_trianglePosV01 = collisionPointV01;
                    newCollisionEvent.m_trianglePosV02 = collisionPointV02;

                    // report advanced collision data
                    if (!a_settings.m_returnMinimalCollisionData)
                    {
                        newCollisionEvent.m_globalPos = cAdd(a_object->getGlobalPos(),
                                                        cMul(a_object->getGlobalRot(),
                                                        newCollisionEvent.m_localPos));
                        newCollisionEvent.m_globalNormal = cMul(a_object->getGlobalRot(),
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
    inline double computeArea()
    {
        // A = 0.5 * | u x v |
        cVector3d u = cSub(getVertex(1)->getLocalPos(),getVertex(0)->getLocalPos());
        cVector3d v = cSub(getVertex(2)->getLocalPos(),getVertex(0)->getLocalPos());
        return (0.5 * (cCross(u,v).length()));
    }


    //-----------------------------------------------------------------------
    /*!
        Computes the triangle normal and if requested, updates the normal to
        each of the three vertices.

        \param      a_applyNormalToVertices  If true, thene update normal of each vertex.
        \return     Returns the triangle surface normal.
    */
    //-----------------------------------------------------------------------
    cVector3d computeNormal(const bool a_applyNormalToVertices)
    {
        // get pointer to vertices
        cVertex* vertex0 = getVertex0();
        cVertex* vertex1 = getVertex1();
        cVertex* vertex2 = getVertex2();

        // retrieve vertex positions
        cVector3d pos0 = vertex0->getLocalPos();
        cVector3d pos1 = vertex1->getLocalPos();
        cVector3d pos2 = vertex2->getLocalPos();

		// compute normal vector
        cVector3d normal, v01, v02;
        pos1.subr(pos0, v01);
        pos2.subr(pos0, v02);
        v01.crossr(v02, normal);
        double length = normal.length();
		if (length > 0.0)
		{
			normal.div(length);
            if (a_applyNormalToVertices)
            {
			    vertex0->setNormal(normal);
			    vertex1->setNormal(normal);
			    vertex2->setNormal(normal);
            }
            return (normal);
		}

        normal.zero();
        return (normal);
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
    cMesh* m_owner;

    //! Is this triangle still active?
    bool m_allocated;

    //! For custom use. No specific purpose.
    int m_tag;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------


