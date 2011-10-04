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
#ifndef CCollisionSpheresGeometryH
#define CCollisionSpheresGeometryH
//---------------------------------------------------------------------------
#include "math/CMaths.h"
#include "graphics/CTriangle.h"
#include <assert.h>
#include <list>
#include <map>
#include <math.h>
//---------------------------------------------------------------------------
using std::map;
using std::less;
using std::list;
//---------------------------------------------------------------------------
class cCollisionSpheresPoint;
class cCollisionSpheresEdge;
class cCollisionSpheresLeaf;
//---------------------------------------------------------------------------
//! Map of points to the edges they form.
typedef map<cCollisionSpheresPoint *, cCollisionSpheresEdge *,
        less<cCollisionSpheresPoint *> > PtEmap;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       cCollisionSpheresGeometry.h

    \brief  
    <b> Collision Detection </b> \n
    Spherical Bounding Box Tree - Implementation.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cCollisionSpheresPoint
    \ingroup    collisions

    \brief    
    cCollisionSpheresPoint defines points used in the primitive shapes.
*/
//===========================================================================
class cCollisionSpheresPoint
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCollisionSpheresPoint.
    cCollisionSpheresPoint(double a_x = 0,
                           double a_y = 0,
                           double a_z = 0)
                           { m_pos.x = a_x;  m_pos.y = a_y;  m_pos.z = a_z; }


    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------
    
    //! Position of the point.
    cVector3d m_pos;

    //! Map of edges which have this point as an endpoint.
    PtEmap m_edgeMap;
};


//===========================================================================
/*!
    \class      cCollisionSpheresEdge
    \ingroup    collisions

    \brief    
    cCollisionSpheresEdge defines edges of shape primitives.
*/
//===========================================================================
class cCollisionSpheresEdge
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCollisionSpheresEdge.
    cCollisionSpheresEdge() { }

    //! Constructor of cCollisionSpheresEdge.
    cCollisionSpheresEdge(cCollisionSpheresPoint *a_a,
                          cCollisionSpheresPoint *a_b)
                          { initialize(a_a,a_b); }

    //! Destructor of cCollisionSpheresEdge.
    virtual ~cCollisionSpheresEdge() {}


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------
    
    //! Initialization.
    void initialize(cCollisionSpheresPoint *a_a, cCollisionSpheresPoint *a_b);

    //! Return the center of the edge.
    inline const cVector3d &getCenter() const  {return m_center;}

    //! Return the radius of the edge.
    inline double getRadius() const
        { if (m_D <= 0.0) return 0.0; return sqrt(m_D)/2; }


  private:
    
    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! The two vertices of the edge.
    cCollisionSpheresPoint *m_end[2];

    //! The center of the edge.
    cVector3d m_center;

    //! The distance between the vertices.
    cVector3d m_d;

    //! The 2-norm of the edge.
    double m_D;
};


//===========================================================================
/*!
    \class      cCollisionSpheresGenericShape
    \ingroup    collisions
    
    \brief    
    cCollisionSpheresGenericShape is an abstract class for shape
    primitives (such as triangles or lines) which are surrounded
    by spheres for the collision detector.
*/
//===========================================================================
class cCollisionSpheresGenericShape
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCollisionSpheresGenericShape.
    cCollisionSpheresGenericShape() : m_sphere(NULL) { }

    //! Destructor of cCollisionSpheresGenericShape.
    virtual ~cCollisionSpheresGenericShape() {}


    //-----------------------------------------------------------------------
    // METHODS:
    //----------------------------------------------------------------------

    //! Return center.
    virtual const cVector3d &getCenter() const = 0;

    //! Return radius.
    virtual double getRadius() const = 0;

    //! Determine whether this primitive intersects the given primitive.
    virtual bool computeCollision(cCollisionSpheresGenericShape *a_other,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings) = 0;

    //! Return pointer to bounding sphere of this primitive shape.
    virtual cCollisionSpheresLeaf* getSphere()  { return m_sphere; }

    //! Set pointer for the bounding sphere of this primitive shape.
    virtual void setSphere(cCollisionSpheresLeaf* a_sphere)
        { m_sphere = a_sphere; }

    //! Overloaded less than operator (for sorting).
    bool operator<(cCollisionSpheresGenericShape* a_other)
        { return (getCenter().get(m_split) < a_other->getCenter().get(m_split)); }

    
    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Axis on which to sort.
    static int m_split;


  private:
    
    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Pointer to the collision sphere surrounding the primitive.
    cCollisionSpheresLeaf *m_sphere;
};


//===========================================================================
/*!
    \class      cCollisionSpheresTri
    \ingroup    collisions

    \brief    
    cCollisionSpheresTri defines the triangle primitives that
    make up the mesh and are bounded by the collision spheres.
    It is essentially just a wrapper around a cTriangle object,
    to which it has a pointer (m_original).
*/
//===========================================================================
class cCollisionSpheresTri : public cCollisionSpheresGenericShape
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCollisionSpheresTri.
    cCollisionSpheresTri(cVector3d a,
                         cVector3d b,
                         cVector3d c,
                         double a_extendedRadius);

    //! Destructor of cCollisionSpheresTri.
    virtual ~cCollisionSpheresTri() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------
    
    //! Return whether triangle collides with given line.
    bool computeCollision(cCollisionSpheresGenericShape *a_other,
                          cCollisionRecorder& a_recorder,
                          cCollisionSettings& a_settings);

    //! Return the center of the triangle.
    inline const cVector3d &getCenter() const  { return m_center; }

    //! Return the radius of the triangle.
    inline double getRadius() const  { return m_radius; }

    //! Returns the cTriangle object in the mesh associated with this triangle.
    cTriangle* getOriginal() { return m_original; }

    //! Sets the cTriangle object in the mesh associated with this triangle.
    void setOriginal(cTriangle* a_original) { m_original = a_original; }


  protected:
	
    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! The vertices of the triangle.
    cCollisionSpheresPoint m_corner[3];

    //! The edges of the triangle.
    cCollisionSpheresEdge m_side[3];

    //! The center of the triangle.
    cVector3d m_center;

    //! The radius of the triangle.
    double m_radius;

    //! The cTriangle object in the mesh associated with this triangle.
    cTriangle* m_original;
};


//===========================================================================
/*!
    \class      cCollisionSpheresLine
    \ingroup    collisions

    \brief    
    cCollisionSpheresLine defines a line primitive that may collide with 
    other primitives.  It is used for instance by the proxy algorithm.
*/
//===========================================================================
class cCollisionSpheresLine : public cCollisionSpheresGenericShape
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCollisionSpheresLine.
    cCollisionSpheresLine(cVector3d& a_segmentPointA,
                          cVector3d& a_segmentPointB);

    //! Destructor of cCollisionSpheresLine.
    virtual ~cCollisionSpheresLine() {}


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Return the center of the line.
    inline const cVector3d &getCenter() const  { return m_center; }

    //! Return the radius of the line.
    inline double getRadius() const  { return m_radius; }

    //! Return whether this line intersects the given triangle.
    bool computeCollision(cCollisionSpheresGenericShape *a_other,
                          cCollisionRecorder& a_recorder,
                          cCollisionSettings& a_settings);

    //! Get first endpoint of the line.
    cVector3d getSegmentPointA() { return m_segmentPointA; }

    //! Get direction vector of the line.
    cVector3d getSegmentPointB() { return m_segmentPointB; }


  protected:
  
	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! The center of the line.
    cVector3d m_center;

    //! The radius of the line.
    double m_radius;

    //! The first endpoint of the line.
    cVector3d m_segmentPointA;

    //! The first endpoint of the line.
    cVector3d m_segmentPointB;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
