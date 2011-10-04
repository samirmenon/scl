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
    \author    Dan Morris
    \author    Chris Sewell
    \version   2.0.0 $Rev: 264 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifdef _MSVC
#pragma warning (disable : 4786)
#endif
//---------------------------------------------------------------------------
#ifndef CMeshH
#define CMeshH
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
#include "graphics/CMaterial.h"
#include "graphics/CTexture2D.h"
#include "graphics/CColor.h"
#include <vector>
#include <list>
//---------------------------------------------------------------------------
using std::list;
using std::vector;
//---------------------------------------------------------------------------
class cWorld;
class cTriangle;
class cVertex;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CMesh.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Mesh.
*/
//===========================================================================

//===========================================================================
/*!    
    \class      cMesh
    \ingroup    scenegraph

    \brief      
    cMesh represents a collection of vertices, triangles, materials,
    and texture properties that can be rendered graphically and haptically.
*/
//===========================================================================
class cMesh : public cGenericObject
{

  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cMesh.
    cMesh(cWorld* a_world);

    //! Destructor of cMesh.
    virtual ~cMesh();


    //-----------------------------------------------------------------------
    // METHODS - GENERAL
    //-----------------------------------------------------------------------

    //! Get parent world.
    cWorld* getParentWorld() const { return (m_parentWorld); }

    //! Set parent world.
    void setParentWorld(cWorld* a_world) { m_parentWorld = a_world; }

    //! Load a 3D object file (CHAI currently supports .obj and .3ds files).
    virtual bool loadFromFile(const string& a_fileName);


    //-----------------------------------------------------------------------
    // METHODS - VERTICES
    //-----------------------------------------------------------------------

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const double a_x, const double a_y, const double a_z);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos) { return( newVertex(a_pos.x, a_pos.y, a_pos.z) ); }

    //! Add an array of vertices to the vertex list given an array of vertex positions.
    void addVertices(const cVector3d* a_vertexPositions, const unsigned int& a_numVertices);

    //! Remove the vertex at the specified position in my vertex array.
    bool removeVertex(const unsigned int a_index);

    //! Access the vertex at the specified position in my vertex array (and maybe my childrens' arrays).
    cVertex* getVertex(unsigned int a_index, bool a_includeChildren = false);

	//! Access the vertex at the specified position in my vertex array (and maybe my childrens' arrays).
	inline const cVertex* getVertex(unsigned int a_index, bool a_includeChildren = false) const
    {
        return (const cVertex*)(getVertex(a_index,a_includeChildren));
    }

    //! Read the number of stored vertices, optionally including those of my children.
    unsigned int getNumVertices(bool a_includeChildren = false) const;

    //! Access my vertex list directly (use carefully).
    inline virtual vector<cVertex>* pVertices() { return (&m_vertices); }

    //! Access my vertex list directly (use carefully).
	inline virtual const vector<cVertex>* pVertices() const { return (&m_vertices); }

    //! Access the first non-empty vertex list in any of my children (use carefully).
    virtual vector<cVertex>* pVerticesNonEmpty();


    //-----------------------------------------------------------------------
    // METHODS - TRIANGLES
    //-----------------------------------------------------------------------

    //! Create a new triangle by passing vertex indices.
    unsigned int newTriangle(const unsigned int a_indexVertex0,
                             const unsigned int a_indexVertex1, const unsigned int a_indexVertex2);

    //! Create a new triangle and three new vertices by passing vertex positions.
    unsigned int newTriangle(const cVector3d& a_vertex0, const cVector3d& a_vertex1,
                             const cVector3d& a_vertex2);

    //! Remove a triangle from my triangle array.
    bool removeTriangle(const unsigned int a_index);

    //! Access the triangle at the specified position in my triangle array.
    cTriangle* getTriangle(unsigned int a_index, bool a_includeChildren = false);

    //! Read the number of stored triangles, optionally including those of my children.
    unsigned int getNumTriangles(bool a_includeChildren = false) const;

    //! Clear all triangles and vertices of mesh.
    void clear();

    //! Access my triangle array directly (use carefully).
    inline vector<cTriangle>* pTriangles() { return (&m_triangles); }


    //-----------------------------------------------------------------------
    // METHODS - GRAPHIC RENDERING
    //-----------------------------------------------------------------------

    //! Set the alpha value at each vertex and in all of my material colors.
    virtual void setTransparencyLevel(const float a_level,
                                      const bool a_applyToTextures=false,
                                      const bool a_affectChildren=true);

    //! Set color of each vertex, optionally propagating the operation to my children.
    void setVertexColor(const cColorf& a_color, const bool a_affectChildren=true);

    //! Enable or disable the use of a display list for rendering, optionally propagating the operation to my children.
    void useDisplayList(const bool a_useDisplayList, const bool a_affectChildren=true);

    //! Enable or disable the use vertex arrays for rendering, optionally propagating the operation to my children.
    void useVertexArrays(const bool a_useVertexArrays, const bool a_affectChildren=true);

    //! Ask whether I'm currently rendering with a display list.
    bool getDisplayListEnabled() const { return m_useDisplayList; }

    //! Invalidate any existing display lists.
    void invalidateDisplayList(const bool a_affectChildren=true);

    //! Enable or disable the rendering of vertex normals, optionally propagating the operation to my children.
    void setShowNormals(const bool& a_showNormals, const bool a_affectChildren=true, const bool a_trianglesOnly = false);

    //! Returns whether rendering of normals is enabled.
    bool getShowNormals() const { return m_showNormals; }

    //! Set graphic properties for normal-rendering, optionally propagating the operation to my children.
    void setNormalsProperties(const double a_length, const cColorf& a_color, const bool a_affectChildren);

    //! Are vertex colors currently enabled?
    bool getColorsEnabled() const { return m_useVertexColors; }

    //! Re-initializes textures and display lists.
    virtual void onDisplayReset(const bool a_affectChildren = true);


    //-----------------------------------------------------------------------
    // METHODS - COLLISION DETECTION:
    //-----------------------------------------------------------------------

    //! Set up a brute force collision detector for this mesh and (optionally) for its children.
    virtual void createBruteForceCollisionDetector(bool a_affectChildren, bool a_useNeighbors);

    //! Set up an AABB collision detector for this mesh and (optionally) its children.
    virtual void createAABBCollisionDetector(double a_radius, bool a_affectChildren, bool a_useNeighbors);

    //! Set up a sphere tree collision detector for this mesh and (optionally) its children.
    virtual void createSphereTreeCollisionDetector(double a_radius, bool a_affectChildren, bool a_useNeighbors);

    //! Create a lists for neighbor triangles for each triangle of the mesh.
    void createTriangleNeighborList(bool a_affectChildren);

    //! Search for triangle neighbors.
    void findNeighbors(std::vector<cTriangle*>* search1,
                             std::vector<cTriangle*>* search2, const int& v1, const int& v2);


    //-----------------------------------------------------------------------
    // METHODS - MESH MANIPULATION:
    //-----------------------------------------------------------------------

    //! Compute all triangle normals, optionally propagating the operation to my children.
    void computeAllNormals(const bool a_affectChildren=false);

    //! Extrude each vertex of the mesh by some amount along its normal.
    void extrude(const double a_extrudeDistance, const bool a_affectChildren=false,
      const bool a_updateCollisionDetector=false);

    /*!
        Shifts all vertex positions by the specified amount. \n
        Use setPos() if you want to move the whole mesh for rendering.
    */
    virtual void offsetVertices(const cVector3d& a_offset, 
                                const bool a_affectChildren=false,
                                const bool a_updateCollisionDetector=true);

    //! Scale vertices and normals by the specified scale factors and re-normalize.
    virtual void scaleObject(const cVector3d& a_scaleFactors);

    //! Simple method used to create a new (empty) mesh of my type.
    inline virtual cMesh* createMesh() const { return new cMesh(m_parentWorld); }

    //! Render triangles, material and texture properties.
    virtual void renderMesh(const int a_renderMode=0);

    //! Compute the center of mass of this mesh, based on vertex positions.
    virtual cVector3d getCenterOfMass(const bool a_includeChildren=0);

    //! Reverse all normals on this model.
    virtual void reverseAllNormals(const bool a_affectChildren=0);

    //! Remove redundant triangles from this model.
    virtual void removeRedundantTriangles(const bool a_affectChildren=0);


  protected:

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Render the mesh itself.
    virtual void render(const int a_renderMode=0);

    //! Draw a small line for each vertex normal.
    virtual void renderNormals(const bool a_trianglesOnly=true);

    //! Update the global position of each of my vertices.
    virtual void updateGlobalPositions(const bool a_frameOnly);

    //! Update my boundary box dimensions based on my vertices.
    virtual void updateBoundaryBox();


    //-----------------------------------------------------------------------
    // MEMBERS - DISPLAY PROPERTIES:
    //-----------------------------------------------------------------------

    //! Parent world.
    cWorld *m_parentWorld;

    //! If \b true, then normals are displayed.
    bool m_showNormals;

    //! If \b true, normals are displayed only for vertices that are used in triangles.
    bool m_showNormalsForTriangleVerticesOnly;

    //! Color used to render lines representing normals.
    cColorf m_showNormalsColor;

    //! Length of each normal (for graphic rendering of normals).
    double m_showNormalsLength;

    //! Should we use a display list to render this mesh?
    bool m_useDisplayList;

    //! Should we use vertex arrays to render this mesh?
    bool m_useVertexArrays;

    //! The openGL display list used to draw this mesh, if display lists are enabled.
    int m_displayList;


    //-----------------------------------------------------------------------
    // MEMBERS - ARRAYS:
    //-----------------------------------------------------------------------

    //! Array of vertices.
    vector<cVertex> m_vertices;

    //! List of free slots in the vertex array.
    list<unsigned int> m_freeVertices;

    //! Array of triangles.
    vector<cTriangle> m_triangles;

    //! List of free slots in the triangle array.
    list<unsigned int> m_freeTriangles;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

