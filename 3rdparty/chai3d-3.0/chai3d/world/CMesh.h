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
    \author    Francois Conti
    \author    Dan Morris
    \author    Chris Sewell
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1077 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifdef _MSVC
#pragma warning (disable : 4786)
#endif
//------------------------------------------------------------------------------
#ifndef CMeshH
#define CMeshH
//------------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
#include "graphics/CColor.h"
#include "graphics/CEdge.h"
#include <vector>
#include <list>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cWorld;
class cTriangle;
class cVertex;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMesh.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Mesh.
*/
//==============================================================================

//==============================================================================
/*!    
    \class      cMesh
    \ingroup    scenegraph

    \brief
    3D Mesh object

    \details
    cMesh represents a collection of vertices, triangles, materials,
    and texture properties that can be rendered graphically and haptically.
*/
//==============================================================================
class cMesh : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
    
public:

    //! Constructor of cMesh.
    cMesh(cMaterial* a_material = NULL);

    //! Destructor of cMesh.
    virtual ~cMesh();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL
    //--------------------------------------------------------------------------

public:

    //! Create a copy of itself.
    virtual cMesh* copy(const bool a_duplicateMaterialData = false,
                        const bool a_duplicateTextureData = false, 
                        const bool a_duplicateMeshData = false,
                        const bool a_buildCollisionDetector = false);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - VERTICES
    //--------------------------------------------------------------------------

public:

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const double a_x = 0.0, 
                           const double a_y = 0.0, 
                           const double a_z = 0.0);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const double a_x, 
                           const double a_y, 
                           const double a_z,
                           const double a_normalX, 
                           const double a_normalY, 
                           const double a_normalZ);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const double a_x, 
                           const double a_y, 
                           const double a_z,
                           const double a_normalX, 
                           const double a_normalY, 
                           const double a_normalZ,
                           const double a_textureCoordX,
                           const double a_textureCoordY,
                           const double a_textureCoordZ = 0.0);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos, 
                           const cVector3d& a_normal);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos, 
                           const cVector3d& a_normal,
                           const cVector3d& a_textureCoord);

    //! Create a new vertex and add it to the vertex list.
    unsigned int newVertex(const cVector3d& a_pos, 
                           const cVector3d& a_normal,
                           const cVector3d& a_textureCoord,
                           const cColorf& a_color);

    //! Add an array of vertices to the vertex list given an array of vertex positions.
    void addVertices(const cVector3d* a_vertexPositions, 
                     const unsigned int& a_numVertices);

    //! Remove the vertex at the specified position in my vertex array.
    bool removeVertex(const unsigned int a_index);

    //! Access the vertex at the specified position in my vertex array (and maybe my childrens' arrays).
    inline cVertex* getVertex(unsigned int a_index) { return (&m_vertices->at(a_index)); }

    //! Read the number of stored vertices.
    inline unsigned int getNumVertices() const { return (unsigned int)(m_vertices->size()); }

    
    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TRIANGLES
    //--------------------------------------------------------------------------

public:

    //! Create a new triangle by passing vertex indices.
    unsigned int newTriangle(const unsigned int a_indexVertex0,
                             const unsigned int a_indexVertex1, 
                             const unsigned int a_indexVertex2);

    //! Create a new triangle and three new vertices by passing vertex positions.
    unsigned int newTriangle(const cVector3d& a_vertex0, 
                             const cVector3d& a_vertex1,
                             const cVector3d& a_vertex2);

    //! Create a new triangle and three new vertices by passing vertex positions and normals.
    unsigned int newTriangle(const cVector3d& a_vertex0, 
                             const cVector3d& a_vertex1,
                             const cVector3d& a_vertex2,
                             const cVector3d& a_normal0, 
                             const cVector3d& a_normal1,
                             const cVector3d& a_normal2);

    //! Create a new triangle and three new vertices by passing vertex positions, normals and texture coordinates.
    unsigned int newTriangle(const cVector3d& a_vertex0, 
                             const cVector3d& a_vertex1,
                             const cVector3d& a_vertex2,
                             const cVector3d& a_normal0, 
                             const cVector3d& a_normal1,
                             const cVector3d& a_normal2,
                             const cVector3d& a_textureCoord0, 
                             const cVector3d& a_textureCoord1,
                             const cVector3d& a_textureCoord2);

    //! Create a new triangle and three new vertices by passing vertex positions, normals and texture coordinates.
    unsigned int newTriangle(const cVector3d& a_vertex0, 
                             const cVector3d& a_vertex1,
                             const cVector3d& a_vertex2,
                             const cVector3d& a_normal0, 
                             const cVector3d& a_normal1,
                             const cVector3d& a_normal2,
                             const cVector3d& a_textureCoord0, 
                             const cVector3d& a_textureCoord1,
                             const cVector3d& a_textureCoord2,
                             const cColorf& a_colorVertex0,
                             const cColorf& a_colorVertex1,
                             const cColorf& a_colorVertex2);

    //! Remove a triangle from my triangle array.
    bool removeTriangle(const unsigned int a_index);

    //! Access the triangle at the specified position in my triangle array.
    cTriangle* getTriangle(unsigned int a_index);	

    //! Read the number of stored triangles.
    unsigned int getNumTriangles();

    //! Clear all triangles and vertices of mesh.
    void clear();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - EDGES
    //--------------------------------------------------------------------------

public:

    //! Create a list of edges by providing a threshold angle in degrees.
    void computeAllEdges(double a_angleThresholdDeg = 40.0);

    //! Clear all edges
    void clearAllEdges();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRAPHIC RENDERING
    //--------------------------------------------------------------------------

public:

    //! Invalidate any existing display lists, optionally propagating the operation to my children.
    virtual void invalidateDisplayList(const bool a_affectChildren = false);

    //! Set the alpha value at each vertex and in all of my material colors.
    virtual void setTransparencyLevel(const float a_level,
                                      const bool a_applyToTextures = false,
                                      const bool a_affectChildren = false);

    //! Set color of each vertex.
    void setVertexColor(const cColorf& a_color);

    //! Enable or disable the use vertex arrays for rendering.
    void setUseVertexArrays(const bool a_useVertexArrays);

    //! Ask whether I'm currently rendering with vertex arrays.
    bool getUseVertexArrays() const { return (m_useVertexArrays); }

    //! Enable or disable the rendering of vertex normals.
    void setShowNormals(const bool a_showNormals) { m_showNormals = a_showNormals; }

    //! Returns whether rendering of normals is enabled.
    bool getShowNormals() const { return (m_showNormals); }

    //! Set graphic properties for normal-rendering.
    void setNormalsProperties(const double a_length, 
                              const cColorf& a_color);

    //! Enable or disable the rendering of edges.
    void setShowEdges(const bool a_showEdges) { m_showEdges = a_showEdges; }

    //! Returns whether rendering of edges is enabled.
    bool getShowEdges() const { return (m_showEdges); }

    //! Set graphic properties for edge-rendering.
    void setEdgeProperties(const double a_width, 
                           const cColorf& a_color);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION DETECTION:
    //--------------------------------------------------------------------------

public:

    //! Set up a brute force collision detector for this mesh.
    virtual void createBruteForceCollisionDetector();

    //! Set up an AABB collision detector for this mesh.
    virtual void createAABBCollisionDetector(const double a_radius);

    //! Update the relationship between the tool and the current object.
    void computeLocalInteraction(const cVector3d& a_toolPos,
                                 const cVector3d& a_toolVel,
                                 const unsigned int a_IDN);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MESH MANIPULATION:
    //--------------------------------------------------------------------------

public:

    //! Scale this object by using different scale factors along X,Y and Z axes.
    void scaleXYZ(const double a_scaleX, const double a_scaleY, const double a_scaleZ);

    //! Compute all triangle normals, optionally propagating the operation to my children.
    void computeAllNormals();

    //! Reverse all normals on this model.
    virtual void reverseAllNormals();

    //! Shifts all vertex positions by the specified amount.
    virtual void offsetVertices(const cVector3d& a_offset, 
                                const bool a_updateCollisionDetector = true);

    //! Compute the center of mass of this mesh, based on vertex positions.
    virtual cVector3d getCenterOfMass();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - INTERNAL
    //--------------------------------------------------------------------------

protected:

    //! Render the mesh itself.
    virtual void render(cRenderOptions& a_options);

    //! Draw a small line for each vertex normal.
    virtual void renderNormals();

    //! Draw all edges of mesh.
    virtual void renderEdges();

    //! Render triangles, material and texture properties.
    virtual void renderMesh(cRenderOptions& a_options);

    //! Update the global position of each of my vertices.
    virtual void updateGlobalPositions(const bool a_frameOnly);

    //! Update my boundary box dimensions based on my vertices.
    virtual void updateBoundaryBox();

    //! Scale vertices and normals by the specified scale factors and re-normalize.
    virtual void scaleObject(const double& a_scaleFactor);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - DISPLAY PROPERTIES:
    //--------------------------------------------------------------------------

protected:

    //! If __true__, then normals are displayed.
    bool m_showNormals;

    //! Length of each normal (for graphic rendering of normals).
    double m_normalsLength;

    //! If __true__, then show edges.
    bool m_showEdges;

    //! Width of edge lines.
    double m_edgeLineWidth;

    //! Should we use vertex arrays to render this mesh?
    bool m_useVertexArrays;

    //! Display list for edges.
    cDisplayList m_displayListEdges;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - DISPLAY PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! Color used to render lines representing normals.
    cColorf m_normalsColor;

    //! Color used to render lines representing edges.
    cColorf m_edgeLineColor;


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - TRIANGLE AND VERTEX DATA:
    //--------------------------------------------------------------------------

public:

    //! Array of vertices.
    std::vector<cVertex> *m_vertices;

    //! Array of triangles.
    std::vector<cTriangle> *m_triangles;

    //! Edges
    std::vector<cEdge> *m_edges;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - TRIANGLE AND VERTEX DATA:
    //--------------------------------------------------------------------------

protected:

    //! List of free slots in the vertex array.
    std::list<unsigned int> *m_freeVertices;

    //! List of free slots in the triangle array.
    std::list<unsigned int> *m_freeTriangles;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

