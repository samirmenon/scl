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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 676 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CMultiMeshH
#define CMultiMeshH
//---------------------------------------------------------------------------
#include "world/CMesh.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CMultiMesh.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual MultiMesh.
*/
//===========================================================================

//===========================================================================
/*!    
    \class      cMultiMesh
    \ingroup    scenegraph

    \brief      
    cMultiMesh represents a collection of cMesh objects. Each cMesh object 
	includes one material and texture properties with a set of vertices 
	and triangles. cMultiMesh allows the user to build complex mesh objects 
	composed of different materials and sets of triangles.
*/
//===========================================================================
class cMultiMesh : public cGenericObject
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    public:

    //! Constructor of cMultiMesh.
    cMultiMesh();

    //! Destructor of cMultiMesh.
    virtual ~cMultiMesh();


    //-----------------------------------------------------------------------
    // DISABLE / ENABLE:
    //-----------------------------------------------------------------------

	//! Enable or Disable an object. When an object is disable, haptic and graphic rendering no longer occur.
	virtual void setEnabled(bool a_enabled,
							const bool a_affectChildren = false);

	//! Read status of object.
	bool getEnabled() { return (m_enabled); }


	//-----------------------------------------------------------------------
    // METHODS - COPY:
	//-----------------------------------------------------------------------

    //! Create a copy of itself.
    virtual cMultiMesh* copy(const bool a_duplicateMaterialData = false,
                             const bool a_duplicateTextureData = false, 
                             const bool a_duplicateMeshData = false,
                             const bool a_buildCollisionDetector = true);


	//-----------------------------------------------------------------------
    // METHODS - HAPTIC PROPERTIES:
	//-----------------------------------------------------------------------

    //! Allow this object to be felt (when visible), optionally propagating the change to children.
    virtual void setHapticEnabled(const bool a_hapticEnabled, 
								  const bool a_affectChildren = false);

    //! Set the haptic stiffness, possibly recursively affecting children.
    virtual void setStiffness(const double a_stiffness, 
							  const bool a_affectChildren = false);

    //! Set the static and dynamic friction for this mesh, possibly recursively affecting children.
    virtual void setFriction(double a_staticFriction, 
							 double a_dynamicFriction, 
							 const bool a_affectChildren = false);


	//-----------------------------------------------------------------------
    // METHODS - GRAPHIC PROPERTIES:
	//-----------------------------------------------------------------------

    //! Show or hide this object, optionally propagating the change to children.
    virtual void setShowEnabled(const bool a_show, 
								const bool a_affectChildren = false);

    //! Read the display status of object (true means it's visible).
    inline bool getShowEnabled() const { return (m_showEnabled); }

    //! Enable or disable transparency.
    virtual void setUseTransparency(const bool a_useTransparency, 
									const bool a_affectChildren = true);

    //! Is transparency enabled for this mesh?
    inline bool getUseTransparency() const { return m_useTransparency; }

    //! Set the alpha value at each vertex and in all of my material colors.
    virtual void setTransparencyLevel(const float a_level,
                                      const bool a_applyToTextures = false,
                                      const bool a_affectChildren = true);

    //! Enable or disable wireframe rendering, optionally propagating the operation to my children.
    virtual void setWireMode(const bool a_showWireMode, 
							 const bool a_affectChildren = true);

    //! Return whether wireframe rendering is enabled.
    inline bool getWireMode() const { return (m_triangleMode == GL_LINE); }

    //! Enable or disabling face-culling, optionally propagating the operation to my children.
    virtual void setUseCulling(const bool a_useCulling, 
							   const bool a_affectChildren=true);

    //! Is face-culling currently enabled?
    inline bool getUseCulling() const { return (m_cullingEnabled); }

    //! Enable or disable the use of per-vertex colors, optionally propagating the operation to my children.
    virtual void setUseVertexColors(const bool a_useColors, 
									const bool a_affectChildren=true);

	//! Are per-vertex properties currently enabled?
	inline bool getUseVertexColors() const { return (m_useVertexColors); }


    //! Backup color properties of object, optionally propagating the operation to my children.
    virtual void backupColors(const bool a_material_properties_only = false, 
							  const bool a_affectChildren = false);

    //! Restore color properties of object, optionally propagating the operation to my children.
    virtual void restoreColors(const bool a_material_properties_only = false, 
							   const bool a_affectChildren = false);


    //! Enable or disable the use of a display list for rendering, optionally propagating the operation to my children.
    virtual void setUseDisplayList(const bool a_useDisplayList,
                                   const bool a_affectChildren = false);


    //! Invalidate any existing display lists, optionally propagating the operation to my children.
    virtual void invalidateDisplayList(const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // METHODS - MATERIAL:
	//-----------------------------------------------------------------------

    //! Enable or disable the use of material properties, optionally propagating the operation to my children.
    virtual void setUseMaterial(const bool a_useMaterial, 
								const bool a_affectChildren = true);

    //! Are material properties currently enabled?
    inline bool getUseMaterial() const { return (m_useMaterialProperty); }


    //! Set material to this object, possibly recursively affecting children.
    virtual void setMaterial(cMaterial* a_mat, 
							 const bool a_affectChildren = false);

    //! Set material to this object, possibly recursively affecting children.
    virtual void setMaterial(cMaterial& a_mat, 
							 const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // METHODS - TEXTURE:
	//-----------------------------------------------------------------------

    //! Enable or disable the use of texture-mapping, optionally propagating the operation to my children.
    virtual void setUseTexture(const bool a_useTexture, 
							   const bool a_affectChildren = true);

    //! Is texture-mapping enabled?
    inline bool getUseTexture() const { return (m_useTextureMapping); }

    //! Set texture to this object, possibly recursively affecting children.
    virtual void setTexture(cTexture1d* a_texture, 
							const bool a_affectChildren = false);


	//-----------------------------------------------------------------------
    // METHODS - BOUNDARY BOX:
	//-----------------------------------------------------------------------

    //! Show or hide the boundary box for this object, optionally propagating the change to children.
    virtual void setShowBoundaryBox(const bool a_showBoundaryBox, 
									const bool a_affectChildren = false);

    //! Read the display status of boundary box. (true means it's visible).
    inline bool getShowBoundaryBox() const { return (m_showBoundaryBox); }

    //! Read the minimum point of this object's boundary box.
    inline cVector3d getBoundaryMin() const { return (m_boundaryBoxMin); }

    //! Read the maximum point of this object's boundary box.
    inline cVector3d getBoundaryMax() const { return (m_boundaryBoxMax); }

    //! Compute the center of this object's boundary box.
    inline cVector3d getBoundaryCenter() const { return (m_boundaryBoxMax + m_boundaryBoxMin)/2.0; }

    //! Re-compute this object's bounding box, optionally forcing it to bound child objects.
    virtual void computeBoundaryBox(const bool a_includeChildren = true);


	//-----------------------------------------------------------------------
    // METHODS - COLLISION DETECTION:
	//-----------------------------------------------------------------------

    //! Delete any existing collision detector and set the current cd to null (no collisions).
    virtual void deleteCollisionDetector(const bool a_affectChildren = false);

    //! Compute collision detection using collision detectors.
    virtual bool computeCollisionDetection(cVector3d& a_segmentPointA,
										   cVector3d& a_segmentPointB,
										   cCollisionRecorder& a_recorder,
										   cCollisionSettings& a_settings);

    //! Show or hide the collision tree for this object, optionally propagating the change to children.
    virtual void setShowCollisionDetector(const bool a_showCollisionDetector, 
										  const bool a_affectChildren = false);

     //! Set collision rendering properties.
    virtual void setCollisionDetectorProperties(unsigned int a_displayDepth, 
												cColorf& a_color, 
												const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // MEMBERS - MESH PRIMITIVES:
    //-----------------------------------------------------------------------

    //! Array of meshes.
    vector<cMesh*> *m_meshes;

	//! Create a new mesh primitive.
	cMesh* newMesh();

	//! Add an existing mesh primitives to list of meshes
	bool addMesh(cMesh* a_mesh);

	//! Remove a mesh primitive.
	bool removeMesh(cMesh* a_mesh);

	//! Remove all mesh primitive.
	bool removeAllMesh();

	//! Delete a mesh primitive.
	bool deleteMesh(cMesh* a_mesh);

	//! Delete all meshes
	bool deleteAllMeshes();

	//! Retrieve the number of mesh primitives composing the multimesh.
	int getNumMeshes();

	//! Get access to an individual mesh primitive by passing its index number.
	cMesh* getMesh(unsigned int a_index);


    //-----------------------------------------------------------------------
    // METHODS - VERTICES
    //-----------------------------------------------------------------------
	
    //! Access the vertex at the specified position in my vertex array (and maybe my childrens' arrays).
	cVertex* getVertex(unsigned int a_index);

    //! Read the number of stored vertices, optionally including those of my children.
	unsigned int getNumVertices() const;

  
    //-----------------------------------------------------------------------
    // METHODS - TRIANGLES
    //-----------------------------------------------------------------------

    //! Access the triangle at the specified position in my triangle array.
    cTriangle* getTriangle(unsigned int a_index);

    //! Read the number of stored triangles, optionally including those of my children.
    unsigned int getNumTriangles() const;

    //! Clear all triangles and vertices of mesh.
    void clear();

   
    //-----------------------------------------------------------------------
    // METHODS - GRAPHIC MESH OPTIONS
    //-----------------------------------------------------------------------

    //! Set color of each vertex, optionally propagating the operation to my children.
    void setVertexColor(const cColorf& a_color);

    //! Enable or disable the use vertex arrays for rendering, optionally propagating the operation to my children.
    void setUseVertexArrays(const bool a_useVertexArrays);

    //! Enable or disable the rendering of vertex normals, optionally propagating the operation to my children.
    void setShowNormals(const bool& a_showNormals);

    //! Set graphic properties for normal-rendering, optionally propagating the operation to my children.
    void setNormalsProperties(const double a_length, 
                              const cColorf& a_color);

    //! Enable or disable the rendering of edges.
	void setShowEdges(const bool a_showEdges);

    //! Set graphic properties for edge-rendering.
    void setEdgeProperties(const double a_width, 
                           const cColorf& a_color);

    //! Create a list of edges by providing a threshold angle in degrees.
    void computeAllEdges(double a_angleThresholdDEG = 40.0);

    //! Clear all edges
    void clearAllEdges();


    //-----------------------------------------------------------------------
    // METHODS - COLLISION DETECTION:
    //-----------------------------------------------------------------------

    //! Set up a brute force collision detector for this mesh and (optionally) for its children.
    virtual void createBruteForceCollisionDetector();

    //! Set up an AABB collision detector for this mesh.
    virtual void createAABBCollisionDetector(const double a_radius);


    //-----------------------------------------------------------------------
    // METHODS - COMMANDS:
    //-----------------------------------------------------------------------

    //! Load a 3D object file.
    virtual bool loadFromFile(string a_filename);

    //! Scale this object by a_scaleFactor (uniform scale).
    virtual void scale(const double& a_scaleFactor, 
                       const bool a_affectChildren = true);

    //! Compute all triangle normals, optionally propagating the operation to my children.
    void computeAllNormals();

    //! Reverse all normals on this model.
    virtual void reverseAllNormals();


  protected:

    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Render the mesh itself.
    virtual void render(cRenderOptions& a_options);

    //! Update the global position of each of my meshes.
    virtual void updateGlobalPositions(const bool a_frameOnly);


	//-----------------------------------------------------------------------
    // MEMBERS - GENERAL
	//-----------------------------------------------------------------------

  public:

	//! Computer haptic interaction.
    virtual cVector3d computeInteractions(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN,
                                          cInteractionRecorder& a_interactions);

};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

