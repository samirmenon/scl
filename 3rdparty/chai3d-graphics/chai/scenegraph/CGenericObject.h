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
    \version   2.0.0 $Rev: 270 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericObjectH
#define CGenericObjectH
//---------------------------------------------------------------------------
#include "math/CMaths.h"
#include "graphics/CDraw3D.h"
#include "graphics/CColor.h"
#include "graphics/CMacrosGL.h"
#include "graphics/CMaterial.h"
#include "graphics/CTexture2D.h"
#include "collisions/CCollisionBasics.h"
#include "forces/CInteractionBasics.h"
#include "effects/CGenericEffect.h"
#include "extras/CGenericType.h"
//---------------------------------------------------------------------------
#include <typeinfo>
#include <vector>
#include <list>
//---------------------------------------------------------------------------
using std::vector;
//---------------------------------------------------------------------------
class cTriangle;
class cGenericCollision;
class cGenericPointForceAlgo;
class cMesh;
//---------------------------------------------------------------------------
// TYPE DEFINITION
//---------------------------------------------------------------------------
//! Constants that define specific rendering passes (see cCamera.cpp)
typedef enum {
  CHAI_RENDER_MODE_RENDER_ALL = 0,
  CHAI_RENDER_MODE_NON_TRANSPARENT_ONLY,
  CHAI_RENDER_MODE_TRANSPARENT_BACK_ONLY,
  CHAI_RENDER_MODE_TRANSPARENT_FRONT_ONLY
} chai_render_modes;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGenericObject.h

    \brief 
    <b> Scenegraph </b> \n 
    Base Class.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGenericObject
    \ingroup    scenegraph

    \brief      
    This class is the root of basically every render-able object in CHAI.  
    It defines a reference frame (position and rotation) and virtual methods 
    for rendering, which are overloaded by useful subclasses. \n
 
    This class also defines basic methods for maintaining a scene graph, 
    and propagating rendering passes and reference frame changes through 
    a hierarchy of cGenericObjects. \n

    Besides subclassing, a useful way to extend cGenericObject is to store 
    custom data in the m_tag and m_userData member fields, which are not 
    used by CHAI. \n

    The most important methods to look at here are probably the virtual 
    methods, which are listed last in CGenericObject.h. These methods 
    will be called on each cGenericObject as operations propagate through 
    the scene graph.
*/
//===========================================================================
class cGenericObject : public cGenericType
{

  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGenericObject.
    cGenericObject();

    //! Destructor of cGenericObject.
    virtual ~cGenericObject();


	//-----------------------------------------------------------------------
    // METHODS - TRANSLATION AND ORIENTATION:
	//-----------------------------------------------------------------------

    //! Set the local position of this object.
    void setPos(const cVector3d& a_pos)
    {
        m_localPos = a_pos;
    }

    //! Set the local position of this object.
    void setPos(const double a_x, const double a_y, const double a_z)
    {
        m_localPos.set(a_x, a_y, a_z);
    }

    //! Get the local position of this object.
    inline cVector3d getPos() const { return (m_localPos); }

    //! Get the global position of this object.
    inline cVector3d getGlobalPos() const { return (m_globalPos); }

    //! Set the local rotation matrix for this object.
    inline void setRot(const cMatrix3d& a_rot)
    {
        m_localRot = a_rot;
    }

    //! Get the local rotation matrix of this object.
    inline cMatrix3d getRot() const { return (m_localRot); }

    //! Get the global rotation matrix of this object.
    inline cMatrix3d getGlobalRot() const { return (m_globalRot); }

    //! Translate this object by a specified offset.
    void translate(const cVector3d& a_translation);

    //! Translate this object by a specified offset.
    void translate(const double a_x, const double a_y, const double a_z);

    //! Rotate this object by multiplying with a specified rotation matrix.
    void rotate(const cMatrix3d& a_rotation);

    //! Rotate this object around axis a_axis by angle a_angle (radians).
    void rotate(const cVector3d& a_axis, const double a_angle);


	//-----------------------------------------------------------------------
    // METHODS - GLOBAL / LOCAL POSITIONS:
	//-----------------------------------------------------------------------

    //! Compute the global position and rotation of this object and its children.
    void computeGlobalPositions(const bool a_frameOnly = true,
                                const cVector3d& a_globalPos = cVector3d(0.0, 0.0, 0.0),
                                const cMatrix3d& a_globalRot = cIdentity3d());

    //! Compute the global position and rotation of current object only.
    void computeGlobalCurrentObjectOnly(const bool a_frameOnly = true);

    //! Compute the global position and rotation with relative motion of this object and its children.
    void computeGlobalPositionsAndMotion(const bool a_frameOnly = true,
                                    const cVector3d& a_globalPos = cVector3d(0.0, 0.0, 0.0),
                                    const cMatrix3d& a_globalRot = cIdentity3d());


	//-----------------------------------------------------------------------
    // METHODS - INTERACTIONS, FORCES AND EFFECTS:
	//-----------------------------------------------------------------------

    // Descend through child objects to compute interactions for all cGenericEffects.
    cVector3d computeInteractions(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int a_IDN,
                                  cInteractionRecorder& a_interactions,
                                  cInteractionSettings& a_interactionSettings);

    //! Projection of the latest interaction point with the surface (limits) of the current object.
    cVector3d m_interactionProjectedPoint;

    //! Was the last interaction point located inside the object?
    bool m_interactionInside;

    //! list of haptic effects.
    vector<cGenericEffect*> m_effects;

    //! add an effect.
    void addEffect(cGenericEffect* a_newEffect);


	//-----------------------------------------------------------------------
    // METHODS - HAPTIC PROPERTIES:
	//-----------------------------------------------------------------------
    //! Set the haptic stiffness, possibly recursively affecting children.
    void setStiffness(double a_stiffness, const bool a_affectChildren=0);

    //! Set the static and dynamic friction for this mesh, possibly recursively affecting children.
    void setFriction(double a_staticFriction, double a_dynamicFriction, const bool a_affectChildren=0);


	//-----------------------------------------------------------------------
    // METHODS - GRAPHICS:
	//-----------------------------------------------------------------------

    //! Material properties.
    cMaterial m_material;

    //! Texture property.
    cTexture2D* m_texture;

    //! Show or hide this object, optionally propagating the change to children.
    void setShowEnabled(const bool a_show, const bool a_affectChildren = false);

    //! Read the display status of object (true means it's visible).
    bool getShowEnabled() const { return (m_show); }

    //! Allow this object to be felt (when visible), optionally propagating the change to children.
    void setHapticEnabled(const bool a_hapticEnabled, const bool a_affectChildren = false);

    //! Read the haptic status of object (true means it can be felt when visible).
    bool getHapticEnabled() const { return (m_hapticEnabled); }

    //! Show or hide the child/parent tree, optionally propagating the change to children.
    void setShowTree(const bool a_showTree, const bool a_affectChildren = false);

    //! Read the display status of the tree (true means it's visible).
    bool getShowTree() const { return (m_showTree); }

    //! Set the tree color, optionally propagating the change to children.
    void setTreeColor(const cColorf& a_treeColor, const bool a_affectChildren  = false);

    //! Read the tree color.
    cColorf getTreeColor() const { return (m_treeColor); }

    //! Show or hide the reference frame arrows for this object, optionally propagating the change to children.
    void setShowFrame(const bool a_showFrame, const bool a_affectChildren  = false);

    //! Read the display status of the reference frame (true means it's visible).
    bool getShowFrame(void) const { return (m_showFrame); }

    //! Show or hide the boundary box for this object, optionally propagating the change to children.
    void setShowBox(const bool iShowBox, const bool iAffectChildren = false);

    //! Read the display status of boundary box. (true means it's visible).
    bool getShowBox() const { return (m_showBox); }

    //! Set the color of boundary box for this object, optionally propagating the change to children.
    void setBoxColor(const cColorf& a_boxColor, const bool a_affectChildren = false);

    //! Read the color of boundary box.
    cColorf getBoxColor() const { return (m_boundaryBoxColor); }

    //! Show or hide the collision tree for this object, optionally propagating the change to children.
    void setShowCollisionTree(const bool a_showCollisionTree, const bool a_affectChildren = false);

    //! Read the display status of of the collision tree for this object.
    bool getShowCollisionTree() { return (m_showCollisionTree); }

    //! This function should get called when it's necessary to re-initialize the OpenGL context.
    virtual void onDisplayReset(const bool a_affectChildren = true);

    //! This function tells children that you're not going to change their contents any more.
    virtual void finalize(const bool a_affectChildren = true);

    //! This function tells objects that you may modify their contents.
    virtual void unfinalize(const bool a_affectChildren = true);

    //! Render the entire scene graph, starting from this object.
    virtual void renderSceneGraph(const int a_renderMode=CHAI_RENDER_MODE_RENDER_ALL);


    //-----------------------------------------------------------------------
    // METHODS - GRAPHIC RENDERING:
    //-----------------------------------------------------------------------

    //! Set the material for this mesh, and optionally pass it on to my children.
    void setMaterial(cMaterial& a_mat, const bool a_affectChildren=false, const bool a_applyPhysicalParmetersOnly=false);

    //! Set the alpha value at each vertex and in all of my material colors.
    virtual void setTransparencyLevel(const float a_level,
                                      const bool a_applyToTextures=false,
                                      const bool a_affectChildren=true);

    //! Specify whether this mesh should use multipass transparency (see cCamera).
    void setTransparencyRenderMode(const bool a_useMultipassTransparency, const bool a_affectChildren=true);

    //! Is multipass transparency used for this mesh?
    bool getMultipassTransparencyEnabled() const { return m_useMultipassTransparency; }

    //! Enable or disable transparency (also see setTransparencyRenderMode)... turns the depth mask _off_!
    void setUseTransparency(const bool a_useTransparency, const bool a_affectChildren=true);

    //! Is transparency enabled for this mesh?
    bool getUseTransparency() const { return m_useTransparency; }

    //! Enable or disable wireframe rendering, optionally propagating the operation to my children.
    void setWireMode(const bool a_showWireMode, const bool a_affectChildren=true);

    //! Return whether wireframe rendering is enabled.
    bool getWireMode() const { return m_triangleMode == GL_LINE; }

    //! Enable or disabling face-culling, optionally propagating the operation to my children.
    void setUseCulling(const bool a_useCulling, const bool a_affectChildren=true);

    //! Is face-culling currently enabled?
    bool getUseCulling() const { return m_cullingEnabled; }

    //! Enable or disable the use of per-vertex colors, optionally propagating the operation to my children.
    void setUseVertexColors(const bool a_useColors, const bool a_affectChildren=true);

	//! Are per-vertex properties currently enabled?
	bool getUseVertexColors() const { return m_useVertexColors; }

    //! Enable or disable the use of material properties, optionally propagating the operation to my children.
    void setUseMaterial(const bool a_useMaterial, const bool a_affectChildren=true);

    //! Are material properties currently enabled?
    bool getUseMaterial() const { return m_useMaterialProperty; }

    //! Enable or disable the use of texture-mapping, optionally propagating the operation to my children.
    void setUseTexture(const bool a_useTexture, const bool a_affectChildren=true);

    //! Is texture-mapping enabled?
    bool getUseTexture() const { return m_useTextureMapping; }

    //! Set my texture, possibly recursively affecting children.
    void setTexture(cTexture2D* a_texture, const bool a_affectChildren=0);

    //! Access my texture.
    cTexture2D* getTexture() const { return(m_texture); }


	//-----------------------------------------------------------------------
    // METHODS - BOUNDARY BOX:
	//-----------------------------------------------------------------------

    //! Read the minimum point of this object's boundary box.
    cVector3d getBoundaryMin() const { return (m_boundaryBoxMin); }

    //! Read the maximum point of this object's boundary box.
    cVector3d getBoundaryMax() const { return (m_boundaryBoxMax); }

    //! Compute the center of this object's boundary box.
    cVector3d getBoundaryCenter() const { return (m_boundaryBoxMax+m_boundaryBoxMin)/2.0; }

    //! Re-compute this object's bounding box, optionally forcing it to bound child objects.
    void computeBoundaryBox(const bool a_includeChildren=true);


	//-----------------------------------------------------------------------
	// METHODS - REFERENCE FRAME REPRESENTATION:
	//-----------------------------------------------------------------------

    //! Set the size of the rendered reference frame, optionally propagating the change to children.
    bool setFrameSize(const double a_size=1.0, const double a_thickness=1.0, const bool a_affectChildren = false);

    //! Read the size of the rendered reference frame.
    double getFrameSize() const { return (m_frameSize); }


	//-----------------------------------------------------------------------
    // METHODS - COLLISION DETECTION:
	//-----------------------------------------------------------------------

    //! Set a collision detector for current object.
    void setCollisionDetector(cGenericCollision* a_collisionDetector)
         { m_collisionDetector = a_collisionDetector; }

    //! Get pointer to this object's current collision detector.
    inline cGenericCollision* getCollisionDetector() const { return (m_collisionDetector); }

    //! Set collision rendering properties.
    void setCollisionDetectorProperties(unsigned int a_displayDepth, cColorf& a_color, const bool a_affectChildren = false);

    //! Delete any existing collision detector and set the current cd to null (no collisions).
    void deleteCollisionDetector(const bool a_affectChildren = false);

    //! Compute collision detection using collision trees
    bool computeCollisionDetection(cVector3d& a_segmentPointA,
                                   cVector3d& a_segmentPointB,
                                   cCollisionRecorder& a_recorder,
                                   cCollisionSettings& a_settings);

    //! Adjust collision segment for dynamic objects.
    virtual void adjustCollisionSegment(cVector3d& a_segmentPointA,
                                        cVector3d& a_segmentPointAadjusted);


	//-----------------------------------------------------------------------
    // METHODS - SCENE GRAPH:
	//-----------------------------------------------------------------------

	//! Set parent of current object.
    void setParent(cGenericObject* a_parent) { m_parent = a_parent; }

    //! Read parent of current object.
    cGenericObject* getParent() const { return (m_parent); }

    //! Read an object from my list of children.
    inline cGenericObject* getChild(const unsigned int a_index) const { return (m_children[a_index]); }

    //! Add an object to my list of children.
    void addChild(cGenericObject* a_object);

    //! Remove an object from my list of children, without deleting it.
    bool removeChild(cGenericObject* a_object);

    //! Does this object have the specified object as a child?
    bool containsChild(cGenericObject* a_object, bool a_includeChildren=false);

    //! Remove an object from my list of children and delete it.
    bool deleteChild(cGenericObject *a_object);

    //! Clear all objects from my list of children, without deleting them.
    void clearAllChildren();

    //! Clear and delete all objects from my list of children.
    void deleteAllChildren();

    //! Return the number of children on my list of children.
    inline unsigned int getNumChildren() { return ((unsigned int)m_children.size()); }

    //! Return my total number of descendants, optionally including this object.
    unsigned int getNumDescendants(bool a_includeCurrentObject=false);

    //! Fill this list with all of my descendants.
    void enumerateChildren(std::list<cGenericObject*>& a_childList, bool a_includeCurrentObject=true);

    //! Remove me from my parent's CHAI scene graph.
    inline bool removeFromGraph()
    {
        if (m_parent) return m_parent->removeChild(this);
        else return false;
    }

	//! Makes this object a ghost node.
	void setAsGhost(bool a_ghostStatus);

	//! Read the ghost status of this object.
	bool getAsGhost() { return (m_ghostStatus); }


	//-----------------------------------------------------------------------
    // METHODS - LINKS TO EXTERNALS:
	//-----------------------------------------------------------------------

    //! Sets a pointer to an external parent of the current object. Optionally propagating the change to children.
    void setExternalParent(cGenericType* a_externalParent, const bool a_affectChildren = false);

    //! Get the external parent of current object.
    inline cGenericType* getExternalParent() { return (m_externalParent); }

    //! Sets the super parent of the current object. Optionally propagating the change to children.
    void setSuperParent(cGenericObject* a_superParent, const bool a_affectChildren = false);

    //! Get the super parent of current object.
    inline cGenericObject* getSuperParent() { return (m_superParent); }


	//-----------------------------------------------------------------------
    // METHODS - SCALING:
	//-----------------------------------------------------------------------

    //! Scale this object by a_scaleFactor (uniform scale).
    void scale(const double& a_scaleFactor, const bool a_includeChildren = true);

    //! Non-uniform scale.
    void scale(const cVector3d& a_scaleFactors, const bool a_includeChildren = true);


	//-----------------------------------------------------------------------
    // MEMBERS - CUSTOM USER DATA:
	//-----------------------------------------------------------------------

    //! An arbitrary tag, not used by CHAI.
    int m_tag;

    //! Set the tag for this object and - optionally - for my children.
    virtual void setTag(const int a_tag, const bool a_affectChildren=0);

    //! An arbitrary data pointer, not used by CHAI.
    void* m_userData;

    //! Set the m_userData pointer for this object and - optionally - for my children.
    virtual void setUserData(void* a_data, const bool a_affectChildren=0);

    //! A name for this object, automatically assigned by mesh loaders (for example).
    char m_objectName[CHAI_SIZE_NAME];

    //! Set the name for this object and - optionally - for my children.
    virtual void setName(const char* a_name, const bool a_affectChildren=0);


  protected:

	//-----------------------------------------------------------------------
    // MEMBERS - SCENE GRAPH:
	//-----------------------------------------------------------------------

    //! Parent object.
    cGenericObject* m_parent;

    //! My list of children.
    vector<cGenericObject*> m_children;

	//! Ghost status of current object.
	bool m_ghostStatus;


	//-----------------------------------------------------------------------
    // MEMBERS - LINKS TO EXTERNALS:
	//-----------------------------------------------------------------------

    /*!
        A pointer to an external parent located outside of the scenegraph.
        This parameter can typically be used if you want to attach an
        generic object to some other object outside of CHAI3D of to an external
        representation such as a dynamics engine model. See the ODE examples
        to understand how a generic object can be attached to an ODE object.
    */
    cGenericType* m_externalParent;

    /*!
        A super parent points to another object generally located higher up in the scene graph.
        When a mesh is created, the super parent of its children will generally point towards
        the root of the mesh. This parameter is automatically set by the 3D object file loader.
    */
    cGenericObject* m_superParent;


	//-----------------------------------------------------------------------
    // MEMBERS - POSITION & ORIENTATION:
	//-----------------------------------------------------------------------

    //! The position of this object in my parent's reference frame.
    cVector3d m_localPos;

    //! The position of this object in the world's reference frame.
    cVector3d m_globalPos;

    //! The rotation matrix that rotates my reference frame into my parent's reference frame.
    cMatrix3d m_localRot;

    //! The rotation matrix that rotates my reference frame into the world's reference frame.
    cMatrix3d m_globalRot;

    //! The previous position of this of this object in the parent's reference frame.
    cVector3d m_prevLocalPos;

    //! The previous position of this of this object in the parent's reference frame.
    cMatrix3d m_prevLocalRot;


	//-----------------------------------------------------------------------
    // MEMBERS - DYNAMIC OBJECTS:
	//-----------------------------------------------------------------------

    //! A previous position; exact interpretation up to user.
    cVector3d m_prevGlobalPos;

    //! A previous rotation; exact interpretation up to user.
    cMatrix3d m_prevGlobalRot;


	//-----------------------------------------------------------------------
    // MEMBERS - BOUNDARY BOX
	//-----------------------------------------------------------------------

    //! Minimum position of boundary box.
    cVector3d m_boundaryBoxMin;

    //! Maximum position of boundary box.
    cVector3d m_boundaryBoxMax;


	//-----------------------------------------------------------------------
    // MEMBERS - FRAME REPRESENTATION [X,Y,Z]:
	//-----------------------------------------------------------------------

    //! Size of graphical representation of frame (X-Y-Z).
    double m_frameSize;

	//! Pen thickness of graphical representation of frame (X-Y-Z).
    double m_frameThicknessScale;


	//-----------------------------------------------------------------------
    // MEMBERS - GRAPHICS:
	//-----------------------------------------------------------------------

    //! If \b true, this object is rendered.
    bool m_show;

    //! IF \b true, this object can be felt.
    bool m_hapticEnabled;

    //! If \b true, this object's reference frame is rendered as a set of arrows.
    bool m_showFrame;

    //! If \b true, this object's boundary box is displayed as a set of lines.
    bool m_showBox;

    //! If \b true, the skeleton of the scene graph is rendered at this node.
    bool m_showTree;

    //! If \b true, the collision tree is displayed (if available) at this node.
    bool m_showCollisionTree;

    //! The color of the collision tree.
    cColorf m_treeColor;

    //! The color of the bounding box.
    cColorf m_boundaryBoxColor;

    //! Should texture mapping be used?
    bool m_useTextureMapping;

    //! Should material properties be used?
    bool m_useMaterialProperty;

    //! Should per-vertex colors be used?
    bool m_useVertexColors;

    //! The polygon rendering mode (GL_FILL or GL_LINE).
    int m_triangleMode;

    /*!
        If true, transparency is enabled... this turns alpha on when the mesh is
        rendered, and - if multipass transparency is enabled in the rendering camera -
        uses the camera's multiple rendering passes to approximate back-to-front
        sorting via culling.
    */
    bool m_useTransparency;

    /*!
        If true, multipass transparency is permitted for this mesh... this means
        that if the rendering camera is using multipass transparency, this mesh
        will render back and front faces separately. \n

        Note that m_useTransparency also has to be \b true for this variable to
        be meaningful.
    */
    bool m_useMultipassTransparency;

     /*!
        Should culling be used when rendering triangles? \n

        Note that this option only applies when multipass transparency is
        disabled or during the non-transparent rendering pass when multipass
        transparency is enabled... \n

        Also note that currently only back-faces are culled during non-transparent
        rendering; you can't cull front-faces.
    */
    bool m_cullingEnabled;


	//-----------------------------------------------------------------------
    // MEMBERS - COLLISION DETECTION:
	//-----------------------------------------------------------------------

    //! The collision detector used to test for contact with this object.
    cGenericCollision* m_collisionDetector;


	//-----------------------------------------------------------------------
    // GENERAL VIRTUAL METHODS::
	//-----------------------------------------------------------------------

    //! Render this object in OpenGL.
    virtual void render(const int a_renderMode=CHAI_RENDER_MODE_RENDER_ALL);

    //! Update the m_globalPos and m_globalRot properties of any members of this object (e.g. all triangles).
    virtual void updateGlobalPositions(const bool a_frameOnly) {};

    //! Update the bounding box of this object, based on object-specific data (e.g. triangle positions).
    virtual void updateBoundaryBox() {};

    //! Scale current object with scale factors along x, y and z.
    virtual void scaleObject(const cVector3d& a_scaleFactors) {};

    //! Update the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN);

    //! computes any other interaction with object.
    virtual cVector3d computeOtherInteractions(const cVector3d& a_toolPos,
                                               const cVector3d& a_toolVel,
                                               const unsigned int a_IDN,
                                               cInteractionRecorder& a_interactions,
                                               cInteractionSettings& a_interactionSettings) { return cVector3d(0,0,0); }

    //! Compute any collisions other than the default collision detector.
    virtual bool computeOtherCollisionDetection(cVector3d& a_segmentPointA,
                                                cVector3d& a_segmentPointB,
                                                cCollisionRecorder& a_recorder,
                                                cCollisionSettings& a_settings) {return(false);}


	//-----------------------------------------------------------------------
    // MEMBERS - OPEN GL:
	//-----------------------------------------------------------------------

    //! OpenGL matrix describing my position and orientation transformation.
    cMatrixGL m_frameGL;
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

