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
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CCameraH
#define CCameraH
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
#include "math/CMaths.h"
#include "files/CImageLoader.h"
//---------------------------------------------------------------------------
class cWorld;
//---------------------------------------------------------------------------

//! Constant specifying specific stereo rendering frames (Monoscopic View).
const int CHAI_MONO             =  0;

//! Constant specifying specific stereo rendering frames (Stereoscopic View - Left Eye).
const int CHAI_STEREO_LEFT      = -1;

//! Constant specifying specific stereo rendering frames (Monoscopic View  - Right Eye).
const int CHAI_STEREO_RIGHT     =  1;

/*!
    This constant is used to tell the _viewport_ that he should decide
    which frame(s) to render, and send MONO, LEFT, and/or RIGHT to the
    camera.
*/
const int CHAI_STEREO_DEFAULT   =  -1000;

//! The maximum number of arbitrary clip planes.
#define CHAI_MAX_CLIP_PLANES 6

//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CCamera.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Camera.
*/
//===========================================================================

//===========================================================================
/*!
      \struct     cClippingPlane
      \ingroup    scenegraph
      \brief      Struct used for enabling arbitrary clipping planes after 
                  the viewing matrix has been set up
*/
//===========================================================================
struct cClippingPlane
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cClippingPlane
    cClippingPlane()
    {
        enabled = -1;
        peqn[0] = peqn[1] = peqn[2] = peqn[3] = 0.0f;
    }


    //-----------------------------------------------------------------------
    // MEMBERS:
	//-----------------------------------------------------------------------

    /*! 
        Is this clip plane enabled?
        0 : disabled
        1 : enabled
        -1 : don't touch
    */
    int enabled;

    //! The plane equation
    double peqn[4];
};


//===========================================================================
/*!
      \class      cCamera
      \ingroup    scenegraph
      \brief      cCamera describes a virtual Camera located inside the world.
                  Its job in life is to set up the OpenGL projection matrix
                  for the current OpenGL rendering context.  The default camera
                  looks down the negative x-axis.  OpenGL folks may wonder why
                  we chose the negative x-axis... it turns out that's a better
                  representation of the standard conventions used in general
                  robotics.
*/
//===========================================================================
class cCamera : public cGenericObject
{
  friend class cWorld;

  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cCamera
    cCamera(cWorld* iParent);

    //! Destructor of cCamera
    virtual ~cCamera() {};


	//-----------------------------------------------------------------------
    // METHODS - MOUSE SELECTION:
	//-----------------------------------------------------------------------

    //! Get pointer to parent world.
    cWorld* getParentWorld() { return (m_parentWorld); }

    //! Query whether the specified position is 'pointing at' any objects in the world.
    virtual bool select(const int a_windowPosX, const int a_windowPosY,
                        const int a_windowWidth, const int a_windowHeight,
                        cCollisionRecorder& a_collisionRecorder,
						cCollisionSettings& a_collisionSettings);


	//-----------------------------------------------------------------------
    // METHODS - POSITION & ORIENTATION:
	//-----------------------------------------------------------------------

    //! Set the position and orientation of the camera.
    virtual bool set(const cVector3d& a_localPosition,
					 const cVector3d& a_localLookAt,
					 const cVector3d& a_localUp);

	//! Get the camera "look at" position vector for this camera.
    cVector3d getLookVector()  const { return m_localRot.getCol0(); }

	//! Get the  "up" vector for this camera.
    cVector3d getUpVector()    const { return m_localRot.getCol2(); }

	//! Get the "right direction" vector for this camera.
    cVector3d getRightVector() const { return m_localRot.getCol1(); }

    //! It's useful to store the last projection matrix, for gluProject'ing things.
    double m_projectionMatrix[16];


	//-----------------------------------------------------------------------
    // METHODS - CLIPPING PLANES:
	//-----------------------------------------------------------------------

    //! Set near and far clipping planes.
    void setClippingPlanes(const double a_distanceNear, const double a_distanceFar);

    //! Get near clipping plane.
    double getNearClippingPlane() { return (m_distanceNear); }

    //! Get far clipping plane.
    double getFarClippingPlane() { return (m_distanceFar); }

    //! Automatically adjust back and front clipping planes.
    void adjustClippingPlanes();

    //! Enable or disable one of the (six) arbitrary clipping planes.
    void enableClipPlane(const unsigned int& index, const int& enable, const double* peqn=0);


	//-----------------------------------------------------------------------
    // METHODS - FIELD OF VIEW & OPTICS: 
	//-----------------------------------------------------------------------

    //! Set field of view angle (in degrees).
    void setFieldViewAngle(double a_fieldViewAngle);

    //! Read field of view angle (in degrees).
    double getFieldViewAngle() { return (m_fieldViewAngle); }

    //! Set stereo focal length.
    int setStereoFocalLength(double a_stereoFocalLength);

    //! Get stereo focal length.
    double getStereoFocalLength() { return (m_stereoFocalLength); }

    //! Set stereo eye separation.
    int setStereoEyeSeparation(double a_stereoEyeSeparation);

    //! Get stereo eye separation.
    double getStereoEyeSeparation() { return (m_stereoEyeSeparation); }


	//-----------------------------------------------------------------------
    // METHODS - RENDERING AND IMAGING:
	//-----------------------------------------------------------------------

    //! Render the camera in OpenGL (i.e. set up the projection matrix)...
    virtual void renderView(const int a_windowWidth, const int a_windowHeight, const int a_imageIndex = CHAI_MONO);

    //! Copy output image data to image structure.
    void copyImageData(cImageLoader* a_image);

    //! Enable or disable additional rendering passes for transparency (see full comment).
    virtual void enableMultipassTransparency(bool enable);

    //! Resets textures and displays for the world associated with this camera.
    virtual void onDisplayReset(const bool a_affectChildren = true);


	//-----------------------------------------------------------------------
    // MEMBERS - FRONT AND BACK PLANES:
	//-----------------------------------------------------------------------

	/*
		These are special 'children' of the camera that are rendered independently of
		all other objects, intended to contain 2d objects only.  The 'back' scene is
		rendered before the 3d objects; the 'front' scene is rendered after the
		3d object.  These are made public variables to allow convenient access to
		the scenegraph management functions.

		These objects are rendered through an orthographic projection matrix, so the
		positive z axis faces the camera.  Depth is currently not used.  Lighting
		is disabled during rendering.
    */

	//! Front plane scenegraph which can be used to attach widgets.
	cGenericObject m_front_2Dscene;

	//! Black plane scenegraph which can be used to attach widgets.
    cGenericObject m_back_2Dscene;


  protected:

	//-----------------------------------------------------------------------
    // MEMBERS:
	//-----------------------------------------------------------------------

    //! Parent world.
    cWorld *m_parentWorld;

    //! Distance to near clipping plane.
    double m_distanceNear;

    //! Distance to far clipping plane.
    double m_distanceFar;

    //! Other clipping planes.
    cClippingPlane m_clipPlanes[CHAI_MAX_CLIP_PLANES];

    //! Field of view angle in degrees.
    double m_fieldViewAngle;

    // Stereo Parameters:

    //! Focal length.
    double m_stereoFocalLength;

    //! Eye separation.
    double m_stereoEyeSeparation;

    //! If true, three rendering passes are performed to approximate back-front sorting (see long comment)
    bool m_useMultipassTransparency;

    //! Render a 2d scene within this camera's view.
    void render2dSceneGraph(cGenericObject* a_graph, int a_width, int a_height);

    //! Some apps may have the camera as a child of the world, which would cause recursion when resetting the display
    bool m_performingDisplayReset;

    //! last width size of the window.
    unsigned int m_lastDisplayWidth;

	//! last height size of the window.
    unsigned int m_lastDisplayHeight;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

