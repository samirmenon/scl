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
    \author    Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 492 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "display/CCamera.h"
//---------------------------------------------------------------------------
#include "world/CWorld.h"
#include "lighting/CSpotLight.h"
//---------------------------------------------------------------------------
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
      Create a camera by passing the parent world as a parameter.

      \fn         cCamera::cCamera(cWorld* a_parentWorld)
      \param      a_parentWorld  Parent world camera.
*/
//===========================================================================
cCamera::cCamera(cWorld* a_parentWorld)
{
    // set parent world
    m_parentWorld = a_parentWorld;
	
	// set default values for clipping planes
    setClippingPlanes(0.1, 1000.0);

    // set default field of view angle
    setFieldViewAngle(45);

    // position and orient camera, looking down the negative x-axis
    // (the robotics convention)
    set(
          cVector3d(0,0,0),       // Local Position of camera.
          cVector3d(-1,0,0),      // Local Look At position
          cVector3d(0,0,1)        // Local Up Vector
        );


    // by default we use a persepctive camera
    m_perspectiveMode = true;

    // width of orthographic view. (not active by default)
    m_orthographicWidth = 0.0;

    // set default stereo parameters
    m_stereoFocalLength		= 2.0;
    m_stereoEyeSeparation	= 0.07;

    // disable multipass transparency rendering by default
    m_useMultipassTransparency = false;

	// enable shadow rendering
	m_useShadowCasting = false;

    // enable stereo display
    m_useStereo = false;

    // reset display status
    m_resetDisplay = false;

    // create front and back layers
    m_frontLayer = new cWorld();
    m_backLayer = new cWorld();
}


//===========================================================================
/*!
      Set the position and orientation of the camera. Three vectors are
      required: \n
      
      [iPosition] which describes the position in local coordinates
      of the camera. \n

      [iLookAt] which describes a point at which the camera is looking

      [iUp] to orient the camera around its rolling axis. [iUp] always points
      to the top of the image. \n

      These vectors are used in the usual gluLookAt sense.

      \fn         bool cCamera::set(const cVector3d& a_localPosition, 
				  const cVector3d& a_localLookAt,
                  const cVector3d& a_localUp)

      \param      a_localPosition  The position of the camera in local coordinates
      \param      a_localLookAt  The Point in local space at which the camera looks
      \param      a_localUp  A vector giving the rolling orientation (points toward
                  the top of the image)
*/
//===========================================================================
bool cCamera::set(const cVector3d& a_localPosition, 
				  const cVector3d& a_localLookAt,
                  const cVector3d& a_localUp)
{
    // copy new values to temp variables
    cVector3d pos = a_localPosition;
    cVector3d lookAt = a_localLookAt;
    cVector3d up = a_localUp;
    cVector3d Cy;

    // check validity of vectors
    if (pos.distancesq(lookAt) < C_SMALL) { return (false); }
    if (up.lengthsq() < C_SMALL) { return (false); }

    // compute new rotation matrix
    pos.sub(lookAt);
    pos.normalize();
    up.normalize();
    up.crossr(pos, Cy);
    if (Cy.lengthsq() < C_SMALL) { return (false); }
    Cy.normalize();
    pos.crossr(Cy,up);

    // update frame with new values
    setLocalPos(a_localPosition);
    cMatrix3d localRot;
    localRot.setCol(pos, Cy, up);
    setLocalRot(localRot);

    // return success
    return (true);
}


//===========================================================================
/*!
      Set the camera in orthographic mode.

      \fn         void cCamera::setOrthographicView(double a_viewWidth)
      \param      a_width  Width of orthographic view.
*/
//===========================================================================
void cCamera::setOrthographicView(double a_viewWidth)
{
    // setup orthographic mode
    m_orthographicWidth = cClamp0(a_viewWidth);
    m_fieldViewAngle = 0.0;
    m_perspectiveMode = false;
}


//===========================================================================
/*!
      Set the field of view angle in \e degrees

      \fn         void cCamera::setFieldViewAngle(double a_fieldViewAngle)
      \param      a_fieldViewAngle  Field of view angle in \e degrees
                  (should be between 0 and 180)
*/
//===========================================================================
void cCamera::setFieldViewAngle(double a_fieldViewAngle)
{
    m_fieldViewAngle = cClamp(a_fieldViewAngle, 0.0, 180.0);
    m_orthographicWidth = 0.0;
    m_perspectiveMode = true;
}


//===========================================================================
/*!
      Return aspect ratio of output image.

      \fn		double cCamera::getAspectRatio()
      \return	Return aspect ratio of image. Returns 1.0 if value cannot be
		        computed.
*/
//===========================================================================
double cCamera::getAspectRatio()
{
	double ratio = 1.0;
	if (m_lastDisplayHeight > 0)
	{
		ratio = (((double)m_lastDisplayWidth / (double)m_lastDisplayHeight));
	}
	return (ratio);
}


//===========================================================================
/*!
      Enable or disable 3D stereo rendering. This option is supported on
      all graphics card which support OpenGL quad buffers.

      \fn         void cCamera::setUseStereo(bool a_enabled)
      \param      a_enable  Focal length.
*/
//===========================================================================
void cCamera::setUseStereo(bool a_enabled)
{
    m_useStereo = a_enabled;
}


//===========================================================================
/*!
      Set stereo focal length

      \fn         int cCamera::setStereoFocalLength(double a_stereoFocalLength)
      \param      a_stereoFocalLength  Focal length.
*/
//===========================================================================
void cCamera::setStereoFocalLength(double a_stereoFocalLength)
{
    m_stereoFocalLength = a_stereoFocalLength;

    // Prevent 0 or negative focal lengths
    if (m_stereoFocalLength < C_SMALL)
	{
		m_stereoFocalLength = C_SMALL;
	}   
}


//===========================================================================
/*!
      Set stereo eye separation

      \fn         void cCamera::setStereoEyeSeparation(double a_stereoEyeSeparation)
      \param      a_stereoEyeSeparation  Distance between the left and right eyes.
                  
      Note that the stereo pair can be reversed by supplying a negative
      eye separation.
*/
//===========================================================================
void cCamera::setStereoEyeSeparation(double a_stereoEyeSeparation)
{
    m_stereoEyeSeparation = a_stereoEyeSeparation; 
}


//===========================================================================
/*!
      Set the positions of the near and far clip planes

      \fn       void cCamera::setClippingPlanes(const double a_distanceNear,
                const double a_distanceFar)
      \param    a_distanceNear  Distance to near clipping plane
      \param    a_distanceFar   Distance to far clipping plane
*/
//===========================================================================
void cCamera::setClippingPlanes(const double a_distanceNear, const double a_distanceFar)
{
    // check values of near and far clipping planes
    if ((a_distanceNear > 0.0) &&
        (a_distanceFar > 0.0) &&
        (a_distanceFar > a_distanceNear))
    {
        m_distanceNear = a_distanceNear;
        m_distanceFar = a_distanceFar;
    }
}


//===========================================================================
/*!
    Check for collision detection between an x-y position (typically a mouse
    click) and an object in the scene

    \fn         bool cCamera::select(const int a_windowPosX,
                     const int a_windowPosY,
                     const int a_windowWidth,
                     const int a_windowHeight,
                     cCollisionRecorder& a_collisionRecorder,
                     cCollisionSettings& a_collisionSettings)
     
     \param     a_windowPosX        X coordinate position of mouse click.
     \param     a_windowPosY        Y coordinate position of mouse click.
     \param     a_windowWidth       Width of window display (pixels)
     \param     a_windowHeight      Height of window display (pixels)
     \param     a_collisionRecorder Recorder used to store all collisions between mouse and objects
     \param     a_collisionSettings Settings related to collision detection
     \return    Returns \b true if an object has been hit, else false
*/
//===========================================================================
bool cCamera::select(const int a_windowPosX,
                     const int a_windowPosY,
                     const int a_windowWidth,
                     const int a_windowHeight,
                     cCollisionRecorder& a_collisionRecorder,
                     cCollisionSettings& a_collisionSettings)
{
    // sanity check
    if ((a_windowWidth <= 0) || (a_windowHeight <= 0)) return (false);
    
    // clear collision recorder
    a_collisionRecorder.clear();

    // update my m_globalPos and m_globalRot variables
    m_parentWorld->computeGlobalPositions(false);

    // init variable to store result
    bool result = false;
    if (m_perspectiveMode)
    {

        // make sure we have a legitimate field of view
        if (fabs(m_fieldViewAngle) < 0.001f) { return (false); }

        // compute the ray that leaves the eye point at the appropriate angle
        //
        // m_fieldViewAngle / 2.0 would correspond to the _top_ of the window
        double distCam = (a_windowHeight / 2.0f) / cTanDeg(m_fieldViewAngle / 2.0f);

        cVector3d selectRay;
        selectRay.set(-distCam,
                       (a_windowPosX - (a_windowWidth / 2.0f)),
                       ((a_windowHeight / 2.0f) - a_windowPosY));
        selectRay.normalize();

        selectRay = cMul(m_globalRot, selectRay);

        // create a point that's way out along that ray
        cVector3d selectPoint = cAdd(m_globalPos, cMul(100000, selectRay));

        // search for intersection between the ray and objects in the world
        result = m_parentWorld->computeCollisionDetection(
                                    m_globalPos,
                                    selectPoint,
                                    a_collisionRecorder,
                                    a_collisionSettings);
    }
    else
    {
        double hw = (double)(a_windowWidth) * 0.5;
        double hh = (double)(a_windowHeight)* 0.5;
        double aspect = hw / hh;
        
        double offsetX = ((a_windowPosX - hw) / hw) * 0.5 * m_orthographicWidth;
        double offsetY =-((a_windowPosY - hh) / hh) * 0.5 * (m_orthographicWidth / aspect);

        cVector3d pos = cAdd(m_globalPos, 
                             cMul(offsetX, m_globalRot.getCol1()), 
                             cMul(offsetY, m_globalRot.getCol2()));

        // create a point that's way out along that ray
        cVector3d selectPoint = cAdd(pos, cMul(100000,  cNegate(m_globalRot.getCol0())));

        result = m_parentWorld->computeCollisionDetection(pos,
                                                          selectPoint,
                                                          a_collisionRecorder,
                                                          a_collisionSettings);
    }

    // return result
    return (result);
}


//===========================================================================
/*!
      Set up the OpenGL perspective projection matrix, and nukes the contents
      of the GL buffers.  This function assumes the caller (typically cViewport)
      has set the appropriate buffer to be current.

      \fn         void cCamera::renderView(const int a_windowWidth, 
										   const int a_windowHeight)
      \param      a_windowWidth  Width of viewport.
      \param      a_windowHeight  Height of viewport.
*/
//===========================================================================
void cCamera::renderView(const int a_windowWidth, 
						 const int a_windowHeight)
{
	//-----------------------------------------------------------------------
	// (1) COMPUTE SHADOW MAPS
	//-----------------------------------------------------------------------

	// initialize a temporary variable that will inform us if shadow casting is used 
	// by our application and also supported by the hardware. 
	bool useShadowCasting = false;
    list<cShadowMap*> shadowMaps;
    shadowMaps.clear();

	// shadow casting has been requested, we will first need to verify if the hardware
	// can support it
	if (m_useShadowCasting)
	{
		// we verify if shadow casting is supported by hardware
		if (isShadowCastingSupported())
		{
            // we check every light source. if it is a spot light we verify if shadow casting is enabled
            for (unsigned int i=0; i<m_parentWorld->m_lights.size(); i++)
            {
			    cSpotLight* light =  dynamic_cast<cSpotLight*>(m_parentWorld->getLightSource(i));
			    if (light != NULL)
			    {
				    if (light->getShadowMapEnabled())
				    {
                        // update shadow map
				        light->updateShadowMap();

                        // add shadowmap to list
                        shadowMaps.push_back(light->m_shadowMap);

                        // shadow mapping is used by at least one light source!
					    useShadowCasting = true;
				    }
			    }
            }
		}
	}


	//-----------------------------------------------------------------------
	// (2) INITIALIZE CURRENT VIEWPORT
	//-----------------------------------------------------------------------

    // check window size
    if (a_windowHeight == 0) { return; }

    // store most recent size of display
    m_lastDisplayWidth = a_windowWidth;
    m_lastDisplayHeight = a_windowHeight;

	// setup viewport
	glViewport(0, 0, a_windowWidth, a_windowHeight);

    // compute aspect ratio
    double glAspect = ((double)a_windowWidth / (double)a_windowHeight);

    // compute global pose
    computeGlobalPositionsFromRoot(true);

    // set background color
    glClearColor(m_parentWorld->getBackgroundColor().getR(),
                 m_parentWorld->getBackgroundColor().getG(),
                 m_parentWorld->getBackgroundColor().getB(),
                 1.0);


	//-----------------------------------------------------------------------
	// (3) VERIFY IF STEREO DISPLAY IS ENABLED
	//-----------------------------------------------------------------------
    GLboolean stereo = false;
    unsigned int numStereoPass = 1;
    if (m_useStereo)
    {
        // verify if stereo is available by the graphics hardware and camera is of perspective model
        glGetBooleanv(GL_STEREO, &stereo);  
        if (stereo && m_perspectiveMode)
        {
            // stereo is available - we shall perform 2 rendering passes for LEFT and RIGHT eye.
            numStereoPass = 2;
        }
        else
        {
            stereo = false;
        }   
    }


	//-----------------------------------------------------------------------
	// (4) RENDER THE ENTIRE SCENE
	//-----------------------------------------------------------------------
    for (unsigned int i=0; i<numStereoPass; i++)
    {
	    //-------------------------------------------------------------------
	    // (4.1) SELECTING THE DISPLAY BUFFER (MONO / STEREO)
	    //-------------------------------------------------------------------
        if (stereo)
        {
            if (i == 0)
            {
                glDrawBuffer(GL_BACK_LEFT);
            }
            else
            {
                glDrawBuffer(GL_BACK_RIGHT);
            }
        }
        else
        {
            glDrawBuffer(GL_BACK);
        }

	    //-------------------------------------------------------------------
	    // (4.2) CLEAR CURRENT BUFFER
	    //-------------------------------------------------------------------

        // clear the color and depth buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	    glShadeModel(GL_SMOOTH);

	    //-------------------------------------------------------------------
	    // (4.3)  RENDER BACK PLANE
	    //-------------------------------------------------------------------

        // render the 2D backlayer
        // it will set up its own projection matrix
        if (m_backLayer->getNumChildren())
	    {
		    renderLayer(m_backLayer,
                        a_windowWidth,
                        a_windowHeight);
	    }

        // clear depth buffer
        glClear(GL_DEPTH_BUFFER_BIT);

	    //-------------------------------------------------------------------
	    // (4.4a) SETUP CAMERA  (MONO RENDERING)
	    //-------------------------------------------------------------------
        if (!stereo)
        {
            // init projection matrix
	        glMatrixMode(GL_PROJECTION);
	        glLoadIdentity();

            // create projection matrix depending of camera mode
            if (m_perspectiveMode)
            {
		        // setup perspective camera
		        glMatrixMode(GL_PROJECTION);
		        glLoadIdentity();
		        gluPerspective(
				        m_fieldViewAngle,   // Field of View angle.
				        glAspect,           // Aspect ratio of viewing volume.
				        m_distanceNear,     // Distance to Near clipping plane.
				        m_distanceFar);     // Distance to Far clipping plane.
            }
            else
            {
                // setup orthographic camera
                double left     = -m_orthographicWidth / 2.0;
                double right    = -left;
                double bottom   = left / glAspect;
                double top      = -bottom;

                glOrtho(left,               // Left vertical clipping plane.
                        right,              // Right vertical clipping plane.            
                        bottom,             // Bottom vertical clipping plane.
                        top,                // Top vertical clipping plane.
                        m_distanceNear,     // Distance to Near clipping plane.
				        m_distanceFar       // Distance to Far clipping plane.
                    );
            }

            // setup cameta position
            glMatrixMode(GL_MODELVIEW);
	        glLoadIdentity();

	        // compute camera location
	        cVector3d lookAt = m_globalRot.getCol0();
	        cVector3d lookAtPos;
	        m_globalPos.subr(lookAt, lookAtPos);
	        cVector3d up = m_globalRot.getCol2();

	        // setup modelview matrix
	        gluLookAt( m_globalPos(0) ,  m_globalPos(1) ,  m_globalPos(2) ,
			           lookAtPos(0) ,    lookAtPos(1) ,    lookAtPos(2) ,
			           up(0) ,           up(1) ,           up(2)  );
        }


	    //-------------------------------------------------------------------
	    // (4.4b) SETUP CAMERA  (STEREO RENDERING)
	    //-------------------------------------------------------------------
        else
        {
		    //-----------------------------------------------------------------
		    // Based on Paul Bourke's stereo rendering tutorial:
		    // http://local.wasp.uwa.edu.au/~pbourke/miscellaneous/stereographics/stereorender/
		    //-----------------------------------------------------------------

		    double radians = ((C_PI / 180.0) * m_fieldViewAngle / 2.0f);
		    double wd2 = m_distanceNear * tan(radians);
		    double ndfl = m_distanceNear / m_stereoFocalLength;

		    // compute the look, up, and cross vectors
		    cVector3d lookv = m_globalRot.getCol0();
		    lookv.mul(-1.0);

		    cVector3d upv = m_globalRot.getCol2();
		    cVector3d offsetv = cCross(lookv,upv);

		    offsetv.mul(m_stereoEyeSeparation / 2.0);

		    if (i == 0) offsetv.mul(-1.0);

		    // decide whether to offset left or right
		    double stereo_multiplier = (i == 0) ? 1.0f : -1.0f;  // (i == 0) correspond to LEFT IMAGE, (i == 1) correspond to RIGHT IMAGE

		    double left   = -1.0 * glAspect * wd2 + stereo_multiplier * 0.5 * m_stereoEyeSeparation * ndfl;
		    double right  =        glAspect * wd2 + stereo_multiplier * 0.5 * m_stereoEyeSeparation * ndfl;
		    double top    =        wd2;
		    double bottom = -1.0 * wd2;

		    // setup projection matrix
		    glMatrixMode(GL_PROJECTION);
		    glLoadIdentity();
		    glFrustum(left,right,bottom,top,m_distanceNear,m_distanceFar);

		    // initialize modelview matrix
		    glMatrixMode(GL_MODELVIEW);
		    glLoadIdentity();

		    // compute the offset we should apply to the current camera position
		    cVector3d pos = cAdd(m_globalPos,offsetv);

		    // compute the shifted camera position
		    cVector3d lookAtPos;
		    pos.addr(lookv, lookAtPos);

		    // setup modelview matrix
		    gluLookAt(pos(0) ,       pos(1) ,       pos(2) ,
				      lookAtPos(0) , lookAtPos(1) , lookAtPos(2) ,
				      upv(0) ,       upv(1) ,       upv(2) 
				      );
        }

        // Backup the view and projection matrix for future reference
        glGetDoublev(GL_PROJECTION_MATRIX,m_projectionMatrix.getData());
	    glGetDoublev(GL_MODELVIEW_MATRIX, m_modelviewMatrix.getData());


	    //-------------------------------------------------------------------
	    // (4.5) RENDER THE 3D WORLD
	    //-------------------------------------------------------------------

        // Set up reasonable default OpenGL state
        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
	    glDepthFunc(GL_LEQUAL);

	    // rendering options
	    cRenderOptions options;

        // optionally perform multiple rendering passes for transparency
        if (m_useMultipassTransparency) 
	    {
		    ////////////////////////////////////////////////////////////////////
		    // MULTI PASS - USING SHADOW CASTING
		    ////////////////////////////////////////////////////////////////////
		    if (useShadowCasting)
		    {
                // setup rendering options
                options.m_camera                                = this;
                options.m_single_pass_only						= false;
                options.m_render_opaque_objects_only			= true;
                options.m_render_transparent_front_faces_only	= false;
                options.m_render_transparent_back_faces_only	= false;
                options.m_enable_lighting						= true;
                options.m_render_materials						= true;
                options.m_render_textures						= true;
                options.m_creating_shadow_map					= false;
                options.m_rendering_shadow						= true;
                options.m_shadow_light_level					= 1.0 - m_parentWorld->getShadowIntensity();
                options.m_storeObjectPositions					= false;
                options.m_resetDisplay                          = m_resetDisplay;

                // render 1st pass (opaque objects - shadowed regions)
                m_parentWorld->renderSceneGraph(options);

                // setup rendering options
                options.m_rendering_shadow						= false;
                options.m_shadow_light_level					= 1.0;

                // render 2nd pass (opaque objects - non shadowed regions)
                list<cShadowMap*>::iterator i;
                for(i = shadowMaps.begin(); i !=shadowMaps.end(); ++i)
                {
                    cShadowMap *shadowMap = *i;
                    shadowMap->render(options);

                    glEnable(GL_POLYGON_OFFSET_FILL);
                    m_parentWorld->renderSceneGraph(options);
                    glDisable(GL_POLYGON_OFFSET_FILL);

                    // restore states
                    glActiveTexture(GL_TEXTURE0_ARB);
                    glDisable(GL_TEXTURE_2D);
                    glDisable(GL_TEXTURE_GEN_S);
                    glDisable(GL_TEXTURE_GEN_T);
                    glDisable(GL_TEXTURE_GEN_R);
                    glDisable(GL_TEXTURE_GEN_Q);
                    glDisable(GL_ALPHA_TEST);
                }

                // setup rendering options
                options.m_render_opaque_objects_only			= false;
                options.m_render_transparent_back_faces_only	= true;

                // render 3rd pass (transparent objects - back faces only)
                m_parentWorld->renderSceneGraph(options);

                // modify rendering options for third pass
                options.m_render_transparent_back_faces_only	= false;
                options.m_render_transparent_front_faces_only	= true;

                // render 4th pass (transparent objects - back faces only)
                m_parentWorld->renderSceneGraph(options);
		    }


		    ////////////////////////////////////////////////////////////////////
		    // MULTI PASS - WITHOUT SHADOWS
		    ////////////////////////////////////////////////////////////////////
		    else
		    {
			    // setup rendering options for first pass
                options.m_camera                                = this;
			    options.m_single_pass_only						= false;
			    options.m_render_opaque_objects_only			= true;
			    options.m_render_transparent_front_faces_only	= false;
			    options.m_render_transparent_back_faces_only	= false;
			    options.m_enable_lighting						= true;
			    options.m_render_materials						= true;
			    options.m_render_textures						= true;
			    options.m_creating_shadow_map					= false;
			    options.m_rendering_shadow						= false;
			    options.m_shadow_light_level					= 1.0;
			    options.m_storeObjectPositions					= true;
                options.m_resetDisplay                          = m_resetDisplay;

			    // render 1st pass (opaque objects - all faces)
			    m_parentWorld->renderSceneGraph(options);

			    // modify rendering options
			    options.m_render_opaque_objects_only			= false;
			    options.m_render_transparent_back_faces_only	= true;
			    options.m_storeObjectPositions					= false;

			    // render 2nd pass (transparent objects - back faces only)
			    m_parentWorld->renderSceneGraph(options);

			    // modify rendering options
			    options.m_render_transparent_back_faces_only	= false;
			    options.m_render_transparent_front_faces_only	= true;

			    // render 3rd pass (transparent objects - front faces only)
			    m_parentWorld->renderSceneGraph(options);
		    }
        }
        else
        {
		    ////////////////////////////////////////////////////////////////////
		    // SINGLE PASS - USING SHADOW CASTING 
		    ////////////////////////////////////////////////////////////////////
		    if (useShadowCasting)
		    {
			    // setup rendering options for single pass
                options.m_camera                                = this;
			    options.m_single_pass_only						= true;
			    options.m_render_opaque_objects_only			= false;
			    options.m_render_transparent_front_faces_only	= false;
			    options.m_render_transparent_back_faces_only	= false;
			    options.m_enable_lighting						= true;
			    options.m_render_materials						= true;
			    options.m_render_textures						= true;
			    options.m_creating_shadow_map					= false;
			    options.m_rendering_shadow						= true;
			    options.m_shadow_light_level					= 1.0 - m_parentWorld->getShadowIntensity();
			    options.m_storeObjectPositions					= false;
                options.m_resetDisplay                          = m_resetDisplay;

			    // render 1st pass (opaque objects - all faces - shadowed regions)
			    m_parentWorld->renderSceneGraph(options);

			    // setup rendering options
			    options.m_rendering_shadow						= false;
			    options.m_shadow_light_level					= 1.0;

			    // render 2nd pass (opaque objects - all faces - non shadowed regions)
                list<cShadowMap*>::iterator i;
                for(i = shadowMaps.begin(); i !=shadowMaps.end(); ++i)
                {
                    cShadowMap *shadowMap = *i;
                    shadowMap->render(options);

                    glEnable(GL_POLYGON_OFFSET_FILL);
                    m_parentWorld->renderSceneGraph(options);
                    glDisable(GL_POLYGON_OFFSET_FILL);

                    // restore states
                    glActiveTexture(GL_TEXTURE0_ARB);
                    glDisable(GL_TEXTURE_2D);
                    glDisable(GL_TEXTURE_GEN_S);
                    glDisable(GL_TEXTURE_GEN_T);
                    glDisable(GL_TEXTURE_GEN_R);
                    glDisable(GL_TEXTURE_GEN_Q);
                    glDisable(GL_ALPHA_TEST);
                }

			    // restore states
			    glActiveTexture(GL_TEXTURE0_ARB);
			    glDisable(GL_TEXTURE_2D);
			    glDisable(GL_TEXTURE_GEN_S);
			    glDisable(GL_TEXTURE_GEN_T);
			    glDisable(GL_TEXTURE_GEN_R);
			    glDisable(GL_TEXTURE_GEN_Q);
			    glDisable(GL_ALPHA_TEST);
		    }


		    ////////////////////////////////////////////////////////////////////
		    // SINGLE PASS - WITHOUT SHADOWS
		    ////////////////////////////////////////////////////////////////////
		    else
		    {
			    // setup rendering options for single pass
                options.m_camera                                = this;
			    options.m_single_pass_only						= true;
			    options.m_render_opaque_objects_only			= false;
			    options.m_render_transparent_front_faces_only	= false;
			    options.m_render_transparent_back_faces_only	= false;
			    options.m_enable_lighting						= true;
			    options.m_render_materials						= true;
			    options.m_render_textures						= true;
			    options.m_creating_shadow_map					= false;
			    options.m_rendering_shadow						= false;
			    options.m_shadow_light_level					= 1.0;
			    options.m_storeObjectPositions					= true;
                options.m_resetDisplay                          = m_resetDisplay;

			    // render single pass (all objects)
			    m_parentWorld->renderSceneGraph(options);                
		    }
        }        

	    //-------------------------------------------------------------------
	    // (4.6) RENDER FRONT PLANE
	    //-------------------------------------------------------------------

        // clear depth buffer
        glClear(GL_DEPTH_BUFFER_BIT);

        // render the 'front' 2d object layer; it will set up its own
        // projection matrix
        if (m_frontLayer->getNumChildren() > 0)
	    {
		    renderLayer(m_frontLayer, 
                        a_windowWidth, 
                        a_windowHeight);
	    }

        // if requested, display reset has now been completed
        m_resetDisplay = false;
    }
}


//===========================================================================
/*!
      Copies the opengl image buffer to a cImage class structure.

      \fn         void cCamera::copyImageData(cImageLoader* a_image)
      \param      a_image  Destination image
*/
//===========================================================================
void cCamera::copyImageData(cImage* a_image)
{
    // check image structure
    if (a_image == NULL) { return; }

    // check size
    if ((m_lastDisplayWidth  != a_image->getWidth()) ||
        (m_lastDisplayHeight != a_image->getHeight()))
    {
        a_image->allocate(m_lastDisplayWidth, m_lastDisplayHeight, GL_RGBA);
    }

    // copy pixel data if required
    glReadPixels(
		0,
		0,
		m_lastDisplayWidth,
		m_lastDisplayHeight,
        a_image->getFormat(),
		GL_UNSIGNED_BYTE,
		a_image->getData()
    );
}


//===========================================================================
/*!
    Enable or disable multipass transparency... when this option is
    enabled (it's disabled by default), each time the camera is
    asked to render the scene, it will perform three rendering
    passes: a pass for non-transparent items, a pass for the backfaces
    of transparent items, and a pass for the frontfaces of transparent
    items.

    Objects being rendered are told which pass is current via the 
    parameter supplied to the render() function.

    We will hopefully find a cleaner way to do this in the future, but
    for now be careful when you enable this feature...

    \fn     void cCamera::setUseMultipassTransparency(bool a_enabled)
    \param  a_enabled   value.
*/
//===========================================================================
void cCamera::setUseMultipassTransparency(bool a_enabled) 
{
    m_useMultipassTransparency = a_enabled;
}


//===========================================================================
/*!
    Enable or disable shadow casting.

    \fn     void cCamera::setUseShadowCasting(bool a_enabled)
    \param  a_enabled   value.
*/
//===========================================================================
void cCamera::setUseShadowCasting(bool a_enabled)
{
	m_useShadowCasting = a_enabled;
}


//===========================================================================
/*!
    Verifies if shadow casting is supported on this hardware.

    \fn     bool cCamera::isShadowCastingSupported();
	\return	Return \true if shadow casting is supported, otherwise \b false.
*/
//===========================================================================
bool cCamera::isShadowCastingSupported()
{
	//Check for necessary OpenGL extensions
	if(!GLEE_ARB_depth_texture || 
	   !GLEE_ARB_shadow)
	{
		return (false);
	}
	else
	{
		return (true);
	}
}


//===========================================================================
/*!
    This call automatically adjusts the front and back clipping planes to
    optimize usage of the z-buffer.

    \fn     void cCamera::adjustClippingPlanes();
*/
//===========================================================================
void cCamera::adjustClippingPlanes()
{
    // check if world is valid
    cWorld* world = getParentWorld();
    if (world == NULL) { return; }

    // compute size of the world
    world->computeBoundaryBox(true);

    // compute a distance slightly larger the world size
    cVector3d max = world->getBoundaryMax();
    cVector3d min = world->getBoundaryMin();
    double distance = 2.0 * cDistance(min, max);

    // update clipping plane:
    if (distance > 0.0)
	{
		setClippingPlanes(distance / 1000.0, distance);
	}
}


//===========================================================================
/*!
    Render a 2d scene within the viewport.

    \fn     void cCamera::renderLayer(cGenericObject* a_graph, 
                          int a_width, 
                          int a_height)
    \param  a_graph  The root of the 2d scenegraph to be rendered.
    \param  a_width  The size of the rendering window
    \param  a_height The size of the rendering window
*/
//===========================================================================
void cCamera::renderLayer(cGenericObject* a_graph, 
                          int a_width, 
                          int a_height)
{
    // set up an orthographic projection matrix
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    // set orthographic rendering mode. The z-buffer front and back clipping planes
    // can be set to any desired default values.
    glOrtho(0, a_width, 0, a_height, -10000, 10000);

    // reset modeling view matrix.
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // we disable lighting in our 2D world and widgets are considered to be
	// non ligthing sensitive.
    glDisable(GL_LIGHTING);

	// setup rendering options for single pass
	cRenderOptions options;
    options.m_camera                                = this;
	options.m_single_pass_only						= true;
	options.m_render_opaque_objects_only			= false;
	options.m_render_transparent_front_faces_only	= false;
	options.m_render_transparent_back_faces_only	= false;
	options.m_enable_lighting						= true;
	options.m_render_materials						= true;
	options.m_render_textures						= true;
	options.m_creating_shadow_map					= false;
	options.m_rendering_shadow						= false;
	options.m_shadow_light_level					= 1.0;
	options.m_storeObjectPositions					= true;
    options.m_resetDisplay                          = false;

    // render widget scene graph
    a_graph->renderSceneGraph(options);

    // put OpenGL back into a useful state
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);

    // restore modelview and projection matrices
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}


//===========================================================================
/*!
    Called by the user or by the viewport when the world needs to have
    textures and display lists reset (e.g. after a switch to or from
    fullscreen).

    \fn     void cCamera::onDisplayReset() 
*/
//===========================================================================
void cCamera::onDisplayReset() 
{
    m_resetDisplay = true;
}

