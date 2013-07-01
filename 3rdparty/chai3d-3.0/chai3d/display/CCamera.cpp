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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 492 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "display/CCamera.h"
using namespace std;
//------------------------------------------------------------------------------
#include "world/CWorld.h"
#include "lighting/CSpotLight.h"
//------------------------------------------------------------------------------
#ifdef C_USE_OPENGL
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Create a camera by passing the parent world as a parameter.

    \param  a_parentWorld  Parent world camera.
*/
//==============================================================================
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


    // by default we use a perspective camera
    m_perspectiveMode = true;

    // width of orthographic view. (not active by default)
    m_orthographicWidth = 0.0;

    // set default stereo parameters
    m_stereoMode            = C_STEREO_DISABLED;
    m_stereoFocalLength		= 2.0;
    m_stereoEyeSeparation	= 0.07;

    // disable multipass transparency rendering by default
    m_useMultipassTransparency = false;

    // enable shadow rendering
    m_useShadowCasting = false;

    // reset display status
    m_resetDisplay = false;

    // create front and back layers
    m_frontLayer = new cWorld();
    m_backLayer = new cWorld();

    // mirroring
    m_mirrorHorizontal = false;
    m_mirrorVertical = false;
    m_mirrorStatus = false;
    m_scaleH = 1.0;
    m_scaleV = 1.0;

}


//==============================================================================
/*!
    Set the position and orientation of the camera. Three vectors are
    required: \n
      
    [iPosition] which describes the position in local coordinates
    of the camera. \n

    [iLookAt] which describes a point at which the camera is looking

    [iUp] to orient the camera around its rolling axis. [iUp] always points
    to the top of the image. \n

    These vectors are used in the usual gluLookAt sense.

    \param  a_localPosition  The position of the camera in local coordinates
    \param  a_localLookAt  The Point in local space at which the camera looks
    \param  a_localUp  A vector giving the rolling orientation (points toward
            the top of the image)
*/
//==============================================================================
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


//==============================================================================
/*!
    Set the camera in orthographic mode.

    \param  a_viewWidth  Width of orthographic view.
*/
//==============================================================================
void cCamera::setOrthographicView(double a_viewWidth)
{
    // setup orthographic mode
    m_orthographicWidth = cClamp0(a_viewWidth);
    m_fieldViewAngle = 0.0;
    m_perspectiveMode = false;
}


//==============================================================================
/*!
    Set the field of view angle in \e degrees

    \param  a_fieldViewAngle  Field of view angle in _degrees_  (0-180).
*/
//==============================================================================
void cCamera::setFieldViewAngle(double a_fieldViewAngle)
{
    m_fieldViewAngle = cClamp(a_fieldViewAngle, 0.0, 180.0);
    m_orthographicWidth = 0.0;
    m_perspectiveMode = true;
}


//==============================================================================
/*!
    Return aspect ratio of output image.

    \return Return aspect ratio of image. Returns 1.0 if value cannot be
            computed.
*/
//==============================================================================
double cCamera::getAspectRatio()
{
    double ratio = 1.0;
    if (m_lastDisplayHeight > 0)
    {
        ratio = (((double)m_lastDisplayWidth / (double)m_lastDisplayHeight));
    }
    return (ratio);
}


//==============================================================================
/*!
    Set the 3D stereo rendering mode. The following rendering modes are 
    supported: \n\n
      
    * C_STEREO_ACTIVE: Active stereo, requires OpenGL quad buffers. \n
    * C_STEREO_PASSIVE_LEFT_RIGHT: Passive stereo. Left and Right eye images are 
      rendered next to each other. \n
    * C_STEREO_PASSIVE_TOP_BOTTOM: Passive stereo. Left and Right eye images are 
      rendered above each other. \n
    * C_STEREO_DISABLED: Disable stereo. \n
      
    \param  a_stereoMode  Stereo mode.
*/
//==============================================================================
void cCamera::setStereoMode(cStereoMode a_stereoMode)
{
    if (m_perspectiveMode)
    {
        m_stereoMode = a_stereoMode;
    }
    else
    {
        m_stereoMode = C_STEREO_DISABLED;
    }
}


//==============================================================================
/*!
    Set stereo focal length

    \param  a_stereoFocalLength  Focal length.
*/
//==============================================================================
void cCamera::setStereoFocalLength(double a_stereoFocalLength)
{
    m_stereoFocalLength = a_stereoFocalLength;

    // Prevent 0 or negative focal lengths
    if (m_stereoFocalLength < C_SMALL)
    {
        m_stereoFocalLength = C_SMALL;
    }   
}


//==============================================================================
/*!
    Set stereo eye separation. 

    \param  a_stereoEyeSeparation  Distance between the left and right eyes.
*/
//==============================================================================
void cCamera::setStereoEyeSeparation(double a_stereoEyeSeparation)
{
    m_stereoEyeSeparation = a_stereoEyeSeparation; 
}


//! 
//==============================================================================
/*!
    Enable or disable output image mirroring horizontally.

    \param  a_enabled  If __true_ then mirroring is enabled, __false__ otherwise.
*/
//==============================================================================
void cCamera::setMirrorHorizontal(bool a_enabled)
{
    // update state
    m_mirrorHorizontal = a_enabled;

    // update scale factor
    if (m_mirrorHorizontal)
    {
        m_scaleH = -1.0;
    }
    else
    {
        m_scaleH = 1.0;
    }

    // update mirror status
    if (((m_mirrorHorizontal) && (!m_mirrorVertical)) ||
        ((!m_mirrorHorizontal) && (m_mirrorVertical)))
    {
        m_mirrorStatus = true;
    }
    else
    {
        m_mirrorStatus = false;
    }
}


//==============================================================================
/*!
    Enable or disable output image mirroring vertically.

    \param  a_enabled  If __true_ then mirroring is enabled, __false__ otherwise.
*/
//==============================================================================
void cCamera::setMirrorVertical(bool a_enabled)
{
    // update state
    m_mirrorVertical = a_enabled;

    // update scale factor
    if (m_mirrorVertical)
    {
        m_scaleV = -1.0;
    }
    else
    {
        m_scaleV = 1.0;
    }

    // update mirror status
    if (((m_mirrorHorizontal) && (!m_mirrorVertical)) ||
        ((!m_mirrorHorizontal) && (m_mirrorVertical)))
    {
        m_mirrorStatus = true;
    }
    else
    {
        m_mirrorStatus = false;
    }
}



//==============================================================================
/*!
    Set the positions of the near and far clip planes.

    \param  a_distanceNear  Distance to near clipping plane.
    \param  a_distanceFar  Distance to far clipping plane.
*/
//==============================================================================
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


//==============================================================================
/*!
    Check for collision detection between an x-y position (typically a mouse
    click) and an object in the scene.

    \param  a_windowPosX  X coordinate position of mouse click.
    \param  a_windowPosY  Y coordinate position of mouse click.
    \param  a_windowWidth  Width of window display (pixels)
    \param  a_windowHeight  Height of window display (pixels)
    \param  a_collisionRecorder  Recorder used to store all collisions between mouse and objects
    \param  a_collisionSettings  Settings related to collision detection

    \return Returns __true__ if an object has been hit, else false
*/
//==============================================================================
bool cCamera::select(const int a_windowPosX,
                     const int a_windowPosY,
                     const int a_windowWidth,
                     const int a_windowHeight,
                     cCollisionRecorder& a_collisionRecorder,
                     cCollisionSettings& a_collisionSettings)
{
    // sanity check
    if ((a_windowWidth <= 0) || (a_windowHeight <= 0)) return (false);

    // store values
    int windowPosX = a_windowPosX;
    int windowPosY = a_windowPosY;
    int windowWidth = a_windowWidth;
    int windowHeight = a_windowHeight;
    double scaleFactor = 1.0;

    // adjust values when passive stereo is used
    if (m_stereoMode == C_STEREO_PASSIVE_LEFT_RIGHT)
    {
        double center = 0.5 * windowWidth;
        if (windowPosX > center)
        {
            windowPosX = windowPosX - center;
        }
        windowWidth = center;
        scaleFactor = 2.0;
    }
    else if (m_stereoMode == C_STEREO_PASSIVE_TOP_BOTTOM)
    {
        double center = 0.5 * windowHeight;
        if (windowPosY > center)
        {
            windowPosY = windowPosY - center;
        }
        windowHeight = center;
        scaleFactor = 0.5;
    }

    // adjust values when image is mirrored horizontally
    if (m_mirrorHorizontal)
    {
        windowPosX = windowWidth - windowPosX;
    }

    // adjust values when image is mirrored vertically
    if (m_mirrorVertical)
    {
        windowPosY = windowHeight - windowPosY;
    }

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
        double distCam = (windowHeight / 2.0f) / cTanDeg(m_fieldViewAngle / 2.0f);

        cVector3d selectRay;
        selectRay.set(-distCam,
                      scaleFactor * (windowPosX - (windowWidth / 2.0f)),
                      ((windowHeight / 2.0f) - windowPosY));
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
        double hw = (double)(windowWidth) * 0.5;
        double hh = (double)(windowHeight)* 0.5;
        double aspect = a_windowWidth / a_windowHeight;
        
        double offsetX = ((windowPosX - hw) / hw) * 0.5 * m_orthographicWidth;
        double offsetY =-((windowPosY - hh) / hh) * 0.5 * (m_orthographicWidth / aspect);

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


//==============================================================================
/*!
    Set up the OpenGL perspective projection matrix, and nukes the contents
    of the GL buffers.  This function assumes the caller (typically cViewport)
    has set the appropriate buffer to be current.

    \param  a_windowWidth  Width of viewport.
    \param  a_windowHeight  Height of viewport.
*/
//==============================================================================
void cCamera::renderView(const int a_windowWidth, 
                         const int a_windowHeight)
{
#ifdef C_USE_OPENGL

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
                        if (light->updateShadowMap(m_scaleH, m_scaleV))
                        {
                            // add shadow map to list
                            shadowMaps.push_back(light->m_shadowMap);

                            // shadow mapping is used by at least one light source!
                            useShadowCasting = true;
                        }
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
    if ((m_stereoMode != C_STEREO_DISABLED) && (m_perspectiveMode))
    {
        /////////////////////////////////////////////////////////////////////
        // ACTIVE STEREO
        /////////////////////////////////////////////////////////////////////
        if (m_stereoMode == C_STEREO_ACTIVE)
        {
            // verify if stereo is available by the graphics hardware and camera is of perspective model
            glGetBooleanv(GL_STEREO, &stereo); 

            if (stereo)
            {
                // stereo is available - we shall perform 2 rendering passes for LEFT and RIGHT eye.
                numStereoPass = 2;
            }  
        }

        /////////////////////////////////////////////////////////////////////
        // PASSIVE STEREO (LEFT/RIGHT)
        /////////////////////////////////////////////////////////////////////
        else if (m_stereoMode == C_STEREO_PASSIVE_LEFT_RIGHT)
        {
            stereo = true;
            numStereoPass = 2;
        }

        /////////////////////////////////////////////////////////////////////
        // PASSIVE STEREO (TOP/BOTTOM)
        /////////////////////////////////////////////////////////////////////
        else if (m_stereoMode == C_STEREO_PASSIVE_TOP_BOTTOM)
        {
            stereo = true;
            numStereoPass = 2;
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
            /////////////////////////////////////////////////////////////////
            // LEFT EYE
            /////////////////////////////////////////////////////////////////
            if (i == 0)
            {
                switch (m_stereoMode)
                {
                    case C_STEREO_ACTIVE:
                        glViewport(0, 0, a_windowWidth, a_windowHeight);
                        glDrawBuffer(GL_BACK_LEFT);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        break;

                    case C_STEREO_PASSIVE_LEFT_RIGHT:
                        glViewport(0, 0, a_windowWidth/2, a_windowHeight);
                        glDrawBuffer(GL_BACK);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        break;

                    case C_STEREO_PASSIVE_TOP_BOTTOM:
                        glViewport(0, 0, a_windowWidth, a_windowHeight/2);
                        glDrawBuffer(GL_BACK);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        break;
                    
                    default: break;
                }
            }

            /////////////////////////////////////////////////////////////////
            // RIGHT EYE
            /////////////////////////////////////////////////////////////////
            else
            {
                switch (m_stereoMode)
                {
                    case C_STEREO_ACTIVE:
                        glDrawBuffer(GL_BACK_RIGHT);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        break;

                    case C_STEREO_PASSIVE_LEFT_RIGHT:
                        glViewport(a_windowWidth/2, 0, a_windowWidth/2, a_windowHeight);
                        break;

                    case C_STEREO_PASSIVE_TOP_BOTTOM:
                        glViewport(0, a_windowHeight/2, a_windowWidth, a_windowHeight/2);
                        break;
                    
                    default: break;
                }
            }
        }
        else
        {
            glViewport(0, 0, a_windowWidth, a_windowHeight); 
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glDrawBuffer(GL_BACK);
        }

        //-------------------------------------------------------------------
        // (4.2) SETUP GENERAL RENDERING SETTINGS
        //-------------------------------------------------------------------

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

            // adjust display for mirroring
            glScalef (m_scaleH, m_scaleV, 1.0);
            if (m_mirrorStatus)
            {
                glFrontFace(GL_CW);
            }
            else
            {
                glFrontFace(GL_CCW);
            }

            // create projection matrix depending of camera mode
            if (m_perspectiveMode)
            {
                // setup perspective camera
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

            // setup camera position
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
            double stereo_multiplier = (i == 0) ? -1.0f : 1.0f;  // (i == 0) correspond to LEFT IMAGE, (i == 1) correspond to RIGHT IMAGE

            double left   = -1.0 * glAspect * wd2 + stereo_multiplier * 0.5 * m_stereoEyeSeparation * ndfl;
            double right  =        glAspect * wd2 + stereo_multiplier * 0.5 * m_stereoEyeSeparation * ndfl;
            double top    =        wd2;
            double bottom = -1.0 * wd2;

            // setup projection matrix
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            // adjust display for mirroring
            glScalef (m_scaleH, m_scaleV, 1.0);
            if (m_mirrorStatus)
            {
                glFrontFace(GL_CW);
            }
            else
            {
                glFrontFace(GL_CCW);
            }

            glFrustum(left, right, bottom, top, m_distanceNear, m_distanceFar);

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
                //--------------------------------------------------------------
                // OPAQUE OBJECTS
                //--------------------------------------------------------------

                // setup rendering options
                options.m_camera                                = this;
                options.m_single_pass_only                      = false;
                options.m_render_opaque_objects_only            = true;
                options.m_render_transparent_front_faces_only   = false;
                options.m_render_transparent_back_faces_only    = false;
                options.m_enable_lighting                       = true;
                options.m_render_materials                      = true;
                options.m_render_textures                       = true;
                options.m_creating_shadow_map                   = false;
                options.m_rendering_shadow                      = true;
                options.m_shadow_light_level                    = 1.0 - m_parentWorld->getShadowIntensity();
                options.m_storeObjectPositions                  = false;
                options.m_resetDisplay                          = m_resetDisplay;

                // render 1st pass (opaque objects - shadowed regions)
                m_parentWorld->renderSceneGraph(options);

                // setup rendering options
                options.m_rendering_shadow                      = false;
                options.m_shadow_light_level                    = 1.0;

                // render 2nd pass (opaque objects - non shadowed regions)
                list<cShadowMap*>::iterator lst;
                for(lst = shadowMaps.begin(); lst !=shadowMaps.end(); ++lst)
                {
                    cShadowMap *shadowMap = *lst;
                    shadowMap->render(options);

                    m_parentWorld->renderSceneGraph(options);

                    // restore states
                    glActiveTexture(GL_TEXTURE0_ARB);
                    glDisable(GL_TEXTURE_2D);
                    glDisable(GL_TEXTURE_GEN_S);
                    glDisable(GL_TEXTURE_GEN_T);
                    glDisable(GL_TEXTURE_GEN_R);
                    glDisable(GL_TEXTURE_GEN_Q);
                    glDisable(GL_ALPHA_TEST);
                }

                //--------------------------------------------------------------
                // TRANSPARENT OBJECTS
                //--------------------------------------------------------------

                // setup rendering options
                options.m_render_opaque_objects_only            = false;
                options.m_render_transparent_back_faces_only    = true;
                options.m_render_transparent_front_faces_only   = false;
                options.m_rendering_shadow                      = false;

                // render 3rd pass (transparent objects - back faces only)
                m_parentWorld->renderSceneGraph(options);

                // modify rendering options for third pass
                options.m_render_opaque_objects_only            = false;
                options.m_render_transparent_back_faces_only    = false;
                options.m_render_transparent_front_faces_only   = true;
                options.m_rendering_shadow                      = true;
                options.m_shadow_light_level                    = 1.0 - m_parentWorld->getShadowIntensity();

                // render 4th pass (transparent objects - front faces only - shadowed areas)
                m_parentWorld->renderSceneGraph(options);
                
                for(lst = shadowMaps.begin(); lst !=shadowMaps.end(); ++lst)
                {
                    cShadowMap *shadowMap = *lst;
                    shadowMap->render(options);

                    m_parentWorld->renderSceneGraph(options);

                    // restore states
                    glActiveTexture(GL_TEXTURE0_ARB);
                    glDisable(GL_TEXTURE_2D);
                    glDisable(GL_TEXTURE_GEN_S);
                    glDisable(GL_TEXTURE_GEN_T);
                    glDisable(GL_TEXTURE_GEN_R);
                    glDisable(GL_TEXTURE_GEN_Q);
                    glDisable(GL_ALPHA_TEST);
                }


                // modify rendering options for 5th pass
                options.m_rendering_shadow						= false;
                options.m_shadow_light_level					= 1.0;

                // render 5th pass (transparent objects - front faces only - lighted regions)
                for(lst = shadowMaps.begin(); lst !=shadowMaps.end(); ++lst)
                {
                    cShadowMap *shadowMap = *lst;
                    shadowMap->render(options);

                    m_parentWorld->renderSceneGraph(options);

                    // restore states
                    glActiveTexture(GL_TEXTURE0_ARB);
                    glDisable(GL_TEXTURE_2D);
                    glDisable(GL_TEXTURE_GEN_S);
                    glDisable(GL_TEXTURE_GEN_T);
                    glDisable(GL_TEXTURE_GEN_R);
                    glDisable(GL_TEXTURE_GEN_Q);
                    glDisable(GL_ALPHA_TEST);
                }
            }


            ////////////////////////////////////////////////////////////////////
            // MULTI PASS - WITHOUT SHADOWS
            ////////////////////////////////////////////////////////////////////
            else
            {
                // setup rendering options for first pass
                options.m_camera                                = this;
                options.m_single_pass_only                      = false;
                options.m_render_opaque_objects_only            = true;
                options.m_render_transparent_front_faces_only   = false;
                options.m_render_transparent_back_faces_only    = false;
                options.m_enable_lighting                       = true;
                options.m_render_materials                      = true;
                options.m_render_textures                       = true;
                options.m_creating_shadow_map                   = false;
                options.m_rendering_shadow                      = false;
                options.m_shadow_light_level                    = 1.0;
                options.m_storeObjectPositions                  = true;
                options.m_resetDisplay                          = m_resetDisplay;

                // render 1st pass (opaque objects - all faces)
                m_parentWorld->renderSceneGraph(options);

                // modify rendering options
                options.m_render_opaque_objects_only            = false;
                options.m_render_transparent_back_faces_only    = true;
                options.m_storeObjectPositions                  = false;

                // render 2nd pass (transparent objects - back faces only)
                m_parentWorld->renderSceneGraph(options);

                // modify rendering options
                options.m_render_transparent_back_faces_only    = false;
                options.m_render_transparent_front_faces_only   = true;

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
                options.m_single_pass_only                      = true;
                options.m_render_opaque_objects_only            = true;
                options.m_render_transparent_front_faces_only   = false;
                options.m_render_transparent_back_faces_only    = false;
                options.m_enable_lighting                       = true;
                options.m_render_materials                      = true;
                options.m_render_textures                       = true;
                options.m_creating_shadow_map                   = false;
                options.m_rendering_shadow                      = true;
                options.m_shadow_light_level                    = 1.0 - m_parentWorld->getShadowIntensity();
                options.m_storeObjectPositions                  = false;
                options.m_resetDisplay                          = m_resetDisplay;

                // render 1st pass (opaque objects - all faces - shadowed regions)
                m_parentWorld->renderSceneGraph(options);

                // setup rendering options
                options.m_rendering_shadow                      = false;
                options.m_shadow_light_level                    = 1.0;

                // render 2nd pass (opaque objects - all faces - lighted regions)
                list<cShadowMap*>::iterator lst;
                for(lst = shadowMaps.begin(); lst !=shadowMaps.end(); ++lst)
                {
                    cShadowMap *shadowMap = *lst;
                    shadowMap->render(options);

                    m_parentWorld->renderSceneGraph(options);

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
                options.m_render_opaque_objects_only            = false;
                options.m_render_transparent_front_faces_only   = true;
                options.m_render_transparent_back_faces_only    = true;

                // render 3rd pass (transparent objects - all faces)
                m_parentWorld->renderSceneGraph(options);
            }


            ////////////////////////////////////////////////////////////////////
            // SINGLE PASS - WITHOUT SHADOWS
            ////////////////////////////////////////////////////////////////////
            else
            {
                // setup rendering options for single pass
                options.m_camera                                = this;
                options.m_single_pass_only                      = true;
                options.m_render_opaque_objects_only            = false;
                options.m_render_transparent_front_faces_only   = false;
                options.m_render_transparent_back_faces_only    = false;
                options.m_enable_lighting                       = true;
                options.m_render_materials                      = true;
                options.m_render_textures                       = true;
                options.m_creating_shadow_map                   = false;
                options.m_rendering_shadow                      = false;
                options.m_shadow_light_level                    = 1.0;
                options.m_storeObjectPositions                  = true;
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
#endif
}


//==============================================================================
/*!
    Copies the OpenGL image buffer to a cImage class structure.

    \param  a_image  Destination image
*/
//==============================================================================
void cCamera::copyImageData(cImage* a_image)
{
#ifdef C_USE_OPENGL

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

#endif
}


//==============================================================================
/*!
    Enable or disable multipass transparency... when this option is
    enabled (it's disabled by default), each time the camera is
    asked to render the scene, it will perform three rendering
    passes: a pass for non-transparent items, a pass for the back faces
    of transparent items, and a pass for the front faces of transparent
    items.

    Objects being rendered are told which pass is current via the 
    parameter supplied to the render() function.

    We will hopefully find a cleaner way to do this in the future, but
    for now be careful when you enable this feature...

    \param  a_enabled  If __true__, multipass is enabled, __false__ otherwise..
*/
//==============================================================================
void cCamera::setUseMultipassTransparency(bool a_enabled) 
{
    m_useMultipassTransparency = a_enabled;
}


//==============================================================================
/*!
    Enable or disable shadow casting.

    \param  a_enabled  If __true__, shadow casting is enabled, __false__ otherwise.
*/
//==============================================================================
void cCamera::setUseShadowCasting(bool a_enabled)
{
    m_useShadowCasting = a_enabled;
}


//==============================================================================
/*!
    Verifies if shadow casting is supported on this hardware.

    \return Return __true__ if shadow casting is supported, otherwise __false__.
*/
//==============================================================================
bool cCamera::isShadowCastingSupported()
{
#ifdef C_USE_OPENGL
    //Check for necessary OpenGL extensions
    if(!GLEW_ARB_depth_texture || 
       !GLEW_ARB_shadow)
    {
        return (false);
    }
    else
    {
        return (true);
    }
#else
    return (false);
#endif
}


//==============================================================================
/*!
    This call automatically adjusts the front and back clipping planes to
    optimize usage of the z-buffer.
*/
//==============================================================================
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


//==============================================================================
/*!
    Render a 2d scene within the viewport.

    \param  a_graph  The root of the 2d scenegraph to be rendered.
    \param  a_width  The size of the rendering window
    \param  a_height The size of the rendering window
*/
//==============================================================================
void cCamera::renderLayer(cGenericObject* a_graph, 
                          int a_width, 
                          int a_height)
{
#ifdef C_USE_OPENGL

    // set up an orthographic projection matrix
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    // adjust display for mirroring
    glScalef (m_scaleH, m_scaleV, 1.0);
    if (m_mirrorStatus)
    {
        glFrontFace(GL_CW);
    }
    else
    {
        glFrontFace(GL_CCW);
    }

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
    // non lighting sensitive.
    glDisable(GL_LIGHTING);

    // setup rendering options for single pass
    cRenderOptions options;
    options.m_camera                                = this;
    options.m_single_pass_only                      = true;
    options.m_render_opaque_objects_only            = false;
    options.m_render_transparent_front_faces_only   = false;
    options.m_render_transparent_back_faces_only    = false;
    options.m_enable_lighting                       = true;
    options.m_render_materials                      = true;
    options.m_render_textures                       = true;
    options.m_creating_shadow_map                   = false;
    options.m_rendering_shadow                      = false;
    options.m_shadow_light_level                    = 1.0;
    options.m_storeObjectPositions                  = true;
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

#endif
}


//==============================================================================
/*!
    Called by the user or by the viewport when the world needs to have
    textures and display lists reset (e.g. after a switch to or from
    fullscreen).
*/
//==============================================================================
void cCamera::onDisplayReset() 
{
    m_resetDisplay = true;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
