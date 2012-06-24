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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 828 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
//---------------------------------------------------------------------------
#include "display/CViewport.h"
//---------------------------------------------------------------------------
#include "GL/glu.h"
//---------------------------------------------------------------------------
cViewport* cViewport::lastActiveViewport = 0;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cViewport.

    \fn         cViewport::cViewport(HWND a_winHandle, cCamera *a_camera,
                const bool a_stereoEnabled, PIXELFORMATDESCRIPTOR* a_pixelFormat=0)
    \param      a_winHandle    Handle to the actual win32 window.
    \param      a_camera       The camera through which this viewport should be rendered.
    \param      a_stereoEnabled    If \b true, a stereo rendering context is created.
    \param      a_pixelFormat  If non-zero, this custom pixel format is used 
                               to initialize the viewport.
*/
//===========================================================================
cViewport::cViewport(HWND a_winHandle, cCamera *a_camera, const bool a_stereoEnabled, PIXELFORMATDESCRIPTOR* a_pixelFormat)
{

    // If no viewport has been created at all, creation is enough to make this
    // the active viewport
    if (lastActiveViewport == 0) lastActiveViewport = this;

    memset(m_glViewport,0,sizeof(m_glViewport));

    // set the camera through which this viewport should be rendered
    setCamera(a_camera);

    // stereo status
    m_stereoEnabled = a_stereoEnabled;

    // update wincontrol
    m_winHandle = a_winHandle;

    // No post-render callback by default (see setPostRenderCallback() for details)
    m_postRenderCallback = 0;

    // ----------------------------
    // Initialize an OpenGL context
    // ----------------------------

    m_glDC = 0;

    // If the user requested a specific pixel format, use that as our
    // requested format for initializing the display context
    if (a_pixelFormat != 0)
    {
        m_pixelFormat = *a_pixelFormat;
    }

    // Otherwise use a default format descriptor...
    else
    {
        PIXELFORMATDESCRIPTOR pfd = {
          sizeof(PIXELFORMATDESCRIPTOR),       // size of this pfd
          1,                                   // version number
          PFD_DRAW_TO_WINDOW |                 // support window
          PFD_SUPPORT_OPENGL |                 // support OpenGL
          (m_stereoEnabled ? PFD_STEREO : 0) | // optionally enable stereo
          PFD_DOUBLEBUFFER,                    // double buffered
          PFD_TYPE_RGBA,                       // RGBA type
          32,                                  // 32-bit color depth
          0, 0, 0, 0, 0, 0,                    // color bits ignored
          0,                                   // no alpha buffer
          0,                                   // shift bit ignored
          0,                                   // no accumulation buffer
          0, 0, 0, 0,                          // accum bits ignored
          32,                                  // 32-bit z-buffer
          0,                                   // no stencil buffer
          0,                                   // no auxiliary buffer
          PFD_MAIN_PLANE,                      // main layer
          0,                                   // reserved
          0, 0, 0                              // layer masks ignored
        };

        m_pixelFormat = pfd;
    }

    m_forceRenderArea.left   = -1;
    m_forceRenderArea.right  = -1;
    m_forceRenderArea.top    = -1;
    m_forceRenderArea.bottom = -1;

    if (m_winHandle != NULL)
    {
        // This actually creates the context...
        update();
    }
}


//===========================================================================
/*!
        Destructor of cViewport.

        \fn     cViewport::~cViewport()
*/
//===========================================================================
cViewport::~cViewport()
{
    cleanup();
}


//===========================================================================
/*!
      Enable or disable stereo rendering on this viewport. \n

      Note that it is not possible to change the pixel format of a window
      in Windows, so if you create a viewport that doesn't have stereo support,
      you can't enable stereo rendering without creating a new window/viewport.

      \fn     cViewport::setStereoOn(bool a_stereoEnabled)
*/
//===========================================================================
void cViewport::setStereoOn(bool a_stereoEnabled)
{
    // check if new mode is not already active
    if (a_stereoEnabled == m_stereoEnabled) { return; }

    // update stereo rendering state
    m_stereoEnabled = a_stereoEnabled;

    // See whether stereo is _really_ enabled
    PIXELFORMATDESCRIPTOR pfd;
    int formatIndex = GetPixelFormat(m_glDC);
    DescribePixelFormat(m_glDC, formatIndex, sizeof(PIXELFORMATDESCRIPTOR), &pfd);

    // if stereo was enabled but can not be displayed, switch over to mono.
    if (((pfd.dwFlags & PFD_STEREO) == 0) && m_stereoEnabled)
    {
        m_stereoEnabled = false;
    }
}


//===========================================================================
/*!
        Clean up the current rendering context.

        \fn     bool cViewport::cleanup()
*/
//===========================================================================
bool cViewport::cleanup()
{
    bool status = true;

    // delete display context
    int result = ReleaseDC(m_winHandle, m_glDC);
    if (result == 0) status = false;

    result = wglDeleteContext(m_glContext);
    if (result == 0) status = false;

    m_glContext = 0;
    m_glDC = 0;
    m_glReady = false;
    return status;
}


//===========================================================================
/*!
        If the window has been modified, or just created, call this function
        to update the OpenGL display context.

        \fn         bool cViewport::update(bool resizeOnly)
        \param      resizeOnly  If false (default), reinitializes the GL context.
        \return     Return true if operation succeeded.
*/
//===========================================================================
bool cViewport::update(bool resizeOnly)
{

    // Clean up the old rendering context if necessary
    if ((resizeOnly == false) && m_glDC) cleanup();

    // declare variables
    int formatIndex;

    // viewport is not yet enabled
    m_enabled = false;

    // gl display not yet ready
    m_glReady = false;

    // check display handle
    if (m_winHandle == NULL) { return (false); }

    // Find out the rectangle to which we should be rendering

    // If we're using the entire window...
    if (m_forceRenderArea.left == -1)
    {
        if (GetWindowRect(m_winHandle, &m_activeRenderingArea) == 0) { return (false); }

        // Convert from screen to window coordinates
        m_activeRenderingArea.right -= m_activeRenderingArea.left;
        m_activeRenderingArea.left = 0;

        m_activeRenderingArea.bottom -= m_activeRenderingArea.top;
        m_activeRenderingArea.top = 0;

        // Convert from y-axis-down to y-axis-up, since that's how we store
        // our rendering area.
        int height = m_activeRenderingArea.bottom;
        m_activeRenderingArea.top = height - m_activeRenderingArea.top;
        m_activeRenderingArea.bottom = height - m_activeRenderingArea.bottom;
    }

    // Otherwise use whatever rectangle the user wants us to use...
    else
    {
        m_activeRenderingArea = m_forceRenderArea;
    }

    // retrieve handle of the display device context
    m_glDC = ::GetDC(m_winHandle);

    if (m_glDC == 0)
    {
       return(false);
    }

    if (resizeOnly == false)
    {
        // find pixel format supported by the device context. If error return false.
        formatIndex = ChoosePixelFormat(m_glDC, &m_pixelFormat);
        if (formatIndex == 0)
        {
            return(false);
        }

        // sets the specified device context's pixel format. If error return false
        if (!SetPixelFormat(m_glDC, formatIndex, &m_pixelFormat))
        {
            return(false);
        }

        formatIndex = GetPixelFormat (m_glDC);
        DescribePixelFormat (m_glDC, formatIndex, sizeof(PIXELFORMATDESCRIPTOR), &m_pixelFormat);

        // if stereo was enabled but can not be displayed, switch over to mono.
        if (((m_pixelFormat.dwFlags & PFD_STEREO) == 0) && m_stereoEnabled)
        {
            m_stereoEnabled = false;
        }

        // create display context
        m_glContext = wglCreateContext(m_glDC);
        if (m_glContext == 0)
        {
            return(false);
        }

        wglMakeCurrent(m_glDC, m_glContext);
    }

    // OpenGL is now ready for rendering
    m_glReady = true;

    // store this current view as the last active one
    lastActiveViewport = this;

    // enable viewport
    m_enabled = true;

    if (resizeOnly == false) onDisplayReset();

    // return success
    return(true);
}


//===========================================================================
/*!
    Call this method to render the OpenGL world inside the viewport.

    \fn         bool cViewport::render(int imageIndex)
    \return     Return \b true if operation succeeded.
*/
//===========================================================================
bool cViewport::render()
{
    lastActiveViewport = this;
    bool result = renderView();
    return (result);
}


//===========================================================================
/*!
    Renders the OpenGL scene.

    \fn         void cViewport::renderView(const int a_imageIndex)
    \return     Return \b true if operation succeeded.
*/
//===========================================================================
bool cViewport::renderView()
{
    // Make sure the viewport is really ready for rendering
    if ( (m_glReady == 0) || (m_enabled == 0) || (m_camera == NULL) ) return false;

    // Find out whether we need to update the size of our viewport...

    // If we're using the whole window, see whether the window has
    // changed size...
    if (m_forceRenderArea.left == -1)
    {
        RECT sizeWin;
        if (GetWindowRect(m_winHandle, &sizeWin) == 0) { return (false); }

        unsigned int width   = sizeWin.right  - sizeWin.left;
        unsigned int height  = sizeWin.bottom - sizeWin.top;

        if (
            (m_activeRenderingArea.left   != 0)     ||
            (m_activeRenderingArea.bottom != 0)     ||
            (m_activeRenderingArea.right  != (int)width) ||
            (m_activeRenderingArea.top    != (int)height)
           )
        {
            update();
        }
    }

    // Otherwise the user is telling us to use a particular rectangle; see
    // whether that rectangle has changed...
    else
    {
        if ( (m_activeRenderingArea.left   != m_forceRenderArea.left)  ||
             (m_activeRenderingArea.right  != m_forceRenderArea.right) ||
             (m_activeRenderingArea.top    != m_forceRenderArea.top)   ||
             (m_activeRenderingArea.bottom != m_forceRenderArea.bottom) )
        {
            update();
        }
    }

    // Activate display context
    //
    // Note that in the general case, this is not strictly necessary,
    // but if a user is using multiple viewports, we don't want him
    // to worry about the current rendering context, so we incur a bit
    // of overhead here.
    if (!wglMakeCurrent(m_glDC, m_glContext))
    {

        // Try once to re-initialize the context...
        if (!(update()))
          // And return an error if this doesn't work out...
          return(false);
    }

    // set viewport size
    int width = m_activeRenderingArea.right - m_activeRenderingArea.left;
    int height = m_activeRenderingArea.top - m_activeRenderingArea.bottom;
    glViewport(m_activeRenderingArea.left, 
               m_activeRenderingArea.bottom,
               width,
               height);

    glGetIntegerv(GL_VIEWPORT,m_glViewport);

    // render world
    m_camera->renderView(width, height);

    // swap buffers
    SwapBuffers(m_glDC);
    
    // deactivate display context (not necessary)
    // wglMakeCurrent(m_glDC, 0);

    // operation succeeded
    return (true);
}


//===========================================================================
/*!
     Select an object on displayed in the viewport. This method casts a
     virtual ray through the viewport and asks the world for the first
     object hit by that ray. \n

     It's most useful if you want to allow the user to use the mouse to
     click on objects in your virtual scene. \n

     Use getLastSelectedObject(), getLastSelectedTriangle(), and
     getLastSelectedPoint() to extract information about the results of
     this operation. \n

     \fn        bool cViewport::select(const unsigned int a_windowPosX,
                       const unsigned int a_windowPosY,
                       cCollisionSettings* a_collisionSettings)
     \param     a_windowPosX  X coordinate position of mouse click.
     \param     a_windowPosY  Y coordinate position of mouse click.
     \param     a_collisionSettings  Settings used for performing collision detection.
     \return    Return \b true if an object has been hit.
*/
//===========================================================================
bool cViewport::select(const unsigned int a_windowPosX,
                       const unsigned int a_windowPosY,
                       cCollisionSettings* a_collisionSettings)
{
    // check if camera is valid
    if (m_camera == 0) return false;

    // compute width and height of rendering area in scene
    int width = m_activeRenderingArea.right - m_activeRenderingArea.left;
    int height = m_activeRenderingArea.top - m_activeRenderingArea.bottom;

    // apply some default collision settings in none have been specified
    cCollisionSettings collisionSettings;

    if (a_collisionSettings == NULL)
    {
        collisionSettings.m_checkVisibleObjects = true;
        collisionSettings.m_checkHapticObjects = false;
        collisionSettings.m_collisionRadius = 0.0;
    }
    else
    {
        collisionSettings = *a_collisionSettings;
    }

    // search for intersection between ray and objects in world
    bool result = m_camera->select( a_windowPosX,
                                    a_windowPosY,
                                    width,
                                    height,
                                    m_collisionRecorder,
                                    collisionSettings
                                );

    // return result. True if and object has been hit, else false.
    return(result);
}


//===========================================================================
/*!
    Set camera. The viewport will now display the image filmed by this
    virtual camera.

    \fn     void cViewport::setCamera(cCamera *a_camera)
    \param  a_camera  Virtual camera in world.
*/
//===========================================================================
void cViewport::setCamera(cCamera *a_camera)
{
    // set camera
    m_camera = a_camera;
}


//===========================================================================
/*!
    You can use this to specify a specific rectangle to which you want this
    viewport to render within the window.  Supply -1 for each coordinate
    to return to the default behavior (rendering to the whole window).
    The \e _positive_ y axis goes \e _up_.

    \fn     void cViewport::setRenderArea(RECT& r)
    \param  r  The rendering area within the GL context
*/
//===========================================================================
void cViewport::setRenderArea(RECT& r)
{
    m_forceRenderArea = r;
}


//===========================================================================
/*!
    Clients should call this when the scene associated with
    this viewport may need re-initialization, e.g. after a
    switch to or from fullscreen.  Automatically called from update()
    when the viewport creates a new GL context.

    \fn     void cViewport::onDisplayReset()
*/
//===========================================================================
void cViewport::onDisplayReset()
{
    if (m_camera) m_camera->onDisplayReset();
}


//===========================================================================
/*!
    Project a world-space point from 3D to 2D, using my viewport xform, my
    camera's projection matrix, and his world's modelview matrix.

    \fn     void cViewport::projectPoint(cVector3d& a_point)
    \param  a_point     The point to transform
    \return             The transformed point in window space
*/
//===========================================================================
cVector3d cViewport::projectPoint(cVector3d& a_point)
{
    cVector3d result;

    int* viewport = m_glViewport;
	double* projection = m_camera->m_projectionMatrix.getData();
    double* modelview = m_camera->getParentWorld()->m_worldModelView;

    gluProject(a_point(0) , a_point(1) , a_point(2) ,
               modelview, 
			   projection,
               viewport,
			   &result(0) ,&result(1) ,&result(2) );

    return (result);
}

//---------------------------------------------------------------------------
#endif // WIN32
//---------------------------------------------------------------------------
