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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 995 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CViewportH
#define CViewportH
//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
//------------------------------------------------------------------------------
#include "world/CWorld.h"
#include "display/CCamera.h"
#include "math/CMatrix3d.h"
#include "math/CVector3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file	    CViewport.h
    \brief  
    <b> Viewport </b> \n 
    General Viewport Display.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cViewport
    \ingroup    display  

    \brief

    cViewport describes a two-dimensional window for rendering an OpenGL scene.
    Basically this class encapsulates an OpenGL rendering context.  Creating
    a window is left to the application programmer, since that will depend
    on the development environment that you're using.  Once you have a window
    handle, use this class to bind it to an OpenGL context.

    Typically a cViewport is connected to a cCamera for rendering, and a cCamera
    is typically connected to a cWorld, where objects actually live.

*/
//==============================================================================
class cViewport
{
  public:
    
     //---------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //---------------------------------------------------------------------

    //! Constructor of cViewport
    cViewport(HWND a_winHandle,
              cCamera *a_camera,
              const bool a_stereoEnabled = false,
              PIXELFORMATDESCRIPTOR* a_pixelFormat = 0);

    //! Destructor of cViewport.
    ~cViewport();


    //---------------------------------------------------------------------
    // GENERAL SEETINGS
    //---------------------------------------------------------------------

    //! Get height of active viewport area.
    unsigned int getHeight() const { return (m_activeRenderingArea.top - m_activeRenderingArea.bottom); }

    //! Get width of active viewport area.
    unsigned int getWidth() const { return (m_activeRenderingArea.right - m_activeRenderingArea.left); }

    //! Set the camera through which this viewport will be rendered.
    void setCamera(cCamera *a_camera);

    //! Get the camera through which this viewport is being rendered.
    cCamera* getCamera() const { return (m_camera); }

    //! Enable or disable rendering of this viewport.
    void setEnabled( const bool& a_enabled ) { m_enabled = a_enabled; }

    //! Get the rendering status of this viewport.
    bool getEnabled() const { return (m_enabled); }


    //---------------------------------------------------------------------
    // STEREO DISPLAY SUPPORT
    //---------------------------------------------------------------------

    //! Enable or disable stereo rendering.
    void setStereoOn(bool a_stereoEnabled);

    //! Is stereo rendering enabled?
    bool getStereoOn() const { return (m_stereoEnabled); }


    //---------------------------------------------------------------------
    // MOUSE SELECTION
    //---------------------------------------------------------------------

    //! Tell the viewport to figure out whether the (x,y) viewport coordinate is within a visible object.
    bool select(const unsigned int a_windowPosX,
                const unsigned int a_windowPosY,
                cCollisionSettings* a_collisionSettings = NULL);
                
    //! Get last selected mesh.
    cGenericObject* getLastSelectedObject() { return (m_collisionRecorder.m_nearestCollision.m_object); }

    //! Get last selected triangle.
    cTriangle* getLastSelectedTriangle() { return (m_collisionRecorder.m_nearestCollision.m_triangle); }

    //! Get last selected point position.
    cVector3d getLastSelectedPoint(void) { return(m_collisionRecorder.m_nearestCollision.m_globalPos); }

    //! Get last selected point normal.
    cVector3d getLastSelectedPointNormal(void) { return(m_collisionRecorder.m_nearestCollision.m_globalNormal);}

    //! Get distance to last selected object.
    double getLastSelectedDistance(void) { return (sqrt(m_collisionRecorder.m_nearestCollision.m_squareDistance)); }

    //! Last collision events with mouse.
    cCollisionRecorder m_collisionRecorder;


    //---------------------------------------------------------------------
    // RENDERING
    //---------------------------------------------------------------------

    //! Call this method to render the scene in OpenGL
    bool render();

    /*!
        Clients should call this when the scene associated with this viewport 
        may need re-initialization, e.g. after a switch to or from fullscreen.
    */
    virtual void onDisplayReset();

    //! Return a direct handle to the OpenGL viewing context.
    HDC getGLDC() { return m_glDC; }

    //! Returns the pixel format used by this viewport.
    const PIXELFORMATDESCRIPTOR* getPixelFormat() { return (&m_pixelFormat); }

    //! Reconfigures the display context.
    bool update(bool resizeOnly=false);

    //! It's useful to store the last viewport transformation, for gluProject'ing things.
    int m_glViewport[4];


    //---------------------------------------------------------------------
    // SUB AREA RENDERING
    //---------------------------------------------------------------------

    /*!
		You can use this to specify a specific rectangle to which you want this
		viewport to render within the window.  Supply -1 for each coordinate
		to return to the default behavior (rendering to the whole window).
		The _positive_ y axis goes _up_.
	*/
    void setRenderArea(RECT& r);
    
	//! Read the rendering area.
	void getRenderArea(RECT& r) { r = this->m_forceRenderArea; }

	//! Return the last activated viewport. (Last Viewport for which the render() function was called).
    static cViewport* getLastActiveViewport() { return lastActiveViewport; }

    /*! 
		Project a world-space point from 3D to 2D, using my viewport xform, my
		camera's projection matrix, and his world's modelview matrix.
	*/
    cVector3d projectPoint(cVector3d& a_point);


  protected:

    //---------------------------------------------------------------------
    // GENERAL PROPERTIES:
    //---------------------------------------------------------------------

    //!  Virtual camera connected to this viewport.
    cCamera* m_camera;

    //! Status of viewport.
    bool m_enabled;

    //! Stereo status.
    bool m_stereoEnabled;

    //! Handle to window display.
    HWND m_winHandle;

    //! OpenGL display context.
    HGLRC m_glContext;

    //! display context.
    HDC m_glDC;

    //! GL Status.
    bool m_glReady;

	/*!
		The rectangle to which we're rendering within the GL window, equal to
		the window size by default.  The _positive_ y axis goes _up_.
	*/
    RECT m_activeRenderingArea;

	/*!
		If we're forcing rendering to a particular rectangle within the viewport,
		this rectangle contains those coordinates.  Otherwise all coordinates are
		-1, which tells cViewport to use the whole window.  The _positive_ y axis
		goes _up_.
	*/
    RECT m_forceRenderArea;

    //! Descriptor of the context opened for OpenGL rendering
    PIXELFORMATDESCRIPTOR m_pixelFormat;

	/*!
		If non-zero, this object will get rendered immediately 
		before the GL buffer is swapped out, after all other.
	*/
    cGenericObject* m_postRenderCallback;

	/*!
		The most recent viewport to initiate rendering; useful
		for finding global opengl state information
	*/
    static cViewport* lastActiveViewport;


    //---------------------------------------------------------------------
    // PROTECTED METHODS:
    //---------------------------------------------------------------------

    //! Clean up the current rendering context.
    bool cleanup();

    //! Render the scene in OpenGL.
    virtual bool renderView();   
    
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif // WIN32
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

