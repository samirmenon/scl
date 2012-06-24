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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 493 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "lighting/CShadowMap.h"
//---------------------------------------------------------------------------
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    A texture contains a 2D bitmap which can be projected onto the
    polygons of a 3D solid.

    \fn         cShadowMap::cShadowMap()
    \return     Return a pointer to new texture instance.
*/
//===========================================================================
cShadowMap::cShadowMap()
{
    // initialize internal variables
    reset();    
	m_enabled = false;
}


//===========================================================================
/*!
    Destructor of cShadowMap.

    \fn         cShadowMap::~cShadowMap()
*/
//===========================================================================
cShadowMap::~cShadowMap()
{
    if (m_textureID != 0)
    {
        glDeleteTextures(1,&m_textureID);
        m_textureID = 0;
    }

    if (glIsFramebuffer(m_fbo))
    {
        glDeleteFramebuffers(1, &m_fbo);
        m_fbo = -1;
    }

	delete m_image;
}


//===========================================================================
/*!
    Reset internal variables. This function should be called only by constructors.

    \fn         void cShadowMap::reset()
*/
//===========================================================================
void cShadowMap::reset()
{
    // framebuffer
    if (glIsFramebuffer(m_fbo))
    {
        glDeleteFramebuffers(1, &m_fbo);
        m_fbo = -1;
    }
	
    // id number provided by OpenGL once texture is stored in graphics
    // card memory
    m_textureID = 0;

    // texture has not yet been rendered
    m_updateTextureFlag = true;

	// create image
	m_image = new cImage();

	// allocate size of map
	setSize(1024, 1024);
}


//===========================================================================
/*!
    Set the size of shadowmap by defining the width and height in pixel.
    It is generally recommended to define same values for width and height
    in powers of 2. For instance: (512x512, 1024x1024 or 2048x2048).

    \param  a_width     Width of image.
    \param  a_height    Height of image.
*/
//===========================================================================
void cShadowMap::setSize(const unsigned int a_width, const unsigned int a_height)
{
    if (m_image != NULL)
    {
        m_image->allocate(a_width, a_height, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT);
    }
}


//===========================================================================
/*!
      Enable texturing and set this texture as the current texture

      \fn       void cShadowMap::render(cRenderOptions& a_options)
	  \param	a_options  Rendering options.
*/
//===========================================================================
void cShadowMap::render(cRenderOptions& a_options)
{
	// check if shadow texture map has been initialized 
    if (m_image->isInitialized() == 0) return;

    // Only check residency in memory if we weren't going to
    // update the texture anyway...
    if (m_updateTextureFlag == false)
    {
        GLboolean texture_is_resident;
        glAreTexturesResident(1, &m_textureID, &texture_is_resident);

        if (texture_is_resident == false)
        {
            m_updateTextureFlag = true;
        }
    }

    // is texture being rendered for the first time?
    if (m_updateTextureFlag)
    {
        update();
        m_updateTextureFlag = false;
    }

	//Calculate texture matrix for projection
	//This matrix takes us from eye space to the light's clip space
	//It is postmultiplied by the inverse of the current view matrix when specifying texgen
	cTransform biasMatrix;
	biasMatrix.set(0.5f, 0.0f, 0.0f, 0.0f,
				   0.0f, 0.5f, 0.0f, 0.0f,
				   0.0f, 0.0f, 0.5f, 0.0f,
				   0.5f, 0.5f, 0.5f, 1.0f);	//bias from [-1, 1] to [0, 1]
	
	// textureMatrix = biasMatrix * lightProjectionMatrix * lightViewMatrix
    cTransform textureMatrix = biasMatrix * m_lightProjectionMatrix * m_lightViewMatrix;

	//Set up texture coordinate generation.
	double matRow0[4];
	matRow0[0] = textureMatrix.m[0][0];
	matRow0[1] = textureMatrix.m[1][0];
	matRow0[2] = textureMatrix.m[2][0];
	matRow0[3] = textureMatrix.m[3][0];

	double matRow1[4];
	matRow1[0] = textureMatrix.m[0][1];
	matRow1[1] = textureMatrix.m[1][1];
	matRow1[2] = textureMatrix.m[2][1];
	matRow1[3] = textureMatrix.m[3][1];
	
	double matRow2[4];
	matRow2[0] = textureMatrix.m[0][2];
	matRow2[1] = textureMatrix.m[1][2];
	matRow2[2] = textureMatrix.m[2][2];
	matRow2[3] = textureMatrix.m[3][2];
	
	double matRow3[4];
	matRow3[0] = textureMatrix.m[0][3];
	matRow3[1] = textureMatrix.m[1][3];
	matRow3[2] = textureMatrix.m[2][3];
	matRow3[3] = textureMatrix.m[3][3];

    ///////////////////////////////////////////////////////////////////////////

    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glActiveTextureARB(GL_TEXTURE0);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);

    glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_REPLACE);
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB, GL_PRIMARY_COLOR);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_RGB, GL_SRC_COLOR);

    glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_ALPHA, GL_REPLACE);
    glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_ALPHA, GL_TEXTURE);
    glTexEnvi(GL_TEXTURE_ENV, GL_OPERAND0_ALPHA, GL_SRC_ALPHA);

    ///////////////////////////////////////////////////////////////////////////

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);       // GL_LINEAR
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);       // GL_NEAREST
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);  // GL_CLAMP
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);  // GL_CLAMP

    ///////////////////////////////////////////////////////////////////////////

	//static const GLfloat hwBorderColor[4] = { 1.0, 0.0, 0.0, 1.0 };
	//glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, hwBorderColor);

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_SGIX, GL_TRUE);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_OPERATOR_SGIX, GL_TEXTURE_LEQUAL_R_SGIX);

    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

    ///////////////////////////////////////////////////////////////////////////

	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
	glTexGendv(GL_S, GL_EYE_PLANE, &matRow0[0]);
	glEnable(GL_TEXTURE_GEN_S);

	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
	glTexGendv(GL_T, GL_EYE_PLANE, &matRow1[0]);
	glEnable(GL_TEXTURE_GEN_T);

	glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
	glTexGendv(GL_R, GL_EYE_PLANE, &matRow2[0]);
	glEnable(GL_TEXTURE_GEN_R);

	glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
	glTexGendv(GL_Q, GL_EYE_PLANE, &matRow3[0]);
	glEnable(GL_TEXTURE_GEN_Q);

	//Bind & enable shadow map texture
	glBindTexture(GL_TEXTURE_2D, m_textureID);
	glEnable(GL_TEXTURE_2D);

	//Enable shadow comparison
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

	//Shadow comparison should be true (ie not in shadow) if r<=texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

	//Shadow comparison should generate an INTENSITY result
	glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);

	//Set alpha test to discard false comparisons
	glAlphaFunc(GL_GEQUAL, 0.99f);
	//glAlphaFunc(GL_GREATER, 0.1);

	glEnable(GL_ALPHA_TEST);
}


//===========================================================================
/*!
      Generate texture from memory data, to prepare for rendering.

      \fn		void cShadowMap::update()
*/
//===========================================================================
void cShadowMap::update()
{
	// check if shadow is enabled
	if (!m_enabled) { return; }

    if (m_textureID != 0)
    {
        // Deletion can make for all kinds of new hassles, particularly
        // when re-initializing a whole display context, since OpenGL
        // automatically starts re-assigning texture ID's.
         glDeleteTextures(1,&m_textureID);
         m_textureID = 0;
    }

    // Generate a texture ID and bind to it
    glGenTextures(1, &m_textureID);
    glBindTexture(GL_TEXTURE_2D, m_textureID);

    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_DEPTH_COMPONENT,
                 m_image->getWidth(),
                 m_image->getHeight(),
                 0,
                 m_image->getFormat(),
                 m_image->getType(),
				 0
        );
}


//===========================================================================
/*!
	Create a shadow map by providing a world and view point information.

	\fn         bool cShadowMap::updateMap(cWorld* a_world, 
						   const cVector3d& a_lightPos, 
						   const cVector3d& a_lightLookat, 
						   const cVector3d& a_lightUp, 
						   const double a_lightFieldViewAngle,
						   const double a_distanceNear,
						   const double a_distanceFar)

	\param      a_world  World in which to create shadow
	\param	    a_lightPos  Position of eye.
	\param	    a_lightLookat  Eye lookat position
	\param	    a_lightUp  Eye orientation (up vector).
	\param	    a_fieldViewAngle  Field of view of light source
	\param	    a_distanceNear  Distance to near clipping plane.
	\param	    a_distanceFar  Distance to far clipping plane.

	\return     Return \b true if shadow map was created succesfully. \b false otherwise. 
*/
//===========================================================================
bool cShadowMap::updateMap(cWorld* a_world, 
						   const cVector3d& a_lightPos, 
						   const cVector3d& a_lightLookat, 
						   const cVector3d& a_lightUp, 
						   const double a_lightFieldViewAngle,
						   const double a_distanceNear,
						   const double a_distanceFar)
{
	// check if shadow is enabled
	if (!m_enabled) { return (false); }

	// check for necessary OpenGL extensions
	if(!GLEE_ARB_depth_texture || !GLEE_ARB_shadow)
	{
		return (false);
	}

	// verify size of image
	int width  = m_image->getWidth();
	int height = m_image->getHeight();
	if (!((width > 0) && (height > 0))) { return (false); }


    //-----------------------------------------------------------------------------
	// setup the output framebuffer to be the shadow texture. In other words, we
    // will render the world from the light source, produce a depth map, and write
    // the result into the shadow texture.
    //-----------------------------------------------------------------------------
    
    // As with the other objects in OpenGL (texture object, pixel buffer objects 
    // and vertex buffer object) before you can use a FBO you have to create a 
    // valid handle to it:

    if (glIsFramebuffer(m_fbo) == false)
    {
        glGenFramebuffers(1, &m_fbo);
    }

    // To perform any operations on a FBO you need to bind it, much like you 
    // would a VBO or texture, so that the operations can be performed on it, 
    // this is done via the following code
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

	// reserve space for a texture   
	update();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	
	// connect framebuffer to texture
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_textureID, 0);

   
    //-----------------------------------------------------------------------------
	// render the world from the point of view of the camera.
    // here we are only interested in rendering the depth buffer
    //-----------------------------------------------------------------------------
    
	glPushAttrib(GL_VIEWPORT_BIT);

	// we shall not render any colors, only depth information
    glDrawBuffer(GL_NONE);

	// setup our viewport
    glViewport(0, 0, width, height);

	// compute aspect ratio
    double glAspect = ((double)width / (double)height);
	
	// setup projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(
			2.0 * a_lightFieldViewAngle,   // Field of View Angle.
			glAspect,           // Aspect ratio of viewing volume.
			a_distanceNear,     // Distance to Near clipping plane.
			a_distanceFar);     // Distance to Far clipping plane.


	// setup modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt( a_lightPos(0) ,    a_lightPos(1) ,    a_lightPos(2) ,
			   a_lightLookat(0) , a_lightLookat(1) , a_lightLookat(2) ,
			   a_lightUp(0) ,     a_lightUp(1) ,     a_lightUp(2)  );

    // Back up the view and projection matrix for future reference
	cTransform projectionMatrix;
	cTransform viewMatrix;
    glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix.getData());
	glGetDoublev(GL_MODELVIEW_MATRIX, viewMatrix.getData());

	m_lightProjectionMatrix = projectionMatrix;
	m_lightViewMatrix = viewMatrix;

	// clear depth buffer
    glClear(GL_DEPTH_BUFFER_BIT);

	// draw back faces only into the shadow map
	glCullFace(GL_FRONT);
	
	// disable color writes, and use flat shading for speed
	glShadeModel(GL_FLAT);
	glColorMask(0, 0, 0, 0);

    // set up reasonable default OpenGL state
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
	glEnable(GL_POLYGON_OFFSET_FILL);

	// draw the scene
	cRenderOptions options;
    options.m_camera                                = NULL;
	options.m_single_pass_only						= true;
	options.m_render_opaque_objects_only			= false;
	options.m_render_transparent_front_faces_only	= false;
	options.m_render_transparent_back_faces_only	= false;
	options.m_enable_lighting						= true;
	options.m_render_materials						= true;
	options.m_render_textures						= true;
	options.m_creating_shadow_map					= true;
	options.m_rendering_shadow						= true;
	options.m_shadow_light_level					= 1.0;
	options.m_storeObjectPositions					= true;
    options.m_resetDisplay                          = false;

    // set alpha test - all triangles with opacity lower than the selected level
    // shall be considered transparent. 
    // glAlphaFunc(GL_GEQUAL, 0.50f);
    // glEnable(GL_ALPHA_TEST);

	// render single pass (all objects)
	a_world->renderSceneGraph(options);

	// restore OpenGL settings
    glDisable(GL_ALPHA_TEST);
	glDisable(GL_POLYGON_OFFSET_FILL);  
	glColorMask(1, 1, 1, 1);
	glPopAttrib();
	glCullFace(GL_BACK);
	glShadeModel(GL_SMOOTH);

	// cleanup
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// return success
	return (true);
}

//! 
//===========================================================================
/*!
    Copy shadowmap from GPU memory to m_image object in RAM memory. 

    \fn       bool cShadowMap::copyMapFromGPUtoImage()
    \return   Returns nothing.
*/
//===========================================================================
bool cShadowMap::copyMapFromGPUtoImage()
{
    return (copyMapFromGPUtoImage(m_image));
}


//===========================================================================
/*!
    Copy shadowmap from GPU memory to an image object in RAM memory.

    \fn       bool cShadowMap::copyMapFromGPUtoImage(cImage* a_image)
    \param    a_image  Image object to which shadowmap is copied.
    \return   Returns \b true if success, \b false otherwise.
*/
//===========================================================================
bool cShadowMap::copyMapFromGPUtoImage(cImage* a_image)
{
    // check if framebuffer is available
    if (glIsFramebuffer(m_fbo) == false)
    {
        return (false);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    unsigned int w   = m_image->getWidth();
    unsigned int h   = m_image->getHeight();
    GLenum pixelType = m_image->getType();

    if (a_image->allocate(w, h, GL_LUMINANCE, pixelType) == false)
    {
        return (false);
    }

    glReadPixels(
        0,
        0,
        w,
        h,
        GL_DEPTH_COMPONENT,
        GL_UNSIGNED_INT,
        a_image->getData()
        );

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return (true);
}
