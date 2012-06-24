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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 449 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "lighting/CSpotLight.h"
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cSpotLight.

    \fn       cSpotLight::cSpotLight(cWorld* a_world):
                cGenericLight(a_world),
                cPositionalLight(a_world),
                cDirectionalLight(a_world)
*/
//===========================================================================
cSpotLight::cSpotLight(cWorld* a_world):cGenericLight(a_world),cPositionalLight(a_world),cDirectionalLight(a_world)
{   
    // set default cutoff angle
    m_cutOffAngleDEG = 45.0;

    // set default spot exponent
    m_spotExponent = 1.0;

	// create shadow map
	m_shadowMap = new cShadowMap();
    m_shadowNearClippingPlane = 0.1;
    m_shadowFarClippingPlane = 10.0;

    // set color properties of display model of light source
    m_displaySourceColor.setYellowGold();
	m_displayConeColor.setGrayDarkSlate();
}


//===========================================================================
/*!
    Destructor of cSpotLight.

    \fn       cSpotLight::~cSpotLight()
*/
//===========================================================================
cSpotLight::~cSpotLight()
{
    // delete shadow map
	delete m_shadowMap;
}


//===========================================================================
/*!
    Set the cutoff angle (in degrees) of the light beam. Accepted values
    range from 0 to 90 degrees for spot lights.

    \fn       void cSpotLight::setCutOffAngleDEG(const GLfloat& a_angleDEG)
    \param    a_angleDEG  Cutoff angle of light beam in degrees.
*/
//===========================================================================
void cSpotLight::setCutOffAngleDeg(const GLfloat& a_angleDeg)
{
    m_cutOffAngleDEG = cClamp(a_angleDeg, (GLfloat)0.0, (GLfloat)90.0);
}


//===========================================================================
/*!
    Render this light source in OpenGL.

    \fn     void cSpotLight::renderLightSource(cRenderOptions& a_options)
	\param	a_options  Rendering options.
*/
//===========================================================================
void cSpotLight::renderLightSource(cRenderOptions& a_options)
{
	// check if light sources should be rendered
	if ((!a_options.m_enable_lighting) || (!m_enabled))
    {
        // disable OpenGL light source
        glDisable(m_glLightNumber);
        return;
    }

    // enable this light in OpenGL
    glEnable(m_glLightNumber);

    // enable or disable two side light model
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, m_useTwoSideLightModel);

    // compute global position of current light in world
    computeGlobalPositionsFromRoot();

    // set lighting components
	if (a_options.m_rendering_shadow)
	{
		cColorf diffuse((GLfloat)(a_options.m_shadow_light_level * m_diffuse.getR()),
						(GLfloat)(a_options.m_shadow_light_level * m_diffuse.getG()),
						(GLfloat)(a_options.m_shadow_light_level * m_diffuse.getB()));
		cColorf specular(0.0, 0.0, 0.0);
		glLightfv(m_glLightNumber, GL_AMBIENT,  m_ambient.pColor());
		glLightfv(m_glLightNumber, GL_DIFFUSE,  diffuse.pColor() );
		glLightfv(m_glLightNumber, GL_SPECULAR, specular.pColor());
	}
	else
	{	
		glLightfv(m_glLightNumber, GL_AMBIENT,  m_ambient.pColor());
		glLightfv(m_glLightNumber, GL_DIFFUSE,  m_diffuse.pColor() );
		glLightfv(m_glLightNumber, GL_SPECULAR, m_specular.pColor());
	}

    // define the position of the light source. 
    float position[4];
    position[0] = (float)m_globalPos(0) ;
    position[1] = (float)m_globalPos(1) ;
    position[2] = (float)m_globalPos(2) ;
    position[3] = 1.0f; // defines light to be of type positional
    glLightfv(m_glLightNumber, GL_POSITION, (const float *)&position);

    cVector3d dir = m_globalRot.getCol0();
    float direction[4];
    direction[0] = (float)dir(0) ;
    direction[1] = (float)dir(1) ;
    direction[2] = (float)dir(2) ;
    direction[3] = 0.0f;
    glLightfv(m_glLightNumber, GL_SPOT_DIRECTION, (const float *)&direction);

    // set spot exponent
    glLightf(m_glLightNumber, GL_SPOT_EXPONENT, m_spotExponent);   

    // set cutoff angle
    glLightf(m_glLightNumber, GL_SPOT_CUTOFF, m_cutOffAngleDEG); 
}


//===========================================================================
/*!
    Set display settings of light source. To be used for debugging purposes 
    to display location and direction of light source.

    \fn       void cSpotLight::setDisplaySettings(const double& a_sourceRadius, 
                                    const double& a_coneLength, 
                                    const bool a_displayEnabled)

    \param    a_sourceRadius  Radius of displayed light source.
    \param    a_coneLength  Length of cone light.
    \param    a_displayEnabled  Display status.
*/
//===========================================================================
void cSpotLight::setDisplaySettings(const double& a_sourceRadius, 
                                    const double& a_coneLength, 
                                    const bool a_displayEnabled)
{
    m_displaySourceRadius = a_sourceRadius;
    m_displayConeLength = a_coneLength;
    m_displayEnabled = a_displayEnabled;
}


//===========================================================================
/*!
    Render a graphic representation of the light spot in OpenGL.
    This is used for debuging purposes when one wants to display 
    the cone illuminated by the light

    \fn       void cSpotLight::render(cRenderOptions& a_options)
    \param    a_options  Render options.
*/
//===========================================================================
void cSpotLight::render(cRenderOptions& a_options)
{
    // is display rendering of light source enabled?
    if (!m_displayEnabled) { return; }

	/////////////////////////////////////////////////////////////////////////
	// Render parts that are always opaque
	/////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options) && (!a_options.m_creating_shadow_map))
	{      
        // disable lighting
		glDisable(GL_LIGHTING);

        // render cone
        double sizeS = m_displayConeLength * cSinDeg(m_cutOffAngleDEG);
        double sizeX = m_displayConeLength * cCosDeg(m_cutOffAngleDEG);

        glLineWidth(1.0);
        m_displayConeColor.render();
  
        glEnable(GL_LINE_STIPPLE);
        glPushAttrib (GL_LINE_BIT);
        glLineStipple (1, 0xF0F0);

        glBegin(GL_LINES);
			glVertex3d( 0.0, 0.0, 0.0);
            glVertex3d( sizeX, sizeS, sizeS);
            glVertex3d( 0.0, 0.0, 0.0);
            glVertex3d( sizeX,-sizeS, sizeS);
            glVertex3d( 0.0, 0.0, 0.0);
            glVertex3d( sizeX,-sizeS,-sizeS);
            glVertex3d( 0.0, 0.0, 0.0);
            glVertex3d( sizeX, sizeS,-sizeS);
            
            glVertex3d( sizeX, sizeS, sizeS);
            glVertex3d( sizeX,-sizeS, sizeS);            
            glVertex3d( sizeX,-sizeS, sizeS);
            glVertex3d( sizeX,-sizeS,-sizeS);
            glVertex3d( sizeX,-sizeS,-sizeS);
            glVertex3d( sizeX, sizeS,-sizeS);
            glVertex3d( sizeX, sizeS,-sizeS);
            glVertex3d( sizeX, sizeS, sizeS);
		glEnd();

        glPopAttrib();
        glDisable(GL_LINE_STIPPLE);

        // render light source
        if (m_enabled)
            m_displaySourceColor.render();
        else
            glColor3f(0.2f, 0.2f, 0.2f);
        cDrawSphere(m_displaySourceRadius, 8, 8);

		// enable lighting again
		glEnable(GL_LIGHTING);
	}
}


//===========================================================================
/*!
    Update shadow map for current light source.

    \fn       void cSpotLight::updateShadowMap()
    \return   Returns nothing.
*/
//===========================================================================
void cSpotLight::updateShadowMap()
{
    // sanity check
    if ((!m_enabled) || (m_shadowMap == NULL)) { return; }

    // update shadow map
    cVector3d tmp0 = getGlobalPos();
    cVector3d tmp1 = cAdd(getGlobalPos(), getGlobalRot().getCol0());
    cVector3d tmp2 = getGlobalRot().getCol2();
    
    m_shadowMap->updateMap(m_worldParent,
                           tmp0,
                           tmp1,
                           tmp2,
                           m_cutOffAngleDEG,
                           m_shadowNearClippingPlane,
                           m_shadowFarClippingPlane);
}


//===========================================================================
/*!
    Define the near and far clipping planes of the shadow map. For higher 
    quality rendering, it is important to set the values accordingly to the
    size of the environment.

    \fn       void cSpotLight::setShadowMapProperties(const double& a_nearClippingPlane, 
                                        const double& a_farClippingPlane)

    \param    a_nearClippingPlane  
    \param    a_farClippingPlane  
*/
//===========================================================================
void cSpotLight::setShadowMapProperties(const double& a_nearClippingPlane, 
                                        const double& a_farClippingPlane)
{
    m_shadowNearClippingPlane = cMin(cAbs(a_nearClippingPlane), cAbs(a_farClippingPlane));
    m_shadowFarClippingPlane  = cMax(cAbs(a_nearClippingPlane), cAbs(a_farClippingPlane));
}


//===========================================================================
/*!
    Enable of Disable the shadow map for current spot light.

    \fn       void cSpotLight::setMapShadowEnabled(const bool a_enabled)

    \param    a_enabled  Shadow status.
*/
//===========================================================================
void cSpotLight::setShadowMapEnabled(const bool a_enabled)
{
    if (m_shadowMap != NULL)
    {
        m_shadowMap->setEnabled(a_enabled);
    }
}


//===========================================================================
/*!
    Read status of current shadow map.

    \fn       bool cSpotLight::getShadowMapEnabled() const

    \return   Return shadowmap status.
*/
//===========================================================================
bool cSpotLight::getShadowMapEnabled() const
{
    if (m_shadowMap != NULL)
    {
        return((m_enabled) && (m_shadowMap->getEnabled()));
    }  
    return(false);
}