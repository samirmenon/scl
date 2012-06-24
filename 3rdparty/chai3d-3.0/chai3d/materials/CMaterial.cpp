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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 473 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "materials/CMaterial.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cMaterial.

    \fn     cMaterial::cMaterial()
*/
//===========================================================================
cMaterial::cMaterial()
{
    // default graphic settings
    m_ambient.set(0.3f, 0.3f, 0.3f, 1.0f);
    m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);
    m_shininess = 64;

    // default haptic settings
    m_viscosity             = 0.0;
    m_stiffness             = 0.0;
    m_dampingLevel          = 0.0;
    m_staticFriction        = 0.0;
    m_dynamicFriction       = 0.0;
    m_textureLevel          = 0.0;
    m_vibrationFrequency    = 0.0;
    m_vibrationAmplitude    = 0.0;
    m_stickSlipForceMax     = 0.0;
    m_stickSlipStiffness    = 0.0;

    m_useHapticFriction             = false;
    m_useHapticTexture              = false;
    m_useHapticShading              = false;
    m_renderFrontSideOfTriangles    = true;
    m_renderBackSideOfTriangles     = false;

    // set all modification flags to false
    setModificationFlags(false);
}


//===========================================================================
/*!
    Create copy of current instance.

    \fn     cMaterial* cMaterial::copy()
	\return Return point to new instance.
*/
//===========================================================================
cMaterial* cMaterial::copy()
{
	// create new instance
	cMaterial* obj = new cMaterial();

	// copy material properties
    obj->m_ambient				= m_ambient;
    obj->m_diffuse				= m_diffuse;
    obj->m_specular				= m_specular;
    obj->m_emission				= m_emission;
    obj->m_shininess			= m_shininess;
    obj->m_viscosity			= m_viscosity;
    obj->m_stiffness			= m_stiffness;
    obj->m_dampingLevel         = m_dampingLevel;
    obj->m_staticFriction		= m_staticFriction;
    obj->m_dynamicFriction		= m_dynamicFriction;
    obj->m_textureLevel			= m_textureLevel;
    obj->m_vibrationFrequency	= m_vibrationFrequency;
    obj->m_vibrationAmplitude	= m_vibrationAmplitude;
    obj->m_magnetMaxForce		= m_magnetMaxForce;
    obj->m_magnetMaxDistance	= m_magnetMaxDistance;
    obj->m_stickSlipForceMax	= m_stickSlipForceMax;
    obj->m_stickSlipStiffness	= m_stickSlipStiffness;

    obj->m_useHapticFriction                = m_useHapticFriction;
    obj->m_useHapticTexture                 = m_useHapticTexture;
    obj->m_useHapticShading                 = m_useHapticShading;
    obj->m_flag_renderFrontSideOfTriangles  = m_flag_renderFrontSideOfTriangles;
    obj->m_flag_renderBackSideOfTriangles   = m_flag_renderBackSideOfTriangles;

	// reset all flags
	obj->setModificationFlags(false);

	// return
	return (obj);
}


//===========================================================================
/*!
    Set the transparency level (by setting the alpha value for all color 
    properties).

    \fn     void cMaterial::setTransparencyLevel(const float a_levelTransparency)
    \param  a_levelTransparency  Level of transparency.
*/
//===========================================================================
void cMaterial::setTransparencyLevel(const float a_levelTransparency)
{
    // make sur value is in range [0.0 - 1.0]
    float level = cClamp(a_levelTransparency, 0.0f, 1.0f);

    // apply new value
    m_ambient.setA(level);
    m_diffuse.setA(level);
    m_specular.setA(level);
    m_emission.setA(level);
}


//===========================================================================
/*!
    Set the level of shininess. Value are clamped to range from 0 --> 128.

    \fn     void cMaterial::setShininess(const GLuint a_shininess)
    \param  a_shininess  Level of shininess
*/
//===========================================================================
void cMaterial::setShininess(const GLuint a_shininess)
{
    // update value
    m_shininess = cClamp(a_shininess, (GLuint)0, (GLuint)128);

    // mark variable as modified
    m_flag_shininess = true;
}


//===========================================================================
/*!
    Define the color properties of the material.

    \fn     void cMaterial::setColorf(const float a_R, 
                                      const float a_G, 
                                      const float a_B, 
                                      const float a_A)
    \param  a_red    Red component
    \param  a_green  Green component
    \param  a_blue   Blue component
    \param  a_alpha  Alpha component
*/
//===========================================================================
void cMaterial::setColorf(const GLfloat a_red, 
                          const GLfloat a_green, 
                          const GLfloat a_blue,
                          const GLfloat a_alpha)
{
    m_diffuse.set(a_red, a_green, a_blue, a_alpha);
    updateColors();
}


//===========================================================================
/*!
    Define the color properties of the material.

    \param  a_color  Color.
*/
//===========================================================================
void cMaterial::setColor(cColorf& a_color)
{
    m_diffuse = a_color;
    updateColors();
}


//===========================================================================
/*!
    Define the color properties of the material.

    \param  a_color  Color.
*/
//===========================================================================
void cMaterial::setColor(cColorb& a_color)
{
    m_diffuse = a_color.getColorf();
    updateColors();
}


//===========================================================================
/*!
    If a new color is defined for the material by calling methods such as
    setColorf() or setColorRed(), the color is first set to the diffuse component.
    This function then updates the ambient color by setting it at percentage
    of the 

    \fn     void cMaterial::updateColors()
*/
//===========================================================================
void cMaterial::updateColors()
{
    float level = 0.5;
    m_ambient.setR(level * m_diffuse.getR());
    m_ambient.setG(level * m_diffuse.getG());
    m_ambient.setB(level * m_diffuse.getB());
    m_ambient.setA(m_diffuse.getA());
    m_specular.set(1.0, 1.0, 1.0, m_diffuse.getA());
}


//===========================================================================
/*!
    Set the level of stiffness. Clamped to be a non-negative value.

    \fn     void cMaterial::setStiffness(const double a_stiffness)
    \param  a_stiffness  Level of stiffness.
*/
//===========================================================================
void cMaterial::setStiffness(const double a_stiffness)
{
    // update value
    m_stiffness = cClamp0(a_stiffness);

    // mark variable as modified
    m_flag_stiffness = true;
}


//===========================================================================
/*!
    Set the level of damping between 0 and 1. The force algorithm will adjust
    the output value based on the maximum damping level supported by the 
    haptic device.

    \fn     void cMaterial::setDampingLevel(const double a_dampingLevel)
    \param  a_dampingLevel  Level of damping.
*/
//===========================================================================
void cMaterial::setDampingLevel(const double a_dampingLevel)
{
    // update value
    m_dampingLevel = cClamp0(a_dampingLevel);

    // mark variable as modified
    m_flag_dampingLevel = true;
}


//===========================================================================
/*!
    Set the level of static friction. Clamped to be a non-negative value.

    \fn     void cMaterial::setStaticFriction(const double a_friction)
    \param  a_friction  Level of friction.
*/
//===========================================================================
void cMaterial::setStaticFriction(const double a_friction)
{
    // update value
    m_staticFriction = cClamp0(a_friction);

    // enable friction rendering if required
    if ((m_staticFriction > 0) || (m_dynamicFriction > 0))
    {
        setUseHapticFriction(true);
    }
    else
    {
        setUseHapticFriction(false);
    }
    
    // mark variable as modified
    m_flag_staticFriction = true;
}


//===========================================================================
/*!
    Set the level of dynamic friction. Clamped to be a non-negative value.

    \fn     void cMaterial::setDynamicFriction(const double a_friction)
    \param  a_friction  Level of friction.
*/
//===========================================================================
void cMaterial::setDynamicFriction(const double a_friction)
{
    // update value
    m_dynamicFriction = cClamp0(a_friction);

    // enable friction rendering if required
    if ((m_staticFriction > 0) || (m_dynamicFriction > 0))
    {
        setUseHapticFriction(true);
    }
    else
    {
        setUseHapticFriction(false);
    }
    
    // mark variable as modified
    m_flag_dynamicFriction = true;
}


//===========================================================================
/*!
    Set the intensity at which the texure map of an object will be perceived. 

    \fn     void cMaterial::setTextureLevel(const double a_textureLevel)
    \param  a_textureLevel  Intensity level.
*/
//===========================================================================
void cMaterial::setTextureLevel(const double a_textureLevel)
{
    // update value
    m_textureLevel = a_textureLevel;

    // enable texture rendering if required
    if (m_textureLevel > 0)
    {
        setUseHapticTexture(true);
    }
    else
    {
        setUseHapticTexture(false);
    }

    // mark variable as modified
    m_flag_textureLevel = true;
}


//===========================================================================
/*!
    Set the level of viscosity. Clamped to be a non-negative value.

    \fn     void cMaterial::setViscosity(const double a_viscosity)
    \param  a_viscosity  Level of viscosity.
*/
//===========================================================================
void cMaterial::setViscosity(const double a_viscosity)
{
    // update value
    m_viscosity = cClamp0(a_viscosity);
    
    // mark variable as modified
    m_flag_viscosity = true;
}


//===========================================================================
/*!
    Set the frequency of vibration. Clamped to be a non-negative value.

    \fn     void cMaterial::setVibrationFrequency(const double a_vibrationFrequency)
    \param  a_vibrationFrequency  Frequency of vibration [Hz].
*/
//===========================================================================
void cMaterial::setVibrationFrequency(const double a_vibrationFrequency)
{
    // update value
    m_vibrationFrequency = cClamp0(a_vibrationFrequency);
    
    // mark variable as modified
    m_flag_vibrationFrequency = true;
}


//===========================================================================
/*!
    Set the amplitude of vibration. Clamped to be a non-negative value.

    \fn     void cMaterial::setVibrationAmplitude(double a_vibrationAmplitude)
    \param  a_vibrationAmplitude  Amplitude of vibration [N].
*/
//===========================================================================
void cMaterial::setVibrationAmplitude(const double a_vibrationAmplitude)
{
    // update value
    m_vibrationAmplitude = cClamp0(a_vibrationAmplitude);
    
    // mark variable as modified
    m_flag_vibrationAmplitude = true;
}


//===========================================================================
/*!
    Set the maximum force applied by the magnet [N].

    \fn     void cMaterial::setMagnetMaxForce(const double a_magnetMaxForce)
    \param  a_magnetMaxForce  Maximum force of magnet.
*/
//===========================================================================
void cMaterial::setMagnetMaxForce(const double a_magnetMaxForce)
{
    // update value
    m_magnetMaxForce = cClamp0(a_magnetMaxForce);
    
    // mark variable as modified
    m_flag_magnetMaxForce = true;
}


//===========================================================================
/*!
    Set the maximum distance from the object where the force can be perceived [m]

    \fn     void cMaterial::setMagnetMaxDistance(const double a_magnetMaxDistance)
    \param  a_magnetMaxDistance  Maximum distance from object where 
                                 magnet is active.
*/
//===========================================================================
void cMaterial::setMagnetMaxDistance(const double a_magnetMaxDistance)
{
    // update value
    m_magnetMaxDistance = cClamp0(a_magnetMaxDistance);
    
    // mark variable as modified
    m_flag_magnetMaxDistance = true;
}


//===========================================================================
/*!
    Set the maximum force threshold for the stick and slip model [N].

    \fn     void cMaterial::setStickSlipForceMax(const double a_stickSlipForceMax)
    \param  a_stickSlipForceMax  Maximum force threshold.
*/
//===========================================================================
void cMaterial::setStickSlipForceMax(const double a_stickSlipForceMax)
{
    // update value
    m_stickSlipForceMax = cClamp0(a_stickSlipForceMax);
    
    // mark variable as modified
    m_flag_stickSlipForceMax = true;
}


//===========================================================================
/*!
    Set the stiffness for the stick and slip model [N/m]

    \fn     void cMaterial::setStickSlipStiffness(const double a_stickSlipStiffness)
    \param  a_stickSlipStiffness  Stiffness property.
*/
//===========================================================================
void cMaterial::setStickSlipStiffness(const double a_stickSlipStiffness)
{
    // update value
    m_stickSlipStiffness = cClamp0(a_stickSlipStiffness);
    
    // mark variable as modified
    m_flag_stickSlipStiffness = true;
}


//===========================================================================
/*!
    Enable/Disable rendering of haptic friction.

    \fn     void cMaterial::setUseHapticFriction(const bool a_useHapticFriction)
    \param  a_useHapticFriction  If \b true, haptic friction rendering in enabled.
*/
//===========================================================================
void cMaterial::setUseHapticFriction(const bool a_useHapticFriction)
{
    // update value
    m_useHapticFriction = a_useHapticFriction;
    
    // mark variable as modified
    m_flag_useHapticFriction = true;
}


//===========================================================================
/*!
    Enable/Disable rendering of textures haptically.

    \fn     void cMaterial::setStickSlipStiffness(const double a_stickSlipStiffness)
    \param  a_useHapticTexture  If \b true, haptic texture rendering in enabled.
*/
//===========================================================================
void cMaterial::setUseHapticTexture(const bool a_useHapticTexture)
{
    // update value
    m_useHapticTexture = a_useHapticTexture;
    
    // mark variable as modified
    m_flag_useHapticTexture = true;
}


//===========================================================================
/*!
    Enable/Disable haptic shading.

    \fn     void cMaterial::setUseHapticShading(const bool a_useHapticShading)
    \param  a_useHapticShading  If \b true, haptic shading in enabled.
*/
//===========================================================================
void cMaterial::setUseHapticShading(const bool a_useHapticShading)
{
    // update value
    m_useHapticShading = a_useHapticShading;
    
    // mark variable as modified
    m_flag_useHapticShading = true;
}


//===========================================================================
/*!
    Enable/disable if haptic rendering occurs on front side of Mesh triangles.
    This option applies to Mesh objects which are rendered using the
    proxy force algorithm.

    \fn     void cMaterial::setRenderFrontSideOfTriangles(const bool a_enabled)
    \param  a_enabled  If \b true, then haptic rendering is enabled.
*/
//===========================================================================
void cMaterial::setRenderFrontSideOfTriangles(const bool a_enabled)
{
    // update value
    m_renderFrontSideOfTriangles = a_enabled;
    
    // mark variable as modified
    m_flag_renderFrontSideOfTriangles = true;
}


//===========================================================================
/*!
    Enable/disable if haptic rendering occurs on back side of Mesh triangles.
    This option applies to Mesh objects which are rendered using the
    proxy force algorithm.

    \fn     void cMaterial::setRenderBackSideOfTriangles(const bool a_enabled)
    \param  a_enabled  If \b true, then haptic rendering is enabled.
*/
//===========================================================================
void cMaterial::setRenderBackSideOfTriangles(const bool a_enabled)
{
    // update value
    m_renderBackSideOfTriangles = a_enabled;
    
    // mark variable as modified
    m_flag_renderBackSideOfTriangles = true;
}


//===========================================================================
/*!
    Enable/Disable rendering of front and back sides of mesh triangles.

    \fn     void cMaterial::setRenderTriangles(const bool a_enableFrontSide, 
                                   const bool a_enableBackSide)
    \param  a_enableFrontSide  If \b true, then haptic rendering is enabled for front sides.
    \param  a_enableBackSide  If \b true, then haptic rendering is enabled for back sides.
*/
//===========================================================================
void cMaterial::setRenderTriangles(const bool a_enableFrontSide, 
                                   const bool a_enableBackSide)
{
    // update front side
    setRenderFrontSideOfTriangles(a_enableFrontSide);
    
    // update back side
    setRenderBackSideOfTriangles(a_enableBackSide);
}


//===========================================================================
/*!
    Render this material in OpenGL.

    \fn     void cMaterial::render(cRenderOptions& a_options)
	\param	a_options  Rendering options.
*/
//===========================================================================
void cMaterial::render(cRenderOptions& a_options)
{
	// check if materials should be rendered
	if (!a_options.m_render_materials) { return; }

	// render material
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, (const float *)&m_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, (const float *)&m_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (const float *)&m_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, (const float *)&m_emission);
	glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, m_shininess);
}


//===========================================================================
/*!
    Sets all the modification flags to a desired value.

    \fn     void cMaterial::setModificationFlags(const bool a_value)
	\param	a_value  Value to be assigned to modification flags.
*/
//===========================================================================
void cMaterial::setModificationFlags(const bool a_value)
{
    m_flag_shininess                    = a_value;
    m_flag_viscosity                    = a_value;
    m_flag_stiffness                    = a_value;
    m_flag_dampingLevel                 = a_value;
    m_flag_staticFriction               = a_value;
    m_flag_dynamicFriction              = a_value;
    m_flag_textureLevel                 = a_value;
    m_flag_vibrationFrequency           = a_value;
    m_flag_vibrationAmplitude           = a_value;
    m_flag_magnetMaxForce               = a_value;
    m_flag_magnetMaxDistance            = a_value;
    m_flag_stickSlipForceMax            = a_value;
    m_flag_stickSlipStiffness           = a_value;
    m_flag_useHapticTexture             = a_value;
    m_flag_useHapticShading             = a_value;
    m_flag_renderFrontSideOfTriangles   = a_value;
    m_flag_renderBackSideOfTriangles    = a_value;


    m_ambient.setModificationFlags(a_value);
    m_diffuse.setModificationFlags(a_value);
    m_specular.setModificationFlags(a_value);
    m_emission.setModificationFlags(a_value);
}


//===========================================================================
/*!
    Copy modified variables to another material object.

    \fn     void cMaterial::copyTo(cMaterial& a_material)
	\param	a_material  Destination material where values are copied to.
*/
//===========================================================================
void cMaterial::copyTo(cMaterial& a_material)
{
   copyTo(&a_material);
}


//===========================================================================
/*!
    Copy modified variables to another material object.

    \fn     void cMaterial::copyTo(cMaterial* a_material)
	\param	a_material  Destination material where values are copied to.
*/
//===========================================================================
void cMaterial::copyTo(cMaterial* a_material)
{
    if (m_flag_shininess)         
        a_material->setShininess(m_shininess);
    if (m_flag_viscosity)
        a_material->setViscosity(m_viscosity); 
    if (m_flag_stiffness)      
        a_material->setStiffness(m_stiffness);
    if (m_flag_dampingLevel)      
        a_material->setDampingLevel(m_dampingLevel);
    if (m_flag_staticFriction)  
        a_material->setStaticFriction(m_staticFriction);
    if (m_flag_dynamicFriction)    
        a_material->setDynamicFriction(m_dynamicFriction);
    if (m_flag_textureLevel)    
        a_material->setTextureLevel(m_textureLevel);
    if (m_flag_vibrationFrequency)  
        a_material->setVibrationFrequency(m_vibrationFrequency);
    if (m_flag_vibrationAmplitude)  
        a_material->setVibrationAmplitude(m_vibrationAmplitude);
    if (m_flag_magnetMaxForce)      
        a_material->setMagnetMaxForce(m_magnetMaxForce);
    if (m_flag_magnetMaxDistance)    
        a_material->setMagnetMaxDistance(m_magnetMaxDistance);
    if (m_flag_stickSlipForceMax)   
        a_material->setStickSlipForceMax(m_stickSlipForceMax);
    if (m_flag_stickSlipStiffness)  
        a_material->setStickSlipStiffness(m_stickSlipStiffness);
    if (m_flag_renderFrontSideOfTriangles)
        a_material->setRenderFrontSideOfTriangles(m_renderFrontSideOfTriangles);
    if (m_flag_renderBackSideOfTriangles)
        a_material->setRenderBackSideOfTriangles(m_renderBackSideOfTriangles);

    m_ambient.copyTo(a_material->m_ambient);
    m_diffuse.copyTo(a_material->m_diffuse);
    m_specular.copyTo(a_material->m_specular);
    m_emission.copyTo(a_material->m_emission);
}

