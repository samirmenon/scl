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
    \version   2.0.0 $Rev: 245 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "graphics/CMaterial.h"
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
    m_viscosity = 0.0;
    m_stiffness = 0.0;
    m_static_friction = 0.0;
    m_dynamic_friction = 0.0;
    m_vibrationFrequency = 0.0;
    m_vibrationAmplitude = 0.0;
    m_stickSlipForceMax = 0.0;
    m_stickSlipStiffness = 0.0;
}


//===========================================================================
/*!
    Set the transparency level (by setting the alpha value for all color 
    properties).

    \fn     void cMaterial::setTransparencyLevel(const float a_levelTransparency)
    \param  a_levelTransparency  Level of transparency.
*/
//===========================================================================
void cMaterial::setTransparencyLevel(float a_levelTransparency)
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
void cMaterial::setShininess(GLuint a_shininess)
{
    m_shininess = cClamp(a_shininess, (GLuint)0, (GLuint)128);
}


//===========================================================================
/*!
    Set the level of stiffness. Clamped to be a non-negative value.

    \fn     void cMaterial::setStiffness(const double a_stiffness)
    \param  a_stiffness  Level of stiffness.
*/
//===========================================================================
void cMaterial::setStiffness(double a_stiffness)
{
    m_stiffness = cClamp0(a_stiffness);
}


//===========================================================================
/*!
    Set the level of static friction. Clamped to be a non-negative value.

    \fn     void cMaterial::setStaticFriction(const double a_friction)
    \param  a_friction  Level of friction.
*/
//===========================================================================
void cMaterial::setStaticFriction(double a_friction)
{
    m_static_friction = cClamp0(a_friction);
}


//===========================================================================
/*!
    Set the level of dynamic friction. Clamped to be a non-negative value.

    \fn     void cMaterial::setDynamicFriction(const double a_friction)
    \param  a_friction  Level of friction.
*/
//===========================================================================
void cMaterial::setDynamicFriction(double a_friction)
{
    m_dynamic_friction = cClamp0(a_friction);
}


//===========================================================================
/*!
    Set the level of viscosity. Clamped to be a non-negative value.

    \fn     void cMaterial::setViscosity(double a_viscosity)
    \param  a_viscosity  Level of viscosity.
*/
//===========================================================================
void cMaterial::setViscosity(double a_viscosity)
{
    m_viscosity = cClamp0(a_viscosity);
}


//===========================================================================
/*!
    Set the frequency of vibration. Clamped to be a non-negative value.

    \fn     void cMaterial::setVibrationFrequency(double a_vibrationFrequency)
    \param  a_vibrationFrequency  Frequency of vibration [Hz].
*/
//===========================================================================
void cMaterial::setVibrationFrequency(double a_vibrationFrequency)
{
    m_vibrationFrequency = cClamp0(a_vibrationFrequency);
}


//===========================================================================
/*!
    Set the amplitude of vibration. Clamped to be a non-negative value.

    \fn     void cMaterial::setVibrationAmplitude(double a_vibrationAmplitude)
    \param  a_vibrationAmplitude  Amplitude of vibration [N].
*/
//===========================================================================
void cMaterial::setVibrationAmplitude(double a_vibrationAmplitude)
{
    m_vibrationAmplitude = cClamp0(a_vibrationAmplitude);
}


//===========================================================================
/*!
    Set the maximum force applied by the magnet [N].

    \fn     void cMaterial::setMagnetMaxForce(double a_magnetMaxForce)
    \param  a_magnetMaxForce  Maximum force of magnet.
*/
//===========================================================================
void cMaterial::setMagnetMaxForce(double a_magnetMaxForce)
{
    m_magnetMaxForce = cClamp0(a_magnetMaxForce);
}


//===========================================================================
/*!
    Set the maximum distance from the object where the force can be perceived [m]

    \fn     void cMaterial::setMagnetMaxDistance(double a_magnetMaxDistance)
    \param  a_magnetMaxDistance  Maximum distance from object where 
                                 magnet is active.
*/
//===========================================================================
void cMaterial::setMagnetMaxDistance(double a_magnetMaxDistance)
{
    m_magnetMaxDistance = cClamp0(a_magnetMaxDistance);
}


//===========================================================================
/*!
    Set the maximum force threshold for the stick and slip model [N].

    \fn     void cMaterial::setStickSlipForceMax(double a_stickSlipForceMax)
    \param  a_stickSlipForceMax  Maximum force threshold.
*/
//===========================================================================
void cMaterial::setStickSlipForceMax(double a_stickSlipForceMax)
{
    m_stickSlipForceMax = cClamp0(a_stickSlipForceMax);
}


//===========================================================================
/*!
    Set the stiffness for the stick and slip model [N/m]

    \fn     void cMaterial::setStickSlipStiffness(double a_stickSlipStiffness)
    \param  a_stickSlipStiffness  Stiffness property.
*/
//===========================================================================
void cMaterial::setStickSlipStiffness(double a_stickSlipStiffness)
{
    m_stickSlipStiffness = cClamp0(a_stickSlipStiffness);
}


//===========================================================================
/*!
    Render this material in OpenGL.

    \fn     void cMaterial::render()
*/
//===========================================================================
void cMaterial::render()
{
    glDisable(GL_COLOR_MATERIAL);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, (const float *)&m_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, (const float *)&m_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (const float *)&m_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, (const float *)&m_emission);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, m_shininess);
}


