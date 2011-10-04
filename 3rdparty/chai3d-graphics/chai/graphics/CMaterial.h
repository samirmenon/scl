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
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CMaterialH
#define CMaterialH
//---------------------------------------------------------------------------
#include "graphics/CColor.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CMaterial.h
    
    \brief  
    <b> Graphics </b> \n 
    Material Properties.
*/
//===========================================================================

//===========================================================================
/*!
      \class      cMaterial
      \ingroup    graphics

      \brief      
      cMaterial provide a description for handling OpenGL graphic material 
      properties. These include: ambient color, diffuse color, specular color, 
      emissive color, and shininess. \n
    
      Haptic properties are also defined in this class. Properties include 
      stiffness, dynamic friction, and static friction, viscosity, vibration
      and magnetic effects. Force rendering algorithms will lookup the
      material properties of an object to compute the desired force rendering
      effect.
*/
//===========================================================================
struct cMaterial
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    
      //! Constructor of cMaterial.
    cMaterial();

    //! Destructor of cMaterial.
    ~cMaterial() {};

    //! Render the material in OpenGL.
    virtual void render();


    //-----------------------------------------------------------------------
    // METHODS - GRAPHIC PROPERTIES:
    //-----------------------------------------------------------------------

    //! Set shininess (the exponent used for specular lighting).
    void setShininess(GLuint a_shininess);

    //! Get shininess.
    GLuint getShininess() { return (m_shininess); }

    //! set transparency level (sets the alpha value for all color properties).
    void setTransparencyLevel(float a_levelTransparency);

    //! tells you whether this material includes partial transparency.
    inline bool isTransparent() const
    {
        return (m_ambient[4] < 1.0 ||
                m_diffuse[4] < 1.0 ||
                m_specular[4] < 1.0 ||
                m_emission[4]);
    }


    //-----------------------------------------------------------------------
    // MEMBERS - GRAPHIC PROPERTIES:
    //-----------------------------------------------------------------------

    //! Ambient color.
    cColorf m_ambient;

    //! Diffuse color.
    cColorf m_diffuse;

    //! Specular color.
    cColorf m_specular;

    //! Emissive color.
    cColorf m_emission;


    //-----------------------------------------------------------------------
    // METHODS - HAPTIC PROPERTIES:
    //-----------------------------------------------------------------------

    //! Set stiffness level [N/m]
    void setStiffness(double a_stiffness);

    //! Get stiffness level [N/m]
    inline double getStiffness() const { return (m_stiffness); }

    //! Set static friction level
    void setStaticFriction(double a_friction);

    //! Get static friction level
    inline double getStaticFriction() const { return (m_static_friction); }

    //! Set dynamic friction level
    void setDynamicFriction(double a_friction);

    //! Get dynamic friction level
    inline double getDynamicFriction() const { return (m_dynamic_friction); }

    //! Set level of viscosity
    void setViscosity(double a_viscosity);

    //! Get level of viscosity
    inline double getViscosity() { return (m_viscosity); }

    //! Set vibration frequency [Hz]
    void setVibrationFrequency(double a_vibrationFrequency);

    //! Get vibration frequency [Hz]
    inline double getVibrationFrequency() {return (m_vibrationFrequency); }

    //! Set vibration amplitude [max N]
    void  setVibrationAmplitude(double a_vibrationAmplitude);

    //! Get vibration amplitude [max N]
    inline double getVibrationAmplitude() {return (m_vibrationAmplitude); }

    //! Set the maximum force applied by the magnet [N]
    void setMagnetMaxForce(double a_magnetMaxForce);

    //! Get the maximum force applied by the magnet [N]
    inline double getMagnetMaxForce() { return (m_magnetMaxForce); }

    //! Set the maximum distance from the object where the force can be perceived [m]
    void setMagnetMaxDistance(double a_magnetMaxDistance);

    //! Get the maximum distance from the object where the force can be perceived [m]
    inline double getMagnetMaxDistance() { return (m_magnetMaxDistance); }

    //! Set the maximum force threshold for the stick and slip model [N]
    void setStickSlipForceMax(double a_stickSlipForceMax);

    //! Get the maximum force threshold for the stick and slip model [N]
    inline double getStickSlipForceMax() { return (m_stickSlipForceMax); }

    //! Set the stiffness for the stick and slip model [N/m]
    void setStickSlipStiffness(double a_stickSlipStiffness);

    //! Get the stiffness for the stick and slip model [N/m]
    inline double getStickSlipStiffness() { return (m_stickSlipStiffness); }


  protected:

    //-----------------------------------------------------------------------
    // MEMBERS - GRAPHICS PROPERTIES:
    //-----------------------------------------------------------------------

    //! OpenGL shininess
    GLuint m_shininess;


    //-----------------------------------------------------------------------
    // MEMBERS - HAPTIC PROPERTIES:
    //-----------------------------------------------------------------------

    //! Level of viscosity.
    double m_viscosity;

    //! Stiffness [Netwons per meter].
    double m_stiffness;

    //! Static friction constant.
    double m_static_friction;

    //! Dynamic friction constant.
    double m_dynamic_friction;

    //! Frequency of vibrations.
    double m_vibrationFrequency;

    //! Amplitude of vibrations.
    double m_vibrationAmplitude;

    //! Maximum force applied by magnet effect.
    double m_magnetMaxForce;

    //! Maximum distance from the object where the magnetic force can be perceived.
    double m_magnetMaxDistance;

    //! Force threshold for stick and slip effect.
    double m_stickSlipForceMax;

    //! Spring stiffness of stick slip model.
    double m_stickSlipStiffness;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

