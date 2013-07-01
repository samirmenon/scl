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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 474 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CMaterialH
#define CMaterialH
//------------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "graphics/CRenderOptions.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file    CMaterial.h
    
    \brief  
    <b> Materials </b> \n 
    Material Properties.
*/
//==============================================================================

//==============================================================================
/*!
      \class      cMaterial
      \ingroup    materials

      \brief      
      Material properties.

      \details
      cMaterial provide a description for handling OpenGL graphic material 
      properties. These include: ambient color, diffuse color, specular color, 
      emissive color, and shininess. \n
    
      Haptic properties are also defined in this class. Properties include 
      stiffness, dynamic friction, and static friction, viscosity, vibration
      and magnetic effects. Force rendering algorithms will lookup the
      material properties of an object to compute the desired force rendering
      effect.
*/
//==============================================================================
struct cMaterial
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
 
public:

    //! Constructor of cMaterial.
    cMaterial();

    //! Destructor of cMaterial.
    virtual ~cMaterial() {};

    //! Render material in OpenGL.
    virtual void render(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COPY:
    //--------------------------------------------------------------------------

public:

    //! Create a copy of current object.
    cMaterial* copy();

    //! Set value to all modification flags. 
    void setModificationFlags(const bool a_value);

    //! Copy all modified members to another material object.
    void copyTo(cMaterial* a_material);

    //! Copy all modified members to another material object.
    void copyTo(cMaterial& a_material);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRAPHIC PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! Set shininess (the exponent used for specular lighting).
    void setShininess(const GLuint a_shininess);

    //! Get shininess level.
    GLuint getShininess() const { return (m_shininess); }

    //! Set transparency level (alpha value) of all color components.
    void setTransparencyLevel(const float a_levelTransparency);

    //! Returns __true__ if material includes partial or full transparency components.
    inline bool isTransparent() const
    {
        return (m_ambient[4] < 1.0 ||
                m_diffuse[4] < 1.0 ||
                m_specular[4] < 1.0 ||
                m_emission[4]);
    }

    //! Backup color properties.
    inline void backupColors()
    {
        m_ambient.backup();
        m_diffuse.backup();
        m_specular.backup();
        m_emission.backup();
    }

    //! Restore color properties from values stored in the backup members.
    inline void restoreColors()
    {
        m_ambient.restore();
        m_diffuse.restore();
        m_specular.restore();
        m_emission.restore();
    }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - GRAPHIC PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! Ambient color.
    cColorf m_ambient;

    //! Diffuse color.
    cColorf m_diffuse;

    //! Specular color.
    cColorf m_specular;

    //! Emission color.
    cColorf m_emission;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC EFFECTS PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! Set general stiffness level [N/m].
    void setStiffness(const double a_stiffness);

    //! Get general stiffness level [N/m].
    inline double getStiffness() const { return (m_stiffness); }

    //! Set level of viscosity.
    void setViscosity(const double a_viscosity);

    //! Get level of viscosity.
    inline double getViscosity() const { return (m_viscosity); }

    //! Set active vibration frequency [Hz].
    void setVibrationFrequency(const double a_vibrationFrequency);

    //! Get active vibration frequency [Hz].
    inline double getVibrationFrequency() const {return (m_vibrationFrequency); }

    //! Set active vibration force amplitude [N].
    void setVibrationAmplitude(const double a_vibrationAmplitude);

    //! Get active vibration force amplitude [N].
    inline double getVibrationAmplitude() const { return (m_vibrationAmplitude); }

    //! Set maximum force applied by the magnet [N]
    void setMagnetMaxForce(const double a_magnetMaxForce);

    //! Get maximum force applied by the magnet [N]
    inline double getMagnetMaxForce() const { return (m_magnetMaxForce); }

    //! Set maximum distance threshold from where magnetic force is perceived [m].
    void setMagnetMaxDistance(const double a_magnetMaxDistance);

    //! Get maximum distance threshold from where magnetic force is perceived [m].
    inline double getMagnetMaxDistance() const { return (m_magnetMaxDistance); }

    //! Set maximum force threshold of stick and slip model [N].
    void setStickSlipForceMax(const double a_stickSlipForceMax);

    //! Get maximum force threshold of stick and slip model [N].
    inline double getStickSlipForceMax() const { return (m_stickSlipForceMax); }

    //! Set stiffness of stick and slip model [N/m]
    void setStickSlipStiffness(double const a_stickSlipStiffness);

    //! Get stiffness of stick and slip model [N/m]
    inline double getStickSlipStiffness() const { return (m_stickSlipStiffness); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC PROPERTIES (MESH OBJECTS ONLY)
    //--------------------------------------------------------------------------

public:

    //! Set damping coefficient.
    void setDamping(const double a_dampingCoefficient);

    //! Get damping coefficient.
    inline double getDamping() const { return (m_damping); }

    //! \b Enable or \b disable haptic friction. (Applies to mesh objects only).
    void setUseHapticFriction(const bool a_useHapticFriction);

    //! Get status of haptic friction. (Applies to mesh objects only).
    inline bool getUseHapticFriction() const { return (m_useHapticFriction); }

    //! Set static friction level [N].
    void setStaticFriction(const double a_friction);

    //! Get static friction level [N].
    inline double getStaticFriction() const { return (m_staticFriction); }

    //! Set dynamic friction level.
    void setDynamicFriction(const double a_friction);

    //! Get dynamic friction level.
    inline double getDynamicFriction() const { return (m_dynamicFriction); }

    //! \b Enable or \b disable haptic texture. (Applies to mesh objects only).
    void setUseHapticTexture(const bool a_useHapticTexture);

    //! Get status of haptic texture. (Applies to mesh objects only).
    inline bool getUseHapticTexture() const { return (m_useHapticTexture); }

    //! Set texture level.
    void setTextureLevel(const double a_textureLevel);

    //! Get texture level.
    inline double getTextureLevel() const { return (m_textureLevel); }

    //! \b Enable or \b disable haptic shading. (Applies to mesh objects only).
    void setUseHapticShading(const bool a_useHapticShading);

    //! Get status of haptic shading. (Applies to mesh objects only).
    inline bool getUseHapticShading() const { return (m_useHapticShading); }

    //! \b Enable or \b disable haptic rendering on \b front side of mesh triangles.
    void setHapticTriangleFrontSide(const bool a_enabled);

    //! Get status about front side triangle haptic rendering. (Applies to mesh objects only).
    inline bool getHapticTriangleFrontSide() const { return (m_hapticFrontSideOfTriangles); }

    //! Enable/Disable rendering of back side of mesh triangles.
    void setHapticTriangleBackSide(const bool a_enabled);

    //! If __true__ then back side of mesh triangles are rendered.
    inline bool getHapticTriangleBackSide() const { return (m_hapticBackSideOfTriangles); }

    //! Enable/Disable rendering of front and back sides of mesh triangles.
    void setHapticTriangleSides(const bool a_enableFrontSide, const bool a_enableBackSide);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COLOR PROPERTIES:
    //--------------------------------------------------------------------------

public:

    //! Define some color properties for material.
    void setColor(cColorf& a_color);

    //! Define some color properties for material.
    void setColor(cColorb& a_color);

    //! Define some color properties for material.
    void setColorf(const GLfloat a_red, 
        const GLfloat a_green, 
        const GLfloat a_blue,
        const GLfloat a_alpha = 1.0f); 


    //--------------------------------------------------------------------------
    //PUBLIC METHODS - RED COLORS
    //--------------------------------------------------------------------------

public:

    //! Set color to Red Indian.
    inline void setRedIndian()              { m_diffuse.setb(0xCD, 0x5C, 0x5C); updateColors();} 

    //! Set color to Light Coral Red. 
    inline void setRedLightCoral()          { m_diffuse.setb(0xF0, 0x80, 0x80); updateColors();}

    //! Set color to Red Salmon.
    inline void setRedSalmon()              { m_diffuse.setb(0xFA, 0x80, 0x72); updateColors();}

    //! Set color to Dark Red Salmon.
    inline void setRedDarkSalmon()          { m_diffuse.setb(0xE9, 0x96, 0x7A); updateColors();}

    //! Set color to Light Red Salmon.
    inline void setRedLightSalmon()         { m_diffuse.setb(0xFF, 0xA0, 0x7A); updateColors();}

    //! Set color to Red Crimson.
    inline void setRedCrimson()             { m_diffuse.setb(0xDC, 0x14, 0x3C); updateColors();}

    //! Set color to Red.
    inline void setRed()                    { m_diffuse.setb(0xFF, 0x00, 0x00); updateColors();}

    //! Set color to Red Fire Brick.
    inline void setRedFireBrick()           { m_diffuse.setb(0xB2, 0x22, 0x22); updateColors();}

    //! Set color to Dark Red.
    inline void setRedDark()                { m_diffuse.setb(0x8B, 0x00, 0x00); updateColors();}

    
    //--------------------------------------------------------------------------
    // PUBLIC METHODS - PINK COLORS
    //--------------------------------------------------------------------------

    //! Set color to Pink.
    inline void setPink() 	                { m_diffuse.setb(0xFF, 0xC0, 0xCB); updateColors();}

    //! Set color to Light Pink.
    inline void setPinkLight() 	            { m_diffuse.setb(0xFF, 0xB6, 0xC); updateColors();}

    //! Set color to Hot Pink.
    inline void setPinkHot() 	            { m_diffuse.setb(0xFF, 0x69, 0xB4); updateColors();}

    //! Set color to Deep Pink.
    inline void setPinkDeep() 	            { m_diffuse.setb(0xFF, 0x14, 0x93); updateColors();}

    //! Set color to Medium Violet Red.
    inline void setPinkMediumVioletRed()    { m_diffuse.setb(0xC7, 0x15, 0x85); updateColors();}

    //! Set color to Pale Violet Red.
    inline void setPinkPaleVioletRed() 	    { m_diffuse.setb(0xDB, 0x70, 0x93); updateColors();}

   
    //--------------------------------------------------------------------------
    // PUBLIC METHODS - ORANGE COLORS
    //--------------------------------------------------------------------------

    //! Set color to Orange Light Salmon.
    inline void setOrangeLightSalmon() 	    { m_diffuse.setb(0xFF, 0xA0, 0x7A); updateColors();}

    //! Set color to Orange Coral.
    inline void setOrangeCoral() 	        { m_diffuse.setb(0xFF, 0x7F, 0x50); updateColors();}

    //! Set color to Orange Tomato.
    inline void setOrangeTomato() 	        { m_diffuse.setb(0xFF, 0x63, 0x47); updateColors();}

    //! Set color to Orange Red.
    inline void setOrangeRed() 	            { m_diffuse.setb(0xFF, 0x45, 0x00); updateColors();}

    //! Set color to Dark Orange.
    inline void setOrangeDark() 	        { m_diffuse.setb(0xFF, 0x8C, 0x00); updateColors();}

    //! Set color to Orange.
    inline void setOrange() 	            { m_diffuse.setb(0xFF, 0xA5, 0x00); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - YELLOW COLORS
    //--------------------------------------------------------------------------

    //! Set color to Gold.
    inline void setYellowGold() 	        { m_diffuse.setb(0xFF, 0xD7, 0x00); updateColors();}

    //! Set color to Yellow.
    inline void setYellow() 	            { m_diffuse.setb(0xFF, 0xFF, 0x00); updateColors();}

    //! Set color to Light Yellow.
    inline void setYellowLight() 	        { m_diffuse.setb(0xFF, 0xFF, 0xE0); updateColors();}

    //! Set color to Lemon Chiffon.
    inline void setYellowLemonChiffon() 	{ m_diffuse.setb(0xFF, 0xFA, 0xCD); updateColors();}

    //! Set color to Light Goldenrod.
    inline void setYellowLightGoldenrod() 	{ m_diffuse.setb(0xFA, 0xFA, 0xD); updateColors();}

    //! Set color to Papaya Whip.
    inline void setYellowPapayaWhip() 	    { m_diffuse.setb(0xFF, 0xEF, 0xD5); updateColors();}

    //! Set color to Moccasin.
    inline void setYellowMoccasin() 	    { m_diffuse.setb(0xFF, 0xE4, 0xB5); updateColors();}

    //! Set color to Peach Puff.
    inline void setYellowPeachPuff() 	    { m_diffuse.setb(0xFF, 0xDA, 0xB9); updateColors();}

    //! Set color to Pale Goldenrod.
    inline void setYellowPaleGoldenrod() 	{ m_diffuse.setb(0xEE, 0xE8, 0xAA); updateColors();}

    //! Set color to Khaki.
    inline void setYellowKhaki() 	        { m_diffuse.setb(0xF0, 0xE6, 0x8C); updateColors();}

    //! Set color to Dark Khaki.
    inline void setYellowDarkKhaki() 	    { m_diffuse.setb(0xBD, 0xB7, 0x6B); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - PURPLE COLORS
    //--------------------------------------------------------------------------

    //! Set color to Lavendar.
    inline void setPurpleLavender() 	    { m_diffuse.setb(0xE6, 0xE6, 0xFA); updateColors();}

    //! Set color to Thistle.
    inline void setPurpleThistle() 	        { m_diffuse.setb(0xD8, 0xBF, 0xD8); updateColors();}

    //! Set color to Plum.
    inline void setPurplePlum() 	        { m_diffuse.setb(0xDD, 0xA0, 0xDD); updateColors();}

    //! Set color to Violet.
    inline void setPurpleViolet() 	        { m_diffuse.setb(0xEE, 0x82, 0xEE); updateColors();}

    //! Set color to Orchid.
    inline void setPurpleOrchid() 	        { m_diffuse.setb(0xDA, 0x70, 0xD6); updateColors();}

    //! Set color to Fuchsia.
    inline void setPurpleFuchsia() 	        { m_diffuse.setb(0xFF, 0x00, 0xFF); updateColors();}

    //! Set color to Magenta.
    inline void setPurpleMagenta() 	        { m_diffuse.setb(0xFF, 0x00, 0xFF); updateColors();}

    //! Set color to Medium Orchid.
    inline void setPurpleMediumOrchid() 	{ m_diffuse.setb(0xBA, 0x55, 0xD3); updateColors();}

    //! Set color to Medium Purple.
    inline void setPurpleMedium() 	        { m_diffuse.setb(0x93, 0x70, 0xDB); updateColors();}

    //! Set color to Amethyst.
    inline void setPurpleAmethyst() 	    { m_diffuse.setb(0x99, 0x66, 0xCC); updateColors();}

    //! Set color to Blue Violet.
    inline void setPurpleBlueViolet() 	    { m_diffuse.setb(0x8A, 0x2B, 0xE2); updateColors();}

    //! Set color to Dark Violet.
    inline void setPurpleDarkViolet() 	    { m_diffuse.setb(0x94, 0x00, 0xD3); updateColors();}

    //! Set color to Dark Orchid.
    inline void setPurpleDarkOrchid() 	    { m_diffuse.setb(0x99, 0x32, 0xCC); updateColors();}

    //! Set color to Dark Magenta.
    inline void setPurpleDarkMagenta() 	    { m_diffuse.setb(0x8B, 0x00, 0x8B); updateColors();}

    //! Set color to Purple.
    inline void setPurple() 	            { m_diffuse.setb(0x80, 0x00, 0x80); updateColors();}

    //! Set color to Indigo.
    inline void setPurpleIndigo() 	        { m_diffuse.setb(0x4B, 0x00, 0x82); updateColors();}

    //! Set color to Slate Blue.
    inline void setPurpleSlateBlue() 	    { m_diffuse.setb(0x6A, 0x5A, 0xCD); updateColors();}

    //! Set color to Dark Slate Blue.
    inline void setPurpleDarkSlateBlue() 	{ m_diffuse.setb(0x48, 0x3D, 0x8B); updateColors();}

    //! Set color to Medium Slate Blue.
    inline void setPurpleMediumSlateBlue() 	{ m_diffuse.setb(0x7B, 0x68, 0xEE); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GREEN COLORS
    //--------------------------------------------------------------------------

    //! Set color to Green Yellow.
    inline void setGreenYellow() 	        { m_diffuse.setb(0xAD, 0xFF, 0x2F); updateColors();}

    //! Set color to Chartreuse.
    inline void setGreenChartreuse() 	    { m_diffuse.setb(0x7F, 0xFF, 0x00); updateColors();}

    //! Set color to Lawn Green.
    inline void setGreenLawn() 	            { m_diffuse.setb(0x7C, 0xFC, 0x00); updateColors();}

    //! Set color to Lime.
    inline void setGreenLime() 	            { m_diffuse.setb(0x00, 0xFF, 0x00); updateColors();}

    //! Set color to Lime Green.
    inline void setGreenLimeGreen() 	    { m_diffuse.setb(0x32, 0xCD, 0x32); updateColors();}

    //! Set color to Pale Green.
    inline void setGreenPale() 	            { m_diffuse.setb(0x98, 0xFB, 0x98); updateColors();}

    //! Set color to Light Green.
    inline void setGreenLight() 	        { m_diffuse.setb(0x90, 0xEE, 0x90); updateColors();}

    //! Set color to Medium Spring Green.
    inline void setGreenMediumSpring() 	    { m_diffuse.setb(0x00, 0xFA, 0x9A); updateColors();}

    //! Set color to Spring Green.
    inline void setGreenSpring() 	        { m_diffuse.setb(0x00, 0xFF, 0x7F); updateColors();}

    //! Set color to Medium Sea Green.
    inline void setGreenMediumSea() 	    { m_diffuse.setb(0x3C, 0xB3, 0x71); updateColors();}

    //! Set color to Sea Green.
    inline void setGreenSea() 	            { m_diffuse.setb(0x2E, 0x8B, 0x57); updateColors();}

    //! Set color to Forest Green.
    inline void setGreenForest() 	        { m_diffuse.setb(0x22, 0x8B, 0x22); updateColors();}

    //! Set color to Green.
    inline void setGreen() 	                { m_diffuse.setb(0x00, 0x80, 0x00); updateColors();}

    //! Set color to Dark Green.
    inline void setGreenDark() 	            { m_diffuse.setb(0x00, 0x64, 0x00); updateColors();}

    //! Set color to Yellow Green.
    inline void setGreenYellowGreen() 	    { m_diffuse.setb(0x9A, 0xCD, 0x32); updateColors();}

    //! Set color to Olive Drab.
    inline void setGreenOliveDrab() 	    { m_diffuse.setb(0x6B, 0x8E, 0x23); updateColors();}

    //! Set color to Olive.
    inline void setGreenOlive() 	        { m_diffuse.setb(0x80, 0x80, 0x00); updateColors();}

    //! Set color to Dark Olive Green.
    inline void setGreenDarkOlive() 	    { m_diffuse.setb(0x55, 0x6B, 0x2F); updateColors();}

    //! Set color to Medium Aquamarine.
    inline void setGreenMediumAquamarine() 	{ m_diffuse.setb(0x66, 0xCD, 0xAA); updateColors();}

    //! Set color to Dark Sea Green.
    inline void setGreenDarkSea() 	        { m_diffuse.setb(0x8F, 0xBC, 0x8F); updateColors();}

    //! Set color to Light Sea Green.
    inline void setGreenLightSea() 	        { m_diffuse.setb(0x20, 0xB2, 0xAA); updateColors();}

    //! Set color to Dark Cyan.
    inline void setGreenDarkCyan() 	        { m_diffuse.setb(0x00, 0x8B, 0x8B); updateColors();}

    //! Set color to Teal.
    inline void setGreenTeal() 	            { m_diffuse.setb(0x00, 0x80, 0x80); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BLUE COLORS
    //--------------------------------------------------------------------------

    //! Set color to Aqua.
    inline void setBlueAqua() 	            { m_diffuse.setb(0x00, 0xFF, 0xFF); updateColors();}

    //! Set color to Cyan.
    inline void setBlueCyan() 	            { m_diffuse.setb(0x00, 0xFF, 0xFF); updateColors();}

    //! Set color to Light Cyan.
    inline void setBlueLightCyan() 	        { m_diffuse.setb(0xE0, 0xFF, 0xFF); updateColors();}

    //! Set color to Pale Turquoise.
    inline void setBluePaleTurquoise() 	    { m_diffuse.setb(0xAF, 0xEE, 0xEE); updateColors();}

    //! Set color to Aquamarine.
    inline void setBlueAquamarine() 	    { m_diffuse.setb(0x7F, 0xFF, 0xD4); updateColors();}

    //! Set color to Turquoise.
    inline void setBlueTurquoise() 	        { m_diffuse.setb(0x40, 0xE0, 0xD0); updateColors();}

    //! Set color to Medium Turquoise.
    inline void setBlueMediumTurquoise() 	{ m_diffuse.setb(0x48, 0xD1, 0xCC); updateColors();}

    //! Set color to Dark Turquoise.
    inline void setBlueDarkTurquoise() 	    { m_diffuse.setb(0x00, 0xCE, 0xD1); updateColors();}

    //! Set color to Cadet Blue.
    inline void setBlueCadet() 	            { m_diffuse.setb(0x5F, 0x9E, 0xA0); updateColors();}

    //! Set color to Steel Blue.
    inline void setBlueSteel() 	            { m_diffuse.setb(0x46, 0x82, 0xB4); updateColors();}

    //! Set color to Light Steel Blue.
    inline void setBlueLightSteel() 	    { m_diffuse.setb(0xB0, 0xC4, 0xDE); updateColors();}

    //! Set color to Powder Blue.
    inline void setBluePowder() 	        { m_diffuse.setb(0xB0, 0xE0, 0xE6); updateColors();}

    //! Set color to Light Blue.
    inline void setBlueLight() 	            { m_diffuse.setb(0xAD, 0xD8, 0xE6); updateColors();}

    //! Set color to Sky Blue.
    inline void setBlueSky() 	            { m_diffuse.setb(0x87, 0xCE, 0xEB); updateColors();}

    //! Set color to Light Sky Blue.
    inline void setBlueLightSky() 	        { m_diffuse.setb(0x87, 0xCE, 0xFA); updateColors();}

    //! Set color to Deep Sky Blue.
    inline void setBlueDeepSky() 	        { m_diffuse.setb(0x00, 0xBF, 0xFF); updateColors();}

    //! Set color to Doger Blue.
    inline void setBlueDodger() 	        { m_diffuse.setb(0x1E, 0x90, 0xFF); updateColors();}

    //! Set color to Cornflower Blue.
    inline void setBlueCornflower() 	    { m_diffuse.setb(0x64, 0x95, 0xED); updateColors();}

    //! Set color to Medium Slate Blue.
    inline void setBlueMediumSlate() 	    { m_diffuse.setb(0x7B, 0x68, 0xEE); updateColors();}

    //! Set color to Royal Blue.
    inline void setBlueRoyal() 	            { m_diffuse.setb(0x41, 0x69, 0xE1); updateColors();}

    //! Set color to Blue.
    inline void setBlue() 	                { m_diffuse.setb(0x00, 0x00, 0xFF); updateColors();}

    //! Set color to Medium Blue.
    inline void setBlueMedium() 	        { m_diffuse.setb(0x00, 0x00, 0xCD); updateColors();}

    //! Set color to Dark Blue.
    inline void setBlueDark() 	            { m_diffuse.setb(0x00, 0x00, 0x8B); updateColors();}

    //! Set color to Navy.
    inline void setBlueNavy() 	            { m_diffuse.setb(0x00, 0x00, 0x80); updateColors();}

    //! Set color to Midnight Blue.
    inline void setBlueMidnight() 	        { m_diffuse.setb(0x19, 0x19, 0x70); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BROWN COLORS
    //--------------------------------------------------------------------------

    //! Set color to Cornsilk.
    inline void setBrownCornsilk() 	        { m_diffuse.setb(0xFF, 0xF8, 0xDC); updateColors();}

    //! Set color to Blanched Almond.
    inline void setBrownBlanchedAlmond() 	{ m_diffuse.setb(0xFF, 0xEB, 0xCD); updateColors();}

    //! Set color to Bisque.
    inline void setBrownBisque() 	        { m_diffuse.setb(0xFF, 0xE4, 0xC4); updateColors();}

    //! Set color to Navajo White.
    inline void setBrownNavajoWhite() 	    { m_diffuse.setb(0xFF, 0xDE, 0xAD); updateColors();}

    //! Set color to Wheat.
    inline void setBrownWheat() 	        { m_diffuse.setb(0xF5, 0xDE, 0xB3); updateColors();}

    //! Set color to Burly Wood.
    inline void setBrownBurlyWood() 	    { m_diffuse.setb(0xDE, 0xB8, 0x87); updateColors();}

    //! Set color to Tan.
    inline void setBrownTan() 	            { m_diffuse.setb(0xD2, 0xB4, 0x8C); updateColors();}

    //! Set color to Rosy Brown.
    inline void setBrownRosy() 	            { m_diffuse.setb(0xBC, 0x8F, 0x8F); updateColors();}

    //! Set color to Sandy Brown.
    inline void setBrownSandy() 	        { m_diffuse.setb(0xF4, 0xA4, 0x60); updateColors();}

    //! Set color to Brown Goldenrod.
    inline void setBrownGoldenrod() 	    { m_diffuse.setb(0xDA, 0xA5, 0x20); updateColors();}

    //! Set color to Dark Brown Goldenrod.
    inline void setBrownDarkGoldenrod() 	{ m_diffuse.setb(0xB8, 0x86, 0x0B); updateColors();}

    //! Set color to Peru.
    inline void setBrownPeru() 	            { m_diffuse.setb(0xCD, 0x85, 0x3F); updateColors();}

    //! Set color to Chocolate.
    inline void setBrownChocolate() 	    { m_diffuse.setb(0xD2, 0x69, 0x1E); updateColors();}

    //! Set color to Saddle Brown.
    inline void setBrownSaddle() 	        { m_diffuse.setb(0x8B, 0x45, 0x13); updateColors();}

    //! Set color to Sienna.
    inline void setBrownSienna() 	        { m_diffuse.setb(0xA0, 0x52, 0x2D); updateColors();}

    //! Set color to Brown.
    inline void setBrown() 	                { m_diffuse.setb(0xA5, 0x2A, 0x2A); updateColors();}

    //! Set color to Maroon.
    inline void setBrownMaroon() 	        { m_diffuse.setb(0x80, 0x00, 0x00); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - WHITE COLORS
    //--------------------------------------------------------------------------

    //! Set color to White.
    inline void setWhite() 	                { m_diffuse.setb(0xFF, 0xFF, 0xFF); updateColors();}

    //! Set color to White Snow.
    inline void setWhiteSnow() 	            { m_diffuse.setb(0xFF, 0xFA, 0xFA); updateColors();}

    //! Set color to Honeydew.
    inline void setWhiteHoneydew() 	        { m_diffuse.setb(0xF0, 0xFF, 0xF0); updateColors();}

    //! Set color to Mint Cream.
    inline void setWhiteMintCream() 	    { m_diffuse.setb(0xF5, 0xFF, 0xFA); updateColors();}

    //! Set color to Azure.
    inline void setWhiteAzure() 	        { m_diffuse.setb(0xF0, 0xFF, 0xFF); updateColors();}

    //! Set color to Alice Blue.
    inline void setWhiteAliceBlue() 	    { m_diffuse.setb(0xF0, 0xF8, 0xFF); updateColors();}

    //! Set color to Ghost White.
    inline void setWhiteGhost() 	        { m_diffuse.setb(0xF8, 0xF8, 0xFF); updateColors();}

    //! Set color to White Smoke.
    inline void setWhiteSmoke() 	        { m_diffuse.setb(0xF5, 0xF5, 0xF5); updateColors();}

    //! Set color to Seashell.
    inline void setWhiteSeashell() 	        { m_diffuse.setb(0xFF, 0xF5, 0xEE); updateColors();}

    //! Set color to Beige.
    inline void setWhiteBeige() 	        { m_diffuse.setb(0xF5, 0xF5, 0xDC); updateColors();}

    //! Set color to Old Lace.
    inline void setWhiteOldLace() 	        { m_diffuse.setb(0xFD, 0xF5, 0xE6); updateColors();}

    //! Set color to Floral White.
    inline void setWhiteFloral() 	        { m_diffuse.setb(0xFF, 0xFA, 0xF0); updateColors();}

    //! Set color to Ivory.
    inline void setWhiteIvory() 	        { m_diffuse.setb(0xFF, 0xFF, 0xF0); updateColors();}

    //! Set color to Antique White.
    inline void setWhiteAntique() 	        { m_diffuse.setb(0xFA, 0xEB, 0xD7); updateColors();}

    //! Set color to Linen.
    inline void setWhiteLinen() 	        { m_diffuse.setb(0xFA, 0xF0, 0xE6); updateColors();}

    //! Set color to Lavender Blush.
    inline void setWhiteLavenderBlush() 	{ m_diffuse.setb(0xFF, 0xF0, 0xF5); updateColors();}

    //! Set color to Misty Rose.
    inline void setWhiteMistyRose() 	    { m_diffuse.setb(0xFF, 0xE4, 0xE1); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GRAY COLORS
    //--------------------------------------------------------------------------

    //! Set color to Gainsboro.
    inline void setGrayGainsboro() 	        { m_diffuse.setb(0xDC, 0xDC, 0xDC); updateColors();}

    //! Set color to Light Gray.
    inline void setGrayLight() 	            { m_diffuse.setb(0xD3, 0xD3, 0xD3); updateColors();}

    //! Set color to Silver.
    inline void setGraySilver() 	        { m_diffuse.setb(0xC0, 0xC0, 0xC0); updateColors();}

    //! Set color to Dark Gray.
    inline void setGrayDark() 	            { m_diffuse.setb(0xA9, 0xA9, 0xA9); updateColors();}

    //! Set color to Gray.
    inline void setGray() 	                { m_diffuse.setb(0x80, 0x80, 0x80); updateColors();}

    //! Set color to Dim Gray.
    inline void setGrayDim() 	            { m_diffuse.setb(0x69, 0x69, 0x69); updateColors();}

    //! Set color to Light Slate Gray.
    inline void setGrayLightSlate() 	    { m_diffuse.setb(0x77, 0x88, 0x99); updateColors();}

    //! Set color to Slate Gray.
    inline void setGraySlate() 	            { m_diffuse.setb(0x70, 0x80, 0x90); updateColors();}

    //! Set color to Dark Slate Gray.
    inline void setGrayDarkSlate() 	        { m_diffuse.setb(0x2F, 0x4F, 0x4F); updateColors();}

    //! Set color to Black.
    inline void setBlack() 	                { m_diffuse.setb(0x00, 0x00, 0x00); updateColors();}


    //--------------------------------------------------------------------------
    // PUBLIC METHODS -CUSTOM GRAY COLOR
    //--------------------------------------------------------------------------

public:

    // Set custom gray level.
    inline void setGrayLevel(const GLfloat a_level) { m_diffuse.set(a_level, a_level, a_level); updateColors(); }


    //--------------------------------------------------------------------------
    // PROTECTED METHODS - GRAPHICS:
    //--------------------------------------------------------------------------
    
protected:

    //! Takes current \b diffuse color and updates \b ambient and \b specular components.
    void updateColors();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - GRAPHICS PROPERTIES:
    //--------------------------------------------------------------------------

protected:

    //! Material shininess level.
    GLuint m_shininess;

    //! Flag to track if the member has been modified.
    bool m_flag_shininess;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - HAPTIC PROPERTIES:
    //--------------------------------------------------------------------------

protected:

    //! Level of viscosity.
    double m_viscosity;

    //! Flag to track if related member has been modified.
    bool m_flag_viscosity;

    //! Stiffness [N/m].
    double m_stiffness;

    //! Flag to track if related member has been modified.
    bool m_flag_stiffness;

    // Damping level.
    double m_damping;

    //! Flag to track if related member has been modified.
    bool m_flag_damping;

    //! Static friction constant [N].
    double m_staticFriction;

    //! Flag to track if related member has been modified.
    bool m_flag_staticFriction;

    //! Dynamic friction constant [N].
    double m_dynamicFriction;

    //! Flag to track if related member has been modified.
    bool m_flag_dynamicFriction;

    //! Dynamic friction constant.
    double m_textureLevel;

    //! Flag to track if related member has been modified.
    bool m_flag_textureLevel;

    //! Frequency of vibrations [Hz].
    double m_vibrationFrequency;

    //! Flag to track if related member has been modified.
    bool m_flag_vibrationFrequency;

    //! Amplitude of vibrations [Hz].
    double m_vibrationAmplitude;

    //! Flag to track if related member has been modified.
    bool m_flag_vibrationAmplitude;

    //! Maximum force applied by magnetic effect [N].
    double m_magnetMaxForce;

    //! Flag to track if related member has been modified.
    bool m_flag_magnetMaxForce;

    //! Maximum distance from the object where the magnetic force can be perceived.
    double m_magnetMaxDistance;

    //! Flag to track if related member has been modified.
    bool m_flag_magnetMaxDistance;

    //! Force threshold for stick and slip effect [N].
    double m_stickSlipForceMax;

    //! Flag to track if related member has been modified.
    bool m_flag_stickSlipForceMax;

    //! Stiffness of stick slip model.
    double m_stickSlipStiffness;

    //! Flag to track if related member has been modified.
    bool m_flag_stickSlipStiffness;

    //! If __true__, haptic friction rendering is enabled.
    bool m_useHapticFriction;

    //! Flag to track if related member has been modified.
    bool m_flag_useHapticFriction;

    //! If __true__, haptic texture rendering is enabled.
    bool m_useHapticTexture;

    //! Flag to track if related member has been modified.
    bool m_flag_useHapticTexture;

    //! If __true__, haptic shading is enabled.
    bool m_useHapticShading;

    //! Flag to track if related member has been modified.
    bool m_flag_useHapticShading;

    //! If __true__, then front side of triangles are rendered haptically (used by the proxy algorithm).
    bool m_hapticFrontSideOfTriangles;

    //! Flag to track if related member has been modified.
    bool m_flag_hapticFrontSideOfTriangles;

    //! If __true__, then back side of triangles are rendered haptically (used by the proxy algorithm).
    bool m_hapticBackSideOfTriangles;

    //! Flag to track if related member has been modified.
    bool m_flag_hapticBackSideOfTriangles;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

