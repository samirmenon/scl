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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 724 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CRenderOptionsH
#define CRenderOptionsH
//---------------------------------------------------------------------------
class cCamera;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CRenderOptions.h
    
    \brief  
    <b> Graphics </b> \n 
    World rendering options.
*/
//===========================================================================


//===========================================================================
/*!
      \class      cRenderOptions
      \ingroup    graphics

      \brief      
      cRenderOptions provides a description on what elements need to
	  be rendered in the world during a pass. \n
*/
//===========================================================================
struct cRenderOptions
{
    //! Pointer to the current camera from which the scene is being rendered.
    cCamera* m_camera;

    //! Is set to true if the scene is rendered in a single pass only (Scene with no transparent objects)
	bool m_single_pass_only;

    //! When using multiple rendering passes, set to true to render opaque objects only.
	bool m_render_opaque_objects_only;

    //! When using multiple rendering passes, set to true to render front faces of transparent objects only.
    bool m_render_transparent_front_faces_only;
	
    //! When using multiple rendering passes, set to true to render back faces of transparent objects only.
    bool m_render_transparent_back_faces_only;

    //! If set to true, then enabled light sources are activated.
	bool m_enable_lighting;

    //! If set to true then material properties are rendered.
	bool m_render_materials;

    //! If set to true then texture properties are rendered.
    bool m_render_textures;

    //! If set to true then this means that a depth map is being rendered for shadow casting.
	bool m_creating_shadow_map;

    //! If set to true, then shadows are being rendered if enabled.
	bool m_rendering_shadow;

    //! Temp - To be defined!
	double m_shadow_light_level;

    //! If set to true, the position of objects are frozen,
	bool m_storeObjectPositions;

    //! If set to true, then reset display lists and texture objects.
    bool m_resetDisplay;
};


//===========================================================================
/*!
    Returns \b true if parts using material properties with/without 
    transparency can be rendered.

    \param  a_options  Rendering options
    \param  a_useTransparency  Indicates if current object uses transparency.
    \return Return result as described above.
*/
//===========================================================================
inline bool SECTION_RENDER_PARTS_WITH_MATERIALS(cRenderOptions a_options, bool a_useTransparency)
{
	return (!(
		((a_options.m_render_opaque_objects_only) && (a_useTransparency)) ||
		((a_options.m_render_transparent_front_faces_only || a_options.m_render_transparent_back_faces_only) && (!a_useTransparency))
		));
}


//===========================================================================
/*!
    Returns \b true if opaque parts only can be rendered.

    \param  a_options  Rendering options
    \return Return result as described above.
*/
//===========================================================================
inline bool SECTION_RENDER_OPAQUE_PARTS_ONLY(cRenderOptions a_options)
{
	return(!(a_options.m_render_transparent_back_faces_only ||
		     a_options.m_render_transparent_front_faces_only));
}


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
