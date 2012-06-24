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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 831 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CVertexH
#define CVertexH
//---------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "graphics/CColor.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CVertex.h
    
    \brief  
    <b> Graphics </b> \n 
    3D Vertex Point.
*/
//===========================================================================

//===========================================================================
/*!
    \struct     cVertex
    \ingroup    graphics

    \brief      
    cVertex defines a point in 3 dimensional space and the associated 
    rendering properties (position, color, texture coordinate, 
    and surface normal)                  
*/
//===========================================================================
class cVertex
{
    public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Constructor of cVertex.

		\param	a_x	 X component.
		\param  a_y	 Y component.
		\param	a_z	 Z component.
    */
    //-----------------------------------------------------------------------
    cVertex(const double a_x = 0.0, 
            const double a_y = 0.0, 
            const double a_z = 0.0)
            : m_localPos(a_x, a_y, a_z), 
              m_globalPos(a_x, a_y, a_z), 
              m_normal(0.0, 0.0, 1.0),
              m_index(-1), 
              m_allocated(false), 
              m_nTriangles(0)
    {}
     

    //-----------------------------------------------------------------------
    /*!
        Destructor of cVertex.
    */
    //-----------------------------------------------------------------------
    ~cVertex() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Set the position coordinates of vertex.

        \param	a_x	 X component.
        \param  a_y	 Y component.
        \param	a_z	 Z component.
    */
    //-----------------------------------------------------------------------
    inline void setLocalPos(const double& a_x, 
                       const double& a_y, 
                       const double& a_z)
    {
        m_localPos.set(a_x, a_y, a_z);
    }


    //-----------------------------------------------------------------------
    /*!
        Set local position of vertex.

        \param      a_pos  Local position of vertex.
    */
    //-----------------------------------------------------------------------
    inline void setLocalPos(const cVector3d& a_pos)
    {
        m_localPos = a_pos;
    }


    //-----------------------------------------------------------------------
    /*!
        Translate vertex by defining a translation passed as parameter.

        \param      a_translation  Translation vector.
    */
    //-----------------------------------------------------------------------
    inline void translate(const cVector3d& a_translation)
    {
        m_localPos.add(a_translation);
    }


    //-----------------------------------------------------------------------
    /*!
        Read local position of vertex.

        \return     Return position of vertex.
    */
    //-----------------------------------------------------------------------
    inline cVector3d getLocalPos() const 
    { 
        return (m_localPos); 
    }


    //-----------------------------------------------------------------------
    /*!
        Read global position. This value is only correct if the
        computeGlobalPositions() method is called from the parent world.

        \return     Return global position of vertex in world coordinates.
    */
    //-----------------------------------------------------------------------
    inline cVector3d getGlobalPos() const 
    { 
        return (m_globalPos); 
    }


    //-----------------------------------------------------------------------
    /*!
        Set normal vector of vertex.

        \param      a_normal  Normal vector.
    */
    //-----------------------------------------------------------------------
    inline void setNormal(const cVector3d& a_normal)
    {
        m_normal = a_normal;
    }


    //-----------------------------------------------------------------------
    /*!
        Set normal vector of vertex by passing its X,Y and Z components
        as parameters.

        \param	a_x	 X component.
        \param  a_y	 Y component.
        \param	a_z	 Z component.
    */
    //-----------------------------------------------------------------------
    inline void setNormal(const double& a_x, 
                          const double& a_y, 
                          const double& a_z)
    {
        m_normal.set(a_x, a_y, a_z);
    }


    //-----------------------------------------------------------------------
    /*!
        Set normal vector of vertex.

        \return     Return normal vector.
    */
    //-----------------------------------------------------------------------
    inline cVector3d getNormal() const
    {
        return (m_normal);
    }


    //-----------------------------------------------------------------------
    /*!
        Set texture coordinate of vertex.

        \param      a_texCoord  Texture coordinate.
    */
    //-----------------------------------------------------------------------
    inline void setTexCoord(const cVector3d& a_texCoord)
    {
        m_texCoord = a_texCoord;
    }


    //-----------------------------------------------------------------------
    /*!
        Set texture coordinate by passing its coordinates as parameters.

        \param	    a_tx	 X component.
        \param      a_ty	 Y component.
    */
    //-----------------------------------------------------------------------
    inline void setTexCoord(const double& a_tx, 
                            const double& a_ty)
    {
        m_texCoord.set(a_tx, a_ty, 0.0);
    }


    //-----------------------------------------------------------------------
    /*!
        Read texture coordinate of vertex.

        \return     Return texture coordinate.
    */
    //-----------------------------------------------------------------------
    inline cVector3d getTexCoord() const 
    { 
        return (m_texCoord); 
    }


    //-----------------------------------------------------------------------
    /*!
        Set color of vertex.

        \param      a_color  Color.
    */
    //-----------------------------------------------------------------------
    inline void setColor(const cColorf& a_color) 
    { 
        m_color = a_color; 
    }


    //-----------------------------------------------------------------------
    /*!
        Set color of vertex.

        \param    a_red    Red component.
        \param    a_green  Green component.
        \param    a_blue   Blue component.
        \param    a_alpha  Alpha component.
    */
    //-----------------------------------------------------------------------
    inline void setColor(const float& a_red, 
                         const float& a_green,
                         const float& a_blue, 
                         const float& a_alpha = 1.0 )
    {
        m_color.set(a_red, a_green, a_blue, a_alpha);
    }


    //-----------------------------------------------------------------------
    /*!
        Set color of vertex.

        \param    a_color  Color component to be set to vertex.
    */
    //-----------------------------------------------------------------------
    inline void setColor(const cColorb& a_color)
    {
        m_color = a_color.getColorf();
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the global position of vertex given the global position
        and global rotation matrix of the parent object.

        \param    a_globalPos  Global position vector of parent.
        \param    a_globalRot  Global rotation matrix of parent.
    */
    //-----------------------------------------------------------------------
    inline void computeGlobalPosition(const cVector3d& a_globalPos, 
                                      const cMatrix3d& a_globalRot)
    {
        a_globalRot.mulr(m_localPos, m_globalPos);
        m_globalPos.add(a_globalPos);
    }


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    public:

    //! Local position of this vertex.
    cVector3d m_localPos;

    //! Global position of this vertex in world coordinates.
    cVector3d m_globalPos;

    //! Surface normal.
    cVector3d m_normal;

    //! Texture coordinate (uvw).
    cVector3d m_texCoord;

    //! Color.
    cColorf m_color;

    //! My index in the vertex list of the mesh that owns me.
    int m_index;

    //! Is this vertex allocated?
    bool m_allocated;

    //! How many triangles use this vertex?
    int m_nTriangles;

	//! User data.
	int m_tag;
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
