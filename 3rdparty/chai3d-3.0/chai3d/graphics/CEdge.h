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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 733 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CEdgeH
#define CEdgeH
//------------------------------------------------------------------------------
#include "graphics/CVertex.h"
#include "math/CMaths.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CEdge.h
    
    \brief  
    <b> Graphics </b> \n 
    Edge composed of two points.
*/
//==============================================================================

//==============================================================================
/*!
    \struct     cEdge
    \ingroup    graphics

    \brief
    3D Edge (defined by two vertices).

    \details
    cEdge is defined by two vertices (points) in 3 dimensional space.
*/
//==============================================================================
struct cEdge
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cEdge.
    cEdge(){m_tag = 0; m_vertex0 = NULL; m_vertex1 = NULL;};

    //! Constructor of cEdge.
    cEdge(cVertex* a_vertex0, 
          cVertex* a_vertex1);
   
    //! Destructor of cEdge.
    ~cEdge() {};


	//--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Set edge points.
    void set(cVertex* a_vertex0, 
             cVertex* a_vertex1);

	//--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Pointer to vertex 0.
    cVertex* m_vertex0;

    //! Pointer to vertex 1.
    cVertex* m_vertex1;

    //! Triangle reference.
    int m_triangle;

	//! User data.
	int m_tag;
};

//------------------------------------------------------------------------------
// OPERTAOR - (IMPLEMENTED FOR EDGE SORTING PURPOSES)
//--------------------------------------------------------------------------
inline bool operator<(const cEdge& a_edge0, const cEdge& a_edge1)
{
    double tol = 0.0;

    if (cAbs(a_edge0.m_vertex0->getLocalPos().x() - a_edge1.m_vertex0->getLocalPos().x()) <= tol)
    {
        if (cAbs(a_edge0.m_vertex0->getLocalPos().y() - a_edge1.m_vertex0->getLocalPos().y()) <= tol)
        {
            if (cAbs(a_edge0.m_vertex0->getLocalPos().z() - a_edge1.m_vertex0->getLocalPos().z()) <= tol)
            {
                if (cAbs(a_edge0.m_vertex1->getLocalPos().x() - a_edge1.m_vertex1->getLocalPos().x()) <= tol)
                {
                    if (cAbs(a_edge0.m_vertex1->getLocalPos().y() - a_edge1.m_vertex1->getLocalPos().y()) <= tol)
                    {
                        return (a_edge0.m_vertex1->getLocalPos().z() < a_edge1.m_vertex1->getLocalPos().z());
                    }
                    else
                    {
                        return (a_edge0.m_vertex1->getLocalPos().y() < a_edge1.m_vertex1->getLocalPos().y());
                    }
                }
                else
                {
                    return (a_edge0.m_vertex1->getLocalPos().x() < a_edge1.m_vertex1->getLocalPos().x());
                }
            }
            else
            {
                return (a_edge0.m_vertex0->getLocalPos().z() < a_edge1.m_vertex0->getLocalPos().z());
            }
        }
        else
        {
            return (a_edge0.m_vertex0->getLocalPos().y() < a_edge1.m_vertex0->getLocalPos().y());
        }
    }
    else
    {
        return (a_edge0.m_vertex0->getLocalPos().x() < a_edge1.m_vertex0->getLocalPos().x());
    }
}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
