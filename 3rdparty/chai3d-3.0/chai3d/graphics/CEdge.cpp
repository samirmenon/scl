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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 699 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "graphics/CEdge.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cEdge.

    \param  a_vertex0  Vertex 0 of edge.
    \param  a_vertex1  Vertex 1 of edge.
*/
//==============================================================================
cEdge::cEdge(cVertex* a_vertex0, 
             cVertex* a_vertex1)
{
	m_tag = 0;
    set(a_vertex0, a_vertex1);
}


//==============================================================================
/*!
    Set edge points.

    \param  a_vertex0  Vertex 0 of edge.
    \param  a_vertex1  Vertex 1 of edge.
*/
//==============================================================================
void cEdge::set(cVertex* a_vertex0, 
                cVertex* a_vertex1)
{
    // sanity check
    if ((a_vertex0 == NULL) || (a_vertex1 == 0))
    {
        return;
    }

    // sort order of points
    if (a_vertex0->getLocalPos().x() > a_vertex1->getLocalPos().x())
    {
        m_vertex0 = a_vertex0;
        m_vertex1 = a_vertex1;
    }
    else if (a_vertex0->getLocalPos().x() < a_vertex1->getLocalPos().x())
    {
        m_vertex1 = a_vertex0;
        m_vertex0 = a_vertex1;
    }
    else
    {
        if (a_vertex0->getLocalPos().y() > a_vertex1->getLocalPos().y())
        {
            m_vertex0 = a_vertex0;
            m_vertex1 = a_vertex1;
        }
        else if (a_vertex0->getLocalPos().y() < a_vertex1->getLocalPos().y())
        {
            m_vertex1 = a_vertex0;
            m_vertex0 = a_vertex1;
        }
        else
        {
            if (a_vertex0->getLocalPos().z() > a_vertex1->getLocalPos().z())
            {
                m_vertex0 = a_vertex0;
                m_vertex1 = a_vertex1;
            }
            else if (a_vertex0->getLocalPos().z() < a_vertex1->getLocalPos().z())
            {
                m_vertex1 = a_vertex0;
                m_vertex0 = a_vertex1;
            }
            else
            {
                m_vertex0 = a_vertex0;
                m_vertex1 = a_vertex1;
            }
        }
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
