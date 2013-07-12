//===========================================================================
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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1047 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CGELSkeletonLink.h"
#include "CGELMesh.h"
using namespace chai3d;
//---------------------------------------------------------------------------

//===========================================================================
// DEFINITION - DEFAULT VALUES:
//===========================================================================

// Physical properties:
double cGELSkeletonLink::default_kSpringElongation = 100.0; // [N/m]
double cGELSkeletonLink::default_kSpringFlexion    = 0.1;   // [Nm/RAD]
double cGELSkeletonLink::default_kSpringTorsion    = 0.1;   // [Nm/RAD]

// Graphical properties:
cColorf cGELSkeletonLink::default_color(0.2f, 0.2f, 1.0f);


//===========================================================================
/*!
    Constructor of cGELSkeletonLink.

    \fn     cGELSkeletonLink::cGELSkeletonLink(cGELSkeletonNode* a_node0,
                                               cGELSkeletonNode* a_node1)
    \param  a_node0  Node 0.
    \param  a_node1  Node 1.
*/
//===========================================================================
cGELSkeletonLink::cGELSkeletonLink(cGELSkeletonNode* a_node0, cGELSkeletonNode* a_node1)
{
    // Set nodes:
    m_node0 = a_node0;
    m_node1 = a_node1;
	m_enabled = true;

    // set position references
    m_wLink01 = cSub(m_node1->m_pos, m_node0->m_pos);
    m_wLink10 = cSub(m_node0->m_pos, m_node1->m_pos);

	if (m_wLink01.length() == 0)
	{
		m_enabled = false;
	}

    m_wzLink01 = m_wLink01;
    m_wzLink10 = m_wLink10;

    m_nzLink01 = cMul(cTranspose(m_node0->m_rot), m_wzLink01);
    m_nzLink10 = cMul(cTranspose(m_node1->m_rot), m_wzLink10);

    // compute reference frames
    cVector3d v(1.0, 0.0, 0.0);
    double ang = cAngle(v, m_wLink01);
    if ((ang < 0.01) || (ang > 3.13)) { v.set(0.0, 1.0, 0.0); }

    cVector3d A0 = cNormalize( cCross(m_wLink01, v) );
    cVector3d A1 = A0;
    // (seb 20090514) had to rename B0 to _B0 due to name collision, we should figure out where
    //                and probably use a CHAI3D namespace to avoid future collisions for the user
    cVector3d _B0 = cNormalize( cCross(m_wLink01, A0) );

    m_A0 = cMul(cTranspose(m_node0->m_rot), A0);
    m_B0 = cMul(cTranspose(m_node0->m_rot), _B0);
    m_A1 = cMul(cTranspose(m_node1->m_rot), A1);

    // set default color
    m_color = default_color;

    // compute initial length
    m_length0 = cDistance(m_node1->m_pos, m_node0->m_pos);
    m_length = m_length0;

    // set elongation spring constant [N/M]
    m_kSpringElongation = default_kSpringElongation;

    // set flexion angular spring constant [NM/RAD]
    m_kSpringFlexion = default_kSpringFlexion;

    // set torsion angular spring constant [NM/RAD]
    m_kSpringTorsion = default_kSpringTorsion;
}


//===========================================================================
/*!
     Destructor of cGELSkeletonLink.

      \fn       cGELSkeletonLink::~cGELSkeletonLink()
*/
//===========================================================================
cGELSkeletonLink::~cGELSkeletonLink()
{

}

