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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 717 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGELSkeletonLinkH
#define CGELSkeletonLinkH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CGELSkeletonNode.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELSkeletonLink.h

    \brief 
    <b> GEL Module </b> \n 
    Skeleton Elastic Link.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELSkeletonLink
    \ingroup    GEL

    \brief      
    cGELSkeletonLink models a link between two nodes.
*/
//===========================================================================
class cGELSkeletonLink
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGELSkeletonLink.
    cGELSkeletonLink(cGELSkeletonNode* a_node0, cGELSkeletonNode* a_node1);

    //! Destructor of cGELSkeletonLink.
    ~cGELSkeletonLink();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
         Render link in OpenGL.

         \fn       void cGELSkeletonLink::render()
    */
    //-----------------------------------------------------------------------
    inline void render()
    {
        // render link
        glColor4fv( (const float *)&m_color);
        glBegin(GL_LINES);
          glVertex3dv( (const double *)&m_node0->m_pos);
          glVertex3dv( (const double *)&m_node1->m_pos);
        glEnd();
    }


    //-----------------------------------------------------------------------
    /*!
         Compute forces applied on mass nodes.

         \fn       void cGELSkeletonLink::computeForces()
    */
    //-----------------------------------------------------------------------
    inline void computeForces()
    {
        // update basic parameters of current link
        m_wLink01 = cSub(m_node1->m_pos, m_node0->m_pos);
        m_wLink10 = cMul(-1, m_wLink01);
        m_length = m_wLink01.length();

        m_wzLink01 = cMul((m_node0->m_rot), m_nzLink01);
        m_wzLink10 = cMul((m_node1->m_rot), m_nzLink10);

        //-------------------------------------------------------------
        // ELONGATION:
        //-------------------------------------------------------------
        // if distance too small, no forces are applied
        if (m_length < 0.000001) { return; }

        // elongation compute force
        double f = m_kSpringElongation * (m_length - m_length0);

        // apply force
        if (m_length > 0.000001)
        {
            cVector3d force = cMul(f/m_length, m_wLink01);
            m_node0->addForce(force);
            cVector3d tmpfrc = cMul(-1, force);
            m_node1->addForce(tmpfrc);
        }

        //-------------------------------------------------------------
        // FLEXION:
        //-------------------------------------------------------------
        // compute flexion forces and torques
        cVector3d torqueDir0 = cCross(m_wzLink01, m_wLink01);
        cVector3d torqueDir1 = cCross(m_wzLink10, m_wLink10);
        double angle0 = cAngle(m_wzLink01, m_wLink01);
        double angle1 = cAngle(m_wzLink10, m_wLink10);
        double torqueMag0 = angle0 * m_kSpringFlexion;
        double torqueMag1 = angle1 * m_kSpringFlexion;

        if ((m_length > 0.000001) && (m_enabled))
        {
			if (torqueMag0 > 0.0001)
            {
                cVector3d torque0 = cMul(torqueMag0, cNormalize(torqueDir0));
                m_node0->addTorque(torque0);
                cVector3d force1 = cMul((torqueMag0/m_length), cNormalize(cCross(m_wLink01, torque0)));
                m_node1->addForce(force1);
                cVector3d tmpfrc = cMul(-1,force1);
                m_node0->addForce(tmpfrc);
            }

            if (torqueMag1 > 0.0001)
            {
                cVector3d torque1 = cMul(torqueMag1, cNormalize(torqueDir1));
                m_node1->addTorque(torque1);
                cVector3d force0 = cMul((torqueMag1/m_length), cNormalize(cCross(m_wLink10, torque1)));
                m_node0->addForce(force0);
                cVector3d tmpfrc = cMul(-1,force0);
                m_node1->addForce(tmpfrc);
            }
        }

        //-----------------------------------------------------------------------
        // TORSION:
        //-----------------------------------------------------------------------

		if (m_enabled)
		{
			// update frame vectors in world coordinates.
			m_node0->m_rot.mulr(m_A0, m_wA0);
			m_node0->m_rot.mulr(m_B0, m_wB0);
			m_node1->m_rot.mulr(m_A1, m_wA1);

			// compute torsional angle and torque
			cVector3d v0, v1;
			m_wA0.crossr(m_wLink01, v0);
			m_wA1.crossr(m_wLink01, v1);
			v0.normalize();
			v1.normalize();

			cVector3d torque;
			v0.crossr(v1, m_torsionAxisAngle);
			m_torsionAxisAngle.mulr(m_kSpringTorsion, torque);

			m_node0->addTorque(torque);
			cVector3d tmptrq = -1.0 * torque;
			m_node1->addTorque(tmptrq);
		}
    }


	//-----------------------------------------------------------------------
    // MEMBERS - MASS NODES:
    //-----------------------------------------------------------------------

    //! Node 0 of current link.
    cGELSkeletonNode *m_node0;

    //! Node 1 of current link.
    cGELSkeletonNode *m_node1;


	//-----------------------------------------------------------------------
    // MEMBERS - GRAPHICAL PROPERTIES:
    //-----------------------------------------------------------------------

    //! Color used to display NN-links.
    cColorf m_color;


	//-----------------------------------------------------------------------
    // MEMBERS - PHYSICAL PROPERTIES:
    //-----------------------------------------------------------------------

    //! Elongation spring constant.
    double m_kSpringElongation;

    //! Angular spring constant.
    double m_kSpringFlexion;

    //! Torsional spring constant.
    double m_kSpringTorsion;

	//! Is link enabled.
	bool m_enabled;

    //! Linear damping constant.
    //double m_kDamperElongation;

    //! Angular damping constant.
    //double m_kDamperFlexion;

    //! Torsion damping constant.
    //double m_kDamperTorsion;


  public:


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Initial link vector between node 0 and node 1 in node0 reference frame.
    cVector3d m_nzLink01;

    //! Initial link vector between node 0 and node 1 in world coordinates.
    cVector3d m_wzLink01;

    //! Initial link vector between node 1 and node 0 in node1 reference frame.
    cVector3d m_nzLink10;

    //! Initial link vector between node 1 and node 0 in world coordinates.
    cVector3d m_wzLink10;

    //! Current link between node 0 and node 1 in world coordinates.
    cVector3d m_wLink01;

    //! Current link between node 1 and node 0 in world coordinates.
    cVector3d m_wLink10;

    //! Torsional angle.
    cVector3d m_torsionAxisAngle;

    //! Reference vector frame A on sphere 0 (local coordinates).
    cVector3d m_A0;

    //! Reference vector frame A on sphere 0 (world coordinates).
    cVector3d m_wA0;

    //! Reference vector frame B on sphere 0 (local coordinates).
    cVector3d m_B0;

    //! Reference vector frame B on sphere 0 (world coordinates).
    cVector3d m_wB0;

    //! Reference vector frame A on sphere 1 (local coordinates).
    cVector3d m_A1;

    //! Reference vector frame A on sphere 1 (world coordinates).
    cVector3d m_wA1;

    //! Initial length of link.
    double m_length0;

    //! Current length of link.
    double m_length;


  public:

	//-----------------------------------------------------------------------
    // MEMBERS - DEFAULT SETTINGS:
    //-----------------------------------------------------------------------
      
    //! Default property - linear stiffness.
    static double default_kSpringElongation;   // [N/m]

    //! Default property - angular stiffness.
    static double default_kSpringFlexion;      // [Nm/RAD]

    //! Default property - torsional stiffness.
    static double default_kSpringTorsion;      // [Nm/RAD]

    //! Default property - color property.
    static cColorf default_color;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

