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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 791 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGELMeshH
#define CGELMeshH
//---------------------------------------------------------------------------
#include "CGELSkeletonNode.h"
#include "CGELSkeletonLink.h"
#include "CGELLinearSpring.h"
#include "CGELVertex.h"
#include "chai3d.h"
#include <typeinfo>
#include <vector>
#include <list>
//---------------------------------------------------------------------------
using std::vector;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELMesh.h

    \brief 
    <b> GEL Module </b> \n 
    Deformable Mesh.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELMesh
    \ingroup    GEL

    \brief      
    cGELMesh inherits from cMesh and integrate a skeleton model for 
    deformation simulation.
*/
//===========================================================================
class cGELMesh : public cMultiMesh
{

  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cMesh.
    cGELMesh(){ initialise(); };

    //! Destructor of cMesh.
    virtual ~cGELMesh() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Build dynamic vertices for deformable mesh.
    void buildVertices();

    //! Connect each vertex to skeleton.
    void connectVerticesToSkeleton(bool a_connectToNodesOnly);

    //! Update position of vertices connected to skeleton.
    void updateVertexPosition();

    //! Clear forces.
    void clearForces();

    //! Clear external forces.
    void clearExternalForces();

    //! Compute forces.
    void computeForces();

    //! Compute next pose.
    void computeNextPose(double iTimeInterval);

    //! Apply new computed pose.
    void applyNextPose();

    //! Render deformable mesh.
    virtual void render(cRenderOptions& a_options);


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! List of nodes composing the skeleton.
    list<cGELSkeletonNode*> m_nodes;

    //! List of links connecting the different nodes.
    list<cGELSkeletonLink*> m_links;

    //! List of linear springs connecting vertices together.
    list<cGELLinearSpring*> m_linearSprings;

    //! List of deformable vertices.
    vector<cGELVertex> m_gelVertices;

    //! If \b true then display skeleton.
    bool m_showSkeletonModel;

    //! If \b true then display mass particle model.
    bool m_showMassParticleModel;

    //! Use skeleton model.
    bool m_useSkeletonModel;

    //! Use vertex mass particle model.
    bool m_useMassParticleModel;


  private:

	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Initialize deformable mesh.
    void initialise();
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
