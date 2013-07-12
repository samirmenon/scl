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
#ifndef CGELWorldH
#define CGELWorldH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CGELMesh.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGELWorld.h

    \brief 
    <b> GEL Module </b> \n 
    Virtual World.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGELWorld
    \ingroup    GEL

    \brief      
    cGELWorld implements a world to handle deformable objects within CHAI3D.
*/
//===========================================================================
class cGELWorld : public chai3d::cGenericObject
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGELWorld.
    cGELWorld();

    //! Destructor of cGELWorld.
    virtual ~cGELWorld();


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Compute dynamic simulation.
    void updateDynamics(double a_time);

    //! Clear external forces on all objects.
    void clearExternalForces();

    //! Update vertices of all objects.
    void updateSkins();


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! List of deformable solids.
    std::list<cGELMesh*> m_gelMeshes;

    //! Current time of simulation.
    double m_simulationTime;

    //! Integration time.
    double m_integrationTime;

    //! Gravity constant.
    chai3d::cVector3d m_gravity;


  private:

	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Render deformable mesh.
    virtual void render(chai3d::cRenderOptions& a_options);
};


//===========================================================================
/*!
    \class      cGELWorldCollision
    \ingroup    GEL

    \brief      
    cGELWorldCollision provides a collision detection model to support 
    deformable objects.
*/
//===========================================================================
class cGELWorldCollision : public chai3d::cGenericCollision
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGELWorldCollision.
    cGELWorldCollision(cGELWorld* a_gelWorld) {m_gelWorld = a_gelWorld;}

    //! Destructor of cGELWorldCollision.
    virtual ~cGELWorldCollision() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Do any necessary initialization, such as building trees.
    virtual void initialize() {};

    //! Provide a visual representation of the method.
    virtual void render() {};

    //! Return the nearest triangle intersected by the given segment, if any.
    virtual bool computeCollision(chai3d::cVector3d& a_segmentPointA,
                                  chai3d::cVector3d& a_segmentPointB,
                                  chai3d::cCollisionRecorder& a_recorder,
                                  chai3d::cCollisionSettings& a_settings);

  private:


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Deformable world
    cGELWorld* m_gelWorld;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------


