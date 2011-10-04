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
    \version   2.0.0 $Rev: 250 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericCollisionH
#define CGenericCollisionH
//---------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
//---------------------------------------------------------------------------
using std::vector;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file   cGenericCollision.h
    
    \brief  
    <b> Collision Detection </b> \n
    Base Class.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cGenericCollision
    \ingroup    collisions  
    
    \brief    
    cGenericCollision is an abstract class for collision-detection
    algorithms for meshes with line segments.
*/
//===========================================================================
class cGenericCollision
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGenericCollision.
    cGenericCollision();

    //! Destructor of cGenericCollision.
    virtual ~cGenericCollision() {};


	//-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Do any necessary initialization, such as building trees.
    virtual void initialize(double a_radius = 0) {};

    //! Provide a visual representation of the method.
    virtual void render() {};

    //! Return the triangles intersected by the given segment, if any.
    virtual bool computeCollision(cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings)
                                  { return (false); }

    //! Set level of collision tree to display.
    void setDisplayDepth(int a_depth) { m_displayDepth = a_depth; }

    //! Read level of collision tree being displayed.
    double getDisplayDepth() const { return (m_displayDepth); }


	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Color properties of the collision object.
    cMaterial m_material;


  protected:

	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    /*!
        Level of collision tree to render... negative values force rendering
        up to and including this level, positive values render _just_ this level.
    */
    int m_displayDepth;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

