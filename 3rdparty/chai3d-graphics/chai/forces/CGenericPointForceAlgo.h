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
    \version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CGenericPointForceAlgoH
#define CGenericPointForceAlgoH
//---------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "scenegraph/CGenericObject.h"
#include <vector>
//---------------------------------------------------------------------------
class cWorld;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CGenericPointForceAlgo.h

    \brief 
    <b> Force Rendering Algorithms </b> \n 
    Force Model Base Class.
*/
//===========================================================================

//===========================================================================
/*!   
    \class      cGenericPointForceAlgo
    \ingroup    forces 
    
    \brief    
    cGenericPointForceAlgo is an abstract class for algorithms that compute 
    single point force contacts.
*/
//===========================================================================
class cGenericPointForceAlgo
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cGenericPointForceAlgo.
    cGenericPointForceAlgo();
    
    //! Destructor of cGenericPointForceAlgo.
    virtual ~cGenericPointForceAlgo() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Get a pointer to the world in which the force algorithm is operating.
    cWorld* getWorld() { return (m_world); }

    //! Initialize the algorithm by passing the initial position of the device.
    virtual void initialize(cWorld* a_world, const cVector3d& a_initialPos) {};

    //! Compute the next force given the updated position of the device.
    virtual cVector3d computeForces(const cVector3d& a_toolPos, const cVector3d& a_toolVel)
        { return (cVector3d(0.0, 0.0, 0.0)); }


  protected:
    
    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Pointer to the world in which the force algorithm operates.
    cWorld* m_world;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

