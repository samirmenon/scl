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
#ifndef CPotentialFieldForceAlgoH
#define CPotentialFieldForceAlgoH
//---------------------------------------------------------------------------
#include "forces/CGenericPointForceAlgo.h"
#include "forces/CInteractionBasics.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CPotentialFieldForceAlgo.h

    \brief 
    <b> Force Rendering Algorithms </b> \n 
    Potential Field.
*/
//===========================================================================

//===========================================================================
/*! 
    \class      cPotentialFieldForceAlgo
    \ingroup    forces 
    
    \brief    
    cPotentialFieldForceAlgo is an abstract class for algorithms that 
    compute single point force contacts.
*/
//===========================================================================
class cPotentialFieldForceAlgo : public cGenericPointForceAlgo
{
  public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    
      //! Constructor of cPotentialFieldForceAlgo.
    cPotentialFieldForceAlgo();

    //! Destructor of cPotentialFieldForceAlgo.
    virtual ~cPotentialFieldForceAlgo() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Initialize the algorithm by passing the initial position of the device.
    void initialize(cWorld* a_world, const cVector3d& a_initialPos) { m_world = a_world; };

    //! Compute the next force given the updated position of the device.
    virtual cVector3d computeForces(const cVector3d& a_toolPos, const cVector3d& a_toolVel);

    //! Interactions recorder settings.
    cInteractionSettings m_interactionSettings;

    //! Interactions recorder.
    cInteractionRecorder m_interactionRecorder;


  private:

    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Identification number for this force algorithm.
    unsigned int m_IDN;

    //! IDN counter for all.
    static unsigned int m_IDNcounter;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
