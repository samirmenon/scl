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
#ifndef CShapeSphereH
#define CShapeSphereH
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
#include "graphics/CMaterial.h"
#include "graphics/CTexture2D.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CShapeSphere.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Shape - Sphere.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cShapeSphere
    \ingroup    scenegraph

    \brief      
    Implementation of a virtual sphere shape.
*/
//===========================================================================
class cShapeSphere : public cGenericObject
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cShapeSphere.
    cShapeSphere(const double& a_radius);
    
    //! Destructor of cSphere.
    virtual ~cShapeSphere() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Render object in OpenGL.
    virtual void render(const int a_renderMode=0);

    //! Update bounding box of current object.
    virtual void updateBoundaryBox();

    //! Object scaling.
    virtual void scaleObject(const cVector3d& a_scaleFactors);

    //! Update the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN);

    //! Set radius of sphere.
    void setRadius(double a_radius) { m_radius = cAbs(a_radius); updateBoundaryBox();}

    //! Get radius of sphere.
    double getRadius() { return (m_radius); }


  protected:
    
    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! radius of sphere.
    double m_radius;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
