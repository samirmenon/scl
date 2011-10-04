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
#ifndef CShapeTorusH
#define CShapeTorusH
//---------------------------------------------------------------------------
#include "scenegraph/CGenericObject.h"
#include "graphics/CMaterial.h"
#include "graphics/CTexture2D.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CShapeTorus.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Shape - Torus.
*/
//===========================================================================

//===========================================================================
/*!    
    \class      cShapeTorus
    \ingroup    scenegraph

    \brief      
    cShapeTorus describes a simple torus potential field.
*/
//===========================================================================
class cShapeTorus : public cGenericObject
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cShapeTorus.
    cShapeTorus(const double& a_insideRadius, const double& a_outsideRadius);

    //! Destructor of cShapeTorus.
    virtual ~cShapeTorus() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Render object in OpenGL.
    virtual void render(const int a_renderMode=0);

    //! Update bounding box of current object.
    virtual void updateBoundaryBox();

    //! object scaling.
    virtual void scaleObject(const cVector3d& a_scaleFactors);

    //! Update the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
                                         const cVector3d& a_toolVel,
                                         const unsigned int a_IDN);

    //! Set inside and outside radius of torus.
    void setSize(const double& a_innerRadius, const double& a_outerRadius) 
         { m_innerRadius = cAbs(a_innerRadius); m_outerRadius = cAbs(a_outerRadius);}

    //! Get inside radius of torus.
    double getInnerRadius() { return (m_innerRadius); }

    //! Get inside radius of torus.
    double getOuterRadius() { return (m_outerRadius); }


  protected:
    
	//-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! Inside radius of torus.
    double m_innerRadius;

    //! Outside radius of torus.
    double m_outerRadius;

    //! Resolution of the GLU graphical model.
    unsigned int m_resolution;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
