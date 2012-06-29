//===========================================================================
/*
    \author    Xiyang Yeh
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CShapeBeltedEllipsoidH
#define CShapeBeltedEllipsoidH
//---------------------------------------------------------------------------
#include "world/CGenericObject.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CShapeBeltedEllipsoid.h

    \brief 
    <b> Scenegraph </b> \n 
    Virtual Shape - belted ellipsoid.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cShapeBeltedEllipsoidc
    \ingroup    scenegraph

    \brief      
    Implementation of a virtual belted ellipsoid shape.
*/
//===========================================================================
class cShapeBeltedEllipsoid : public cGenericObject
{
  public:
    
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //! Constructor of cShapeBeltedEllipsoid.
	  cShapeBeltedEllipsoid();
    
    //! Destructor of cShapeBeltedEllipsoid.
    virtual ~cShapeBeltedEllipsoid() {};


    //-----------------------------------------------------------------------
    // METHODS:
    //-----------------------------------------------------------------------

    //! Render object in OpenGL.
    virtual void render(cRenderOptions& a_options);

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

    //! Setup points
    void drawEllipsoid();

    //! Get radius of sphere.
    double getRadius() { return (m_radius); }

    //! Update the scales for the ellipsoid
    virtual void updateScale(const cVector3d& a_scaleFactors);

    //! Update the rotation for the ellipsoid
    virtual void updateDirection(const cVector3d& a_axis_u,const cVector3d& a_axis_v,const cVector3d& a_axis_w);

  protected:
    
    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------

    //! radius of sphere.
    double m_radius;

    //! scales of ellipsoid.
    cVector3d m_scale;

    //! axis of ellipsoid.
    cVector3d m_axis_u, m_axis_v, m_axis_w;

  private:
    //! number of rings and slices
    int m_rings;
    int m_slices;

    //! Update the effective radius for the ellipsoid
    void updateRadius();
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
