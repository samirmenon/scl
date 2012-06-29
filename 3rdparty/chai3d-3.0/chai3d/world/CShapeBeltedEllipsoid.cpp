//===========================================================================
/*
    \author    Xiyang Yeh
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "world/CShapeBeltedEllipsoid.h"

//---------------------------------------------------------------------------
//typedef GLUquadric GLUquadricObj;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cShapeBeltedEllipsoid.

    \fn     cShapeBeltedEllipsoid::cShapeBeltedEllipsoid()
*/
//===========================================================================
cShapeBeltedEllipsoid::cShapeBeltedEllipsoid()
{
    // set material properties
    m_material->setShininess(100);
    m_material->m_ambient.set((float)0.3, (float)0.3, (float)0.3);
    m_material->m_diffuse.set((float)0.1, (float)0.7, (float)0.8);
    m_material->m_specular.set((float)1.0, (float)1.0, (float)1.0);

    // set ellipsoid properties
    m_rings = 20;
    m_slices = 20;

    // set the default scales
    m_scale = cVector3d(0.8,0.4,0.1);

    // set default rotations
    m_axis_u = cVector3d(1,0,0);
    m_axis_v = cVector3d(0,1,0);
    m_axis_w = cVector3d(0,0,1);
};


//===========================================================================
/*!
    Render sphere in OpenGL

    \fn       void cShapeBeltedEllipsoid::render(const int a_renderMode)
    \param    a_renderMode  See cGenericObject::render()
*/
//===========================================================================
void cShapeBeltedEllipsoid::render(cRenderOptions& a_options)
{
    //-----------------------------------------------------------------------
    // Rendering code here
    //-----------------------------------------------------------------------

    // render material properties
    if (m_useMaterialProperty)
    {
        m_material->render(a_options);
    }

    glEnable(GL_COLOR_MATERIAL);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
    drawEllipsoid();
    glDisable (GL_BLEND);
    glDisable(GL_COLOR_MATERIAL);
}


//===========================================================================
/*!
    From the position of the tool, search for the nearest point located
    at the surface of the current object. Decide if the point is located inside
    or outside of the object

    \fn     void cShapeSphere::computeLocalInteraction(const cVector3d& a_toolPos,
                                                      const cVector3d& a_toolVel,
                                                      const unsigned int a_IDN)
    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void cShapeBeltedEllipsoid::computeLocalInteraction(const cVector3d& a_toolPos,
                                          const cVector3d& a_toolVel,
                                          const unsigned int a_IDN)
{
    // update effective radius
    updateRadius();

    // compute distance from center of sphere to tool
    double distance = a_toolPos.length();

    // from the position of the tool, search for the nearest point located
    // on the surface of the sphere
    if (distance > 0)
    {
        m_interactionProjectedPoint = cMul( (m_radius/distance), a_toolPos);
    }
    else
    {
        m_interactionProjectedPoint = a_toolPos;
    }

    // check if tool is located inside or outside of the sphere
    if (distance <= m_radius)
    {
        m_interactionInside = true;
    }
    else
    {
        m_interactionInside = false;
    }
}


//===========================================================================
/*!
    Update bounding box of current object.

    \fn       void cShapeSphere::updateBoundaryBox()
*/
//===========================================================================
void cShapeBeltedEllipsoid::updateBoundaryBox()
{
    updateRadius();
    m_boundaryBoxMin.set(-m_radius, -m_radius, -m_radius);
    m_boundaryBoxMax.set( m_radius,  m_radius,  m_radius);
}


//===========================================================================
/*!
    Scale object of defined scale factor

    \fn       void cShapeSphere::scaleObject(const cVector3d& a_scaleFactors)
    \param    a_scaleFactors Scale factor
*/
//===========================================================================
void cShapeBeltedEllipsoid::scaleObject(const cVector3d& a_scaleFactors)
{
    m_scale = a_scaleFactors;
}

void cShapeBeltedEllipsoid::drawEllipsoid(void)
{
  static int i, j;
  static float u;            //u is angle of rotation around z-axis
  static float v;            //v is the angle parameter for an ellipse
  static float du, dv;         //size of the increment of u and v respectively
  static float cosu, sinu;       //sine and cosine of the angle of rotation
  static float cosudu, sinudu;   //cosudu = cos (u+du); sinudu = sin (u+du)
  static float cosv, sinv;       //sine and cosine of the angle used to generate the ellipse
  static float cosvdv, sinvdv;   //cosvdv = cos (v+dv); sinvdv = sin (v+dv)
  static float mx, my, mz;       //x, y, and z components of the normal vector

  //calculate the angles of rotation to obtain the correct number of slices
  //and rings.
  du = 2.0 * M_PI / m_slices;     //rotation angle about the z axis to obtain the ellipsoid
  dv = M_PI / m_rings;            //rotation angle about the x axis to get points on the ellipse

  //outer loop iterates over u starting with -180 degrees
  u = -M_PI;
  for (i = m_slices - 1; i >= 0; i--)
  {
    //these are needed for the four points
    cosu = cos(u);
    sinu = sin(u);
    cosudu = cos (u+du);
    sinudu = sin (u+du);

    //inner loop iterates over v
    v = -M_PI / 2.0;
    for (j = m_rings; j >= 0; j--)
    {
      //these are needed for four points
      cosv = cos(v);
      sinv = sin(v);
      cosvdv = cos(v+dv);
      sinvdv = sin(v+dv);

      //this is Newell's method for finding the normals
      mx = (m_scale(1)*cosv*sinu-m_scale(1)*cosv*sinudu)*(m_scale(2)*sinv+m_scale(2)*sinv) +
         (m_scale(1)*cosv*sinudu-m_scale(1)*cosvdv*sinudu)*(m_scale(2)*sinv+m_scale(2)*sinvdv) +
         (m_scale(1)*cosv*sinudu-m_scale(1)*cosvdv*sinudu)*(m_scale(2)*sinvdv+m_scale(2)*sinvdv)+
         (m_scale(1)*cosvdv*sinu-m_scale(1)*cosv*sinu)*(m_scale(2)*sinvdv+m_scale(2)*sinv);
      my = (m_scale(2)*sinv-m_scale(2)*sinv)*(m_scale(0)*cosv*cosu + m_scale(0)*cosv*cosudu)+
         (m_scale(2)*sinv-m_scale(2)*sinvdv)*(m_scale(0)*cosv*cosudu+m_scale(0)*cosvdv*cosudu) +
         (m_scale(2)*sinvdv-m_scale(2)*sinvdv)*(m_scale(0)*cosvdv*cosudu+m_scale(0)*cosvdv*cosu)+
         (m_scale(2)*sinvdv-m_scale(2)*sinv)*(m_scale(0)*cosvdv*cosu+m_scale(0)*cosv*cosu);
      mz = (m_scale(0)*cosv*cosu - m_scale(0)*cosv*cosudu)* (m_scale(1)*cosv*sinu + m_scale(1)*cosv*sinudu)+
         (m_scale(0)*cosv*cosudu-m_scale(0)*cosvdv*cosudu)*(m_scale(1)*cosv*sinudu+m_scale(1)*cosvdv*sinudu)+
         (m_scale(0)*cosvdv*cosudu-m_scale(0)*cosvdv*cosu)*(m_scale(1)*cosv*sinudu+m_scale(1)*cosvdv*sinudu)+
         (m_scale(0)*cosvdv*cosu-m_scale(0)*cosv*cosu)* (m_scale(1)*cosvdv*sinu+m_scale(1)*cosv*sinu);

      //For each quadrilateral, tell OpenGL the four vertices and the
      //normal vector.
      glBegin (GL_QUADS);
      //glBegin( GL_LINE_LOOP );
        glColor4f (1.0f, 0.0f, 0.0f, 0.5f);

        cVector3d n(mx,my,mz);
        n.normalize();

        cVector3d q1(m_scale(0) * cosv * cosu, m_scale(1) * cosv * sinu, m_scale(2) * sinv);
        cVector3d q2(m_scale(0) * cosv * cosudu, m_scale(1) * cosv * sinudu, m_scale(2) * sinv);
        cVector3d q3(m_scale(0) * cosvdv * cosudu, m_scale(1) * cosvdv * sinudu, m_scale(2) * sinvdv);
        cVector3d q4(m_scale(0) * cosvdv * cosu, m_scale(1) * cosvdv * sinu, m_scale(2) * sinvdv);

        q1 *= q1.length();
        q2 *= q2.length();
        q3 *= q3.length();
        q4 *= q4.length();

        q1 = (q1(0))*m_axis_u+(q1(1))*m_axis_v+(q1(2))*m_axis_w;
        q2 = (q2(0))*m_axis_u+(q2(1))*m_axis_v+(q2(2))*m_axis_w;
        q3 = (q3(0))*m_axis_u+(q3(1))*m_axis_v+(q3(2))*m_axis_w;
        q4 = (q4(0))*m_axis_u+(q4(1))*m_axis_v+(q4(2))*m_axis_w;
        n = (n(0))*m_axis_u+(n(1))*m_axis_v+(n(2))*m_axis_w;

        glNormal3f(n(0),n(1),n(2));
        glVertex3f(q1(0),q1(1),q1(2));
        glVertex3f(q2(0),q2(1),q2(2));
        glVertex3f(q3(0),q3(1),q3(2));
        glVertex3f(q4(0),q4(1),q4(2));

      glEnd();

      //increment the v angle
      v += dv;
    }
    //increment the u angle
    u += du;
  }
}

void cShapeBeltedEllipsoid::updateScale(const cVector3d& a_scaleFactors)
{
  m_scale = a_scaleFactors;
}

void cShapeBeltedEllipsoid::updateDirection(const cVector3d& a_axis_u,const cVector3d& a_axis_v,const cVector3d& a_axis_w)
{
  m_axis_u = a_axis_u;
  m_axis_v = a_axis_v;
  m_axis_w = a_axis_w;
}

void cShapeBeltedEllipsoid::updateRadius()
{
  static double max_xy_;
  max_xy_ = (m_scale(0) > m_scale(1))? m_scale(0) : m_scale(1);
  m_radius = (m_scale(2) > max_xy_)? m_scale(2) : max_xy_;
}
