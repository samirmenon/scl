//==============================================================================
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
    \author    Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 733 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CTransformH
#define CTransformH
//------------------------------------------------------------------------------
#include "math/CMatrix3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CTransform.h

    \brief
    <b> Math </b> \n 
    3D Transformation Frames.
*/
//==============================================================================


//==============================================================================
/*!
    \struct   cTransform
    \ingroup  math

    \brief    
    to be completed
*/
//==============================================================================
struct cTransform
{
  public:

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    /*!
          Default constructor of cTransform.
    */
    //--------------------------------------------------------------------------
    cTransform()
    {
        identity();
    }


    //--------------------------------------------------------------------------
    /*!
          Constructor of cTransform. Build transformation matrix from
          a position vector and rotation matrix.
    */
    //--------------------------------------------------------------------------
    cTransform(const cVector3d& a_pos, const cMatrix3d& a_rot)
    {
        identity();
        set(a_pos, a_rot);
    }


    //--------------------------------------------------------------------------
    /*!
          Constructor of cTransform. Build transformation matrix from
          a position vector.
    */
    //--------------------------------------------------------------------------
    cTransform(const cVector3d& a_pos)
    {
        identity();
        setLocalPos(a_pos);
    }


    //--------------------------------------------------------------------------
    /*!
          Constructor of cTransform. Build transformation matrix from
          a rotation matrix.
    */
    //--------------------------------------------------------------------------
    cTransform(const cMatrix3d& a_rot)
    {
        identity();
        setLocalRot(a_rot);
    }


    //--------------------------------------------------------------------------
    /*!
          Builds a transformation matrix from a position vector passed as
          parameter.

          \param    a_pos   Input vector.
    */
    //--------------------------------------------------------------------------
    inline void setLocalPos(const cVector3d& a_pos)
    {
        m[3][0] = a_pos(0); m[3][1] = a_pos(1);  m[3][2] = a_pos(2);
    }


    //--------------------------------------------------------------------------
    /*!
          Builds a transformation matrix from a 3x3 rotation matrix passed
          as a parameter.

          \param    a_rot  The source rotation matrix.
    */
    //--------------------------------------------------------------------------
    void setLocalRot(const cMatrix3d& a_rot)
    {
        m[0][0] = a_rot(0,0);  m[0][1] = a_rot(1,0);  m[0][2] = a_rot(2,0); 
        m[1][0] = a_rot(0,1);  m[1][1] = a_rot(1,1);  m[1][2] = a_rot(2,1);
        m[2][0] = a_rot(0,2);  m[2][1] = a_rot(1,2);  m[2][2] = a_rot(2,2);
    }


    //--------------------------------------------------------------------------
    /*!
          Builds a transformation matrix from a position vector amd a 3x3 
          rotation matrix passed as a parameter.

          \param    a_rot  The source rotation matrix.
    */
    //--------------------------------------------------------------------------
    void set(const cVector3d& a_pos, 
             const cMatrix3d& a_rot)
    {
        m[0][0] = a_rot(0,0);  m[0][1] = a_rot(1,0);  m[0][2] = a_rot(2,0);
        m[1][0] = a_rot(0,1);  m[1][1] = a_rot(1,1);  m[1][2] = a_rot(2,1);
        m[2][0] = a_rot(0,2);  m[2][1] = a_rot(1,2);  m[2][2] = a_rot(2,2);
        m[3][0] = a_pos(0);    m[3][1] = a_pos(1);    m[3][2] = a_pos(2); 
    }


    //--------------------------------------------------------------------------
    /*!
          Set all values of matrix.

          \param    a_pos   Input vector.
    */
    //--------------------------------------------------------------------------
    inline void set(const double& a_00, const double& a_01, const double& a_02, const double& a_03,
                    const double& a_10, const double& a_11, const double& a_12, const double& a_13,
                    const double& a_20, const double& a_21, const double& a_22, const double& a_23,
                    const double& a_30, const double& a_31, const double& a_32, const double& a_33)
    {
        m[0][0] = a_00;      m[0][1] = a_01;       m[0][2] = a_02;       m[0][3] = a_03;
        m[1][0] = a_10;      m[1][1] = a_11;       m[1][2] = a_12;       m[1][3] = a_13;
        m[2][0] = a_20;		 m[2][1] = a_21;       m[2][2] = a_22;       m[2][3] = a_23;
        m[3][0] = a_30;		 m[3][1] = a_31;	   m[3][2] = a_32;		 m[3][3] = a_33;
    }


    //--------------------------------------------------------------------------
    /*!
        Create a frustum matrix, as defined by the glFrustum function.
    */
    //--------------------------------------------------------------------------
    inline void setFrustumMatrix(double l, 
                                 double r, 
                                 double b, 
                                 double t,
                                 double n, 
                                 double f)
    {
        m[0][0] = (2.0*n) / (r-l);
        m[0][1] = 0.0;
        m[0][2] = 0.0;
        m[0][3] = 0.0;

        m[1][0] = 0.0;
        m[1][1] = (2.0*n) / (t-b);
        m[1][2] = 0.0;
        m[1][3] = 0.0;

        m[2][0] = (r+l) / (r-l);
        m[2][1] = (t+b) / (t-b);
        m[2][2] = -(f+n) / (f-n);
        m[2][3] = -1.0;

        m[3][0] = 0.0;
        m[3][1] = 0.0;
        m[3][2] = -(2.0*f*n) / (f-n);
        m[3][3] = 0.0;
    }


    //--------------------------------------------------------------------------
    /*!
        Build a 4x4 matrix transform, according to the gluLookAt function.
    */
    //--------------------------------------------------------------------------
    inline void setLookAtMatrix(const double a_eyeX, 
                                const double a_eyeY, 
                                const double a_eyeZ,
                                const double a_centerX, 
                                const double a_centerY, 
                                const double a_centerZ,
                                const double a_upX, 
                                const double a_upY, 
                                const double a_upZ)
    {
        double x[3], y[3], z[3];
        double mag;

        // create rotation matrix

        // Z vector
        z[0] = a_eyeX - a_centerX;
        z[1] = a_eyeY - a_centerY;
        z[2] = a_eyeZ - a_centerZ;

        mag = sqrt( z[0]*z[0] + z[1]*z[1] + z[2]*z[2] );
        if (mag > 0.0) 
        {  
          z[0] /= mag;
          z[1] /= mag;
          z[2] /= mag;
        }

        // Y vector
        y[0] = a_upX;
        y[1] = a_upY;
        y[2] = a_upZ;

        // X vector = Y cross Z
        x[0] =  y[1]*z[2] - y[2]*z[1];
        x[1] = -y[0]*z[2] + y[2]*z[0];
        x[2] =  y[0]*z[1] - y[1]*z[0];

        // Recompute Y = Z cross X
        y[0] =  z[1]*x[2] - z[2]*x[1];
        y[1] = -z[0]*x[2] + z[2]*x[0];
        y[2] =  z[0]*x[1] - z[1]*x[0];


        // Normalize
        mag = sqrt( x[0]*x[0] + x[1]*x[1] + x[2]*x[2] );
        if (mag) {
          x[0] /= mag;
          x[1] /= mag;
          x[2] /= mag;
        }

        mag = sqrt( y[0]*y[0] + y[1]*y[1] + y[2]*y[2] );
        if (mag) {
          y[0] /= mag;
          y[1] /= mag;
          y[2] /= mag;
        }

        m[0][0] = x[0];  m[1][0] = x[1];  m[2][0] = x[2];  m[3][0] = -x[0]*a_eyeX + -x[1]*a_eyeY + -x[2]*a_eyeZ;
        m[0][1] = y[0];  m[1][1] = y[1];  m[2][1] = y[2];  m[3][1] = -y[0]*a_eyeX + -y[1]*a_eyeY + -y[2]*a_eyeZ;
        m[0][2] = z[0];  m[1][2] = z[1];  m[2][2] = z[2];  m[3][2] = -z[0]*a_eyeX + -z[1]*a_eyeY + -z[2]*a_eyeZ;
        m[0][3] = 0.0;   m[1][3] = 0.0;   m[2][3] = 0.0;   m[3][3] = 1.0;
    }


    //--------------------------------------------------------------------------
    /*!
        Build a 4x4 matrix transform, according to the gluLookAt function.
    */
    //--------------------------------------------------------------------------
    inline void setLookAtMatrix(const cVector3d& a_eye, 
                                const cVector3d& a_lookAt, 
                                const cVector3d& a_up)
    {
        setLookAtMatrix(a_eye(0), a_eye(1), a_eye(2),
                        a_lookAt(0), a_lookAt(1), a_lookAt(2),
                        a_up(0), a_up(1), a_up(2));
    }

    
    //--------------------------------------------------------------------------
    /*!
        Build a perspective matrix, according to the gluPerspective function.
    */
    //--------------------------------------------------------------------------
    inline void setPerspectiveMatrix(const double a_fovy,
                                     const double a_aspect,
                                     const double a_zNear, 
                                     const double a_zFar)
    {
        double xMin, xMax, yMin, yMax;

        yMax = a_zNear * tan(a_fovy * C_PI / 360.0);
        yMin = -yMax;

        xMin = yMin * a_aspect;
        xMax = yMax * a_aspect;

        setFrustumMatrix(xMin, xMax, yMin, yMax, a_zNear, a_zFar);
    }


    //--------------------------------------------------------------------------
    /*!
          Returns a pointer to the matrix array in memory.

          \return   Returns a pointer of type \e double.
    */
    //--------------------------------------------------------------------------
    inline double* getData() 
    { 
        return (&(m[0][0])); 
    }


    //--------------------------------------------------------------------------
    /*!
          Extract the translational component of this matrix.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getLocalPos() const
    {
        return cVector3d(m[3][0],m[3][1],m[3][2]);
    }


    //--------------------------------------------------------------------------
    /*!
          Extract the rotational component of this matrix.
    */
    //--------------------------------------------------------------------------
    inline cMatrix3d getLocalRot() const
    {
        cMatrix3d mat;
        mat.set(m[0][0],m[1][0],m[2][0],
                m[0][1],m[1][1],m[2][1],
                m[0][2],m[1][2],m[2][2]);
        return (mat);
    }


    //--------------------------------------------------------------------------
    /*!
        Copy the current matrix to an external matrix passed as a parameter.

        \param    a_destination  Destination matrix.
    */
    //--------------------------------------------------------------------------
    inline void copyto(cTransform& a_destination) const
    {
        a_destination.m[0][0] = m[0][0];  a_destination.m[0][1] = m[0][1];
        a_destination.m[0][2] = m[0][2];  a_destination.m[0][3] = m[0][3];
        a_destination.m[1][0] = m[1][0];  a_destination.m[1][1] = m[1][1];
        a_destination.m[1][2] = m[1][2];  a_destination.m[1][3] = m[1][3];
        a_destination.m[2][0] = m[2][0];  a_destination.m[2][1] = m[2][1];
        a_destination.m[2][2] = m[2][2];  a_destination.m[2][3] = m[2][3];
        a_destination.m[3][0] = m[3][0];  a_destination.m[3][1] = m[3][1];
        a_destination.m[3][2] = m[3][2];  a_destination.m[3][3] = m[3][3];
    }


    //--------------------------------------------------------------------------
    /*!
        Copy values from an external matrix passed as parameter to this matrix.

        \param    a_source  Source matrix.
    */
    //--------------------------------------------------------------------------
    inline void copyfrom(const cTransform& a_source)
    {
        m[0][0] = a_source.m[0][0];  m[0][1] = a_source.m[0][1];
        m[0][2] = a_source.m[0][2];  m[0][3] = a_source.m[0][3];
        m[1][0] = a_source.m[1][0];  m[1][1] = a_source.m[1][1];
        m[1][2] = a_source.m[1][2];  m[1][3] = a_source.m[1][3];
        m[2][0] = a_source.m[2][0];  m[2][1] = a_source.m[2][1];
        m[2][2] = a_source.m[2][2];  m[2][3] = a_source.m[2][3];
        m[3][0] = a_source.m[3][0];  m[3][1] = a_source.m[3][1];
        m[3][2] = a_source.m[3][2];  m[3][3] = a_source.m[3][3];
    }


    //--------------------------------------------------------------------------
    /*!
        Set this matrix to be equal to the identity matrix.
    */
    //--------------------------------------------------------------------------
    inline void identity()
    {
        m[0][0] = 1.0;  m[0][1] = 0.0;  m[0][2] = 0.0;  m[0][3] = 0.0;
        m[1][0] = 0.0;  m[1][1] = 1.0;  m[1][2] = 0.0;  m[1][3] = 0.0;
        m[2][0] = 0.0;  m[2][1] = 0.0;  m[2][2] = 1.0;  m[2][3] = 0.0;
        m[3][0] = 0.0;  m[3][1] = 0.0;  m[3][2] = 0.0;  m[3][3] = 1.0;
    }


    //--------------------------------------------------------------------------
    /*!
        Left-multiply the current matrix by an external matrix passed as
        a parameter.  That is, compute: \n

        \e this = \e a_matrix * \e this \n

        Remember that all matrices are column-major.  That's why the following
        code looks like right-multiplication...

        \param    a_matrix  Matrix with which multiplication is performed.
    */
    //--------------------------------------------------------------------------

    inline void mul(const cTransform& a_matrix)
    {
        double m00 = m[0][0] * a_matrix.m[0][0] + m[1][0] * a_matrix.m[0][1] +
                     m[2][0] * a_matrix.m[0][2] + m[3][0] * a_matrix.m[0][3];
        double m01 = m[0][0] * a_matrix.m[1][0] + m[1][0] * a_matrix.m[1][1] +
                     m[2][0] * a_matrix.m[1][2] + m[3][0] * a_matrix.m[1][3];
        double m02 = m[0][0] * a_matrix.m[2][0] + m[1][0] * a_matrix.m[2][1] +
                     m[2][0] * a_matrix.m[2][2] + m[3][0] * a_matrix.m[2][3];
        double m03 = 0.0;

        double m10 = m[0][1] * a_matrix.m[0][0] + m[1][1] * a_matrix.m[0][1] +
                     m[2][1] * a_matrix.m[0][2] + m[3][1] * a_matrix.m[0][3];
        double m11 = m[0][1] * a_matrix.m[1][0] + m[1][1] * a_matrix.m[1][1] +
                     m[2][1] * a_matrix.m[1][2] + m[3][1] * a_matrix.m[1][3];
        double m12 = m[0][1] * a_matrix.m[2][0] + m[1][1] * a_matrix.m[2][1] +
                     m[2][1] * a_matrix.m[2][2] + m[3][1] * a_matrix.m[2][3];
        double m13 = m[0][1] * a_matrix.m[3][0] + m[1][1] * a_matrix.m[3][1] +
                     m[2][1] * a_matrix.m[3][2] + m[3][1] * a_matrix.m[3][3];

        double m20 = m[0][2] * a_matrix.m[0][0] + m[1][2] * a_matrix.m[0][1] +
                     m[2][2] * a_matrix.m[0][2] + m[3][2] * a_matrix.m[0][3];
        double m21 = m[0][2] * a_matrix.m[1][0] + m[1][2] * a_matrix.m[1][1] +
                     m[2][2] * a_matrix.m[1][2] + m[3][2] * a_matrix.m[1][3];
        double m22 = m[0][2] * a_matrix.m[2][0] + m[1][2] * a_matrix.m[2][1] +
                     m[2][2] * a_matrix.m[2][2] + m[3][2] * a_matrix.m[2][3];
        double m23 = m[0][2] * a_matrix.m[3][0] + m[1][2] * a_matrix.m[3][1] +
                     m[2][2] * a_matrix.m[3][2] + m[3][2] * a_matrix.m[3][3];

        double m30 = m[0][3] * a_matrix.m[0][0] + m[1][3] * a_matrix.m[0][1] +
                     m[2][3] * a_matrix.m[0][2] + m[3][3] * a_matrix.m[0][3];
        double m31 = m[0][3] * a_matrix.m[1][0] + m[1][3] * a_matrix.m[1][1] +
                     m[2][3] * a_matrix.m[1][2] + m[3][3] * a_matrix.m[1][3];
        double m32 = m[0][3] * a_matrix.m[2][0] + m[1][3] * a_matrix.m[2][1] +
                     m[2][3] * a_matrix.m[2][2] + m[3][3] * a_matrix.m[2][3];
        double m33 = m[0][3] * a_matrix.m[3][0] + m[1][3] * a_matrix.m[3][1] +
                     m[2][3] * a_matrix.m[3][2] + m[3][3] * a_matrix.m[3][3];

        // return values to current matrix
        m[0][0] = m00;  m[0][1] = m10;  m[0][2] = m20;  m[0][3] = m30;
        m[1][0] = m01;  m[1][1] = m11;  m[1][2] = m21;  m[1][3] = m31;
        m[2][0] = m02;  m[2][1] = m12;  m[2][2] = m22;  m[2][3] = m32;
        m[3][0] = m03;  m[3][1] = m13;  m[3][2] = m23;  m[3][3] = m33;
    }


    //--------------------------------------------------------------------------
    /*!
        Left-multiply the current matrix by an external matrix passed as
        a parameter, storing the result externally.  That is, compute: \n

        \e a_result = \e a_matrix * \e this;

        Remember that all matrices are column-major.  That's why the following
        code looks like right-multiplication...

        \param    a_matrix  Matrix with which multiplication is performed.
        \param    a_result  Matrix where the result is stored.
    */
    //--------------------------------------------------------------------------
    inline void mulr(const cTransform& a_matrix, 
                     cTransform& a_result) const
    {
        double m00 = m[0][0] * a_matrix.m[0][0] + m[1][0] * a_matrix.m[0][1] +
                     m[2][0] * a_matrix.m[0][2] + m[3][0] * a_matrix.m[0][3];
        double m01 = m[0][0] * a_matrix.m[1][0] + m[1][0] * a_matrix.m[1][1] +
                     m[2][0] * a_matrix.m[1][2] + m[3][0] * a_matrix.m[1][3];
        double m02 = m[0][0] * a_matrix.m[2][0] + m[1][0] * a_matrix.m[2][1] +
                     m[2][0] * a_matrix.m[2][2] + m[3][0] * a_matrix.m[2][3];
        double m03 = m[0][0] * a_matrix.m[3][0] + m[1][0] * a_matrix.m[3][1] +
                     m[2][0] * a_matrix.m[3][2] + m[3][0] * a_matrix.m[3][3];

        double m10 = m[0][1] * a_matrix.m[0][0] + m[1][1] * a_matrix.m[0][1] +
                     m[2][1] * a_matrix.m[0][2] + m[3][1] * a_matrix.m[0][3];
        double m11 = m[0][1] * a_matrix.m[1][0] + m[1][1] * a_matrix.m[1][1] +
                     m[2][1] * a_matrix.m[1][2] + m[3][1] * a_matrix.m[1][3];
        double m12 = m[0][1] * a_matrix.m[2][0] + m[1][1] * a_matrix.m[2][1] +
                     m[2][1] * a_matrix.m[2][2] + m[3][1] * a_matrix.m[2][3];
        double m13 = m[0][1] * a_matrix.m[3][0] + m[1][1] * a_matrix.m[3][1] +
                     m[2][1] * a_matrix.m[3][2] + m[3][1] * a_matrix.m[3][3];

        double m20 = m[0][2] * a_matrix.m[0][0] + m[1][2] * a_matrix.m[0][1] +
                     m[2][2] * a_matrix.m[0][2] + m[3][2] * a_matrix.m[0][3];
        double m21 = m[0][2] * a_matrix.m[1][0] + m[1][2] * a_matrix.m[1][1] +
                     m[2][2] * a_matrix.m[1][2] + m[3][2] * a_matrix.m[1][3];
        double m22 = m[0][2] * a_matrix.m[2][0] + m[1][2] * a_matrix.m[2][1] +
                     m[2][2] * a_matrix.m[2][2] + m[3][2] * a_matrix.m[2][3];
        double m23 = m[0][2] * a_matrix.m[3][0] + m[1][2] * a_matrix.m[3][1] +
                     m[2][2] * a_matrix.m[3][2] + m[3][2] * a_matrix.m[3][3];

        double m30 = m[0][3] * a_matrix.m[0][0] + m[1][3] * a_matrix.m[0][1] +
                     m[2][3] * a_matrix.m[0][2] + m[3][3] * a_matrix.m[0][3];
        double m31 = m[0][3] * a_matrix.m[1][0] + m[1][3] * a_matrix.m[1][1] +
                     m[2][3] * a_matrix.m[1][2] + m[3][3] * a_matrix.m[1][3];
        double m32 = m[0][3] * a_matrix.m[2][0] + m[1][3] * a_matrix.m[2][1] +
                     m[2][3] * a_matrix.m[2][2] + m[3][3] * a_matrix.m[2][3];
        double m33 = m[0][3] * a_matrix.m[3][0] + m[1][3] * a_matrix.m[3][1] +
                     m[2][3] * a_matrix.m[3][2] + m[3][3] * a_matrix.m[3][3];

        // return values to current matrix
        a_result.m[0][0] = m00;  a_result.m[0][1] = m10;  a_result.m[0][2] = m20;  a_result.m[0][3] = m30;
        a_result.m[1][0] = m01;  a_result.m[1][1] = m11;  a_result.m[1][2] = m21;  a_result.m[1][3] = m31;
        a_result.m[2][0] = m02;  a_result.m[2][1] = m12;  a_result.m[2][2] = m22;  a_result.m[2][3] = m32;
        a_result.m[3][0] = m03;  a_result.m[3][1] = m13;  a_result.m[3][2] = m23;  a_result.m[3][3] = m33;
    }

    //--------------------------------------------------------------------------
    /*!
        Transpose this matrix.
    */
    //--------------------------------------------------------------------------
    inline void trans()
    {
        double t;

        t = m[0][1]; m[0][1] = m[1][0]; m[1][0] = t;
        t = m[0][2]; m[0][2] = m[2][0]; m[2][0] = t;
        t = m[0][3]; m[0][3] = m[3][0]; m[3][0] = t;
        t = m[1][2]; m[1][2] = m[2][1]; m[2][1] = t;
        t = m[1][3]; m[1][3] = m[3][1]; m[3][1] = t;
        t = m[2][3]; m[2][3] = m[3][2]; m[3][2] = t;
    }


    //--------------------------------------------------------------------------
    /*!
        Transpose this matrix and store the result in a_result.

        \param      a_result  Result is stored here.
    */
    //--------------------------------------------------------------------------
    inline void transr(cTransform& a_result) const
    {
        a_result.m[0][0] = m[0][0];
        a_result.m[0][1] = m[1][0];
        a_result.m[0][2] = m[2][0];
        a_result.m[0][3] = m[3][0];

        a_result.m[1][0] = m[0][1];
        a_result.m[1][1] = m[1][1];
        a_result.m[1][2] = m[2][1];
        a_result.m[1][3] = m[3][1];

        a_result.m[2][0] = m[0][2];
        a_result.m[2][1] = m[1][2];
        a_result.m[2][2] = m[2][2];
        a_result.m[2][3] = m[3][2];

        a_result.m[3][0] = m[0][3];
        a_result.m[3][1] = m[1][3];
        a_result.m[3][2] = m[2][3];
        a_result.m[3][3] = m[3][3];
    }


    //--------------------------------------------------------------------------
    /*!
        Invert this matrix.

        \return     Returns __true__ if operation succeeds. Otherwise __false__.
    */
    //--------------------------------------------------------------------------
    bool inline invert()
    {

        // Macros used during inversion
        #ifndef DOXYGEN_SHOULD_SKIP_THIS
        #define SWAP_ROWS(a, b) { double *_tmp = a; (a)=(b); (b)=_tmp; }
        #define MAT(m,r,c) (m)[(c)*4+(r)]
        #endif  // DOXYGEN_SHOULD_SKIP_THIS 
            
        double *mat = m[0];

        double wtmp[4][8];
        double m0, m1, m2, m3, s;
        double *r0, *r1, *r2, *r3;

        r0 = wtmp[0], r1 = wtmp[1], r2 = wtmp[2], r3 = wtmp[3];

        r0[0] = MAT(mat,0,0), r0[1] = MAT(mat,0,1),
        r0[2] = MAT(mat,0,2), r0[3] = MAT(mat,0,3),
        r0[4] = 1.0, r0[5] = r0[6] = r0[7] = 0.0,

        r1[0] = MAT(mat,1,0), r1[1] = MAT(mat,1,1),
        r1[2] = MAT(mat,1,2), r1[3] = MAT(mat,1,3),
        r1[5] = 1.0, r1[4] = r1[6] = r1[7] = 0.0,

        r2[0] = MAT(mat,2,0), r2[1] = MAT(mat,2,1),
        r2[2] = MAT(mat,2,2), r2[3] = MAT(mat,2,3),
        r2[6] = 1.0, r2[4] = r2[5] = r2[7] = 0.0,

        r3[0] = MAT(mat,3,0), r3[1] = MAT(mat,3,1),
        r3[2] = MAT(mat,3,2), r3[3] = MAT(mat,3,3),
        r3[7] = 1.0, r3[4] = r3[5] = r3[6] = 0.0;

        // choose pivot
        if (fabs(r3[0])>fabs(r2[0])) SWAP_ROWS(r3, r2);
        if (fabs(r2[0])>fabs(r1[0])) SWAP_ROWS(r2, r1);
        if (fabs(r1[0])>fabs(r0[0])) SWAP_ROWS(r1, r0);
        if (0.0 == r0[0])
        {
            return (false);
        }

        // eliminate first variable
        m1 = r1[0]/r0[0]; m2 = r2[0]/r0[0]; m3 = r3[0]/r0[0];
        s = r0[1]; r1[1] -= m1 * s; r2[1] -= m2 * s; r3[1] -= m3 * s;
        s = r0[2]; r1[2] -= m1 * s; r2[2] -= m2 * s; r3[2] -= m3 * s;
        s = r0[3]; r1[3] -= m1 * s; r2[3] -= m2 * s; r3[3] -= m3 * s;
        s = r0[4];
        if (s != 0.0) { r1[4] -= m1 * s; r2[4] -= m2 * s; r3[4] -= m3 * s; }
        s = r0[5];
        if (s != 0.0) { r1[5] -= m1 * s; r2[5] -= m2 * s; r3[5] -= m3 * s; }
        s = r0[6];
        if (s != 0.0) { r1[6] -= m1 * s; r2[6] -= m2 * s; r3[6] -= m3 * s; }
        s = r0[7];
        if (s != 0.0) { r1[7] -= m1 * s; r2[7] -= m2 * s; r3[7] -= m3 * s; }

        // choose pivot
        if (fabs(r3[1])>fabs(r2[1])) SWAP_ROWS(r3, r2);
        if (fabs(r2[1])>fabs(r1[1])) SWAP_ROWS(r2, r1);
        if (0.0 == r1[1])
        {
        return (false);
        }

        // eliminate second variable
        m2 = r2[1]/r1[1]; m3 = r3[1]/r1[1];
        r2[2] -= m2 * r1[2]; r3[2] -= m3 * r1[2];
        r2[3] -= m2 * r1[3]; r3[3] -= m3 * r1[3];
        s = r1[4]; if (0.0 != s) { r2[4] -= m2 * s; r3[4] -= m3 * s; }
        s = r1[5]; if (0.0 != s) { r2[5] -= m2 * s; r3[5] -= m3 * s; }
        s = r1[6]; if (0.0 != s) { r2[6] -= m2 * s; r3[6] -= m3 * s; }
        s = r1[7]; if (0.0 != s) { r2[7] -= m2 * s; r3[7] -= m3 * s; }

        // choose pivot
        if (fabs(r3[2])>fabs(r2[2])) SWAP_ROWS(r3, r2);
        if (0.0 == r2[2])
        {
        return (false);
        }

        // eliminate third variable
        m3 = r3[2]/r2[2];
        r3[3] -= m3 * r2[3], r3[4] -= m3 * r2[4],
        r3[5] -= m3 * r2[5], r3[6] -= m3 * r2[6],
        r3[7] -= m3 * r2[7];

        // last check
        if (0.0 == r3[3])
        {
        return (false);
        }

        s = 1.0/r3[3];
        r3[4] *= s; r3[5] *= s; r3[6] *= s; r3[7] *= s;

        m2 = r2[3];
        s  = 1.0/r2[2];
        r2[4] = s * (r2[4] - r3[4] * m2), r2[5] = s * (r2[5] - r3[5] * m2),
        r2[6] = s * (r2[6] - r3[6] * m2), r2[7] = s * (r2[7] - r3[7] * m2);
        m1 = r1[3];
        r1[4] -= r3[4] * m1, r1[5] -= r3[5] * m1,
        r1[6] -= r3[6] * m1, r1[7] -= r3[7] * m1;
        m0 = r0[3];
        r0[4] -= r3[4] * m0, r0[5] -= r3[5] * m0,
        r0[6] -= r3[6] * m0, r0[7] -= r3[7] * m0;

        m1 = r1[2];
        s  = 1.0/r1[1];
        r1[4] = s * (r1[4] - r2[4] * m1), r1[5] = s * (r1[5] - r2[5] * m1),
        r1[6] = s * (r1[6] - r2[6] * m1), r1[7] = s * (r1[7] - r2[7] * m1);
        m0 = r0[2];
        r0[4] -= r2[4] * m0, r0[5] -= r2[5] * m0,
        r0[6] -= r2[6] * m0, r0[7] -= r2[7] * m0;

        m0 = r0[1];
        s  = 1.0/r0[0];
        r0[4] = s * (r0[4] - r1[4] * m0), r0[5] = s * (r0[5] - r1[5] * m0),
        r0[6] = s * (r0[6] - r1[6] * m0), r0[7] = s * (r0[7] - r1[7] * m0);

        MAT(mat,0,0) = r0[4]; MAT(mat,0,1) = r0[5],
        MAT(mat,0,2) = r0[6]; MAT(mat,0,3) = r0[7],
        MAT(mat,1,0) = r1[4]; MAT(mat,1,1) = r1[5],
        MAT(mat,1,2) = r1[6]; MAT(mat,1,3) = r1[7],
        MAT(mat,2,0) = r2[4]; MAT(mat,2,1) = r2[5],
        MAT(mat,2,2) = r2[6]; MAT(mat,2,3) = r2[7],
        MAT(mat,3,0) = r3[4]; MAT(mat,3,1) = r3[5],
        MAT(mat,3,2) = r3[6]; MAT(mat,3,3) = r3[7];

        return (true);

        // Macros used during inversion
        #undef MAT
        #undef SWAP_ROWS
    }


    //--------------------------------------------------------------------------
    /*!
        Convert the current matrix into an std::string.

        \param    a_precision  Number of digits
        \return   Return output string.
    */
    //--------------------------------------------------------------------------
    inline std::string str(const int a_precision)
    {
        std::string result;
        result.append("[ ");
        for (int i=0; i<4; i++)
        {
            result.append("( ");
            for (int j=0; j<4; j++)
            {
                result.append(cStr(m[j][i], a_precision));
                if (j<3)
                {
                    result.append(", ");
                }
            }
            result.append(" ) ");
        }
        result.append("]");

        return (result);
    }


  public:

    //-----------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

    //! array of type \e double, defining the actual transformation
    double  m[4][4];
};


//==============================================================================
// Operators on cTransform
//==============================================================================

//------------------------------------------------------------------------------
/*!
    An overloaded * operator for matrix/matrix multiplication.
*/
//------------------------------------------------------------------------------
inline cTransform operator*(const cTransform& a_matrix1, 
                            const cTransform& a_matrix2)
{
    cTransform result;
    a_matrix1.mulr(a_matrix2, result);
    return (result);
}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
