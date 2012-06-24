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
    \author    Phil Fong
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 799 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CQuaternionH
#define CQuaternionH
//---------------------------------------------------------------------------
#include "math/CMatrix3d.h"
#include "math/CVector3d.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CQuaternion.h

    \brief  
    <b> Math </b> \n 
    Quaternions.
*/
//===========================================================================

//===========================================================================
/*!
    \struct     cQuaternion
    \ingroup    math
    
    \brief    
    cQuaternion can be used to represents rotations in quaternion form.
    Simple floating point arithmetic operations are provided too.
*/
//===========================================================================
struct cQuaternion 
{
  public:

    //-----------------------------------------------------------------------
    // MEMBERS:
    //-----------------------------------------------------------------------
    //! Component w of quaternion
    double w;

    //! Component x of quaternion
    double x;

    //! Component y of quaternion
    double y;

    //! Component z of quaternion
    double z;


    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
    //! Constructor of cQuaternion.
    cQuaternion() {}

    //! Constructor of cQuaternion.
    cQuaternion(double nw, double nx, double ny, double nz) : w(nw), x(nx), y(ny), z(nz) {}

    //! Constructor of cQuaternion.
    cQuaternion(double const* in) : w(in[0]), x(in[1]), y(in[2]), z(in[3]) {}


    //-----------------------------------------------------------------------
    // OVERLOADED CAST OPERATORS:
    //-----------------------------------------------------------------------
    //! Cast quaternion to a double*
    operator double*() {return &w;}

    //! Cast quaternion to a double const*
    operator double const*() const { return &w;}

    //! Clear quaternion with zeros.
    void zero() { w=0.0; x=0.0; y=0.0; z=0.0; }

    //! Negate current quaternion. \n Result is stored in current quaternion.
    void negate() { w=-w; x=-x; y=-y; z=-z; }
  
    //! Returns quaternion magnitude squared.
    double magsq() const { return (w*w) + (x*x) + (y*y) + (z*z); }

    //! Returns quaternion magnitude squared.
    double lengthsq() const { return magsq(); }

    //! Returns quaternion magnitude.
    double mag() const { return sqrt(magsq()); }

    //! Returns quaternion magnitude.
    double length() const { return mag(); }

    //! Normalize quaternion.
    void normalize()
    {
        double m = mag();
        w /= m;
        x /= m;
        y /= m;
        z /= m;
    }

    //---------------------------------------------------------------
    //! Convert quaternion to rotation matrix.
    /*!
        \param   a_mat The matrix to store the result into.
    */
    //---------------------------------------------------------------
    void toRotMat(cMatrix3d& a_mat) const
    {
        double x2 = 2.0*x*x;
        double y2 = 2.0*y*y;
        double z2 = 2.0*z*z;
        double xy = 2.0*x*y;
        double wz = 2.0*w*z;
        double xz = 2.0*x*z;
        double wy = 2.0*w*y;
        double yz = 2.0*y*z;
        double wx = 2.0*w*x;

        a_mat(0,0) = 1.0 - y2 - z2;
        a_mat(0,1) = xy - wz;
        a_mat(0,2) = xz + wy;
        a_mat(1,0) = xy + wz;
        a_mat(1,1) = 1.0 - x2 - z2;
        a_mat(1,2) = yz - wx;
        a_mat(2,0) = xz - wy;
        a_mat(2,1) = yz + wx;
        a_mat(2,2) = 1.0 - x2 - y2;
    }


    //---------------------------------------------------------------
    /*!
        Convert rotation matrix to quaternion.

        \param   a_mat The rotation matrix to convert.
    */
    //---------------------------------------------------------------
    void fromRotMat(cMatrix3d const& a_mat)
    {
        double trace = 1.0 + a_mat(0,0) + a_mat(1,1) + a_mat(2,2);

        if (trace>0.00000001) {
            double s = 2.0*sqrt(trace);
            x = (a_mat(2,1) - a_mat(1,2))/s;
            y = (a_mat(0,2) - a_mat(2,0))/s;
            z = (a_mat(1,0) - a_mat(0,1))/s;
            w = 0.25*s;
        } else if ((a_mat(0,0) > a_mat(1,1)) && (a_mat(0,0) > a_mat(2,2))) {
            // column 1 has largest diagonal
            double s = 2.0*sqrt(1.0+a_mat(0,0)-a_mat(1,1)-a_mat(2,2));
            x = 0.25*s;
            y = (a_mat(1,0) + a_mat(0,1))/s;
            z = (a_mat(0,2) + a_mat(2,0))/s;
            w = (a_mat(2,1) - a_mat(1,2))/s;
        } else if (a_mat(1,1) > a_mat(2,2)) {
            // column 2 has largest diagonal
            double s = 2.0*sqrt(1.0+a_mat(1,1)-a_mat(0,0)-a_mat(2,2));
            x = (a_mat(1,0) + a_mat(0,1))/s;
            y = 0.25*s;
            z = (a_mat(2,1) + a_mat(1,2))/s;
            w = (a_mat(0,2) - a_mat(2,0))/s;
        } else {
            // column 3 has largest diagonal
            double s = 2.0*sqrt(1.0+a_mat(2,2)-a_mat(0,0)-a_mat(1,1));
            x = (a_mat(0,2) + a_mat(2,0))/s;
            y = (a_mat(2,1) + a_mat(1,2))/s;
            z = 0.25*s;
            w = (a_mat(1,0) - a_mat(0,1))/s;
        }
    }


    //---------------------------------------------------------------
    /*!
        Convert from axis and angle (in radians).

        \param  a_axis  The axis.
        \param  a_angle The angle in radians.
    */
    //---------------------------------------------------------------
    void fromAxisAngle(cVector3d a_axis, double a_angle)
    {
        // not that axis is passed by value so that we can normalize it
        a_axis.normalize();
        double sina = sin(a_angle/2.0);
        double cosa = cos(a_angle/2.0);
        w = cosa;
        x = a_axis[0]*sina;
        y = a_axis[1]*sina;
        z = a_axis[2]*sina;
    }


    //---------------------------------------------------------------
    /*! 
        Convert to axis (not normalized) and angle.
    
        \param a_axis  Where to store the axis.
        \param a_angle Where to store the angle.
    */
    //---------------------------------------------------------------
    void toAxisAngle(cVector3d& a_axis, double& a_angle) const
    {
        double cosa = w/mag();
        a_angle = acos(cosa);
        a_axis[0] = x;
        a_axis[1] = y;
        a_axis[2] = z;
    }

    //! Conjugate of quaternion
    void conj()
    {
        x = -x;
        y = -y;
        z = -z;
    }

    //! Invert quaternion ( inverse is conjugate/magsq )
    void invert()
    {
        double m2 = magsq();
        w =  w/m2;
        x = -x/m2;
        y = -y/m2;
        z = -z/m2;
    }

    //! Multiply operator (grassman product)
    cQuaternion& operator*= (cQuaternion const& a_otherQ)
    {
        double neww = w*a_otherQ.w - x*a_otherQ.x - y*a_otherQ.y - z*a_otherQ.z;
        double newx = w*a_otherQ.x + x*a_otherQ.w + y*a_otherQ.z - z*a_otherQ.y;
        double newy = w*a_otherQ.y - x*a_otherQ.z + y*a_otherQ.w + z*a_otherQ.x;
        double newz = w*a_otherQ.z + x*a_otherQ.y - y*a_otherQ.x + z*a_otherQ.w;
        w = neww;
        x = newx;
        y = newy;
        z = newz;

        return *this;
    }


    //---------------------------------------------------------------
    /*!
        Quaternion multiplication \n
        Multiply this quaternion with another and store result here.

        \param  a_otherQ The other quaternion.
    */
    //---------------------------------------------------------------
    void mul(cQuaternion const& a_otherQ)
    {
        operator*=(a_otherQ);
    }


    //! Scale operator
    cQuaternion& operator*= (double a_scale)
    {
        w *= a_scale; x *= a_scale; y *= a_scale; z *= a_scale;
        return *this;
    }

    //! Scale this quaternion by a scalar
    void mul(double s)
    {
        operator*=(s);
    }

    //! Equality operator
    bool operator==(cQuaternion const& a_otherQ) const
    {
        return (w==a_otherQ.w && x==a_otherQ.x && y==a_otherQ.y && z==a_otherQ.z);
    }

    //---------------------------------------------------------------
    /*!
        Dot product. \n
        Take the dot product with another quaternion and store the result here.

        \param  a_otherQ The other quaternion.

        \return The result of the dot product.
    */
    //---------------------------------------------------------------
    double dot(cQuaternion const& a_otherQ) const
    {
        return (w*a_otherQ.w + x*a_otherQ.x + y*a_otherQ.y + z*a_otherQ.z);
    }

    //! Addition
    cQuaternion& operator+= (cQuaternion const& a_otherQ)
    {
        w+=a_otherQ.w; x+=a_otherQ.x; y+=a_otherQ.y; z+=a_otherQ.z;
        return *this;
    }

    //---------------------------------------------------------------
    /*! 
        Addition \n
        Add another quaternion to this one and store here.

        \param a_otherQ The other quaternion.
    */
    //---------------------------------------------------------------
    void add(cQuaternion const& a_otherQ)
    {
        operator+=(a_otherQ);
    }

    //---------------------------------------------------------------
    /*! 
        Spherical linear interpolation. \n
        Spherically linearly interpolate between quaternions and store 
        the result here.
        
        \param a_level Parameter between 0 (fully at a_q1) and 1.0 (fully at a_q2).
        \param a_q1    Starting quaternion.
        \param a_q2    Ending quaternion.
    */
    //---------------------------------------------------------------
    void slerp(double a_level, cQuaternion const& a_q1, cQuaternion a_q2)
    {
        // a_q2 is passed by value so that we can scale it, etc.
        // compute angle between a_q1 and a_q2
        double costheta = a_q1.dot(a_q2);
        if ((costheta-1.0) < 1e-4 && (costheta-1.0) > -1e-4)
        {
            // quarternions are parallel
            // linearly interpolate and normalize
            *this = a_q1;
            this->mul(1.0-a_level);
            a_q2.mul(a_level);
            this->operator +=(a_q2);
            this->normalize();
        }
        else
        {
            double ratio1, ratio2;
            if ((costheta+1.0) > -1e-4 && (costheta+1.0) < 1e-4) {
                // a_q1 and a_q2 are 180 degrees apart
                // there is no unique path between them
                a_q2.w = a_q1.z;
                a_q2.x = -a_q1.y;
                a_q2.y = a_q1.x;
                a_q2.z = -a_q1.w;
                ratio1 = sin(C_PI*(0.5-a_level));
                ratio2 = sin(C_PI*a_level);
            } else {
                if (costheta < 0.0) {
                    costheta = -costheta;
                    a_q2.negate();
                }
                double theta = acos(costheta);
                double sintheta = sin(theta);

                ratio1 = sin(theta*(1.0-a_level))/sintheta;
                ratio2 = sin(theta*a_level)/sintheta;
            }
            *this = a_q1;
            this->mul(ratio1);
            a_q2.mul(ratio2);
            this->operator +=(a_q2);
        }
    }
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
