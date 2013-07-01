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
    \author    Phil Fong
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 995 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CQuaternionH
#define CQuaternionH
//------------------------------------------------------------------------------
#include "math/CMatrix3d.h"
#include "math/CVector3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CQuaternion.h

    \brief  
    <b> Math </b> \n 
    Quaternion class.
*/
//==============================================================================

//==============================================================================
/*!
    \struct     cQuaternion
    \ingroup    math
    
    \brief 
    Quaternion class.

    \details
    cQuaternion is a class that models quaternions and provides some basic 
    operations. \n

    Unit quaternions provide a convenient mathematical notation for representing 
    orientations and rotations of objects in three dimensions. Compared to Euler 
    angles they are simpler to compose and avoid the problem of gimbal lock. 
*/
//==============================================================================
struct cQuaternion 
{
    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Component __w__ of quaternion.
    double w;

    //! Component __x__ of quaternion.
    double x;

    //! Component __y__ of quaternion.
    double y;

    //! Component __z__ of quaternion.
    double z;


    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cQuaternion.
    cQuaternion() {}

    //! Constructor of cQuaternion.
    cQuaternion(double a_w, 
                double a_x, 
                double a_y, 
                double a_z) : w(a_w), x(a_x), y(a_y), z(a_z) {}

    //! Constructor of cQuaternion.
    cQuaternion(double const* in) : w(in[0]), x(in[1]), y(in[2]), z(in[3]) {}


    //--------------------------------------------------------------------------
    // OPERATORS:
    //--------------------------------------------------------------------------

public:

    //! Cast quaternion to a __double*__.
    inline operator double*() {return &w;}

    //! Cast quaternion to a __double const*__.
    inline operator double const*() const { return &w;}

    //! <b> *= </b> operator (Grassman product).
    inline cQuaternion& operator*= (cQuaternion const& a_quaternion)
    {
        double neww = w*a_quaternion.w - x*a_quaternion.x - y*a_quaternion.y - z*a_quaternion.z;
        double newx = w*a_quaternion.x + x*a_quaternion.w + y*a_quaternion.z - z*a_quaternion.y;
        double newy = w*a_quaternion.y - x*a_quaternion.z + y*a_quaternion.w + z*a_quaternion.x;
        double newz = w*a_quaternion.z + x*a_quaternion.y - y*a_quaternion.x + z*a_quaternion.w;
        w = neww;
        x = newx;
        y = newy;
        z = newz;

        return (*this);
    }

    //! <b> *= </b> operator.
    inline cQuaternion& operator*= (double a_scale)
    {
        w *= a_scale; 
        x *= a_scale; 
        y *= a_scale; 
        z *= a_scale;
        return (*this);
    }
    
    //! <b> += </b> operator
    inline cQuaternion& operator+= (cQuaternion const& a_quaternion)
    {
        w+=a_quaternion.w; 
        x+=a_quaternion.x; 
        y+=a_quaternion.y; 
        z+=a_quaternion.z;
        return (*this);
    }

    //! <b> -= </b> operator
    inline cQuaternion& operator-= (cQuaternion const& a_quaternion)
    {
        w-=a_quaternion.w; 
        x-=a_quaternion.x; 
        y-=a_quaternion.y; 
        z-=a_quaternion.z;
        return (*this);
    }

    //! <b> == </b> operator.
   inline  bool operator==(cQuaternion const& a_quaternion) const
    {
        return ( (w==a_quaternion.w) && 
                 (x==a_quaternion.x) && 
                 (y==a_quaternion.y) && 
                 (z==a_quaternion.z) );
    }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Clear quaternion with zeros.
    inline void zero() { w=0.0; x=0.0; y=0.0; z=0.0; }

    //! Negate current quaternion. Result is stored in current quaternion.
    inline void negate() { w=-w; x=-x; y=-y; z=-z; }
  
    //! Returns quaternion magnitude squared.
    inline double magsq() const { return (w*w) + (x*x) + (y*y) + (z*z); }

    //! Returns quaternion magnitude squared.
    inline double lengthsq() const { return magsq(); }

    //! Returns quaternion magnitude.
    inline double mag() const { return sqrt(magsq()); }

    //! Returns quaternion magnitude.
    inline double length() const { return mag(); }

    //! Normalize quaternion.
    inline void normalize()
    {
        double m = mag();
        w /= m;
        x /= m;
        y /= m;
        z /= m;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Convert __quaternion__ into a __rotation matrix__.
    
        \details
        Convert __quaternion__ into a __rotation matrix__ passed as argument. \n
        Result is stored in matrix _a_matrix_ past as argument.

        \param      a_matrix  Output result.
    */
    //--------------------------------------------------------------------------
    inline void toRotMat(cMatrix3d& a_matrix) const
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

        a_matrix(0,0) = 1.0 - y2 - z2;
        a_matrix(0,1) = xy - wz;
        a_matrix(0,2) = xz + wy;
        a_matrix(1,0) = xy + wz;
        a_matrix(1,1) = 1.0 - x2 - z2;
        a_matrix(1,2) = yz - wx;
        a_matrix(2,0) = xz - wy;
        a_matrix(2,1) = yz + wx;
        a_matrix(2,2) = 1.0 - x2 - y2;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Convert a __rotation matrix__ into a __quaternion__.
    
        \details
        Convert a __rotation matrix__ into a __quaternion__.
        Result is stored in current __quaternion__.

        \param      a_matrix  Input rotation matrix.
    */
    //--------------------------------------------------------------------------
    inline void fromRotMat(cMatrix3d const& a_matrix)
    {
        double trace = 1.0 + a_matrix(0,0) + a_matrix(1,1) + a_matrix(2,2);

        if (trace>0.00000001) 
        {
            double s = 2.0*sqrt(trace);
            x = (a_matrix(2,1) - a_matrix(1,2))/s;
            y = (a_matrix(0,2) - a_matrix(2,0))/s;
            z = (a_matrix(1,0) - a_matrix(0,1))/s;
            w = 0.25*s;
        } 
        else if ((a_matrix(0,0) > a_matrix(1,1)) && (a_matrix(0,0) > a_matrix(2,2))) 
        {
            // column 1 has largest diagonal
            double s = 2.0*sqrt(1.0+a_matrix(0,0)-a_matrix(1,1)-a_matrix(2,2));
            x = 0.25*s;
            y = (a_matrix(1,0) + a_matrix(0,1))/s;
            z = (a_matrix(0,2) + a_matrix(2,0))/s;
            w = (a_matrix(2,1) - a_matrix(1,2))/s;
        } 
        else if (a_matrix(1,1) > a_matrix(2,2)) 
        {
            // column 2 has largest diagonal
            double s = 2.0*sqrt(1.0+a_matrix(1,1)-a_matrix(0,0)-a_matrix(2,2));
            x = (a_matrix(1,0) + a_matrix(0,1))/s;
            y = 0.25*s;
            z = (a_matrix(2,1) + a_matrix(1,2))/s;
            w = (a_matrix(0,2) - a_matrix(2,0))/s;
        } 
        else 
        {
            // column 3 has largest diagonal
            double s = 2.0*sqrt(1.0+a_matrix(2,2)-a_matrix(0,0)-a_matrix(1,1));
            x = (a_matrix(0,2) + a_matrix(2,0))/s;
            y = (a_matrix(2,1) + a_matrix(1,2))/s;
            z = 0.25*s;
            w = (a_matrix(1,0) - a_matrix(0,1))/s;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Convert an __axis-angle__ representation into a __quaternion__.
    
        \details
        Convert an __axis-angle__ representation into a __quaternion__. \n
        Result is stored in current __quaternion__.

        \param      a_axis  Axis vector.
        \param      a_angleRad  Angle in radians.
    */
    //--------------------------------------------------------------------------
    inline void fromAxisAngle(cVector3d a_axis, double a_angleRad)
    {
        // not that axis is passed by value so that we can normalize it
        a_axis.normalize();
        double sina = sin(a_angleRad / 2.0);
        double cosa = cos(a_angleRad / 2.0);
        w = cosa;
        x = a_axis(0) * sina;
        y = a_axis(1) * sina;
        z = a_axis(2) * sina;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Convert a __quaternion__ representation into an __axis-angle__
        representation.
    
        \details
        Convert a __quaternion__ representation into an __axis-angle__
        representation \n. Result is stored in both arguments.

        \param      a_axis  Output result axis vector.
        \param      a_angleRad  Output result angle in radians.
    */
    //---------------------------------------------------------------------------
    inline void toAxisAngle(cVector3d& a_axis, double& a_angleRad) const
    {
        double cosa = w / mag();
        a_angleRad = acos(cosa);
        a_axis(0) = x;
        a_axis(1) = y;
        a_axis(2) = z;
    }

    //! Conjugate quaternion.
    inline void conj()
    {
        x = -x;
        y = -y;
        z = -z;
    }

    //! Invert quaternion (the inverse is equal to <em> conjugate/magsq </em> )
    inline void invert()
    {
        double m2 = magsq();
        w =  w/m2;
        x = -x/m2;
        y = -y/m2;
        z = -z/m2;
    }


    //---------------------------------------------------------------
    /*!
        \brief
        Multiplication: <em> this = this * a_quaternion </em>

        \details
        Multiply this __quaternion__ with another __quaternion__ and 
        store result here.

        \param      a_quaternion  Input quaternion.
    */
    //---------------------------------------------------------------
    inline void mul(cQuaternion const& a_quaternion)
    {
        double neww = w*a_quaternion.w - x*a_quaternion.x - y*a_quaternion.y - z*a_quaternion.z;
        double newx = w*a_quaternion.x + x*a_quaternion.w + y*a_quaternion.z - z*a_quaternion.y;
        double newy = w*a_quaternion.y - x*a_quaternion.z + y*a_quaternion.w + z*a_quaternion.x;
        double newz = w*a_quaternion.z + x*a_quaternion.y - y*a_quaternion.x + z*a_quaternion.w;

        w = neww;
        x = newx;
        y = newy;
        z = newz;
    }


    //---------------------------------------------------------------
    /*!
        \brief
        Scale: <em> this = a_scale * this </em>

        \details
        Scale this quaternion by a scalar.

        \param      a_scale  Scale factor.
    */
    //---------------------------------------------------------------
    inline void mul(double a_scale)
    {
        w *= a_scale; 
        x *= a_scale; 
        y *= a_scale; 
        z *= a_scale;
    }


    //---------------------------------------------------------------
    /*!
        \brief
        Dot product: <em> this = this . a_quaternion </em>

        \details
        Compute dot product with another __quaternion__ passed as
        argument and store the result here.

        \param      a_quaternion  Input quaternion.

        \return     Dot product.
    */
    //---------------------------------------------------------------
    inline double dot(cQuaternion const& a_quaternion) const
    {
        return (w*a_quaternion.w + x*a_quaternion.x + y*a_quaternion.y + z*a_quaternion.z);
    }


    //---------------------------------------------------------------
    /*! 
        \brief
        Addition: <em> this = this + a_quaternion </em>.

        \details
        Add another quaternion to this one and store here.

        \param      a_quaternion  Input quaternion.
    */
    //---------------------------------------------------------------
    inline void add(cQuaternion const& a_quaternion)
    {
        w+=a_quaternion.w; 
        x+=a_quaternion.x; 
        y+=a_quaternion.y; 
        z+=a_quaternion.z;
    }


    //---------------------------------------------------------------
    /*! 
        \brief
        Subtraction: <em> this = this - a_quaternion </em>.

        \details
        Add another quaternion to this one and store here.

        \param      a_quaternion  Input quaternion.
    */
    //---------------------------------------------------------------
    inline void sub(cQuaternion const& a_quaternion)
    {
        w-=a_quaternion.w; 
        x-=a_quaternion.x; 
        y-=a_quaternion.y; 
        z-=a_quaternion.z;
    }


    //---------------------------------------------------------------
    /*! 
        \brief
        SLERP - Spherical linear interpolation.

        \details
        Spherically linearly interpolate between quaternions and store 
        the result here.
        
        \param      a_level  Parameter between 0 (fully at a_quaternion0) and 1.0 (fully at a_quaternion1).
        \param      a_quaternion0  Starting quaternion.
        \param      a_quaternion1  Ending quaternion.
    */
    //---------------------------------------------------------------
    inline void slerp(double a_level, 
                      cQuaternion const& a_quaternion0, 
                      cQuaternion a_quaternion1)
    {
        // a_quaternion1 is passed by value so that we can scale it, etc.
        // compute angle between a_quaternion0 and a_quaternion1
        double costheta = a_quaternion0.dot(a_quaternion1);
        if ((costheta-1.0) < 1e-4 && (costheta-1.0) > -1e-4)
        {
            // quaternions are parallel
            // linearly interpolate and normalize
            *this = a_quaternion0;
            this->mul(1.0-a_level);
            a_quaternion1.mul(a_level);
            this->operator +=(a_quaternion1);
            this->normalize();
        }
        else
        {
            double ratio1, ratio2;
            if ((costheta+1.0) > -1e-4 && (costheta+1.0) < 1e-4) 
            {
                // a_quaternion0 and a_quaternion1 are 180 degrees apart
                // there is no unique path between them
                a_quaternion1.w = a_quaternion0.z;
                a_quaternion1.x = -a_quaternion0.y;
                a_quaternion1.y = a_quaternion0.x;
                a_quaternion1.z = -a_quaternion0.w;
                ratio1 = sin(C_PI*(0.5-a_level));
                ratio2 = sin(C_PI*a_level);
            } 
            else 
            {
                if (costheta < 0.0) 
                {
                    costheta = -costheta;
                    a_quaternion1.negate();
                }
                double theta = acos(costheta);
                double sintheta = sin(theta);

                ratio1 = sin(theta*(1.0-a_level))/sintheta;
                ratio2 = sin(theta*a_level)/sintheta;
            }
            *this = a_quaternion0;
            this->mul(ratio1);
            a_quaternion1.mul(ratio2);
            this->operator +=(a_quaternion1);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        String conversion.

        \details
        Convert quaternion into a string.

        \param      a_precision    Number of digits.

        \return     Output string.
    */
    //--------------------------------------------------------------------------
    inline std::string str(const unsigned int a_precision = 2) const
    {
        std::string result;
        result = ("("+
                  cStr(w, a_precision) + "| " +
                  cStr(x, a_precision) + ", " +
                  cStr(y, a_precision) + ", " +
                  cStr(z, a_precision) + 
                  ")");
        return (result);
    }
};


//==============================================================================
// OPERATORS ON CQUATERNION:
//==============================================================================


//! An overloaded <b> * </b> operator for quaternion/scalar multiplication.
inline cQuaternion operator*(const cQuaternion& a_quaternion, const double a_scale)
{
    cQuaternion result = a_quaternion;
    result.mul(a_scale);
    return (result);
}


//! An overloaded <b> * </b> operator for scalar/quaternion multiplication.
inline cQuaternion operator*(const double a_scale, const cQuaternion& a_quaternion)
{
    cQuaternion result = a_quaternion;
    result.mul(a_scale);
    return (result);
}


//! An overloaded <b> * </b> operator for quaternion multiplication.
inline cQuaternion operator*(const cQuaternion& a_quaternion0, const cQuaternion& a_quaternion1)
{
    cQuaternion result = a_quaternion0;
    result.mul(a_quaternion1);
    return (result);
}


//! An overloaded <b> + </b> operator for quaternion addition.
inline cQuaternion operator+(const cQuaternion& a_quaternion0, const cQuaternion& a_quaternion1)
{
    cQuaternion result = a_quaternion0;
    result.add(a_quaternion1);
    return (result);
}


//! An overloaded <b> - </b> operator for quaternion subtraction.
inline cQuaternion operator-(const cQuaternion& a_quaternion0, const cQuaternion& a_quaternion1)
{
    cQuaternion result = a_quaternion0;
    result.sub(a_quaternion1);
    return (result);
}


//! <b> ostream <b> operator. Outputs the cQuaternion's components.
static inline std::ostream &operator << (std::ostream &a_os, cQuaternion const& a_quaternion)
{
    a_os << a_quaternion.str(3);
    return (a_os);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
