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
    \author    Francois Conti
    \author    Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 825 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CVector3dH
#define CVector3dH
//---------------------------------------------------------------------------
#include "system/CString.h"
#include "system/CGlobals.h"
#include "math/CConstants.h"
#include <ostream>
#include <cmath>
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CVector3d.h

    \brief  
    <b> Math </b> \n 
    3D Vectors.
*/
//===========================================================================

//===========================================================================
/*!
    \struct     cVector3d
    \ingroup    math  
    
    \brief    
    This vector class provides storage for a 3 dimensional double precision 
    floating point vector as well as simple floating point arithmetic 
    operations.
*/
//===========================================================================
struct cVector3d : public Vector3d
{
    public:

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Constructors of cVector3d.

        You can initialize a cVector3d from any of the following: \n

        char*                     \n
        string                    \n
        double,double,double      \n
        cVector3d                 \n
        Vector3d (Eigen)          \n
        double*                   \n

        See the set(char*) function for a description of the acceptable
        string formats.
    */
    //-----------------------------------------------------------------------
    cVector3d():Vector3d()
    {
    }

    //! Constructor by passing three doubles to initialize vector.
    cVector3d(const double a_x, const double a_y, const double a_z)
    { 
        (*this)(0) = a_x; 
        (*this)(1) = a_y; 
        (*this)(2) = a_z; 
    }

    //! Constructor by passing a cVector3d vector to initialize vector.
    cVector3d (const cVector3d &other) : Vector3d(other)
    {
        (*this)(0) = other(0) ;
        (*this)(1) = other(1) ;
        (*this)(2) = other(2) ;
    }

    //! Constructor by passing an Eigen Vector3d vector to initialize vector.
    cVector3d (const Vector3d &other)
    {
        (*this)(0) = other(0);
        (*this)(1) = other(1);
        (*this)(2) = other(2);
    }

    //! Constructor by passing a string to initialize vector.
    cVector3d(const char* a_initstr)
    { 
        set(a_initstr); 
    }

    //! Constructor by passing a string to initialize vector.
    cVector3d(const string& a_initstr)
    { 
        set(a_initstr); 
    }


    //-----------------------------------------------------------------------
    // OPERATION METHODS:
    //-----------------------------------------------------------------------

    //-----------------------------------------------------------------------
    /*!
        Get vector components.
    */
    //-----------------------------------------------------------------------
    inline double x() const
    { 
        return((*this)(0)); 
    }
    
    
    inline double y() const
    { 
        return((*this)(1)); 
    }
    
    
    inline double z() const
    {
        return((*this)(2)); 
    }
    

    //-----------------------------------------------------------------------
    /*!
        Set vector components.
    */
    //-----------------------------------------------------------------------
    inline void x(const double a_value) 
    { 
        (*this)(0) = a_value; 
    }
    
    
    inline void y(const double a_value) 
    { 
        (*this)(1) = a_value; 
    }
    
    
    inline void z(const double a_value) 
    { 
        (*this)(2) = a_value; 
    }


    //-----------------------------------------------------------------------
    /*!
        Clear vector with zeros.
    */
    //-----------------------------------------------------------------------
    inline void zero()
    {
        (*this)(0) = 0.0;
        (*this)(1) = 0.0;
        (*this)(2) = 0.0;
    }


    //-----------------------------------------------------------------------
    /*!
        Return the i th component of the vector. \e component = 0 return x,
        \e component = 1 returns y, \e component = 2 returns z.

        \param  a_component  component number

        \return selected vector component
    */
    //-----------------------------------------------------------------------
    inline double get(const unsigned int& a_component) const
    {
        return ((double*)(this))[a_component];
    }


    //-----------------------------------------------------------------------
    /*!
        An overloaded /= operator for vector/scalar division
    */
    //-----------------------------------------------------------------------
    inline void operator/= (const double& a_val)
    {
        double factor = 1.0 / a_val;
        (*this)(0) *= factor;
        (*this)(1) *= factor;
        (*this)(2) *= factor;
    }


    //-----------------------------------------------------------------------
    /*!
        An overloaded *= operator for vector/scalar multiplication
    */
    //-----------------------------------------------------------------------
    inline void operator*= (const double& a_val)
    {
        (*this)(0) *= a_val;
        (*this)(1) *= a_val;
        (*this)(2) *= a_val;
    }


    //-----------------------------------------------------------------------
    /*!
        An overloaded += operator for vector/vector addition
    */
    //-----------------------------------------------------------------------
    inline void operator+= (const cVector3d& a_input)
    {
        (*this)(0) += a_input(0);
        (*this)(1) += a_input(1);
        (*this)(2) += a_input(2);
    }


    //-----------------------------------------------------------------------
    /*!
        An overloaded -= operator for vector/vector subtraction
    */
    //-----------------------------------------------------------------------
    inline void operator-= (const cVector3d& a_input)
    {
        (*this)(0) -= a_input(0);
        (*this)(1) -= a_input(1);
        (*this)(2) -= a_input(2);
    }


    //-----------------------------------------------------------------------
    /*!
        An overloaded = operator
    */
    //-----------------------------------------------------------------------
    inline void operator= (const cVector3d& a_input)
    {
        //Vector3d::operator=(a_input);
        (*this)(0) = a_input(0);
        (*this)(1) = a_input(1);
        (*this)(2) = a_input(2);  
    }
  

    //-----------------------------------------------------------------------
    /*!
        Initialize 3 dimensional vector with parameters \e x, \e y, and \e z.

        \param  a_x  X component.
        \param  a_y  Y component.
        \param  a_z  Z component.
    */
    //-----------------------------------------------------------------------
    inline void set(const double& a_x, const double& a_y, const double& a_z)
    {
        (*this)(0) = a_x;
        (*this)(1) = a_y;
        (*this)(2) = a_z;
    }


    //-----------------------------------------------------------------------
    /*!
        Initialize a vector from a string of the form (x,y,z), the
        same form produced by str().  Will actually accept any of the
        following forms:\n

        (4.3,23,54) \n
        4.3 54 2.1  \n
        4.5,7.8,9.1 \n

        ...i.e., it expects three numbers, optionally preceded
        by '(' and whitespace, and separated by commas or whitespace.

        \param   a_initStr The string to convert

        \return  \b true if conversion was successful.
    */
    //-----------------------------------------------------------------------
    inline bool set(const char* a_initStr)
    {
        // sanity check
        if (a_initStr == 0) return false;

        // look for a valid-format string. ignore leading whitespace and ('s
        const char* curpos = a_initStr;
        while( (*curpos != '\0') &&
               (*curpos == ' ' || *curpos == '\t' || *curpos == '('))
        {
            curpos++;
        }

        // scan data
        double ax, ay, az;
        int result = sscanf(curpos,"%lf%*[ ,\t\n\r]%lf%*[ ,\t\n\r]%lf", &ax, &ay, &az);

        // make sure the conversion worked
        if (result !=3) return (false);

        // copy the values we found
        (*this)(0) = ax; 
        (*this)(1) = ay; 
        (*this)(2) = az;

        // return result
        return (true);
    }


    //-----------------------------------------------------------------------
    /*!
        Initialize a vector from a string of the form (x,y,z) (the
        same form produced by str() )

        \param   a_initStr The string to convert

        \return  \b true if conversion was successful.
    */
    //-----------------------------------------------------------------------
    inline bool set(const string& a_initStr)
    {
        return ( set(a_initStr.c_str()) );
    }


    //-----------------------------------------------------------------------
    /*!
        Copy current vector to external vector as parameter.

        \param  a_destination  Destination vector.
    */
    //-----------------------------------------------------------------------
    inline void copyto(cVector3d& a_destination) const
    {
        a_destination(0) = (*this)(0);
        a_destination(1) = (*this)(1);
        a_destination(2) = (*this)(2);
    }


    //-----------------------------------------------------------------------
    /*!
        Copy external vector as parameter to current vector.

        \param  a_source  Source vector.
    */
    //-----------------------------------------------------------------------
    inline void copyfrom(const cVector3d &a_source)
    {
        (*this)(0) = a_source(0);
        (*this)(1) = a_source(1);
        (*this)(2) = a_source(2);
    }


    //-----------------------------------------------------------------------
    /*!
        Addition between current vector and  external vector passed as
        parameter. \n
        Result is stored in current vector.

        \param  a_vector  This vector is added to the current one.
    */
    //-----------------------------------------------------------------------
    inline void add(const cVector3d& a_vector)
    {
     
        (*this)(0) = (*this)(0) + a_vector(0);
        (*this)(1) = (*this)(1) + a_vector(1);
        (*this)(2) = (*this)(2) + a_vector(2);
    }


    //-----------------------------------------------------------------------
    /*!
        Addition between current vector and external vector passed as
        parameter. \n
        Result is stored in current vector.

        \param  a_x  X component.
        \param  a_y  Y component.
        \param  a_z  Z component.
    */
    //-----------------------------------------------------------------------
    inline void add(const double& a_x, 
                    const double& a_y, 
                    const double& a_z)
    {
        (*this)(0) = (*this)(0) + a_x;
        (*this)(1) = (*this)(1) + a_y;
        (*this)(2) = (*this)(2) + a_z;
    }


    //-----------------------------------------------------------------------
    /*!
        Addition between current vector and external vector passed as
        parameter.\n  Result is stored in external \e result vector.

        \param  a_vector  Vector which is added to current one.
        \param  a_result  Vector where result is stored.
    */
    //-----------------------------------------------------------------------
    inline void addr(const cVector3d& a_vector, 
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0) + a_vector(0) ;
        a_result(1)  = (*this)(1) + a_vector(1) ;
        a_result(2)  = (*this)(2) + a_vector(2) ;
    }


    //-----------------------------------------------------------------------
    /*!
        Addition between current vector and vector passed by parameter.\n
        Result is stored in \e result vector.

        \param  a_x  X component.
        \param  a_y  Y component.
        \param  a_z  Z component.
        \param  a_result  Vector where result is stored.
    */
    //-----------------------------------------------------------------------
    inline void addr(const double& a_x, 
                     const double& a_y, 
                     const double& a_z, 
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0) + a_x;
        a_result(1)  = (*this)(1) + a_y;
        a_result(2)  = (*this)(2) + a_z;
    }


    //-----------------------------------------------------------------------
    /*!
        Subtraction between current vector and an external vector
        passed as parameter.\n
        Result is stored in current vector.

        \param  a_vector  Vector which is subtracted from current one.
    */
    //-----------------------------------------------------------------------
    inline void sub(const cVector3d& a_vector)
    {
        (*this)(0) = (*this)(0) - a_vector(0) ;
        (*this)(1) = (*this)(1) - a_vector(1) ;
        (*this)(2) = (*this)(2) - a_vector(2) ;
    }


    //-----------------------------------------------------------------------
    /*!
        Subtract an external vector passed as parameter from current
        vector. \n Result is stored in current vector.

        \param  a_x  X component.
        \param  a_y  Y component.
        \param  a_z  Z component.
    */
    //-----------------------------------------------------------------------
    inline void sub(const double& a_x, 
                    const double& a_y, 
                    const double& a_z)
    {
        (*this)(0) = (*this)(0) - a_x;
        (*this)(1) = (*this)(1) - a_y;
        (*this)(2) = (*this)(2) - a_z;
    }


    //-----------------------------------------------------------------------
    /*!
        Subtraction between current vector and external vector passed as
        parameter.\n  Result is stored in external \e a_result vector.

        \param  a_vector  Vector which is subtracted from current one.
        \param  a_result  Vector where result is stored.
    */
    //-----------------------------------------------------------------------
    inline void subr(const cVector3d& a_vector, 
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0) - a_vector(0) ;
        a_result(1)  = (*this)(1) - a_vector(1) ;
        a_result(2)  = (*this)(2) - a_vector(2) ;
    }


    //-----------------------------------------------------------------------
    /*!
        Subtract current vector from vector passed by parameter.\n
        Result is stored in \e result vector.

        \param  a_x  X component.
        \param  a_y  Y component.
        \param  a_z  Z component.
        \param  a_result  Vector where result is stored.
    */
    //-----------------------------------------------------------------------
    inline void subr(const double& a_x, 
                     const double& a_y, 
                     const double& a_z,
                     cVector3d &a_result) const
    {
        a_result(0)  = (*this)(0) - a_x;
        a_result(1)  = (*this)(1) - a_y;
        a_result(2)  = (*this)(2) - a_z;
    }


    //-----------------------------------------------------------------------
    /*!
        Multiply current vector by a scalar. \n
        Result is stored in current vector.

        \param  a_scalar  Scalar value.
    */
    //-----------------------------------------------------------------------
    inline void mul(const double &a_scalar)
    {
        (*this)(0) = a_scalar * (*this)(0);
        (*this)(1) = a_scalar * (*this)(1);
        (*this)(2) = a_scalar * (*this)(2);
    }


    //-----------------------------------------------------------------------
    /*!
        Multiply current vector by a scalar. \n
        Result is stored in \e result vector.

        \param  a_scalar  Scalar value.
        \param  a_result  Result vector.
    */
    //-----------------------------------------------------------------------
    inline void mulr(const double& a_scalar, 
                     cVector3d& a_result) const
    {
        a_result(0)  = a_scalar * (*this)(0);
        a_result(1)  = a_scalar * (*this)(1);
        a_result(2)  = a_scalar * (*this)(2);
    }


    //-----------------------------------------------------------------------
    /*!
        Divide current vector by a scalar. No check for divide-by-zero
        is performed.

        Result is stored in current vector.

        \param  a_scalar  Scalar value.
    */
    //-----------------------------------------------------------------------
    inline void div(const double& a_scalar)
    {
        double factor = 1.0 / a_scalar;
        (*this)(0) = (*this)(0) * factor;
        (*this)(1) = (*this)(1) * factor;
        (*this)(2) = (*this)(2) * factor;
    }


    //-----------------------------------------------------------------------
    /*!
        Divide current vector by a scalar. \n
        Result is stored in \e result vector.

        \param  a_scalar  Scalar value.
        \param  a_result  Result vector.
    */
    //-----------------------------------------------------------------------
    inline void divr(const double& a_scalar, 
                     cVector3d& a_result) const
    {
        double factor = 1.0 / a_scalar;
        a_result(0)  = (*this)(0) * factor;
        a_result(1)  = (*this)(1) * factor;
        a_result(2)  = (*this)(2) * factor;
    }


    //-----------------------------------------------------------------------
    /*!
        Negate current vector. \n
        Result is stored in current vector.
    */
    //-----------------------------------------------------------------------
    inline void negate()
    {
        (*this)(0) = -(*this)(0);
        (*this)(1) = -(*this)(1);
        (*this)(2) = -(*this)(2);
    }


    //-----------------------------------------------------------------------
    /*!
        Negate current vector. \n
        Result is stored in \e result vector.

        \param  a_result  Result vector.
    */
    //-----------------------------------------------------------------------
    inline void negater(cVector3d& a_result) const
    {
        a_result(0)  = -(*this)(0);
        a_result(1)  = -(*this)(1);
        a_result(2)  = -(*this)(2);
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the cross product between current vector and an external
        vector. \n Result is stored in current vector.

        \param  a_vector  Vector with which cross product is computed with.
    */
    //-----------------------------------------------------------------------
    inline void cross(const cVector3d& a_vector)
    {
        // compute cross product
        double a =  ((*this)(1) * a_vector(2) ) - ((*this)(2) * a_vector(1) );
        double b = -((*this)(0) * a_vector(2) ) + ((*this)(2) * a_vector(0) );
        double c =  ((*this)(0) * a_vector(1) ) - ((*this)(1) * a_vector(0) );

        // store result in current vector
        (*this)(0) = a;
        (*this)(1) = b;
        (*this)(2) = c;
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the cross product between current vector and an
        external vector passed as parameter. \n

        Result is returned.  Performance-wise, cross() and crossr() are usually
        preferred, since this version creates a new stack variable.

        \param  a_vector  Vector with which cross product is computed.

        \return Resulting cross product.
    */
    //-----------------------------------------------------------------------
    inline cVector3d crossAndReturn(const cVector3d& a_vector) const
    {
        cVector3d r;
        crossr(a_vector,r);
        return (r);
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the cross product between current vector and an
        external vector passed as parameter. \n
        Result is stored in \e a_result vector.

        \param  a_vector  Vector with which cross product is computed.
        \param  a_result  Vector where result is stored.
    */
    //-----------------------------------------------------------------------
    inline void crossr(const cVector3d& a_vector, 
                       cVector3d& a_result) const
    {
        a_result(0)  =  ((*this)(1) * a_vector(2) ) - ((*this)(2) * a_vector(1) );
        a_result(1)  = -((*this)(0) * a_vector(2) ) + ((*this)(2) * a_vector(0) );
        a_result(2)  =  ((*this)(0) * a_vector(1) ) - ((*this)(1) * a_vector(0) );
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the dot product between current vector and an external vector
        passed as parameter.

        \param  a_vector  Vector with which dot product is computed.

        \return Dot product computed between both vectors.
    */
    //-----------------------------------------------------------------------
    inline double dot(const cVector3d& a_vector) const
    {
        return(((*this)(0) * a_vector(0) ) + ((*this)(1) * a_vector(1) ) + ((*this)(2) * a_vector(2) ));
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the element-by-element product between current vector and an external
        vector and store in the current vector.

        \param  a_vector  Vector with which product is computed.
    */
    //-----------------------------------------------------------------------
    inline void elementMul(const cVector3d& a_vector)
    {
        (*this)(0)*=a_vector(0) ;
        (*this)(1)*=a_vector(1) ;
        (*this)(2)*=a_vector(2) ;
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the element-by-element product between current vector and an external
        vector and store in the supplied output vector.

        \param  a_vector  Vector with which product is computed.
        \param  a_result  Resulting vector.
    */
    //-----------------------------------------------------------------------
    inline void elementMulr(const cVector3d& a_vector, 
                            cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0)*a_vector(0) ;
        a_result(1)  = (*this)(1)*a_vector(1) ;
        a_result(2)  = (*this)(2)*a_vector(2) ;
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the length of current vector.

        \return   Returns length of current vector.
    */
    //-----------------------------------------------------------------------
    inline double length() const
    {
        return(sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2))));
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the square length of current vector.

        \return   Returns square length of current vector.
    */
    //-----------------------------------------------------------------------
    inline double lengthsq() const
    {
        return(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
    }


    //-----------------------------------------------------------------------
    /*!
        Normalize current vector to become a vector of length one.\n
        \b Warning: \n
        Vector should not be equal to (0,0,0) or a division
        by zero error will occur. \n
        Result is stored in current vector.
    */
    //-----------------------------------------------------------------------
    inline void normalize()
    {
        // compute length of vector
        double length = sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
        if (length == 0.0) { return; }
        double factor = 1.0 / length;

        // divide current vector by its length
        (*this)(0) = (*this)(0) * factor;
        (*this)(1) = (*this)(1) * factor;
        (*this)(2) = (*this)(2) * factor;
    }


    //-----------------------------------------------------------------------
    /*!
        Clamp the vector to a maximum desired length. Vectors that are smaller
        than a_maxLength are not affected.

        \param  a_maxLength  Maximum desired length for vector.
    */
    //-----------------------------------------------------------------------
    inline void clamp(const double& a_maxLength)
    {
        double length = sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
        if (a_maxLength == 0)
        {
            (*this)(0) = 0.0;
            (*this)(1) = 0.0;
            (*this)(2) = 0.0;
        }
        else if (length > a_maxLength)
        {
            double factor = a_maxLength / length;
            (*this)(0) = (*this)(0) * factor;
            (*this)(1) = (*this)(1) * factor;
            (*this)(2) = (*this)(2) * factor;              
        }
    }


    //-----------------------------------------------------------------------
    /*!
        Normalize current vector to become a vector of length one. \n
        \b WARNING: Vector should not be equal to (0,0,0) or a division
        by zero error will occur. \n
        Result is stored in \e result vector.

        \param  a_result  Vector where result is stored.
    */
    //-----------------------------------------------------------------------
    inline void normalizer(cVector3d& a_result) const
    {
        // compute length of vector
        double length = sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
        double factor = 1.0 / length;

        // divide current vector by its length
        a_result(0)  = (*this)(0) * factor;
        a_result(1)  = (*this)(1) * factor;
        a_result(2)  = (*this)(2) * factor;
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the distance between current point and an external point
        passed as parameter.

        \param  a_vector  Point to which the distance is measured

        \return Distance between the points
    */
    //-----------------------------------------------------------------------
    inline double distance(const cVector3d& a_vector) const
    {
        // compute distance between both points
        double dx = (*this)(0) - a_vector(0) ;
        double dy = (*this)(1) - a_vector(1) ;
        double dz = (*this)(2) - a_vector(2) ;

        // return result
        return(sqrt( dx * dx + dy * dy + dz * dz ));
    }


    //-----------------------------------------------------------------------
    /*!
        Compute the square distance between the current point and an external
        point.

        \param  a_vector  Point to which squared distance is measured

        \return Squared distance between the points
    */
    //-----------------------------------------------------------------------
    inline double distancesq(const cVector3d& a_vector) const
    {
        // compute distance between both points
        double dx = (*this)(0) - a_vector(0) ;
        double dy = (*this)(1) - a_vector(1) ;
        double dz = (*this)(2) - a_vector(2) ;

        // return result
        return( dx * dx + dy * dy + dz * dz );
    }


    //-----------------------------------------------------------------------
    /*!
        Test whether the current vector and an external vector are equal.

        \param    a_vector  Vector with which equality is checked.
        \param    epsilon  Two vectors will be considered equal if each
                  component is within epsilon units.  Defaults
                  to zero.

        \return   \b true if both vectors are equal, otherwise
                  returns \b false.
    */
    //-----------------------------------------------------------------------
    inline bool equals(const cVector3d& a_vector, 
                       const double epsilon = 0.0) const
    {
        // Accelerated path for exact equality
        if (epsilon == 0.0)
        {
            if ( ((*this)(0) == a_vector(0) ) && ((*this)(1) == a_vector(1) ) && ((*this)(2) == a_vector(2) ) )
            {
                return (true);
            }
            else
            {
                return (false);
            }
        }

        if ((fabs(a_vector(0) - (*this)(0)) < epsilon) &&
            (fabs(a_vector(1) - (*this)(1)) < epsilon) &&
            (fabs(a_vector(2) - (*this)(2)) < epsilon))
        {
            return (true);
        }
        else
        {
            return (false);
        }
    }


    //-----------------------------------------------------------------------
    /*!
        Convert current vector into a string.

        \param   a_precision  Number of digits.

        \return  Return output string.
    */
    //-----------------------------------------------------------------------
    inline string str(const unsigned int a_precision = 2) const
    {
        string result;
        result = ("( ") + cStr((*this)(0), a_precision) + ", "
                        + cStr((*this)(1), a_precision) + ", "
                        + cStr((*this)(2), a_precision) + " )";
        return (result);
    }


    //-----------------------------------------------------------------------
    /*!
        Print the current vector using the cPrint macro.

        \param    a_precision  Number of digits.
    */
    //-----------------------------------------------------------------------
    inline void print(const unsigned int a_precision = 2) const
    {
        string s = str(a_precision);
        cPrint("%s\n", s.c_str());
    }


    //-----------------------------------------------------------------------
    /*!
        Decompose me into components parallel and perpendicular to the input
        vector.

        \param    a_input         Reference vector.
        \param    a_parallel      Parallel component
        \param    a_perpendicular Perpendicular component
    */
    //-----------------------------------------------------------------------
    inline void decompose(const cVector3d& a_input, 
                          cVector3d& a_parallel, 
                          cVector3d& a_perpendicular) const
    {
        double scale = (this->dot(a_input) / (a_input.dot(a_input)));
        a_parallel = a_input;
        a_parallel.mul(scale);
        this->subr(a_parallel,a_perpendicular);
    }


    //-----------------------------------------------------------------------
    /*!
        Spherically linearly interpolate between two vectors and store in this vector.
        Vectors should have the same length

        \param    a_level         Fraction of distance to a_vector2 (0 is fully at a_vector1, 1.0 is fully at a_vector2)
        \param    a_vector1       First vector to interpolate from
        \param    a_vector2       Second vector to interpolate from
    */
    //-----------------------------------------------------------------------
    inline void slerp(double a_level, 
                      cVector3d const& a_vector1, 
                      cVector3d a_vector2)
    {
        // a_vector2 is passed in by value so that we may scale it
        double a_vec1lensq = a_vector1.lengthsq();
        double cosomega = a_vector1.dot(a_vector2)/(a_vec1lensq);
        if ((cosomega-1.0) > -1e-4 && (cosomega-1.0) < 1e-4) {
            // vectors are (almost) parallel
            // linearly interpolate
            *this = a_vector1;
            this->mul(1.0-a_level);
            a_vector2.mul(a_level);
            this->operator +=(a_vector2);
            this->mul(sqrt(a_vec1lensq/this->lengthsq()));
        } else {
            if (cosomega < 0.0) {
                cosomega = -cosomega;
                a_vector2.negate();
            }
            double ratio1, ratio2;
            if ((cosomega+1.0) > -1e-4 && (cosomega+1.0) < 1e-4) {
                // vectors are 180 degrees apart
                // there is no unique path between them
                if ((a_vector1(0)  < a_vector1(1) ) && (a_vector1(0)  < a_vector1(2) )){
                    // x component is the smallest
                    a_vector2(0)  = 0;
                    a_vector2(1)  = -a_vector1(2) ;
                    a_vector2(2)  = a_vector1(1) ;
                } else if (a_vector1(1)  < a_vector1(2) ) {
                    // y component is the smallest
                    a_vector2(0)  = -a_vector1(2) ;
                    a_vector2(1)  = 0;
                    a_vector2(2)  = a_vector1(0) ;
                } else {
                    // z component is the smallest
                    a_vector2(0)  = -a_vector1(1) ;
                    a_vector2(1)  = a_vector1(0) ;
                    a_vector2(2)  = 0;
                }
                // scale it so it is the same length as before
                a_vector2.mul(sqrt(a_vec1lensq/a_vector2.lengthsq()));

                ratio1 = ::sin(C_PI*(0.5-a_level));
                ratio2 = ::sin(C_PI*a_level);
            } else {
                double omega = acos(cosomega);
                double sinomega = ::sin(omega);
                ratio1 = ::sin(omega*(1.0-a_level))/sinomega;
                ratio2 = ::sin(omega*a_level)/sinomega;
            }
            *this = a_vector1;
            this->mul(ratio1);
            a_vector2.mul(ratio2);
            this->add(a_vector2);
        }
    }
};


//===========================================================================
// Operators on cVector3d
//===========================================================================

//---------------------------------------------------------------------------
/*!
    An overloaded * operator for vector/scalar multiplication.
*/
//---------------------------------------------------------------------------
inline cVector3d operator*(const cVector3d& v, const double a_input)
{
    return (cVector3d(v(0) *a_input,v(1) *a_input,v(2) *a_input));
}


//---------------------------------------------------------------------------
/*!
    An overloaded / operator for vector/scalar division.
*/
//---------------------------------------------------------------------------
inline cVector3d operator/(const cVector3d& v, const double a_input)
{
    return (cVector3d(v(0)/a_input, v(1)/a_input, v(2)/a_input));
}


//---------------------------------------------------------------------------
/*!
    An overloaded * operator for scalar/vector multiplication.
*/
//---------------------------------------------------------------------------
inline cVector3d operator*(const double a_input, const cVector3d& v)
{
    return cVector3d(v(0)*a_input, v(1)*a_input, v(2)*a_input);
}


//---------------------------------------------------------------------------
/*!
    An overloaded + operator for vector/vector addition.
*/
//---------------------------------------------------------------------------
inline cVector3d operator+(const cVector3d& v1, const cVector3d& v2)
{
    return cVector3d(v1(0)+v2(0), v1(1)+v2(1), v1(2)+v2(2));
}


//---------------------------------------------------------------------------
/*!
    An overloaded - operator for vector/vector subtraction.
*/
//---------------------------------------------------------------------------
inline cVector3d operator-(const cVector3d& v1, const cVector3d& v2)
{
    return cVector3d(v1(0) -v2(0) ,v1(1) -v2(1) ,v1(2) -v2(2) );
}


//---------------------------------------------------------------------------
/*!
    An overloaded * operator for vector/vector dotting.
*/
//---------------------------------------------------------------------------
inline double operator*(const cVector3d& v1, const cVector3d& v2)
{
    return v1(0) *v2(0) +v1(1) *v2(1) +v1(2) *v2(2) ;
}


//---------------------------------------------------------------------------
/*!
    An overloaded = operator for Eigen vectors.
*/
//---------------------------------------------------------------------------
inline Vector3d operator-(const cVector3d& v)
{
    return (Vector3d(v(0), v(1), v(2)));
}


//---------------------------------------------------------------------------
/*!
    ostream operator. \n
    Outputs the vector's components separated by commas.
*/
//---------------------------------------------------------------------------
static inline std::ostream &operator << (std::ostream &a_os, cVector3d const& a_vec)
{
    a_os << a_vec(0)  << ", " << a_vec(1)  << ", " << a_vec(2) ;
    return a_os;
}


//===========================================================================
/*!
    \struct     cRay3d
    \ingroup    math  

    \brief    
    cRay3d represents a 3D vector with an origin.
*/
//===========================================================================
struct cRay3d
{
    //! Vector representing ray origin.
    cVector3d m_origin;

    //! Unit vector representing ray direction.
    cVector3d m_direction;

    //! Constructor of cRay3d.
    cRay3d();

    //! This constructor assumes that a_direction is normalized already.
    cRay3d(const cVector3d& a_origin, 
           const cVector3d& a_direction) :
           m_origin(a_origin), 
           m_direction(a_direction){}
};


//===========================================================================
/*!
    \struct     cSegment3d
    \ingroup    math  
    
    \brief    
    cSegment3d represents a line segment with a start and an end.
*/
//===========================================================================
struct cSegment3d
{
    //! Constructor of cSegment3d.
    cSegment3d(const cVector3d& a_start, 
               const cVector3d& a_end) :
               m_start(a_start), 
               m_end(a_end){}

    //! Start point of segment.
    cVector3d m_start;

    //! End point of segment
    cVector3d m_end;


    //-----------------------------------------------------------------------
    /*!
        Returns the squared distance from this segment to a_point and the
        position along the segment (from 0.0 to 1.0) of the closest point.

        \param    a_point Point to test.
        \param    a_t return value for the position along the segment.
        \param    a_closestPoint The closest point on this segment to the supplied point.

        \return   The distance from a_point to this segment.
    */
    //-----------------------------------------------------------------------
    double distanceSquaredToPoint(const cVector3d& a_point,
                                  double& a_t,
                                  cVector3d* a_closestPoint)
    {
        double mag = m_start.distance(m_end);

        // Project this point onto the line
        a_t = (a_point - m_start) * (m_end - m_start) / (mag * mag);

        // Clip to segment endpoints
        if (a_t < 0.0)
            a_t = 0.0;
        else if (a_t > 1.0)
            a_t = 1.0;

        // Find the intersection point
        cVector3d intersection = m_start + a_t * (m_end - m_start);
        if (a_closestPoint)
        {
            *a_closestPoint = intersection;
        }

        // Compute distance
        return a_point.distancesq(intersection);
    }
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
