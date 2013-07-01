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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1077 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CVector3dH
#define CVector3dH
//------------------------------------------------------------------------------
#include "system/CString.h"
#include "system/CGlobals.h"
#include "math/CConstants.h"
#include <ostream>
#include <cmath>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CVector3d.h

    \brief  
    <b> Math </b> \n 
    3D Vector class.
*/
//==============================================================================

//==============================================================================
/*!
    \struct     cVector3d
    \ingroup    math  
    
    \brief    
    3D Vector class.

    \details
    This vector class provides storage for a 3 dimensional double precision 
    floating point vector as well as basic floating point arithmetic 
    operations.
*/
//==============================================================================
struct cVector3d
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Constructors of cVector3d. \n

        You can initialize a cVector3d from any of the following: \n

        - char*                     \n
        - string                    \n
        - double, double, double    \n
        - cVector3d                 \n
        - Vector3d (Eigen)          \n
        - double*                   \n

        See the set(const char *a_initStr) function for a description of the 
        acceptable string formats.
    */
    //--------------------------------------------------------------------------
    //! Constructor.
    cVector3d() {}

    //! Constructor by passing three doubles to initialize vector.
    cVector3d(const double a_x, const double a_y, const double a_z)
    { 
        (*this)(0) = a_x; 
        (*this)(1) = a_y; 
        (*this)(2) = a_z; 
    }

#ifdef C_USE_EIGEN

    //! Constructor by passing a cVector3d vector to initialize vector.
    cVector3d (const cVector3d &other)
    {
        (*this)(0) = other(0);
        (*this)(1) = other(1);
        (*this)(2) = other(2);
    }

    //! Constructor by passing an __Eigen Vector3d__ vector to initialize vector.
    cVector3d (const Eigen::Vector3d &other)
    {
        (*this)(0) = other(0);
        (*this)(1) = other(1);
        (*this)(2) = other(2);
    }

#else

    //! Constructor by passing a cVector3d vector to initialize vector.
    cVector3d (const cVector3d &other)
    {
        (*this)(0) = other(0) ;
        (*this)(1) = other(1) ;
        (*this)(2) = other(2) ;
    }

#endif

    //! Constructor by passing an __ANSI string__ to initialize vector.
    cVector3d(const char* a_initstr)
    { 
        set(a_initstr); 
    }

    //! Constructor by passing a __string__ to initialize vector.
    cVector3d(const std::string& a_initstr)
    { 
        set(a_initstr); 
    }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - OPERATIONS:
    //--------------------------------------------------------------------------

#ifdef C_USE_EIGEN

    //! Convert this vector to an Eigen Vector3d.
    Eigen::Vector3d eigen()
    {
        return (Eigen::Vector3d((*this)(0), (*this)(1), (*this)(2)));
    }

#endif

    //! Get vector component __x__. 
    inline double x() const
    { 
        return((*this)(0)); 
    }
    
    //! Get vector component __y__. 
    inline double y() const
    { 
        return((*this)(1)); 
    }
    
    //! Get vector component __z__. 
    inline double z() const
    {
        return((*this)(2)); 
    }
    
    //! Set vector component __x__.
    inline void x(const double a_value) 
    { 
        (*this)(0) = a_value; 
    }
    
    //! Set vector component __y__.
    inline void y(const double a_value) 
    { 
        (*this)(1) = a_value; 
    }
    
    //! Set vector component __z__.
    inline void z(const double a_value) 
    { 
        (*this)(2) = a_value; 
    }

    //! Clear all vector components with zeros.
    inline void zero()
    {
        (*this)(0) = 0.0;
        (*this)(1) = 0.0;
        (*this)(2) = 0.0;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Return the _i_ th component of the vector.

        \details
        Return the _i_ th component of the vector. \n
        - if _a_component_ = 0, then return __x__,
        - if _a_component_ = 1, then return __y__,
        - if _a_component_ = 2, then return __z__,

        \param      a_component  Component index number (0,1,2).

        \return     Selected vector component.
    */
    //--------------------------------------------------------------------------
    inline double get(const unsigned int& a_component) const
    {
        return ((double*)(this))[a_component];
    }


    //--------------------------------------------------------------------------
    // OPERATORS:
    //--------------------------------------------------------------------------

public:

    //! An overloaded <b> /= </b> operator for vector/scalar division.
    inline void operator/= (const double& a_val)
    {
        double factor = 1.0 / a_val;
        (*this)(0) *= factor;
        (*this)(1) *= factor;
        (*this)(2) *= factor;
    }

    //! An overloaded <b> *= </b> operator for vector/scalar multiplication.
    inline void operator*= (const double& a_val)
    {
        (*this)(0) *= a_val;
        (*this)(1) *= a_val;
        (*this)(2) *= a_val;
    }


    //! An overloaded <b> += </b> operator for vector/vector addition.
    inline void operator+= (const cVector3d& a_input)
    {
        (*this)(0) += a_input(0);
        (*this)(1) += a_input(1);
        (*this)(2) += a_input(2);
    }


    //! An overloaded <b> -= </b> operator for vector/vector subtraction.
    inline void operator-= (const cVector3d& a_input)
    {
        (*this)(0) -= a_input(0);
        (*this)(1) -= a_input(1);
        (*this)(2) -= a_input(2);
    }


    //! An overloaded <b> = </b> operator.
    inline void operator= (const cVector3d& a_input)
    {
        (*this)(0) = a_input(0);
        (*this)(1) = a_input(1);
        (*this)(2) = a_input(2);  
    }


    //! An overloaded <b> () </b> operator.
    inline double& operator() (const int a_index)
    {
        return m_data[a_index];  
    }


    //! An overloaded <b> () </b> operator.
    inline const double& operator() (const int a_index) const
    {
        return m_data[a_index];  
    }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Initialize vector with components __x__, __y__, and __z__ passed as arguments.
    inline void set(const double& a_x, const double& a_y, const double& a_z)
    {
        (*this)(0) = a_x;
        (*this)(1) = a_y;
        (*this)(2) = a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Initialize vector from input __ANSI spring__.

        \details
        Initialize vector from an input string of the form <b> (x,y,z) </b>. The function 
        accepts any of the following formats:\n

        - (4.3,23,54) \n
        - 4.3 54 2.1  \n
        - 4.5,7.8,9.1 \n

        The methods expects three numbers, optionally preceded by '(' and 
        whitespace, and separated by commas or whitespace.

        \param      a_initStr The string to convert

        \return     __true__ if conversion was successful, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool set(const char* a_initStr)
    {
        // sanity check
        if (a_initStr == 0) return (false);

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


    //--------------------------------------------------------------------------
    /*!
        \brief
        Initialize vector from input __spring__.

        \details
        Initialize vector from an input string of the form <b> (x,y,z) </b>. The function 
        accepts any of the following formats:\n

        - (4.3,23,54) \n
        - 4.3 54 2.1  \n
        - 4.5,7.8,9.1 \n

        The methods expects three numbers, optionally preceded by '(' and 
        whitespace, and separated by commas or whitespace.

        \param      a_initStr The string to convert

        \return     __true__ if conversion was successful, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool set(const std::string& a_initStr)
    {
        return ( set(a_initStr.c_str()) );
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Copy: <em> a_destination = this </em>

        \details
        Copy components (__x__, __y__, __z__), of current vector to a vector 
        passed as argument.

        \param      a_destination  Destination vector where data is copied.
    */
    //--------------------------------------------------------------------------
    inline void copyto(cVector3d& a_destination) const
    {
        a_destination(0) = (*this)(0);
        a_destination(1) = (*this)(1);
        a_destination(2) = (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Copy: <em> this = a_source </em>

        \details
        Copy components (__x__, __y__, __z__) of a vector passed as argument 
        to current one.

        \param      a_source  Source vector from where data is copied.
    */
    //--------------------------------------------------------------------------
    inline void copyfrom(const cVector3d &a_source)
    {
        (*this)(0) = a_source(0);
        (*this)(1) = a_source(1);
        (*this)(2) = a_source(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Addition: <em> this = this + a_vector </em>
    
        \details
        Addition between current vector and external vector passed as
        argument. \n
        Result is stored in current vector. \n

        \param      a_vector  Vector to be added to current one.
    */
    //--------------------------------------------------------------------------
    inline void add(const cVector3d& a_vector)
    {
        (*this)(0) = (*this)(0) + a_vector(0);
        (*this)(1) = (*this)(1) + a_vector(1);
        (*this)(2) = (*this)(2) + a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Addition: <em> this = this + cVector3d(a_x, a_y, a_z) </em>
    
        \details
        Addition between current vector and external vector passed as
        argument. \n
        Result is stored in current vector.

        \param      a_x  __x__ component.
        \param      a_y  __y__ component.
        \param      a_z  __z__ component.
    */
    //--------------------------------------------------------------------------
    inline void add(const double& a_x, 
                    const double& a_y, 
                    const double& a_z)
    {
        (*this)(0) = (*this)(0) + a_x;
        (*this)(1) = (*this)(1) + a_y;
        (*this)(2) = (*this)(2) + a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Addition: <em> a_result = this + a_vector </em>
    
        \details
        Addition between current vector and external vector passed as
        argument. \n
        Result is stored in output _a_result_ vector passed as second argument.

        \param      a_vector  Vector to be added to current one.
        \param      a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void addr(const cVector3d& a_vector, 
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0) + a_vector(0);
        a_result(1)  = (*this)(1) + a_vector(1);
        a_result(2)  = (*this)(2) + a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Addition: <em> a_result = this + cVector3d(a_x, a_y, a_z) </em>
    
        \details
        Addition between current vector and external vector passed as
        argument. \n
        Result is stored in output _a_result_ vector passed as last argument.


        \param      a_x  __x__ component.
        \param      a_y  __y__ component.
        \param      a_z  __z__ component.
        \param      a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void addr(const double& a_x, 
                     const double& a_y, 
                     const double& a_z, 
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0) + a_x;
        a_result(1)  = (*this)(1) + a_y;
        a_result(2)  = (*this)(2) + a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Subtraction: <em> this = this - a_vector </em>
    
        \details
        Subtraction between current vector and external vector passed as
        argument. \n
        Result is stored in current vector.

        \param      a_vector  Vector to be subtracted from current one.
    */
    //--------------------------------------------------------------------------
    inline void sub(const cVector3d& a_vector)
    {
        (*this)(0) = (*this)(0) - a_vector(0);
        (*this)(1) = (*this)(1) - a_vector(1);
        (*this)(2) = (*this)(2) - a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Subtraction: <em> this = this - cVector3d(a_x, a_y, a_z) </em>

        \details
        Subtraction between current vector and external vector passed as
        argument. \n
        Result is stored in current vector.

        \param      a_x  __x__ component.
        \param      a_y  __y__ component.
        \param      a_z  __z__ component.
    */
    //--------------------------------------------------------------------------
    inline void sub(const double& a_x, 
                    const double& a_y, 
                    const double& a_z)
    {
        (*this)(0) = (*this)(0) - a_x;
        (*this)(1) = (*this)(1) - a_y;
        (*this)(2) = (*this)(2) - a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Subtraction: <em> a_result = this - a_vector </em>

        \details
        Subtraction between current vector and external vector passed as
        argument. \n
        Result is stored in output _a_result_ vector passed as second argument.

        \param      a_vector  Vector to be subtracted from current one.
        \param      a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void subr(const cVector3d& a_vector, 
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0) - a_vector(0);
        a_result(1)  = (*this)(1) - a_vector(1);
        a_result(2)  = (*this)(2) - a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Subtraction: <em> a_result = this - cVector3d(a_x, a_y, a_z) </em>
    
        \details
        Subtraction between current vector and external vector passed as
        argument. \n
        Result is stored in output _a_vector_ passed as last argument.

        \param      a_x  __x__ component.
        \param      a_y  __y__ component.
        \param      a_z  __z__ component.
        \param      a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void subr(const double& a_x, 
                     const double& a_y, 
                     const double& a_z,
                     cVector3d &a_result) const
    {
        a_result(0)  = (*this)(0) - a_x;
        a_result(1)  = (*this)(1) - a_y;
        a_result(2)  = (*this)(2) - a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Multiplication: <em> this = a_scalar * this </em>

        \details
        Multiply current vector by a scalar. \n
        Result is stored in current vector.

        \param      a_scalar  Scalar value.
    */
    //--------------------------------------------------------------------------
    inline void mul(const double &a_scalar)
    {
        (*this)(0) = a_scalar * (*this)(0);
        (*this)(1) = a_scalar * (*this)(1);
        (*this)(2) = a_scalar * (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Multiplication: <em> this = scalars * this </em>

        \details
        Multiply each component of vector by a different scalar. \n
        Result is stored in current vector.

        \param      a_scalar0  Scalar value for component 0.
        \param      a_scalar1  Scalar value for component 1.
        \param      a_scalar2  Scalar value for component 2.
    */
    //--------------------------------------------------------------------------
    inline void mul(const double &a_scalar0, 
                    const double &a_scalar1, 
                    const double &a_scalar2)
    {
        (*this)(0) = a_scalar0 * (*this)(0);
        (*this)(1) = a_scalar1 * (*this)(1);
        (*this)(2) = a_scalar2 * (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Multiplication: <em> a_result = a_scalar * this </em>

        \details
        Multiply current vector by a scalar. \n
        Result is stored in output _a_result_ vector passed as second argument.

        \param      a_scalar  Scalar value.
        \param      a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void mulr(const double& a_scalar, 
                     cVector3d& a_result) const
    {
        a_result(0)  = a_scalar * (*this)(0);
        a_result(1)  = a_scalar * (*this)(1);
        a_result(2)  = a_scalar * (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Multiplication: <em> a_result = scalars * this </em>

        \details
        Multiply each component of vector by a different scalar. \n
        Result is stored in output _a_result_ vector passed as second argument.

        \param      a_scalar0  Scalar value for component 0.
        \param      a_scalar1  Scalar value for component 1.
        \param      a_scalar2  Scalar value for component 2.
        \param      a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void mulr(const double &a_scalar0, 
                     const double &a_scalar1, 
                     const double &a_scalar2,
                     cVector3d& a_result) const
    {
        a_result(0)  = a_scalar0 * (*this)(0);
        a_result(1)  = a_scalar1 * (*this)(1);
        a_result(2)  = a_scalar2 * (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Division: <em> this = (1.0 / a_scalar) * this </em>

        \details
        Divide current vector by a scalar. \n
        Result is stored in current vector.

        \param      a_scalar  Scalar value.
    */
    //--------------------------------------------------------------------------
    inline void div(const double& a_scalar)
    {
        double factor = 1.0 / a_scalar;
        (*this)(0) = (*this)(0) * factor;
        (*this)(1) = (*this)(1) * factor;
        (*this)(2) = (*this)(2) * factor;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Division: <em> a_result = (1.0 / a_scalar) * this </em>

        \details
        Divide current vector by a scalar. \n
        Result is stored in output _a_result_ vector passed as second argument.

        \param      a_scalar  Scalar value.
        \param      a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void divr(const double& a_scalar, 
                     cVector3d& a_result) const
    {
        double factor = 1.0 / a_scalar;
        a_result(0)  = (*this)(0) * factor;
        a_result(1)  = (*this)(1) * factor;
        a_result(2)  = (*this)(2) * factor;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Opposite: <em> this = -this </em>

        \details
        Negate current vector. \n
        Result is stored in current vector.
    */
    //--------------------------------------------------------------------------
    inline void negate()
    {
        (*this)(0) = -(*this)(0);
        (*this)(1) = -(*this)(1);
        (*this)(2) = -(*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Opposite: <em> a_result = -this </em>

        \details
        Negate current vector. \n
        Result is stored in output _a_result_ vector passed as second argument.

        \param      a_result  Result vector.
    */
    //--------------------------------------------------------------------------
    inline void negater(cVector3d& a_result) const
    {
        a_result(0)  = -(*this)(0);
        a_result(1)  = -(*this)(1);
        a_result(2)  = -(*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Cross product: <em> this = this X a_vector </em>

        \details
        Compute the cross product between current vector and external
        vector passed as argument. \n 
        Result is stored in current vector.

        \param      a_vector  Input vector.
    */
    //--------------------------------------------------------------------------
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


    //--------------------------------------------------------------------------
    /*!
        \brief
        Cross product: <em> a_result = this X a_vector </em>

        \details
        Compute the cross product between current vector and external vector 
        passed as argument. \n
        Result is stored in output _a_result_ vector passed as second argument.

        \param      a_vector  Input vector.
        \param      a_result  Resulting cross product.
    */
    //--------------------------------------------------------------------------
    inline void crossr(const cVector3d& a_vector, 
                       cVector3d& a_result) const
    {
        a_result(0)  =  ((*this)(1) * a_vector(2) ) - ((*this)(2) * a_vector(1) );
        a_result(1)  = -((*this)(0) * a_vector(2) ) + ((*this)(2) * a_vector(0) );
        a_result(2)  =  ((*this)(0) * a_vector(1) ) - ((*this)(1) * a_vector(0) );
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Cross product: <em> this X a_vector </em>

        \details
        Return the cross product between current vector and an external vector 
        passed as argument. \n

        Result is returned.  Performance-wise, cross() and crossr() are usually
        preferred, since this version creates a new stack variable.

        \param      a_vector  Input vector.

        \return     Resulting cross product.
    */
    //--------------------------------------------------------------------------
    inline cVector3d crossAndReturn(const cVector3d& a_vector) const
    {
        cVector3d result;
        crossr(a_vector, result);
        return (result);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Dot product: <em> this . a_vector </em>

        \details
        Compute the dot product between current vector and external vector
        passed as argument. \n

        \param      a_vector  Input vector.

        \return     Dot product between both vectors.
    */
    //--------------------------------------------------------------------------
    inline double dot(const cVector3d& a_vector) const
    {
        return(((*this)(0) * a_vector(0) ) + ((*this)(1) * a_vector(1) ) + ((*this)(2) * a_vector(2) ));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Element multiplication

        \details
        Compute the element-by-element product between current vector and an external
        vector. \n 
        Result is stored in current vector.\n
        
        \param  a_vector  Input vector.
    */
    //--------------------------------------------------------------------------
    inline void elementMul(const cVector3d& a_vector)
    {
        (*this)(0)*=a_vector(0) ;
        (*this)(1)*=a_vector(1) ;
        (*this)(2)*=a_vector(2) ;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Element multiplication

        \details
        Compute the element-by-element product between current vector and an external
        vector. \n
        Result is stored in output _a_result_ vector passed as second argument.

        \param  a_vector  Input vector.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void elementMulr(const cVector3d& a_vector, 
                            cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0)*a_vector(0) ;
        a_result(1)  = (*this)(1)*a_vector(1) ;
        a_result(2)  = (*this)(2)*a_vector(2) ;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Euclidean norm: <em> |this| </em>

        \details
        Return the Euclidean norm of current vector.

        \return     Euclidean norm of current vector.
    */
    //--------------------------------------------------------------------------
    inline double length() const
    {
        return(sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2))));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Square of Euclidean norm: <em> |this|^2 </em>

        \details
        Compute the square of the Euclidean norm of current vector.

        \return     Square of Euclidean norm of current vector.
    */
    //--------------------------------------------------------------------------
    inline double lengthsq() const
    {
        return(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Normalization: <em> this = (1.0 / |this|) * this </em>

        \details
        Normalize current vector length one.\n \n

        __WARNING:__ \n
        Vector should not be equal to (0,0,0) or a division by zero error 
        will occur. \n Result is stored in current vector.
    */
    //--------------------------------------------------------------------------
    inline void normalize()
    {
        // compute length of vector
        double len = sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
        if (len == 0.0) { return; }
        double factor = 1.0 / len;

        // divide current vector by its length
        (*this)(0) = (*this)(0) * factor;
        (*this)(1) = (*this)(1) * factor;
        (*this)(2) = (*this)(2) * factor;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Normalization: <em> a_result = (1.0 / |this|) * this </em>

        \details
        Normalize current vector to unit length.\n
        Result is stored in output _a_result_ vector passed as second argument.

        __WARNING:__ \n
        Vector should not be equal to (0,0,0) or a division by zero error 
        will occur.
    */
    //--------------------------------------------------------------------------
    inline void normalizer(cVector3d& a_result) const
    {
        // compute length of vector
        double len = sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
        double factor = 1.0 / len;

        // divide current vector by its length
        a_result(0)  = (*this)(0) * factor;
        a_result(1)  = (*this)(1) * factor;
        a_result(2)  = (*this)(2) * factor;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Clamp current vector to a maximum desired length.

        \details
        Clamp current vector to a maximum desired length. Vectors that are shorter
        than _a_maxLength_ are not affected.

        \param      a_maxLength  Maximum length value.
    */
    //--------------------------------------------------------------------------
    inline void clamp(const double& a_maxLength)
    {
        double len = sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
        if (a_maxLength == 0)
        {
            (*this)(0) = 0.0;
            (*this)(1) = 0.0;
            (*this)(2) = 0.0;
        }
        else if (len > a_maxLength)
        {
            double factor = a_maxLength / len;
            (*this)(0) = (*this)(0) * factor;
            (*this)(1) = (*this)(1) * factor;
            (*this)(2) = (*this)(2) * factor;              
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Distance between two points.

        \details
        Return the distance between current point and external point
        passed as argument.

        \param      a_vector  Input point.

        \return     Distance between two points.
    */
    //--------------------------------------------------------------------------
    inline double distance(const cVector3d& a_vector) const
    {
        // compute distance between both points
        double dx = (*this)(0) - a_vector(0) ;
        double dy = (*this)(1) - a_vector(1) ;
        double dz = (*this)(2) - a_vector(2) ;

        // return result
        return(sqrt(dx*dx + dy*dy + dz*dz));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Square of distance between two points.

        \details
        Return the square distance between current point and external
        point passed as argument.

        \param      a_vector  Input point.

        \return     Square of distance between the points
    */
    //--------------------------------------------------------------------------
    inline double distancesq(const cVector3d& a_vector) const
    {
        // compute distance between both points
        double dx = (*this)(0) - a_vector(0) ;
        double dy = (*this)(1) - a_vector(1) ;
        double dz = (*this)(2) - a_vector(2) ;

        // return result
        return(dx*dx + dy*dy + dz*dz);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Equality test.

        \details
        Test whether the current vector and external vector passed as 
        argument are both equal. Two vectors are considered equal if each
        of their components are distant within an distance _epsilon_ defined
        by the second argument. By default, _epsilon_ is set to zero.

        \param      a_vector Input vector.
        \param      a_epsilon  Tolerance error.

        \return     __true__ if both vectors are equal, otherwise __false__.
    */
    //--------------------------------------------------------------------------
    inline bool equals(const cVector3d& a_vector, 
                       const double a_epsilon = 0.0) const
    {
        // Accelerated path for exact equality
        if (a_epsilon == 0.0)
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

        if ((fabs(a_vector(0) - (*this)(0)) < a_epsilon) &&
            (fabs(a_vector(1) - (*this)(1)) < a_epsilon) &&
            (fabs(a_vector(2) - (*this)(2)) < a_epsilon))
        {
            return (true);
        }
        else
        {
            return (false);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        String conversion.

        \details
        Convert current vector into a string.

        \param      a_precision    Number of digits.

        \return     Output string.
    */
    //--------------------------------------------------------------------------
    inline std::string str(const unsigned int a_precision = 2) const
    {
        std::string result;
        result = (cStr((*this)(0), a_precision) + ", " +
                  cStr((*this)(1), a_precision) + ", " +
                  cStr((*this)(2), a_precision));
        return (result);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Print string.

        \details
        Print the current vector using the __cPrint__ macro.

        \param      a_precision  Number of digits.
    */
    //--------------------------------------------------------------------------
    inline void print(const unsigned int a_precision = 2) const
    {
        std::string s = str(a_precision);
        cPrint("%s\n", s.c_str());
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Decomposition of current vector into two orthogonal vectors.

        \details
        Decompose current vector into two orthogonal vectors. The first output vector is parallel to input
        vector, and the second output vector is orthogonal. \n
        <em> this =  a_parallel + a_perpendicular </em>

        \param      a_input  Reference vector.
        \param      a_parallel  Output vector parallel to _a_input_.
        \param      a_orthogonal  Output vector perpendicular to _a_input_.
    */
    //--------------------------------------------------------------------------
    inline void decompose(const cVector3d& a_input, 
                          cVector3d& a_parallel, 
                          cVector3d& a_orthogonal) const
    {
        double scale = (this->dot(a_input) / (a_input.dot(a_input)));
        a_parallel = a_input;
        a_parallel.mul(scale);
        this->subr(a_parallel, a_orthogonal);
    }



    //--------------------------------------------------------------------------
    // PRIVATE MEMBERS
    //--------------------------------------------------------------------------
    
private:

    //! Vector data.
    double m_data[3];
};


//==============================================================================
// OPERATORS ON CVECTOR3D:
//==============================================================================


//! An overloaded <b> * </b> operator for vector/scalar multiplication.
inline cVector3d operator*(const cVector3d& a_vector, const double a_scale)
{
    return (cVector3d(a_vector(0) * a_scale,
                      a_vector(1) * a_scale,
                      a_vector(2) * a_scale));
}


//! An overloaded <b> * </b> operator for scalar/vector multiplication.
inline cVector3d operator*(const double a_scale, const cVector3d& a_vector)
{
    return (cVector3d(a_vector(0) * a_scale,
                      a_vector(1) * a_scale,
                      a_vector(2) * a_scale));
}


//! An overloaded <b> / </b> operator for vector/scalar division.
inline cVector3d operator/(const cVector3d& a_vector, const double a_scale)
{
    return (cVector3d(a_vector(0) / a_scale,
                      a_vector(1) / a_scale,
                      a_vector(2) / a_scale));
}


//! An overloaded <b> + </b> operator for vector/vector addition.
inline cVector3d operator+(const cVector3d& a_vector0, const cVector3d& a_vector1)
{
    return cVector3d(a_vector0(0) + a_vector1(0), 
                     a_vector0(1) + a_vector1(1), 
                     a_vector0(2) + a_vector1(2));
}


//! An overloaded <b> - </b> operator for vector/vector subtraction.
inline cVector3d operator-(const cVector3d& a_vector0, const cVector3d& a_vector1)
{
    return cVector3d(a_vector0(0) - a_vector1(0), 
                     a_vector0(1) - a_vector1(1), 
                     a_vector0(2) - a_vector1(2));
}


//! An overloaded <b> - </b> operator for vector negation.
inline cVector3d operator-(const cVector3d& a_vector0)
{
    return cVector3d(-a_vector0(0), 
                     -a_vector0(1), 
                     -a_vector0(2));
}


//! An overloaded <b> * </b> operator for vector/vector dotting.
inline double operator*(const cVector3d& a_vector0, const cVector3d& a_vector1)
{
    return (a_vector0(0) * a_vector1(0) + 
            a_vector0(1) * a_vector1(1) + 
            a_vector0(2) * a_vector1(2));
}


#ifdef C_USE_EIGEN

//! An overloaded = operator for Eigen vectors.
inline Eigen::Vector3d operator-(const cVector3d& a_vector)
{
    return (Eigen::Vector3d(a_vector(0), a_vector(1), a_vector(2)));
}

#endif


//! <b> ostream <b> operator. Outputs the vector's components separated by commas.
static inline std::ostream &operator << (std::ostream &a_os, cVector3d const& a_vector)
{
    a_os << a_vector(0)  << ", " << a_vector(1)  << ", " << a_vector(2) ;
    return (a_os);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
