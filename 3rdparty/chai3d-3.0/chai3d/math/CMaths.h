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
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1052 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CMathsH
#define CMathsH
//------------------------------------------------------------------------------
#include "math/CTransform.h"
#include <math.h>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMaths.h
    \ingroup    math

    \brief      
    <b> Math </b> \n 
    General Utility Functions.
*/
//==============================================================================

//==============================================================================
/*!
    Check if \e value is equal or near zero.

    \param    a_value  Value to be checked.

    \return   Returns __true__ if it's almost zero, otherwise __false__.
*/
//==============================================================================
inline bool cZero(const double& a_value)
{
    return ((a_value < C_TINY) && (a_value > -C_TINY));
}


//==============================================================================
/*!
    Check if \e value is strictly positive and less than \e maxBound in case
    \e maxBound is positive.

    \param    a_value  Value to be checked.
    \param    a_boundMax  Positive bound.

    \return   Return __true__ if \e value is greater than 0 and either
              \e maxBound is negative, or \e value is less than\e maxBound.
              Otherwise return __false__.
*/
//==============================================================================
inline bool cPositiveBound(const double& a_value, 
                           const double& a_boundMax)
{
    return ((a_value > C_TINY) && ((a_boundMax < 0) || (a_value < a_boundMax)));
}


//==============================================================================
/*!
    Compute absolute value.

    \param    a_value  Input value.

    \return   Return the absolute value of \e value.
*/
//==============================================================================
template<class T> inline T cAbs(const T& a_value)
{
    return (a_value >= 0 ? a_value : -a_value);
}


//==============================================================================
/*!
    Compute the sign of a value.

    \param    a_value  Input value.

    \return   Return 1 is the value is zero or positive, return -1 otherwise.
*/
//==============================================================================
template<class T> inline double cSign(const T& a_value)
{
    if (a_value < 0) 
    {
        return (-1);
    }
    else
    {	
        return (1);
    }
}


//==============================================================================
/*!
    Compute maximum between two values.

    \param    a_value1  First value.
    \param    a_value2  Second value.

    \return   Return maximum of a_value1 and a_value2.

    Note that this function should \e not be modified to take inputs by
    reference.
*/
//==============================================================================
template<class T> inline T cMax(const T& a_value1, 
                                const T& a_value2)
{
    return (a_value1 >= a_value2 ? a_value1 : a_value2);
}


//==============================================================================
/*!
    Compute minimum between two values.

    \param    a_value1  First value.
    \param    a_value2  Second value.

    \return   Return minimum of a_value1 and a_value2.

    Note that this function should \e not be modified to take inputs by
    reference.
*/
//==============================================================================
template<class T> inline T cMin(const T& a_value1, 
                                const T& a_value2)
{
    return (a_value1 <= a_value2 ? a_value1 : a_value2);
}


//==============================================================================
/*!
    Return maximum of 3 values.

    \param      a_value1  First value.
    \param      a_value2  Second value.
    \param      a_value3  Third value.

    \return     Return maximum of a_value1, a_value2 and a_value3.
*/
//==============================================================================
template<class T> inline T cMax3(const T& a_value1, 
                                 const T& a_value2, 
                                 const T& a_value3)
{
    return (cMax(a_value1, cMax(a_value2, a_value3)));
}


//==============================================================================
/*!
    Return minimum of 3 values.

    \param    a_value1  First value.
    \param    a_value2  Second value.
    \param    a_value3  Third value.

    \return   Return minimum of a_value1, a_value2 and a_value3.
*/
//==============================================================================
template<class T> inline T cMin3(const T& a_value1, 
                                 const T& a_value2, 
                                 const T& a_value3)
{
    return (cMin(a_value1, cMin(a_value2, a_value3)));
}


//==============================================================================
/*!
    Compute maximum of absolute values of 2 numbers.

    \param    a_value1  First value.
    \param    a_value2  Second value.

    \return   Return max(abs(p), abs(q)).

    Note that this function should \e not be modified to take inputs by
    reference.
*/
//==============================================================================
template<class T> inline T cMaxAbs(const T& a_value1, 
                                   const T& a_value2)
{
    return (cAbs(a_value1) >= cAbs(a_value2) ? a_value1 : a_value2);
}


//==============================================================================
/*!
    Compute minimum of absolute values of 2 values.

    \param    a_value1  First value.
    \param    a_value2  Second value.

    \return   Return min(abs(p), abs(q)).

    Note that this function should \e not be modified to take inputs by
    reference.
*/
//==============================================================================
template<class T> inline T cMinAbs(const T& a_value1, 
                                   const T& a_value2)
{
    return (cAbs(a_value1) <= cAbs(a_value2) ? a_value1 : a_value2);
}


//==============================================================================
/*!
    Compute maximum of absolute values of 3 values.

    \param    a_value1  First value.
    \param    a_value2  Second value.
    \param    a_value3  Third value.

    \return   Return max(abs(p), abs(q), abs(r)).
*/
//==============================================================================
template<class T> inline T cMax3Abs(const T& a_value1, 
                                    const T& a_value2, 
                                    const T& a_value3)
{
    return cMaxAbs(a_value1, cMaxAbs(a_value2, a_value3));
}


//==============================================================================
/*!
    Compute minimum of absolute values of 3 values.

    \param    a_value1  First value.
    \param    a_value2  Second value.
    \param    a_value3  Third value.

    \return   Return min(abs(p), abs(q), abs(r)).
*/
//==============================================================================
template<class T> inline T cMin3Abs(const T& a_value1, 
                                    const T& a_value2, 
                                    const T& a_value3)
{
    return cMinAbs(a_value1, cMinAbs(a_value2, a_value3));
}


//==============================================================================
/*!
    Swap two elements.

    \param    a_value1  First value.
    \param    a_value2  Second value.
*/
//==============================================================================
template<class T> inline void cSwap(T& a_value1, 
                                    T& a_value2)
{
    T value = a_value1;
    a_value1 = a_value2;
    a_value2 = value;
}


//==============================================================================
/*!
    Linear interpolation from \e value0 (when a=0) to \e value1 (when a=1).

    \param    a_level  Interpolation parameter.
    \param    a_value1  First value.
    \param    a_value2  Second value.

    \return   Return an interpolated result: (1-a_level) * a_value1 + a_level * a_value2
*/
//==============================================================================
template<class T> inline T cLerp(const double& a_level, 
                                 const T& a_value1, 
                                 const T& a_value2)
{
    return (a_value2 * a_level + a_value1 * (1 - a_level));
}


//==============================================================================
/*!
    Clamp the input to the specified range.

    \param    a_value  Input value.
    \param    a_low  Low boundary.
    \param    a_high  High boundary.

    \return   Return clamped value.

    Note that this function should \e not be modified to take inputs by
    reference.
*/
//==============================================================================
template<class T> inline T cClamp(const T& a_value, 
                                  const T& a_low, 
                                  const T& a_high)
{
    return (a_value < a_low ? a_low : a_value > a_high ? a_high : a_value);
}


//==============================================================================
/*!
    Clamp the input to the range 0 - \e infinity.

    \param    a_value  Input value.

    \return   Return clamped value.
*/
//==============================================================================
template<class T> inline T cClamp0(const T& a_value)
{
    return cMax<T>(0, a_value);
}


//==============================================================================
/*!
    Clamp the input to the range [0,1].

    \param    a_value  Input value of type double.

    \return   Return clamped value.
*/
//==============================================================================
inline double cClamp01(const double& a_value)
{
    return (cClamp(a_value, 0.0, 1.0));
}


//==============================================================================
/*!
    Check whether \e value is in the range of [low, high].

    \param    a_value  Input value.
    \param    a_low  Low boundary.
    \param    a_high  High boundary.

    \return   Return __true__ if \e value is in the rage of [low, high]
                and __false__ otherwise.
*/
//==============================================================================
template<class T, class V> inline bool cContains(const T& a_value, 
                                                 const V& a_low, 
                                                 const V& a_high)
{
    return ((a_value >= a_low) && (a_value <= a_high));
}


//==============================================================================
/*!
    Compute the square of a scalar.

    \param    a_value  Input value.

    \return   Return (/e value * /e value).
*/
//==============================================================================
inline double cSqr(const double& a_value)
{
    return(a_value*a_value);
}


//==============================================================================
/*!
    Compute the cosine of an angle defined in degrees.

    \param    a_angleDeg  Angle in degrees.

    \return   Return the cosine of angle.
*/
//==============================================================================
inline double cCosDeg(const double& a_angleDeg)
{
    return (cos(a_angleDeg * C_DEG2RAD));
}


//==============================================================================
/*!
    Compute the sine of an angle defined in degrees.

    \param    a_angleDeg  Angle in degrees.

    \return   Return the sine of angle.
*/
//==============================================================================
inline double cSinDeg(const double& a_angleDeg)
{
    return (sin(a_angleDeg * C_DEG2RAD));
}


//==============================================================================
/*!
    Compute the tangent of an angle defined in degrees.

    \param    a_angleDeg  Angle in degrees.

    \return   Return the tangent of angle.
*/
//==============================================================================
inline double cTanDeg(const double& a_angleDeg)
{
    return (tan(a_angleDeg * C_DEG2RAD));
}


//==============================================================================
/*!
    Return the cosine of an angle defined in radians.

    \param    a_angleRad  Angle in radians.

    \return   Return the cosine of angle.
*/
//==============================================================================
inline double cCosRad(const double& a_angleRad)
{
    return (cos(a_angleRad));
}


//==============================================================================
/*!
    Return the sine of an angle defined in radians.

    \param    a_value Angle in radians.

    \return   Return the sine of angle a_value.
*/
//==============================================================================
inline double cSinRad(const double& a_value)
{
    return (sin(a_value));
}


//==============================================================================
/*!
    Return the tangent of an angle defined in radians.

    \param    a_value  Angle in radians.

    \return   Return the tangent of angle a_value.
*/
//==============================================================================
inline double cTanRad(const double& a_value)
{
    return (tan(a_value));
}


//==============================================================================
/*!
    Convert an angle from degrees to radians.

    \param    a_angleDeg  Angle in degrees.

    \return   Return angle in radians.
*/
//==============================================================================
inline double cDegToRad(const double& a_angleDeg)
{
    return (a_angleDeg * C_DEG2RAD);
}


//==============================================================================
/*!
    Convert an angle from radians to degrees

    \param    a_angleRad  Angle in radians.

    \return   Return angle in degrees.
*/
//==============================================================================
inline double cRadToDeg(const double& a_angleRad)
{
    return (a_angleRad * C_RAD2DEG);
}


//==============================================================================
/*!
    Compute the addition between two vectors. \n
    \e Result = \e Vector1 + \e Vector2

    \param    a_vector1  First vector.
    \param    a_vector2  Second vector.

    \return   Return the addition of both vectors.
*/
//==============================================================================
inline cVector3d cAdd(const cVector3d& a_vector1, 
                      const cVector3d& a_vector2)
{
    return cVector3d(
      a_vector1(0)+a_vector2(0),
      a_vector1(1)+a_vector2(1),
      a_vector1(2)+a_vector2(2));
}


//==============================================================================
/*!
    Compute the addition between three vectors. \n
    \e Result = \e Vector1 + \e Vector2 + \e Vector3

    \param    a_vector1  First vector.
    \param    a_vector2  Second vector.
    \param    a_vector3  Third vector.

    \return   Return the addition of all three vectors.
*/
//==============================================================================
inline cVector3d cAdd(const cVector3d& a_vector1, 
                      const cVector3d& a_vector2, 
                      const cVector3d& a_vector3)
{
    return cVector3d(
      a_vector1(0) +a_vector2(0) +a_vector3(0) ,
      a_vector1(1) +a_vector2(1) +a_vector3(1) ,
      a_vector1(2) +a_vector2(2) +a_vector3(2) );
}


//==============================================================================
/*!
    Compute the subtraction between two vectors. \n
    \e Result = \e Vector1 - \e Vector2

    \param    a_vector1  First vector.
    \param    a_vector2  Second vector.

    \return   Return result of subtraction.
*/
//==============================================================================
inline cVector3d cSub(const cVector3d& a_vector1, 
                      const cVector3d& a_vector2)
{
  return cVector3d(
    a_vector1(0) -a_vector2(0) ,
    a_vector1(1) -a_vector2(1) ,
    a_vector1(2) -a_vector2(2) );
}


//==============================================================================
/*!
    Compute the negated vector of a input vector.

    \param    a_vector  Input vector.

    \return   Return (-a_vector).
*/
//==============================================================================
inline cVector3d cNegate(const cVector3d& a_vector)
{
    return cVector3d(a_vector(0) *-1.0,
        a_vector(1) *-1.0,
        a_vector(2) *-1.0);
}


//==============================================================================
/*!
    Multiply a vector by a scalar.

    \param    a_value   Scalar.
    \param    a_vector  Input vector.

    \return   Returns result of multiplication.
*/
//==============================================================================
inline cVector3d cMul(const double& a_value, 
                      const cVector3d& a_vector)
{
    return cVector3d(a_vector(0) *a_value,
        a_vector(1) *a_value,
        a_vector(2) *a_value);
}


//==============================================================================
/*!
    Divide a vector by a scalar.

    \param    a_value   Scalar.
    \param    a_vector  Input vector.

    \return   Returns result of division.
*/
//==============================================================================
inline cVector3d cDiv(const double& a_value, 
                      const cVector3d& a_vector)
{
    return cVector3d(a_vector(0) /a_value,
        a_vector(1) /a_value,
        a_vector(2) /a_value);
}


//==============================================================================
/*!
    Divide a scalar by components of a 3D vector and return vector

    \param    a_value   Scalar.
    \param    a_vector  Input vector.

    \return   Returns a vector. \e result = \e value / \e vector.
*/
//==============================================================================
inline cVector3d cDivVect(const double& a_value, 
                          const cVector3d& a_vector)
{
    return cVector3d( a_value / a_vector(0),
        a_value / a_vector(1),
        a_value / a_vector(2));
}


//==============================================================================
/*!
    Compute the cross product between two 3D vectors.

    \param    a_vector1  Vector1.
    \param    a_vector2  Vector2.

    \return   Returns the cross product between \e Vector1 and \e Vector2.
*/
//==============================================================================
inline cVector3d cCross(const cVector3d& a_vector1, 
                        const cVector3d& a_vector2)
{
    cVector3d result;
    a_vector1.crossr(a_vector2, result);
    return (result);
}


//==============================================================================
/*!
    Compute the dot product between two vectors.

    \param    a_vector1  Vector1.
    \param    a_vector2  Vector2.

    \return   Returns the dot product between between both vectors.
*/
//==============================================================================
inline double cDot(const cVector3d& a_vector1, 
                   const cVector3d& a_vector2)
{
    return(a_vector1.dot(a_vector2));
}


//==============================================================================
/*!
    Compute the normalized vector (\e length = 1) of an input vector.

    \param    a_vector  Input vector.

    \return   Returns the normalized vector.
*/
//==============================================================================
inline cVector3d cNormalize(const cVector3d& a_vector)
{
    cVector3d result;
    a_vector.normalizer(result);
    return (result);
}


//==============================================================================
/*!
    Compute the distance between two points.

    \param    a_point1  First point.
    \param    a_point2  Second point.

    \return   Return distance between points
*/
//==============================================================================
inline double cDistance(const cVector3d& a_point1, 
                        const cVector3d& a_point2)
{
    return ( a_point1.distance(a_point2) );
}


//==============================================================================
/*!
    Compute the squared distance between two points.

    \param    a_point1  First point.
    \param    a_point2  Second point.

    \return   Return squared distance between points.
*/
//==============================================================================
inline double cDistanceSq(const cVector3d& a_point1, 
                          const cVector3d& a_point2)
{
    return ( a_point1.distancesq(a_point2) );
}


//==============================================================================
/*!
     Determine whether two vectors represent the same point.

     \fn       bool inline cEqualPoints(const cVector3d& v1,
                                        const cVector3d& v2,
                                        const double epsilon=C_SMALL)
     \param    v1  First point.
     \param    v2  Second point.
     \param    epsilon  Two points will be considered equal if each
                        component is within epsilon units.  Defaults
                        to \e C_SMALL.

     \return   Return whether the two vectors represent the same point.
*/
//==============================================================================
bool inline cEqualPoints(const cVector3d& v1,
                         const cVector3d& v2,
                         const double epsilon = C_SMALL)
{
    // Accelerated path for exact equality
    if (epsilon == 0.0) 
    {
        if ( (v1(0)  == v2(0) ) && (v1(1)  == v2(1) ) && (v1(2)  == v2(2) ) ) 
            return true;
        else 
            return false;
    }

    if ((fabs(v1(0) -v2(0) ) < epsilon) &&
        (fabs(v1(1) -v2(1) ) < epsilon) &&
        (fabs(v1(2) -v2(2) ) < epsilon)) 
        return true;
    else 
        return false;
}


//==============================================================================
/*!
    Return the Identity Matrix .

    \return   Return the identity matrix.
*/
//==============================================================================
inline cMatrix3d cIdentity3d(void)
{
    cMatrix3d result;
    result.identity();
    return (result);
}


//==============================================================================
/*!
    Build a rotation matrix from a set of Euler angles and co-moving 
    axes of rotations.

    \param      a_angleRad1  Angle in radians of the first rotation in the sequence.
    \param      a_angleRad2  Angle in radians of the second rotation in the sequence.
    \param      a_angleRad3  Angle in radians of the third rotation in the sequence.
    \param      a_eulerOrder  The order of the axes about which the rotations 
                are to be applied
    \param      a_useIntrinsicEulerModel  If __true__ use _intrinsic_ Euler 
                model, if __false__ use _extrinsic_ Euler model.
*/
//==============================================================================
inline cMatrix3d cRotEulerRad(const double& a_angleRad1,
    const double& a_angleRad2,
    const double& a_angleRad3,
    const cEulerOrder a_eulerOrder,
    const bool a_useIntrinsicEulerModel = true)
{
    // create matrix
    cMatrix3d rot;
    if (a_useIntrinsicEulerModel)
        rot.setIntrinsicEulerRotationRad(a_angleRad1, a_angleRad2, a_angleRad3, a_eulerOrder);
    else
        rot.setExtrinsicEulerRotationRad(a_angleRad1, a_angleRad2, a_angleRad3, a_eulerOrder);

    // return result
    return (rot);
}


//==============================================================================
/*!
    Build a rotation matrix from a set of Euler angles and co-moving 
    axes of rotations.

    \param      a_angleDeg1  Angle in degrees of the first rotation in the sequence.
    \param      a_angleDeg2  Angle in degrees of the second rotation in the sequence.
    \param      a_angleDeg3  Angle in degrees of the third rotation in the sequence.
    \param      a_eulerOrder  The order of the axes about which the rotations 
                are to be applied
    \param      a_useIntrinsicEulerModel  If __true__ use _intrinsic_ Euler 
                model, if __false__ use _extrinsic_ Euler model.
*/
//==============================================================================
inline cMatrix3d cRotEulerDeg(const double& a_angleDeg1,
    const double& a_angleDeg2,
    const double& a_angleDeg3,
    const cEulerOrder a_eulerOrder,
    const bool a_useIntrinsicEulerModel = true)
{
    // create matrix
    cMatrix3d rot;
    if (a_useIntrinsicEulerModel)
        rot.setExtrinsicEulerRotationDeg(a_angleDeg1, a_angleDeg2, a_angleDeg3, a_eulerOrder);
    else
        rot.setExtrinsicEulerRotationDeg(a_angleDeg1, a_angleDeg2, a_angleDeg3, a_eulerOrder);

    // return result
    return (rot);
}


//==============================================================================
/*!
    Build a rotation matrix from an __axis-angle__ representation.

    \param      a_axisX  __x__ component of axis.
    \param      a_axisY  __y__ component of axis.
    \param      a_axisZ  __z__ component of axis.
    \param      a_angleRad  Angle in radian.
*/
//==============================================================================
inline cMatrix3d cRotAxisAngleRad(const double& a_axisX,
    const double& a_axisY,
    const double& a_axisZ,
    const double& a_angleRad)
{
    // create matrix
    cMatrix3d rot;
    rot.setAxisAngleRotationRad(a_axisX,
                                a_axisY,
                                a_axisZ,
                                a_angleRad);

    // return result
    return (rot);
}


//==============================================================================
/*!
    Build a rotation matrix from an __axis-angle__ representation.

    \param      a_axisX  __x__ component of axis.
    \param      a_axisY  __y__ component of axis.
    \param      a_axisZ  __z__ component of axis.
    \param      a_angleDeg  Angle in degrees.
*/
//==============================================================================
inline cMatrix3d cRotAxisAngleDeg(const double& a_axisX,
    const double& a_axisY,
    const double& a_axisZ,
    const double& a_angleDeg)
{
    // create matrix
    cMatrix3d rot;
    rot.setAxisAngleRotationDeg(a_axisX,
                                a_axisY,
                                a_axisZ,
                                a_angleDeg);

    // return result
    return (rot);
}


//==============================================================================
/*!
    Compute the multiplication between two matrices.

    \param    a_matrix1  First matrix.
    \param    a_matrix2  Second matrix.

    \return   Returns multiplication of /e matrix1 * /e matrix2.
*/
//==============================================================================
inline cMatrix3d cMul(const cMatrix3d& a_matrix1, 
                      const cMatrix3d& a_matrix2)
{
    cMatrix3d result;
    a_matrix1.mulr(a_matrix2, result);
    return (result);
}


//==============================================================================
/*!
    Compute the multiplication of a matrix and a vector.

    \param    a_matrix  Input matrix.
    \param    a_vector  Input vector.

    \return   Returns the multiplication of the matrix and vector.
*/
//==============================================================================
inline cVector3d cMul(const cMatrix3d& a_matrix, 
                      const cVector3d& a_vector)
{
    cVector3d result;
    a_matrix.mulr(a_vector, result);
    return(result);
}


//==============================================================================
/*!
    Compute the transpose of a matrix

    \param    a_matrix  Input matrix.

    \return   Returns the transpose of the input matrix.
*/
//==============================================================================
inline cMatrix3d cTranspose(const cMatrix3d& a_matrix)
{
    cMatrix3d result;
    a_matrix.transr(result);
    return(result);
}


//==============================================================================
/*!
    Compute the inverse of a matrix.

    \param    a_matrix  Input matrix.

    \return   Returns the inverse of the input matrix.
*/
//==============================================================================
inline cMatrix3d cInverse(const cMatrix3d& a_matrix)
{
    cMatrix3d result;
    a_matrix.invertr(result);
    return(result);
}


//==============================================================================
/*!
    Compute the angle in radians between two vectors.

    \param    a_vector0  Input vector.
    \param    a_vector1  Input vector.

    \return   Returns the angle in radians between the input vectors.
*/
//==============================================================================
inline double cAngle(const cVector3d& a_vector0, 
                     const cVector3d& a_vector1)
{
    // compute length of vectors
    double n0 = a_vector0.length();
    double n1 = a_vector1.length();
    double val = n0 * n1;

    // check if lengths of vectors are not zero
    if (cAbs(val) < C_SMALL)
    {
        return (0.0);
    }

    // compute angle
    double result = a_vector0.dot(a_vector1)/(val);
    if (result > 1.0) { result = 1.0; }
    else if (result < -1.0) { result = -1.0; }

    return(acos(result));
}


//==============================================================================
/*!
    Compute the cosine of the angle between two vectors.

    \param    a_vector0  Input vector.
    \param    a_vector1  Input vector.

    \return   Returns the cosine of the angle between the input vectors.
*/
//==============================================================================
inline double cCosAngle(const cVector3d& a_vector0, 
                        const cVector3d& a_vector1)
{
    // compute length of vectors
    double n0 = a_vector0.length();
    double n1 = a_vector1.length();
    double val = n0 * n1;

    // check if lengths of vectors are not zero
    if (cAbs(val) < C_SMALL)
    {
        return (0.0);
    }

    // compute angle
    return(a_vector0.dot(a_vector1)/(val));
}


//==============================================================================
/*!
    Compute the projection of a point on a plane. the plane is expressed
    by a point and a surface normal.

    \param    a_point  Point to project on plane.
    \param    a_planePoint   Point on plane.
    \param    n              Plane normal.

    \return   Returns the projection of \e point on \e plane.
*/
//==============================================================================
inline cVector3d cProjectPointOnPlane(const cVector3d& a_point,
                                      const cVector3d& a_planePoint,
                                      const cVector3d& n)
{
    // compute a projection matrix
    cMatrix3d projectionMatrix;

    projectionMatrix.set(
       (n(1) * n(1)) + (n(2) * n(2)), -(n(0) * n(1) ),-(n(0) * n(2)),
      -(n(1) * n(0)), (n(0) * n(0)) + (n(2) * n(2) ),-(n(1) * n(2)),
      -(n(2) * n(0)),-(n(2) * n(1)), (n(0) * n(0) ) + (n(1) * n(1))
      );

    // project point on plane and return projected point.
    cVector3d point;
    a_point.subr(a_planePoint, point);
    projectionMatrix.mul(point);
    point.add(a_planePoint);

    // return result
    return (point);
}


//==============================================================================
/*!
    Compute the area of a triangle by passing the position of its three
    vertices.

    \param    a_vertex0  Point 0 of triangle.
    \param    a_vertex1  Point 1 of triangle.
    \param    a_vertex2  Point 2 of triangle.

    \return   Returns the area of the triangle.
*/
//==============================================================================
inline double cTriangleArea(const cVector3d& a_vertex0,
                            const cVector3d& a_vertex1,
                            const cVector3d& a_vertex2)
{
    cVector3d u, v;
    a_vertex1.subr(a_vertex0, u);
    a_vertex2.subr(a_vertex0, v);
    return (0.5 * (cCross(u,v).length()));
}


//==============================================================================
/*!
    Compute the projection of a point on a plane. the plane is expressed
    by a set of three points.

    \param    a_point  Point to project on plane.
    \param    a_planePoint0  Point 0 on plane.
    \param    a_planePoint1  Point 1 on plane.
    \param    a_planePoint2  Point 2 on plane.

    \return   Returns the projection of \e point on \e plane.
*/
//==============================================================================
inline cVector3d cProjectPointOnPlane(const cVector3d& a_point,
                                      const cVector3d& a_planePoint0, 
                                      const cVector3d& a_planePoint1,
                                      const cVector3d& a_planePoint2)
{
    // create two vectors from the three input points lying in the projection
    // plane.
    cVector3d v01, v02;
    a_planePoint1.subr(a_planePoint0, v01);
    a_planePoint2.subr(a_planePoint0, v02);

    // compute the normal vector of the plane
    cVector3d n;
    v01.crossr(v02, n);
    n.normalize();

    return (cProjectPointOnPlane(a_point,a_planePoint0,n));
}


//==============================================================================
/*!
    Projects a 3D point on a plane composed of three points. \n
    This function returns two parameters \e v01 and \e v02 which correspond 
    to the factors that expressed the following relation: 
    \e projectedPoint = \e planePoint0 + \e v01 * \e V01 + \e v02 * \e V02 \n
    where: \n
    \e V01 = \e planePoint1 - \e planePoint0, and \n
    \e V02 = \e planePoint2 - \e planePoint0

    \param    a_point  Point to project on plane.
    \param    a_planePoint0  Point 0 on plane.
    \param    a_planePoint1  Point 1 on plane.
    \param    a_planePoint2  Point 2 on plane.
    \param    a_v01  returned factor.
    \param    a_v02  returned factor.
*/
//==============================================================================
inline void cProjectPointOnPlane(const cVector3d& a_point,
                                 const cVector3d& a_planePoint0, 
                                 const cVector3d& a_planePoint1,
                                 const cVector3d& a_planePoint2, 
                                 double& a_v01, 
                                 double& a_v02)
{
    cVector3d v01 = cSub(a_planePoint1, a_planePoint0);
    cVector3d v02 = cSub(a_planePoint2, a_planePoint0);
    cVector3d point = cSub(a_point, a_planePoint0);

    // matrix
    double m00 = v01(0) * v01(0) + v01(1) * v01(1) + v01(2) * v01(2);
    double m01 = v01(0) * v02(0) + v01(1) * v02(1) + v01(2) * v02(2);
    double m10 = m01;
    double m11 = v02(0) * v02(0) + v02(1) * v02(1) + v02(2) * v02(2);
    double det = m00 * m11 - m10 * m01;

    // vector
    double vm0 = v01(0) * point(0) + v01(1) * point(1) + v01(2) * point(2);
    double vm1 = v02(0) * point(0) + v02(1) * point(1) + v02(2) * point(2);

    // inverse
    a_v01 = (1.0 / det) * ( m11 * vm0 - m01 * vm1);
    a_v02 = (1.0 / det) * (-m10 * vm0 + m00 * vm1);
}


//==============================================================================
/*!
    Compute the projection of a point on a line. the line is expressed
    by a point located on the line and a direction vector.

    \param    a_point  Point to project on the line.
    \param    a_pointOnLine  Point located on the line
    \param    a_directionOfLine  Vector expressing the direction of the line

    \return   Returns the projection of \e a_point on the line.
*/
//==============================================================================
inline cVector3d cProjectPointOnLine(const cVector3d& a_point,
                                     const cVector3d& a_pointOnLine, 
                                     const cVector3d& a_directionOfLine)
{
    // temp variable
    cVector3d point, result;

    // compute norm of line direction
    double lengthDirSq = a_directionOfLine.lengthsq();

    if (lengthDirSq > 0.0)
    {
        a_point.subr(a_pointOnLine, point);

        a_directionOfLine.mulr( (point.dot(a_directionOfLine) / (lengthDirSq)),
                                result);

        result.add(a_pointOnLine);
        return (result);
    }
    else
    {
        return (a_pointOnLine);
    }
}


//==============================================================================
/*!
    Compute the projection of a point on a segment. the segment is described
    by its two extremity points.

    \param    a_point  Point that is projected.
    \param    a_segmentPointA  Point A of segment.
    \param    a_segmentPointB  Point B of segment.

    \return   Returns the projection of \e a_point on the segment.
*/
//==============================================================================
inline cVector3d cProjectPointOnSegment(const cVector3d& a_point,
                                        const cVector3d& a_segmentPointA,
                                        const cVector3d& a_segmentPointB)
{
    // if both points are equal
    if (a_segmentPointA.equals(a_segmentPointB))
    {
        return (a_segmentPointA);
    }

    // compute line
    cVector3d segmentAB;
    a_segmentPointB.subr(a_segmentPointA, segmentAB);

    // project tool onto segment
    cVector3d projection = cProjectPointOnLine(a_point, a_segmentPointA, segmentAB);

    double distanceAB = segmentAB.lengthsq();
    double distanceAP = cDistanceSq(projection, a_segmentPointA);
    double distanceBP = cDistanceSq(projection, a_segmentPointB);

    if (distanceAP > distanceAB)
    {
        return(a_segmentPointB);
    }
    else
    if (distanceBP > distanceAB)
    {
        return(a_segmentPointA);
    }
    else
    {
        return(projection);
    }
}


//==============================================================================
/*!
    Compute the projection of a point on a disk lying in the XY plane.
    The disk is described by its Z coordinate and its radius.

    \param    a_point   Point that is projected.
    \param    a_height  Z coordinate of disk.
    \param    a_radius  Radius of disk.

    \return   Returns the projection of \e a_point on the disk.
*/
//==============================================================================
inline cVector3d cProjectPointOnDiskXY (const cVector3d& a_point,
                                        const double& a_height,
                                        const double& a_radius)
{
  const cVector3d center(0.0, 0.0, a_height);

  cVector3d pointProj(a_point);

  pointProj(2)  = a_height;

  cVector3d ray = cSub (pointProj, center);

  if (ray.length() < a_radius) return pointProj;
  else
  {
    ray.normalize();
    return center + a_radius * ray;
  }
}


//==============================================================================
/*!
    Project a vector \e V0 onto a second vector \e V1.

    \param    a_vector0  Vector 0.
    \param    a_vector1  Vector 1.

    \return   Returns the projection of \e V0 onto \e V1.
*/
//==============================================================================
inline cVector3d cProject(const cVector3d& a_vector0, 
                          const cVector3d& a_vector1)
{
    // temp variable
    cVector3d result;

    // compute projection
    double lengthSq = a_vector1.lengthsq();
    a_vector1.mulr( (a_vector0.dot(a_vector1) / (lengthSq)), result);

    // return result
    return (result);
}


//==============================================================================
/*!
    Compute the distance between a point and a line described by two points.

    \param    a_point  Point.
    \param    a_linePoint1  Line point 1.
    \param    a_linePoint2  Line point 1.

    \return   Return distance between points
*/
//==============================================================================
inline double cDistanceToLine(const cVector3d& a_point, 
                              const cVector3d& a_linePoint1,
                              const cVector3d& a_linePoint2)
{
    cVector3d point = cProjectPointOnLine(a_point, a_linePoint1, cSub(a_linePoint2, a_linePoint1));
    return (cDistance(a_point, point));
}


//==============================================================================
/*!
    Compute the normal of a surface defined by three point passed as
    parameters.

    \param    a_surfacePoint0  Point 0 of surface.
    \param    a_surfacePoint1  Point 1 of surface.
    \param    a_surfacePoint2  Point 2 of surface.

    \return     Return surface normal.
*/
//==============================================================================
inline cVector3d cComputeSurfaceNormal(const cVector3d& a_surfacePoint0,
                                       const cVector3d& a_surfacePoint1, 
                                       const cVector3d& a_surfacePoint2)
{
    // temp variable
    cVector3d v01, v02, result;

    // compute surface normal
    a_surfacePoint1.subr(a_surfacePoint0, v01);
    a_surfacePoint2.subr(a_surfacePoint0, v02);
    v01.normalize();
    v02.normalize();
    v01.crossr(v02, result);
    result.normalize();

    // return result
    return (result);
}


//==============================================================================
/*!
    Returns true if \e point is contained in the bounding box defined by min and max

    \param    a_point Test point
    \param    box_min Minimum coordinate in the boundary box
    \param    box_max Maximum coordinate in the boundary box

    \return   Returns true if p is in [box_min,box_max], inclusive
*/
//==============================================================================
inline bool cBoxContains(const cVector3d& a_point, 
                         const cVector3d& box_min, 
                         const cVector3d& box_max)
{
    if ((a_point(0)  >= box_min(0) ) && (a_point(0)  <= box_max(0) ) &&
        (a_point(1)  >= box_min(1) ) && (a_point(1)  <= box_max(1) ) &&
        (a_point(2)  >= box_min(2) ) && (a_point(2)  <= box_max(2) ))
        return true;
    return false;
}


//==============================================================================
/*!
    Returns the number of intersection points between a segment AB and
    a sphere of radius \e R. \n
    The number of points can be \e 0, \e 1 or \e 2. \n
    If two intersection points are detected, \e collisionPoint0 will be
    the closest one to \e segmentPointA of segment AB. 
    If the segment starts inside the sphere, then the contacts point is ignored.

    \param    a_segmentPointA       First point of segment AB.
    \param    a_segmentPointB       Second point of segment AB.
    \param    a_spherePos           Position of sphere center.
    \param    a_sphereRadius        Radius of sphere.
    \param    a_collisionPoint0     Intersection point 0 if detected.
    \param    a_collisionNormal0    Surface normal at intersection point 0
    \param    a_collisionPoint1     Intersection point 1 if detected.
    \param    a_collisionNormal1    Surface normal at intersection point 1

    \return   Returns the number of intersection points found.
*/
//==============================================================================
inline int cIntersectionSegmentSphere(const cVector3d& a_segmentPointA,
                                      const cVector3d& a_segmentPointB,
                                      const cVector3d& a_spherePos,
                                      const double& a_sphereRadius,
                                      cVector3d& a_collisionPoint0,
                                      cVector3d& a_collisionNormal0,
                                      cVector3d& a_collisionPoint1,
                                      cVector3d& a_collisionNormal1)
{
    // temp variables
    cVector3d AB, CA;
    a_segmentPointB.subr(a_segmentPointA, AB);
    a_segmentPointA.subr(a_spherePos, CA);
    double radiusSq = a_sphereRadius * a_sphereRadius;

    double a = AB.lengthsq();
    double b = 2.0 * cDot(AB, CA);
    double c = CA.lengthsq() - radiusSq;

    // invalid segment
    if (a == 0)
    {
        return (0);
    }

    double d = b*b - 4*a*c;

    // segment ray is located outside of sphere
    if (d < 0)
    {
        return (0);
    }

    // segment ray intersects sphere
    d = sqrt(d);
    double e = 2.0 * a;

    // compute both solutions
    double u0 = (-b + d) / e;
    double u1 = (-b - d) / e;

    // check if the solutions are located along the segment AB
    bool valid_u0 = cContains(u0, 0.0, 1.0);
    bool valid_u1 = cContains(u1, 0.0, 1.0);

    // two intersection points are located along segment AB
    if (valid_u0 && valid_u1)
    {
        if (u0 > u1) { cSwap(u0, u1); }

        // compute point 0
        AB.mulr(u0, a_collisionPoint0);
        a_collisionPoint0.add(a_segmentPointA);

        a_collisionPoint0.subr(a_spherePos, a_collisionNormal0);
        a_collisionNormal0.normalize();

        // compute point 1
        AB.mulr(u1, a_collisionPoint1);
        a_collisionPoint1.add(a_segmentPointA);

        a_collisionPoint1.subr(a_spherePos, a_collisionNormal1);
        a_collisionNormal1.normalize();

        return (2);
    }

    // one intersection point is located along segment AB
    else if (valid_u0)
    {
        // compute point 0
        AB.mulr(u0, a_collisionPoint0);
        a_collisionPoint0.add(a_segmentPointA);

        a_collisionPoint0.subr(a_spherePos, a_collisionNormal0);
        a_collisionNormal0.normalize();

        // check dot product to see if the intial segment point is located
        // inside the sphere.
        double dotProduct = cDot(AB, a_collisionNormal0);
        if (dotProduct < 0.0)
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }

    // one intersection point is located along segment AB
    else if (valid_u1)
    {
        // compute point 0
        AB.mulr(u1, a_collisionPoint0);
        a_collisionPoint0.add(a_segmentPointA);

        a_collisionPoint0.subr(a_spherePos, a_collisionNormal0);
        a_collisionNormal0.normalize();

        // check dot product to see if the intial segment point is located
        // inside the sphere.
        double dotProduct = cDot(AB, a_collisionNormal0);
        if (dotProduct < 0.0)
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }

    // both points are located outside of the segment AB
    else
    {
        return (0);
    }

}


//==============================================================================
/*!
    Returns the number of intersection points between a segment AB and
    a open topless cylinder of radius \e R. \n
    The number of intersection points can be \e 0, \e 1 or \e 2. \n
    If two intersection points are detected, \e collisionPoint0 will
    correspond to the point nearest to \e segmentPointA of segment AB.
    If the segment starts inside the cylinder, then the contacts point is ignored.

    \param    a_segmentPointA       First point of segment AB
    \param    a_segmentPointB       Second point of segment AB
    \param    a_cylinderPointA      Extremity point A of cylinder.
    \param    a_cylinderPointB      Extremity point B of cylinder.
    \param    a_cylinderRadius      Radius of cylinder.
    \param    a_collisionPoint0     Intersection point 0 if detected
    \param    a_collisionNormal0    Surface normal at intersection point 0
    \param    a_collisionPointRelPosAB0  Collision point0 projected on segment described by vertices a_segmentPointA and a_segmentPointB
    \param    a_collisionPoint1     Intersection point 1 if detected
    \param    a_collisionNormal1    Surface normal at intersection point 1
    \param    a_collisionPointRelPosAB1  Collision point1 projected on segment described by vertices a_segmentPointA and a_segmentPointB

    \return   Returns the number of intersection points found
*/
//==============================================================================
inline int cIntersectionSegmentToplessCylinder(const cVector3d& a_segmentPointA,
                                               const cVector3d& a_segmentPointB,
                                               const cVector3d& a_cylinderPointA,
                                               const cVector3d& a_cylinderPointB,
                                               const double& a_cylinderRadius,
                                               cVector3d& a_collisionPoint0,
                                               cVector3d& a_collisionNormal0,
                                               double& a_collisionPointRelPosAB0,
                                               cVector3d& a_collisionPoint1,
                                               cVector3d& a_collisionNormal1,
                                               double& a_collisionPointRelPosAB1)
{
    // compute some initial values
    cVector3d cylinderAB = a_cylinderPointB - a_cylinderPointA;
    double lengthCylinderAB = cylinderAB.length();

    // sanity check
    if (lengthCylinderAB == 0.0) { return (0); }

    // compute more initial values
    cVector3d cylinderDir = cNormalize(cylinderAB);
    cVector3d RC = a_segmentPointA - a_cylinderPointA;
    cVector3d segmentAB = a_segmentPointB - a_segmentPointA;
    cVector3d segmentDir = cNormalize(segmentAB);
    cVector3d n = cCross(segmentDir, cylinderDir);

    // segment is parallel to cylinder
    double length = n.length();
    if (length == 0.0)
    {
        return (0);
    }

    n.normalize();
    double d = cAbs (cDot (RC,n));

    if (d <= a_cylinderRadius)
    {
        cVector3d O = cCross(RC, cylinderDir);
        double t = -cDot(O,n) / length;
        O = cCross(n, cylinderDir);
        O.normalize();
        double s = cAbs(sqrt(a_cylinderRadius*a_cylinderRadius - d*d) / cDot(segmentDir,O));
        double u0 = t - s;
        double u1 = t + s;

        // reorder solutions
        if (u0 > u1) { cSwap (u0,u1); }

        bool valid_u0 = true;
        bool valid_u1 = true;

        // check if solutions along segment
        double lengthAB = segmentAB.length();
        if (!cContains(u0, 0.0, lengthAB)) { valid_u0 = false; }
        if (!cContains(u1, 0.0, lengthAB)) { valid_u1 = false; }

        // check if solutions lay along cylinder
        cVector3d P0, P1;
        cVector3d cylinderDirNeg = cNegate(cylinderDir);
        
        if (valid_u0)
        {
            P0 = a_segmentPointA + u0 * segmentDir;
            cVector3d P0A = cSub(P0, a_cylinderPointA);
            cVector3d P0B = cSub(P0, a_cylinderPointB);

            double cosAngleA = cCosAngle(cylinderDir, P0A);
            double cosAngleB = cCosAngle(cylinderDirNeg, P0B);

            if ((cosAngleA <= 0.0) || (cosAngleB <= 0.0))
            {
                valid_u0 = false;
            }
            else
            {
                a_collisionPointRelPosAB0 = cosAngleA * (P0A.length() / lengthCylinderAB);
            }
        }

        if (valid_u1)
        {
            P1 = a_segmentPointA + u1 * segmentDir;
            cVector3d P1A = cSub(P1, a_cylinderPointA);
            cVector3d P1B = cSub(P1, a_cylinderPointB);

            double cosAngleA = cCosAngle(cylinderDir, P1A);
            double cosAngleB = cCosAngle(cylinderDirNeg, P1B);

            if ((cosAngleA <= 0.0) || (cosAngleB <= 0.0))
            {
                valid_u1 = false;
            }
            else
            {
                a_collisionPointRelPosAB1 = cosAngleA * (P1A.length() / lengthCylinderAB);
            }
        }

        if (valid_u0 && valid_u1)
        {
            a_collisionPoint0 = P0;
            a_collisionNormal0 = cNormalize(cCross(cylinderDir, cCross(cSub(P0, a_cylinderPointA), cylinderDir)));
            a_collisionPoint1 = P1;
            a_collisionNormal1 = cNormalize(cCross(cylinderDir, cCross(cSub(P1, a_cylinderPointA), cylinderDir)));
            return (2);
        }
        else if (valid_u0)
        {
            a_collisionPoint0 = P0;
            a_collisionNormal0 = cNormalize(cCross(cylinderDir, cCross(cSub(P0, a_cylinderPointA), cylinderDir)));

            // check dot product to see if the intial segment point is located inside the cylinder.
            double dotProduct = cDot(segmentAB, a_collisionNormal0);
            if (dotProduct < 0.0)
            {
                return (1);
            }
            else
            {
                return (0);
            }
        }
        else if (valid_u1)
        {
            a_collisionPoint0 = P1;
            a_collisionNormal0 = cNormalize(cCross(cylinderDir, cCross(cSub(P1, a_cylinderPointA), cylinderDir)));

            // check dot product to see if the intial segment point is located inside the cylinder.
            double dotProduct = cDot(segmentAB, a_collisionNormal0);
            if (dotProduct < 0.0)
            {
                return (1);
            }
            else
            {
                return (0);
            }
        }
    }

    return (0);
}


//==============================================================================
/*!
    Returns __true__ if the segment AB intersects the triangle defined by
    its three vertices (\e V0, \e V1, \e V2).

    \param    a_segmentPointA  First point of segment AB.
    \param    a_segmentPointB  Second point of segment AB.
    
    \param    a_triangleVertex0  Vertex 0 of triangle
    \param    a_triangleVertex1  Vertex 1 of triangle
    \param    a_triangleVertex2  Vertex 2 of triangle
    
    \param    a_reportFrontSideCollision  If __true__, then reports collision when triangle is facing segment with front side.
    \param    a_reportBackSideCollision   If __true__, then reports collision when triangle is facing segment with back side.

    \param    a_collisionPoint   If a collision occurs, collision point is returned here.
    \param    a_collisionNormal  If a collision occurs, surface normal is reported here
    
    \param    a_collisionPosVertex01  Collision point projected on segment described 
              by vertices a_triangleVertex0 and a_triangleVertex1
    
    \param    a_collisionPosVertex02  Collision point projected on segment described 
              by vertices a_triangleVertex0 and a_triangleVertex2

    \return   Return __true__ if collision occurs, otherwise return __false__
*/
//==============================================================================
inline bool cIntersectionSegmentTriangle(const cVector3d& a_segmentPointA,
                                         const cVector3d& a_segmentPointB,
                                         const cVector3d& a_triangleVertex0,
                                         const cVector3d& a_triangleVertex1,
                                         const cVector3d& a_triangleVertex2,
                                         const bool a_reportFrontSideCollision,
                                         const bool a_reportBackSideCollision,
                                         cVector3d& a_collisionPoint,
                                         cVector3d& a_collisionNormal,
                                         double& a_collisionPosVertex01,
                                         double& a_collisionPosVertex02
                                         )
{
    // This value controls how close rays can be to parallel to the triangle
    // surface before we discard them
    const double C_INTERSECT_EPSILON = 10e-14f;

    // compute a ray and check its length
    cVector3d rayDir;
    a_segmentPointB.subr(a_segmentPointA, rayDir);
    double segmentLengthSquare = rayDir.lengthsq();
    if (segmentLengthSquare == 0.0) { return (false); }

    // Compute the triangle's normal
    cVector3d t_E0, t_E1, t_N;

    a_triangleVertex1.subr(a_triangleVertex0, t_E0);
    a_triangleVertex2.subr(a_triangleVertex0, t_E1);
    t_E0.crossr(t_E1, t_N);

    // If the ray is parallel to the triangle (perpendicular to the
    // normal), there's no collision
    double dotProduct = t_N.dot(rayDir);
    if (fabs(dotProduct)<10E-15f) return (false);

    if ((dotProduct < 0.0) && (a_reportFrontSideCollision == false)) return (false);
    if ((dotProduct > 0.0) && (a_reportBackSideCollision == false)) return (false);

    double t_T = cDot(t_N, cSub(a_triangleVertex0, a_segmentPointA)) / cDot(t_N, rayDir);

    if (t_T + C_INTERSECT_EPSILON < 0) return (false);

    cVector3d t_Q = cSub(cAdd(a_segmentPointA, cMul(t_T, rayDir)), a_triangleVertex0);
    double t_Q0 = cDot(t_E0,t_Q);
    double t_Q1 = cDot(t_E1,t_Q);
    double t_E00 = cDot(t_E0,t_E0);
    double t_E01 = cDot(t_E0,t_E1);
    double t_E11 = cDot(t_E1,t_E1);
    double t_D = (t_E00 * t_E11) - (t_E01 * t_E01);

    if ((t_D > -C_INTERSECT_EPSILON) && (t_D < C_INTERSECT_EPSILON)) return(false);

    double t_S0 = ((t_E11 * t_Q0) - (t_E01 * t_Q1)) / t_D;
    double t_S1 = ((t_E00 * t_Q1) - (t_E01 * t_Q0)) / t_D;

    // Collision has occurred. It is reported.
    if (
      (t_S0 >= 0.0 - C_INTERSECT_EPSILON) &&
      (t_S1 >= 0.0 - C_INTERSECT_EPSILON) &&
      ((t_S0 + t_S1) <= 1.0 + C_INTERSECT_EPSILON)
      )
    {
        cVector3d t_I = cAdd(a_triangleVertex0, cMul(t_S0,t_E0), cMul(t_S1, t_E1));

        // Square distance between ray origin and collision point.
        double distanceSquare = a_segmentPointA.distancesq(t_I);

        // check if collision occurred within segment. If yes, report collision
        if (distanceSquare <= segmentLengthSquare)
        {
            a_collisionPoint = cAdd(a_segmentPointA, cMul(t_T, rayDir));
            t_N.normalizer(a_collisionNormal);
            if (cCosAngle(a_collisionNormal, rayDir) > 0.0)
            {
                a_collisionNormal.negate();
            }
            
            a_collisionPosVertex01 = t_S0;
            a_collisionPosVertex02 = t_S1;

            return (true);
        }
    }

    // no collision occurred
    return (false);
}

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif  // CMathsH
//------------------------------------------------------------------------------


