/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file DataTypes.hpp
 *
 *  Created on: May 13, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef DATA_TYPES_HPP_
#define DATA_TYPES_HPP_

#include <string>

// Check GCC
#if __GNUC__
#if __x86_64__ || __ppc64__
#define SCL_64_BIT
#else
#define SCL_32_BIT
#endif
#endif

namespace scl
{
  ///////////////////////////////////////////////////////////////
  ////////////////////////////DATA TYPES/////////////////////////
  ///////////////////////////////////////////////////////////////
  /** Standard bool */
  typedef bool sBool;
  /** Standard char */
  typedef char sChar;
#ifdef SCL_64_BIT
  /** Standard unsigned int */
  typedef unsigned int sUInt;
#else
  typedef unsigned long sUInt;
#endif
  /** Standard int (4-byte) */
  typedef int sInt;
  /** Standard longlong */
  typedef long long sLongLong;

  /** Standard floating point ops in general
   * NOTE :
   * Float is precise to 7 significant-digits
   * This is fine for most scl ops. */
  typedef double sFloat;

  /** NOTE TODO : Depracated. Remove this later. */
  struct sFloat3
  { sFloat data_[3]; };

  /** Clock (Can use system clock measurements which require
   * more than a float  */
  typedef double sClock;

  /** Standard NULL */
  #define S_NULL 0

  /** Integration timestep : 1ms   */
  #define SCL_INTEGRATION_TSTEP 0.001

  /** Any position change lesser than 0.1mm will be considered zero.
   * This value sets the spatial resolution for controllers etc.
   *
   * NOTE : In general, SCL will use the following for all computation:
   * sFloat epsilon = std::numeric_limits<sFloat>::epsilon();
   *
   * This define should ONLY be used for controllers to check whether
   * they meet a goal etc.*/
  #define SCL_MINIMUM_POSITION_CHANGE 0.0001

  /** Two strings */
  struct sString2
  { std::string data_[2]; };

  ///////////////////////////////////////////////////////////////
  ////////////////////////////ENUM TYPES/////////////////////////
  ///////////////////////////////////////////////////////////////
  /** Enum for the different joint types in scl
   * Note: Arbitrary joint axis rotations are not supported. You
   * may instead rotate the transformation from the previous link's
   * frame to go to a standard axis (x,y,z). It is best to just use
   * z when you can (our convention). */
  typedef enum {
    JOINT_TYPE_PRISMATIC_X = 0,
    JOINT_TYPE_PRISMATIC_Y = 1,
    JOINT_TYPE_PRISMATIC_Z = 2,
    JOINT_TYPE_REVOLUTE_X = 4,
    JOINT_TYPE_REVOLUTE_Y = 5,
    JOINT_TYPE_REVOLUTE_Z = 6,
    JOINT_TYPE_SPHERICAL = 8,
    JOINT_TYPE_SPLINE = 9,
    JOINT_TYPE_NOTASSIGNED = -1
  }sJointType;

  typedef enum {
    RENDER_TYPE_SPHERE = 0,
    RENDER_TYPE_CYLINDER = 1,
    RENDER_TYPE_ELLIPSOID = 2
  }sRenderType;

  typedef enum {
    ROT_QUATERNION = 0, ROT_AXISANGLE = 1,
    ROT_DH = 2
  }sRotationType;

  ///////////////////////////////////////////////////////////////
  ///////////////////////////THE END/////////////////////////////
}

#endif /* DATA_TYPES_HPP_ */
