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
/* \file SMuscleSystem.hpp
 *
 *  Created on: May 9, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SMUSCLESYSTEM_HPP_
#define SMUSCLESYSTEM_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <sutil/CMappedList.hpp>

#include <Eigen/Dense>

#include <vector>
#include <string>

namespace scl
{
  /** A single muscle point. A muscle passes through
   * multiple points. */
  class SMusclePoint
  {
  public:
    /** xyz coordinates of an attachment point */
    Eigen::Vector3d point_;

    /** The link to which this muscle point attaches */
    std::string parent_link_;

    /** The point's position on the muscle.
     * 0 = the muscle's start point
     * 1 = the next point
     * ... and so on. */
    sUInt position_on_muscle_;

    /** Default constructor. Sets defaults. */
    SMusclePoint();

    /** Default destructor. Does nothing. */
    ~SMusclePoint();

    /** To ensure new aligns the memory. Reqd for Eigen */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** A single muscle's specification:
   * Strings together a set of muscle points. */
  class SMuscle
  {
  public:
    /** The muscle points should be ordered
     * by position_on_muscle_ */
    std::vector<SMusclePoint> points_;
    /** Maximum force that this muscle can exert */
    sFloat max_isometric_force_;
    /** The optimal muscle fiber length */
    sFloat optimal_fiber_length_;
    /** The tendon's slack */
    sFloat tendon_slack_length_;
    /** The pennation angle */
    sFloat pennation_angle_;
    /** The time constants : For muscle force increase & increaes+decrease */
    sFloat activation_time_constt_, deactivation_time_constt_;
    /** Max contraction velocity */
    sFloat max_contraction_vel_;
    /** Max contraction velocity at low and high fiber lengths. Thelen Muscles. */
    sFloat max_contraction_vel_low_,max_contraction_vel_high_;
    /** Max tendon strain : Thelen Muscles. */
    sFloat max_tendon_strain_;
    /** Max muscle strain : Thelen Muscles. */
    sFloat max_muscle_strain_;
    /** The muscle's damping ratio. NOTE TODO : What does this do? */
    sFloat damping_;
    /** The muscle's type. Eg. Thelen2003Muscle, Schutte1993Muscle etc. */
    std::string muscle_type_;
    /** The muscle's name */
    std::string muscle_name_;

    /** Default constructor. Does nothing */
    SMuscle();
    /** Default destructor. Does nothing */
    ~SMuscle();
  };

  /** All the data required to attach muscle actuators
   * to any robot.*/
  class SMuscleSystem : public SObject
  {
  public:
    /** A set of muscles */
    sutil::CMappedList<std::string, SMuscle> muscles_;

    /** This muscle system might not work on all
     * robots. */
    std::string must_use_robot_;

    /** Default constructor. Sets the type. */
    SMuscleSystem();
    /** Default destructor. Does nothing */
    virtual ~SMuscleSystem();
  };

}

#endif /* SMUSCLESYSTEM_HPP_ */
