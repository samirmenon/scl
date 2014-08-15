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
/* \file SActuatorSetMuscleParsed.hpp
 *
 *  Created on: May 9, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SACTUATORSETMUSCLEPARSED_HPP_
#define SACTUATORSETMUSCLEPARSED_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SActuatorSetParsed.hpp>
#include <sutil/CMappedList.hpp>

#include <Eigen/Dense>

#include <vector>
#include <string>

namespace scl
{
  /** A single muscle point. A muscle passes through
   * multiple points. */
  class SMusclePointParsed
  {
  public:
    /** xyz coordinates of an attachment point */
    Eigen::Vector3d pos_in_parent_;

    /** The link to which this muscle point attaches */
    std::string parent_link_;

    /** The point's position on the muscle.
     * 0 = the muscle's start point
     * 1 = the next point
     * ... and so on. */
    sUInt position_on_muscle_;

    /** Default constructor. Sets defaults. */
    SMusclePointParsed();

    /** Default destructor. Does nothing. */
    ~SMusclePointParsed();

    /** To ensure new aligns the memory. Reqd for Eigen */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** A single muscle's specification:
   * Strings together a set of muscle points. */
  class SMuscleParsed
  {
  public:
    /** The muscle points should be ordered
     * by position_on_muscle_ */
    std::vector<SMusclePointParsed> points_;

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

    /** The muscle's stiffness ratio. Hill Muscles */
    sFloat stiffness_;
    /** The muscle's damping ratio. Hill Muscles */
    sFloat damping_;
    /** The muscle tendon's stiffness ratio. Hill Muscles */
    sFloat stiffness_tendon_;

    /** The muscle's type. Eg. Thelen2003Muscle, Schutte1993Muscle etc. */
    std::string muscle_type_;
    /** The muscle's name */
    std::string name_;

    /** Default constructor. Does nothing */
    SMuscleParsed();
    /** Default destructor. Does nothing */
    ~SMuscleParsed();
  };

  /** All the data required to attach muscle actuators
   * to any robot.*/
  class SActuatorSetMuscleParsed : public SActuatorSetParsed
  {
  public:
    /** ******************** DATA *********************** */
    /** A set of muscles */
    sutil::CMappedList<std::string, SMuscleParsed> muscles_;

    /** The muscle numeric id to name map */
    std::vector<std::string> muscle_id_to_name_;

    /** The muscle name to numeric id map */
    sutil::CMappedList<std::string, sUInt> muscle_name_to_id_;

    /** Graphics properties */
    int render_muscle_thickness_;//pixels
    double render_muscle_via_pt_sz_;//size of sphere

    /** ******************** FUNCTIONS *********************** */
    /** Default constructor. Sets the type. */
    SActuatorSetMuscleParsed();
    /** Default destructor. Does nothing */
    virtual ~SActuatorSetMuscleParsed();
  };

}

#endif /* SACTUATORSETMUSCLEPARSED_HPP_ */
