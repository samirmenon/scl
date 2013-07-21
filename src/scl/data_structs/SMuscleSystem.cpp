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
/* \file SMuscleSystem.cpp
 *
 *  Created on: May 9, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "SMuscleSystem.hpp"

namespace scl
{
  SMusclePoint::SMusclePoint()
  {
    point_ = Eigen::Vector3d::Zero();
    parent_link_ = "";
    position_on_muscle_ = 0;
  }

  SMusclePoint::~SMusclePoint()
  { }

  SMuscle::SMuscle()
  {
    //Ref : Schutte93, Thelen03, Delp90
    max_isometric_force_ = 546.0;
    optimal_fiber_length_ = 0.0535;
    tendon_slack_length_ = 0.078;
    pennation_angle_ = 0.13962;
    activation_time_constt_ = 1.0;
    deactivation_time_constt_ = 1.0;
    max_contraction_vel_ = 10.0;// Unit : m/s
    max_contraction_vel_low_ = 10.0;
    max_contraction_vel_high_ = 10.0;
    max_tendon_strain_ = 100;
    max_muscle_strain_ = 100;
    stiffness_ = 100;
    damping_ = 0.0;
    stiffness_tendon_ = 1000;
    muscle_type_ = "";
    name_ = "";
  }

  SMuscle::~SMuscle()
  {}

  SMuscleSystem::SMuscleSystem() : SObject(std::string("SMuscleSystem"))
  {
    muscles_.clear();
    must_use_robot_ = "";
    render_muscle_thickness_ = 3;
    render_muscle_via_pt_sz_ = 0.0;
  }

  SMuscleSystem::~SMuscleSystem()
  { }
}
