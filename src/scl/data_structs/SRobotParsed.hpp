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
/* \file SRobotParsed.hpp
 *
 *  Created on: Jul 22, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 

#ifndef SROBOTPARSED_HPP_
#define SROBOTPARSED_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/data_structs/SActuatorSetParsed.hpp>

#include <sutil/CMappedTree.hpp>
#include <string>

namespace scl
{

/**
 * This structure contains all the information required to construct
 * a robot. Each robot is completely defined by a tree of such
 * links.
 */
class SRobotParsed : public SObject
{
public:
  /** The branching representation will store a tree of SRigidBody nodes
   * and will maintain a mapping between their names and the nodes. */
  sutil::CMappedTree<std::string, SRigidBody> rb_tree_;

  /** The indices of the different links in the mapped tree */
  std::vector<std::string> robot_tree_numeric_id_to_name_;

  /** A set of actuators that move this robot */
  sutil::CMappedPointerList<std::string, SActuatorSetParsed, true> actuator_sets_;

  /** The joint values don't go outside this range */
  Eigen::VectorXd gc_pos_limit_max_, gc_pos_limit_min_;

  /** The joint default positions */
  Eigen::VectorXd gc_pos_default_;

  /** The damping (different for each dof)
   *
   * NOTE TODO : Update the friction model. This could
   * require a more sophisticated data structure than
   * a simple vector.
   *
   * NOTE TODO : Rename this to gc damping. */
  Eigen::VectorXd damping_;

  /** The actuators' max force values don't go outside this range */
  Eigen::VectorXd actuator_forces_max_, actuator_forces_min_;

  /** The number of degrees of freedom of the robot */
  sUInt dof_;

  /** The acceleration due to gravity for the robot */
  Eigen::Vector3d gravity_;

  /** The log file */
  std::string log_file_;

  /** ---------------------------------------------- */
  /** Flags to control the simulation     | Defaults */
  /** ---------------------------------------------- */
  sBool flag_apply_gc_damping_;              //false
  sBool flag_apply_gc_pos_limits_;           //false
  sBool flag_apply_actuator_force_limits_;   //true
  sBool flag_apply_actuator_pos_limits_;     //true
  sBool flag_apply_actuator_vel_limits_;     //true
  sBool flag_apply_actuator_acc_limits_;     //true
  sBool flag_controller_on_;                 //true
  sBool flag_logging_on_;                    //false
  sBool flag_wireframe_on_;                  //false
  sFloat option_axis_frame_size_;            //0.01
  sFloat option_muscle_via_pt_sz_;           //0.00 (not rendered)
  /** ---------------------------------------------- */

  //std::string name_; //Inherited
  //sBool has_been_init_; //Inherited

  SRobotParsed() : SObject(std::string("SRobotParsed") ),
      dof_(0)
  {
    //Flags to control SRobot's behavior
    flag_apply_gc_damping_            = false;
    flag_apply_gc_pos_limits_         = false;
    flag_apply_actuator_force_limits_ = true;
    flag_apply_actuator_pos_limits_   = true;
    flag_apply_actuator_vel_limits_   = true;
    flag_apply_actuator_acc_limits_   = true;
    flag_controller_on_               = true;
    flag_logging_on_                  = false;
    flag_wireframe_on_                = false;
    option_axis_frame_size_           = 0.01;//default
    option_muscle_via_pt_sz_          = 0.00;//default
  }
};

}//end of namespace scl_parser

#endif /*SROBOTPARSED_HPP_*/
