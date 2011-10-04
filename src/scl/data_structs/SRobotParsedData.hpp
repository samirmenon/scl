/* This file is part of Busylizzy, a control and simulation library
for robots and biomechanical models.

Busylizzy is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

Busylizzy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
Busylizzy. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file SRobotParsedData.hpp
 *
 *  Created on: Jul 22, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 

#ifndef SROBOTPARSEDDATA_HPP_
#define SROBOTPARSEDDATA_HPP_

#include <string>

#include <scl/DataTypes.hpp>

#include <sutil/CMappedTree.hpp>
#include <scl/data_structs/SRobotLink.hpp>

#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SWorldParsedData.hpp>

namespace scl
{

/**
 * This structure contains all the information required to construct
 * a robot. Each robot is completely defined by a tree of such
 * links.
 */
struct SRobotParsedData : public SObject
{
public:
  /** The controller parser definition contains a root link vector and
   * a child link vector.
   *
   * The branching representation will store a tree of SRobotLink nodes
   * and will maintain a mapping between their names and the nodes. */
  sutil::CMappedTree<std::string, SRobotLink> robot_br_rep_;

  /** The joint limit doesn't go below this */
  Eigen::VectorXd joint_limit_min_;

  /** The joint limit doesn't go below this */
  Eigen::VectorXd joint_limit_max_;

  /** The joint default positions */
  Eigen::VectorXd joint_default_pos_;

  /** The damping (different for each dof) */
  Eigen::VectorXd damping_;

  /** The actuators' max force limits */
  Eigen::VectorXd max_actuator_forces_;

  /** The actuators' min force limits */
  Eigen::VectorXd min_actuator_forces_;

  /** The number of degrees of freedom of the robot */
  sUInt dof_;

  /** ---------------------------------------------- */
  /** Flags to control the simulation     | Defaults */
  /** ---------------------------------------------- */
  sBool flag_apply_damping_;                 //false
  sBool flag_apply_joint_limits_;            //true
  sBool flag_apply_actuator_force_limits_;   //true
  sBool flag_apply_actuator_pos_limits_;     //true
  sBool flag_apply_actuator_vel_limits_;     //true
  sBool flag_apply_actuator_acc_limits_;     //true
  sBool flag_controller_on_;                 //true
  /** ---------------------------------------------- */

  //std::string name_; //Inherited
  //sBool has_been_init_; //Inherited

  SRobotParsedData() : SObject(std::string("SRobotParsedData") )
  {
    //Flags to control SRobot's behavior
    flag_apply_damping_               = false;
    flag_apply_joint_limits_          = true;
    flag_apply_actuator_force_limits_ = true;
    flag_apply_actuator_pos_limits_   = true;
    flag_apply_actuator_vel_limits_   = true;
    flag_apply_actuator_acc_limits_   = true;
    flag_controller_on_               = true;
  }
};

}//end of namespace scl_parser

#endif /*SROBOTPARSEDDATA_HPP_*/
