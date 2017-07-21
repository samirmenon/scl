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

  /** A set of actuators that move this robot.
   *
   * NOTE : This is the parsed data. To utilize this properly, you'll
   * also require dyn data : Something that subclasses SActuatorSetBase.
   *
   * Typically, the dyn data is stored in the robot io data structure*/
  sutil::CMappedPointerList<std::string, SActuatorSetParsed, true> actuator_sets_;

  /** The joint values don't go outside this range */
  Eigen::VectorXd gc_pos_limit_max_, gc_pos_limit_min_;

  /** The joint default positions */
  Eigen::VectorXd gc_pos_default_;

  /** The damping (different for each dof)
   *
   * NOTE TODO : Update the friction model. This could
   * require a more sophisticated data structure than
   * a simple vector. */
  Eigen::VectorXd damping_gc_;

  /** The actuators' max force values don't go outside this range */
  Eigen::VectorXd actuator_forces_max_, actuator_forces_min_;

  /** The number of degrees of freedom of the robot */
  sUInt dof_=0;

  /** The acceleration due to gravity for the robot */
  Eigen::Vector3d gravity_;

  /** The log file */
  std::string log_file_;

  /** ---------------------------------------------- */
  /** Flags to control the simulation     | Defaults */
  /** ---------------------------------------------- */
  sBool     flag_apply_gc_damping_            = false;
  sBool     flag_apply_gc_pos_limits_         = false;
  sBool     flag_apply_actuator_force_limits_ = true;       //true
  sBool     flag_apply_actuator_pos_limits_   = true;
  sBool     flag_apply_actuator_vel_limits_   = true;
  sBool     flag_apply_actuator_acc_limits_   = true;
  sBool     flag_controller_on_               = true;
  sBool     flag_logging_on_                  = false;
  sBool     flag_wireframe_on_                = false;
  sFloat    option_axis_frame_size_           = 0.01;//default
  sFloat    option_muscle_via_pt_sz_          = 0.00;//default
  /** ---------------------------------------------- */

  //std::string name_; //Inherited
  //sBool has_been_init_; //Inherited

  /** Default constructor. Sets type */
  SRobotParsed() : SObject(std::string("SRobotParsed") ) { }

  /** Initializes the Eigen vectors. This is more time consuming and
   * potentially error prone hence is not in the constructors (avoids
   * weird C++ states).
   */
  bool init()
  {
    bool flag;
    try
    {
      //NOTE TODO : Add more error checks and init stuff here. There are two options:
      // Option 1. Init everything here
      // Option 2. Make sure the scl/Init.hpp stuff can do this and/or is tested..

      // *****************************************************************
      //           Now organize all the links in the data struct etc.
      // *****************************************************************
      dof_ = rb_tree_.size() - 1;//The root node is stationary

      //Connect children to parents in the rb tree.
      flag = rb_tree_.linkNodes();
      if(false == flag)
      { throw(std::runtime_error("Could not link robot's nodes.")); }

      bool link_order_specified = (robot_tree_numeric_id_to_name_.size() > 0);
      if(false == link_order_specified)
      {
        robot_tree_numeric_id_to_name_.resize(rb_tree_.size());

        sutil::CMappedTree<std::string, SRigidBody>::iterator its,itse;//For sorting
        for(auto its = rb_tree_.begin(), itse = rb_tree_.end();
            its!=itse; ++its)
        {// NOTE : This adds links to their default xml position given that there is no link order
          // It also adds the root node at the end (which doesn't need to be specified in the link order)
          if(0 <= its->link_id_)  //Non-root node
          { robot_tree_numeric_id_to_name_[its->link_id_] = its->name_; }
          else if(-1 == its->link_id_) //Root node
          { robot_tree_numeric_id_to_name_[rb_tree_.size()-1] = its->name_; }
        }
      }

      // Now sort the robot
      flag = rb_tree_.sort(robot_tree_numeric_id_to_name_);
      if(false == flag)
      { throw(std::runtime_error(std::string("Could not sort the robot's mapped tree (") + name_ + std::string(")")));  }

      // If the sort was successful, update the link ids.
      // Now also reset the link_id_ to match the order.
      int i=0;
      for(auto its = rb_tree_.begin(), itse = rb_tree_.end()-1;/* ignore root node at end */
          its!=itse; ++its, ++i)
      { its->link_id_ = i;  }

      // Print sorted node order
#ifdef DEBUG
      std::cout<<"\nreadRobotFromFile() : Sorted links for "<<name_;
      for(auto its = rb_tree_.begin(), itse = rb_tree_.end();
          its!=itse; ++its)
      { std::cout<<"\n\t"<<its->link_id_<<" : "<<its->name_;  }
#endif

      has_been_init_ = true;
    }
    catch (std::exception& e)
    {
      std::cout<<"\nSRobotParsed::init() : ERROR : "<<e.what();
      return false;
    }
    return true;
  }

  /** Initializes the Eigen vectors. This is more time consuming and
   * potentially error prone hence is not in the constructors (avoids
   * weird C++ states).
   */
  bool reset()
  {
    rb_tree_.clear();
    robot_tree_numeric_id_to_name_.clear();
    actuator_sets_.clear();
    gc_pos_limit_max_.setZero(3);
    gc_pos_limit_min_.setZero(3);
    gc_pos_default_.setZero(3);
    damping_gc_.setZero(3);
    actuator_forces_max_.setZero(3);
    actuator_forces_min_.setZero(3);
    gravity_.setZero(3);
    return true;
  }
};

}//end of namespace scl_parser

#endif /*SROBOTPARSED_HPP_*/
