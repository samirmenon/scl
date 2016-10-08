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
/* \file CActuatorSetMuscle.cpp
 *
 *  Created on: Jul 21, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CActuatorSetMuscle.hpp"

#include <iostream>
#include <stdexcept>

namespace scl
{
  /** Default constructor. Sets stuff to NULL. */
  CActuatorSetMuscle::CActuatorSetMuscle() :
      CActuatorSetBase("CActuatorSetMuscle"),
      data_(NULL),
      dynamics_(NULL)
  {}

  /** Initializes the actuator. This involves determining whether the muscle
   * matches the robot etc. It also sets up the Jacobians to be computed etc.
   */
  sBool CActuatorSetMuscle::init(
      SActuatorSetMuscle &arg_mset,
      SGcModel &arg_rgcm,
      CDynamicsBase *arg_dynamics)
  {
    bool flag;
    try
    {
      //Check passed objects (should all have been initialized).
      if(false==arg_mset.hasBeenInit())
      { throw(std::runtime_error("Passed unintialized muscle actuator set data struct")); }
      if(false==arg_mset.mset_parsed_->hasBeenInit())
      { throw(std::runtime_error("Passed unintialized muscle actuator set parsed data struct")); }
      if(false==arg_rgcm.hasBeenInit())
      { throw(std::runtime_error("Passed uninitialized robot gc model")); }
      if(false==arg_dynamics->hasBeenInit())
      { throw(std::runtime_error("Passed uninitialized robot dynamics object")); }

      if(NULL == arg_mset.mset_parsed_->robot_)
      { throw(std::runtime_error("Passed mset is linked to a mset_parsed data struct that has a null robot pointer")); }

      // The robot pased data structure..
      const SRobotParsed &rds = *(arg_mset.mset_parsed_->robot_);

      //Set the muscle name
      name_ = arg_mset.name_;

      //Save pointers
      data_ = &arg_mset;
      dynamics_ = arg_dynamics;

      // Initialize all the muscles in the muscle spec
      sutil::CMappedList<std::string, SMuscleParsed>::const_iterator it,ite;
      for (it = data_->mset_parsed_->muscles_.begin(), ite = data_->mset_parsed_->muscles_.end();
          it != ite; ++it)
      {
        // Create a muscle computational object
        CActuatorMuscle* musc = muscles_.create(it->name_);
        if(NULL == musc)
        { throw(std::runtime_error(std::string("Could not create muscle: ")+it->name_)); }

        flag = musc->init(it->name_,*(arg_mset.mset_parsed_), arg_rgcm, arg_dynamics);
        if(false == flag)
        { throw(std::runtime_error(std::string("Could not initialize muscle: ")+it->name_)); }
      }

      flag = muscles_.sort(data_->mset_parsed_->muscle_id_to_name_);
      if(false == flag)
      { throw(std::runtime_error("Could not sort parsed muscle set's computational objects")); }

      has_been_init_ = true;
    }
    catch(std::exception &e)
    {
      std::cout<<"\nCActuatorSetMuscle::init() : Failed. "<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  /** Has this actuator been initialized */
  sBool CActuatorSetMuscle::hasBeenInit()
  {
#ifdef DEBUG
    if(NULL == data_->mset_parsed_->robot_) {  has_been_init_ = false; return false; }
    if(NULL == data_->mset_parsed_) {  has_been_init_ = false; return false; }
    if(false == data_->mset_parsed_->has_been_init_) {  has_been_init_ = false; return false; }
    if(NULL == dynamics_) {  has_been_init_ = false; return false; }
#endif
    return has_been_init_;
  }


  /** Some actuator sets don't directly actuate the generalized coordinates
   * and require a Jacobian to compute their contribution to the generalized
   * forces.
   *
   * Each actuator instance must implement this. */
  sBool CActuatorSetMuscle::computeJacobian(
      const Eigen::VectorXd arg_q,
      Eigen::MatrixXd &ret_J)
  {//This function doesn't use std::exceptions (for speed).
    if(false == hasBeenInit()){ return false; }
    bool flag = true;

    // del-L_m = J del-q
    ret_J.resize(muscles_.size(),data_->mset_parsed_->robot_->dof_);

    row_J_.resize(data_->mset_parsed_->robot_->dof_);

    // Compute a row vector for each muscle to get the full muscle Jacobian
    int i=0;
    sutil::CMappedList<std::string, CActuatorMuscle>::iterator itm,itme;
    for (i=0, itm = muscles_.begin(), itme = muscles_.end();
        itm != itme; ++itm,++i)
    {
      flag = flag && itm->computeJacobian(arg_q, row_J_);
#ifdef DEBUG
      if (false == flag) {  return false; }
#endif
      ret_J.row(i) = row_J_;
    }
    return flag;
  }

} /* namespace scl */
