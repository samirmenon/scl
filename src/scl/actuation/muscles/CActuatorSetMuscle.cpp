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
      name_(""), has_been_init_(false),
      robot_(NULL), robot_io_ds_(NULL),
      msys_(NULL), dynamics_(NULL)
  {}

  /** Initializes the actuator. This involves determining whether the muscle
   * matches the robot etc. It also sets up the Jacobians to be computed etc.
   */
  sBool CActuatorSetMuscle::init(const std::string& arg_name,
      const SRobotParsedData *arg_robot,
      const SRobotIOData *arg_rob_io_ds,
      const SMuscleSystem *arg_msys,
      CDynamicsBase *arg_dynamics)
  {
    bool flag;
    try
    {
      //Check pointers.
      if(NULL==arg_robot)
      { throw(std::runtime_error("Passed NULL robot parsed data struct")); }
      if(NULL==arg_rob_io_ds)
      { throw(std::runtime_error("Passed NULL robot io data struct")); }
      if(NULL==arg_msys)
      { throw(std::runtime_error("Passed NULL muscle spec data struct")); }
      if(false==arg_msys->has_been_init_)
      { throw(std::runtime_error("Passed unintialized muscle spec data struct")); }
      if(NULL==arg_dynamics)
      { throw(std::runtime_error("Passed NULL robot dynamics object")); }

      //Set the muscle name
      name_ = arg_name;

      //Save pointers
      robot_ = arg_robot;
      robot_io_ds_ = arg_rob_io_ds;
      msys_ = arg_msys;
      dynamics_ = arg_dynamics;

      // Initialize all the muscles in the muscle spec
      sutil::CMappedList<std::string, SMuscle>::const_iterator it,ite;
      for (it = arg_msys->muscles_.begin(), ite = arg_msys->muscles_.end();
          it != ite; ++it)
      {
        // Create a muscle computational object
        CActuatorMuscle* musc = muscles_.create(it->name_);
        if(NULL == musc)
        { throw(std::runtime_error(std::string("Could not create muscle: ")+it->name_)); }

        flag = musc->init(it->name_,arg_robot, arg_rob_io_ds, arg_msys, arg_dynamics);
        if(false == flag)
        { throw(std::runtime_error(std::string("Could not initialize muscle: ")+it->name_)); }
      }

      muscle_id_to_name_.resize(muscles_.size());

      // Initialize all the muscles ids in the new muscle objects
      int i=0;
      sutil::CMappedList<std::string, CActuatorMuscle>::const_iterator itm,itme;
      for (i=0, itm = muscles_.begin(), itme = muscles_.end();
          itm != itme; ++itm, ++i)
      {
        muscle_id_to_name_[i] = itm->getName();
        sUInt* tmp = muscle_name_to_id_.create(itm->getName());
        *tmp = i;
      }

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
    if(NULL == robot_) {  has_been_init_ = false; return false; }
    if(NULL == robot_io_ds_) {  has_been_init_ = false; return false; }
    if(NULL == msys_) {  has_been_init_ = false; return false; }
    if(false == msys_->has_been_init_) {  has_been_init_ = false; return false; }
    if(NULL == dynamics_) {  has_been_init_ = false; return false; }
#endif
    return has_been_init_;
  }


  /** Some actuator sets don't directly actuate the generalized coordinates
   * and require a Jacobian to compute their contribution to the generalized
   * forces.
   *
   * Each actuator instance must implement this. */
  sBool CActuatorSetMuscle::computeJacobian(Eigen::MatrixXd &ret_J)
  {//This function doesn't use std::exceptions (for speed).
    if(false == hasBeenInit()){ return false; }
    bool flag = true;

    // del-L_m = J del-q
    ret_J.resize(muscles_.size(),robot_->dof_);

    Eigen::VectorXd row_J;
    row_J.resize(robot_->dof_);

    // Compute a row vector for each muscle to get the full muscle Jacobian
    int i=0;
    sutil::CMappedList<std::string, CActuatorMuscle>::iterator itm,itme;
    for (i=0, itm = muscles_.begin(), itme = muscles_.end();
        itm != itme; ++itm,++i)
    {
      flag = flag && itm->computeJacobian(row_J);
#ifdef DEBUG
      if (false == flag) {  return false; }
#endif
      ret_J.row(i) = row_J;
    }
    return flag;
  }

} /* namespace scl */
