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
/*
 * \file CGcController.cpp
 *
 *  Created on: Dec 29, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/control/gc/CGcController.hpp>

#include <stdexcept>
#include <iostream>

namespace scl
{

  CGcController::CGcController()
  {
    data_ = S_NULL;
  }

  CGcController::~CGcController()
  {}


  sBool CGcController::computeControlForces()
  {
    //Compute the servo torques
    static Eigen::VectorXd tmp1, tmp2, tmp3;

    tmp1 = (data_->des_q_ - data_->io_data_->sensors_.q_);
    tmp1 =  data_->kp_.array() * tmp1.array();

    tmp2 = (data_->des_dq_ - data_->io_data_->sensors_.dq_);
    tmp2 = data_->kv_.array() * tmp2.array();

    //Obtain force to be applied to a unit mass floating about
    //in space (ie. A dynamically decoupled mass).
    tmp3 = data_->des_ddq_ + tmp2 + tmp1;

    //Apply Task's Force Limits
    tmp3 = tmp3.array().min(data_->force_gc_max_.array()); // Remain below the upper bound.
    tmp3 = tmp3.array().max(data_->force_gc_min_.array()); // Remain above the lower bound.

    // We do not use the centrifugal/coriolis forces. They can cause instabilities.
    data_->des_force_gc_ = data_->gc_model_.A_ * tmp3 + data_->gc_model_.g_;

    return true;
  }

  sBool CGcController::computePDControlForces()
  {
    //Compute the servo generalized forces :
    //F_gc_star = M(q) (-kp(q-q_des)-kv(dq/dt)) + b(q,dq/dt) + g(q)
    static Eigen::VectorXd tmp1, tmp2, tmp3;

    tmp1 = data_->io_data_->sensors_.q_ - data_->des_q_;
    tmp1 = data_->kp_.array() * tmp1.array();

    tmp2 = data_->kv_.array() * data_->io_data_->sensors_.dq_.array();

    //Obtain force to be applied to a unit mass floating about
    //in space (ie. A dynamically decoupled mass).
    tmp3 = -tmp2 - tmp1;

    //Apply Task's Force Limits
    tmp3 = tmp3.array().min(data_->force_gc_max_.array()); // Remain below the upper bound.
    tmp3 = tmp3.array().max(data_->force_gc_min_.array()); // Remain above the lower bound.

    // We do not use the centrifugal/coriolis forces. They can cause instabilities.
    data_->des_force_gc_ = data_->gc_model_.A_ * tmp3 + data_->gc_model_.g_;

    return true;
  }

  sBool CGcController::computeFloatForces()
  {
    //Compute the servo generalized forces : Gravity compensation + damping
    //F_gc_star = M(q) (-kp(q-q_des)-kv(dq/dt)) + b(q,dq/dt) + g(q)
    static Eigen::VectorXd tmp1;

    tmp1 = -(data_->kv_.array() * data_->io_data_->sensors_.dq_.array());

    //Apply Task's Force Limits
    tmp1 = tmp1.array().min(data_->force_gc_max_.array()); // Remain below the upper bound.
    tmp1 = tmp1.array().max(data_->force_gc_min_.array()); // Remain above the lower bound.

    data_->des_force_gc_ = data_->gc_model_.A_ * tmp1 + data_->gc_model_.g_;

#ifdef W_TESTING
    Eigen::VectorXd tmp2 = data_->gc_model_.A_ * tmp1;
    std::cout<<"\n******* F* *******\n"<<tmp1.transpose()
        <<"\n******* A *******\n"<<data_->gc_model_.A_
        <<"\n******* g *******\n"<<data_->gc_model_.g_.transpose()
        <<"\n******* q *******\n"<<data_->io_data_->sensors_.q_.transpose()
        <<"\n******* dq *******\n"<<data_->io_data_->sensors_.dq_.transpose()
        <<"\n******* ddq *******\n"<<data_->io_data_->sensors_.ddq_.transpose()
        <<"\n******* kv *******\n"<<data_->kv_.transpose()
        <<"\n******* pd *******\n"<<tmp1.transpose()
        <<"\n******* A*pd *******\n"<<tmp2.transpose();
#endif
    return true;
  }

  sBool CGcController::computeDynamics()
  {
    bool flag = dynamics_->updateModelMatrices(&(data_->io_data_->sensors_),&(data_->gc_model_));
    return flag;
  }

  const Eigen::VectorXd* CGcController::getControlForces()
  { return static_cast<const Eigen::VectorXd*>(& (data_->des_force_gc_)); }


  sBool CGcController::init(SControllerBase* arg_data,
      scl::CDynamicsBase* arg_dynamics)
  {
    try
    {
      //Reset the computational object (remove all the associated data).
      reset();

      if(S_NULL==arg_data)
      { throw(std::runtime_error("NULL data structure passed."));  }
      if(false==arg_data->has_been_init_)
      { throw(std::runtime_error("Uninitialized data structure passed."));  }
      //This ensures that the passed data was properly initialized for gc controllers.
      data_ = dynamic_cast<SGcController*>(arg_data);

      if(NULL==arg_dynamics)
      { throw(std::runtime_error("NULL dynamics object passed."));  }
      if(false==arg_dynamics->hasBeenInit())
      { throw(std::runtime_error("Uninitialized dynamics object passed."));  }
      dynamics_ = arg_dynamics;

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      has_been_init_ = false;
      std::cout<<"\nCGcController::init() Error : "<<e.what();
    }
    return has_been_init_;
  }

  sBool CGcController::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
    return true;
  }

}
