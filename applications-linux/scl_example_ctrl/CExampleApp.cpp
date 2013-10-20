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
/* \file CExampleApp.cpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CExampleApp.hpp"
#include "tasks/CTaskGcExample.hpp"
#include "tasks/CTaskOpExample.hpp"

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SDatabase.hpp>

#include <sutil/CSystemClock.hpp>

#include <chai3d.h>

#include <iostream>
#include <stdexcept>

using namespace chai3d;

namespace scl_app
{

  /** Default constructor. Sets stuff to zero.
   * Uses a task controller*/
  CExampleApp::CExampleApp() : CRobotApp()
  { }

  scl::sBool CExampleApp::initMyController(const std::vector<std::string>& argv,
      scl::sUInt args_parsed)
  {
    try
    {
      //Ctr in array of args_parsed = (args_parsed - 1)
      //So ctr for un-parsed arg = (args_parsed - 1) + 1
      scl::sUInt args_ctr = args_parsed;

      // Check that we haven't finished parsing everything
      while(args_ctr < argv.size())
      {
        /* NOTE : ADD MORE COMMAND LINE PARSING OPTIONS IF REQUIRED */
        // else if (argv[args_ctr] == "-p")
        // { }
        if(true)
        {
          std::cout<<"\n Possible example task options: -xxx (you can change me to suit your needs!)";
          args_ctr++;
        }
      }

      return true;
    }
    catch(std::exception &e)
    { std::cout<<"\nCExampleApp::initMyController() : "<<e.what(); }
    return false;
  }

  scl::sBool CExampleApp::registerCustomDynamicTypes()
  {
    bool flag;
    flag = registerType_TaskGcEmpty();
    flag = flag && registerType_TaskOpExample();
    return flag;
  }

  scl::sBool CExampleApp::setInitialStateForUIAndDynamics()
  {
    bool flag;
    try
    {
      //Compute dynamics and servo once to initialize matrices.
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
      robot_.computeServo();
      robot_.setGeneralizedCoordinatesToZero();
      robot_.setGeneralizedVelocitiesToZero();
      robot_.setGeneralizedAccelerationsToZero();

      //Update the operational point tasks (if any)
      std::vector<SUiCtrlPointData>::iterator it,ite;
      for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
      {
        if(false == it->has_been_init_)
        { throw(std::runtime_error(std::string("UI Task not intialized: ")+it->name_)); }

        if(NULL==it->chai_pos_des_)
        { throw(std::runtime_error(std::string("UI Task's chai position vector is NULL: ")+it->name_)); }

        if(NULL==it->task_)
        { throw(std::runtime_error(std::string("UI Task's control object is null: ")+it->name_)); }

        it->task_->getPos(it->pos_);

        flag = (3 == it->pos_.rows() && 1 == it->pos_.cols()) ||
            (1 == it->pos_.rows() && 3 == it->pos_.cols());
        if( false == flag )
        { throw(std::runtime_error(std::string("UI task's control position vector size is incorrect: ")+it->name_)); }

        db_->s_gui_.ui_point_[it->ui_pt_] = it->pos_;

        //Using a tmp ref to simplify code.
        Eigen::Vector3d& tmp_ref = db_->s_gui_.ui_point_[it->ui_pt_];
        it->chai_pos_des_->setLocalPos(tmp_ref(0),tmp_ref(1),tmp_ref(2));
      }

      return true;
    }
    catch(std::exception &e)
    { std::cerr<<"\nCExampleApp::setInitialStateForUIAndDynamics() : "<<e.what(); }
    return false;
  }

  void CExampleApp::stepMySimulation()
  {
    sutil::CSystemClock::tick(db_->sim_dt_);//Tick the clock.

    //Update the operational point tasks (if any)
    std::vector<SUiCtrlPointData>::iterator it,ite;
    for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
    { it->task_->setGoalPos(db_->s_gui_.ui_point_[it->ui_pt_]); } //Set the goal position.

    if(ctrl_ctr_%5 == 0)           //Update dynamics at a slower rate
    {
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
    }

    if(ctrl_ctr_%20 == 0)           //Update graphics and/or log at a slower rate
    { // Every 2ms
      robot_.logState(true,true,true);

      //Set the positions of the ui points
      for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
      {
        Eigen::Vector3d& tmp_ref = db_->s_gui_.ui_point_[it->ui_pt_];
        it->chai_pos_des_->setLocalPos(tmp_ref(0),tmp_ref(1),tmp_ref(2));

        it->task_->getPos(it->pos_);
        Eigen::VectorXd& tmp_ref2 = it->pos_;
        it->chai_pos_->setLocalPos(tmp_ref2(0),tmp_ref2(1),tmp_ref2(2));
      }
    }
    robot_.computeServo();           //Run the servo loop
    robot_.integrateDynamics();      //Integrate system

    ctrl_ctr_++;//Increment the counter for dynamics computed.
  }
}
