/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
#include "tasks/CTaskOpExample.hpp"
#include "tasks/CTaskGcExample.hpp"

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

      // Set up gc damping
      for(unsigned int i=0;i<rob_ds_->dof_;++i)
      { rob_ds_->damping_gc_(i) = db_->sim_dt_*10; }

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
      robot_.setGeneralizedVelocitiesToZero();
      robot_.setGeneralizedAccelerationsToZero();
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
      robot_.computeServo();

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

    if(ctrl_ctr_%1 == 0)           //Update dynamics at a slower rate
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

    /** Slow down sim to real time */
    sutil::CSystemClock::tick(scl::CDatabase::getData()->sim_dt_);
    double tcurr = sutil::CSystemClock::getSysTime();
    double tdiff = sutil::CSystemClock::getSimTime() - tcurr;
    timespec ts = {0, 0};
    if(tdiff > 0)
    {
      ts.tv_sec = static_cast<int>(tdiff);
      tdiff -= static_cast<int>(tdiff);
      ts.tv_nsec = tdiff*1e9;
      nanosleep(&ts,NULL);
    }

    ctrl_ctr_++;//Increment the counter for dynamics computed.
  }
}
