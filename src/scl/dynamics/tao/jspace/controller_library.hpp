/*
 * Stanford Whole-Body Control Framework http://stanford-scl.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file jspace/controller_library.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_CONTROLLER_LIBRARY_HPP
#define JSPACE_CONTROLLER_LIBRARY_HPP

#include <jspace/Controller.hpp>
#include <string>


namespace jspace {
  
  
  typedef enum {
    COMP_NONE         = 0x00,
    COMP_GRAVITY      = 0x01,
    COMP_CORIOLIS     = 0x02,
    COMP_MASS_INERTIA = 0x04
  } compensation_flags_t;
  
  
  class FloatController
    : public Controller
  {
  public:
    virtual Status setGoal(Eigen::VectorXd const & goal);
    virtual Status getGoal(Eigen::VectorXd & goal) const;
    virtual Status getActual(Eigen::VectorXd & actual) const;
    
    virtual Status setGains(Eigen::VectorXd const & kp, Eigen::VectorXd const & kd);
    virtual Status getGains(Eigen::VectorXd & kp, Eigen::VectorXd & kd) const;
    
    virtual Status latch(Model const & model);
    virtual Status computeCommand(Model const & model, Eigen::VectorXd & tau);
  };
  
  
  class GoalControllerBase
    : public Controller
  {
  public:
    GoalControllerBase(int compensation_flags,
		       Eigen::VectorXd const & default_kp,
		       Eigen::VectorXd const & default_kd);
    
    virtual Status init(Model const & model);
    
    virtual Status setGoal(Eigen::VectorXd const & goal);
    virtual Status getGoal(Eigen::VectorXd & goal) const;
    
    virtual Status setGains(Eigen::VectorXd const & kp, Eigen::VectorXd const & kd);
    virtual Status getGains(Eigen::VectorXd & kp, Eigen::VectorXd & kd) const;
    
  protected:
    int compensation_flags_;
    Eigen::VectorXd default_kp_;
    Eigen::VectorXd default_kd_;
    Eigen::VectorXd goal_;
    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;
  };
  
  
  class JointGoalController
    : public GoalControllerBase
  {
  public:
    JointGoalController(int compensation_flags,
			Eigen::VectorXd const & default_kp,
			Eigen::VectorXd const & default_kd);
    
    virtual Status getActual(Eigen::VectorXd & actual) const;
    virtual Status latch(Model const & model);
    virtual Status computeCommand(Model const & model, Eigen::VectorXd & tau);
    
  protected:
    Eigen::VectorXd actual_;
  };
  
}

#endif // JSPACE_CONTROLLER_LIBRARY_HPP
