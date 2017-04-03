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
/* \file CTaskBase.hpp
 *  Encapsulates a task servo and a task model
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCL_CONTROL_TASKS_CTASKBASE_HPP_
#define SCL_CONTROL_TASKS_CTASKBASE_HPP_

#include <scl/DataTypes.hpp>

#include <scl/control/task/data_structs/STaskBase.hpp>
#include <scl/data_structs/SRobotIO.hpp>

#include <scl/dynamics/CDynamicsBase.hpp>

#ifdef DEBUG
#include <cassert>
#endif

namespace scl
{
  namespace tasks{

    /** Container class to encapsulate a task model and a task servo.
     *
     * NOTE : Virtual class. Subclass and implement functions
     * that compute the task forces.
     */
    class CTaskBase {
    public:
      /* **************************************************************
       *                   Computation Functions
       * ************************************************************** */
      /** Computes the task torques */
      virtual bool computeServo(const SRobotSensors* arg_sensors)=0;

      /** Computes the dynamics (task model) */
      virtual bool computeModel(const SRobotSensors* arg_sensors)=0;

      /* **************************************************************
       *                   Status Get/Set Functions
       * ************************************************************** */
      /** Return this task controller's task data structure.
       *   Use it responsibly!
       * NOTE : Use dynamic casts whenever you downcast the data.
       *        And try not to downcast very often. Perf hit. */
      virtual STaskBase* getTaskData()=0;

      /** Sets the current goal position. Returns false if not supported by task. */
      virtual bool setGoalPos(const Eigen::VectorXd & arg_goal) { return false; }

      /** Sets the current goal velocity. Returns false if not supported by task. */
      virtual bool setGoalVel(const Eigen::VectorXd & arg_goal) { return false; }

      /** Sets the current goal acceleration. Returns false if not supported by task. */
      virtual bool setGoalAcc(const Eigen::VectorXd & arg_goal) { return false; }

      /** Gets the current goal position. Returns false if not supported by task. */
      virtual bool getGoalPos(Eigen::VectorXd & arg_goal) const { return false; }

      /** Gets the current goal velocity. Returns false if not supported by task. */
      virtual bool getGoalVel(Eigen::VectorXd & arg_goal) const { return false; }

      /** Gets the current goal acceleration. Returns false if not supported by task. */
      virtual bool getGoalAcc(Eigen::VectorXd & arg_goal) const { return false; }

      /** Gets the current position. Returns false if not supported by task. */
      virtual bool getPos(Eigen::VectorXd & arg_pos) const { return false; }

      /** Gets the current velocity. Returns false if not supported by task. */
      virtual bool getVel(Eigen::VectorXd & arg_vel) const { return false; }

      /** Gets the current acceleration. Returns false if not supported by task. */
      virtual bool getAcc(Eigen::VectorXd & arg_acc) const { return false; }

      /* **************************************************************
       *                   Initialization Functions
       * ************************************************************** */
      /** Constructor does nothing */
      CTaskBase():
        has_been_init_(false),
        dynamics_(S_NULL) {}

      /** Destructor does nothing */
      virtual ~CTaskBase(){}

      /** Initializes the task object. Create a subclass of
       * STaskBase if your task requires more data than the defaults
       * in STaskBase provide.
       * This function should set has_been_init_ to true*/
      virtual bool init(STaskBase* arg_task_data,
          CDynamicsBase* arg_dynamics)=0;

      /** Resets the task by removing its data. */
      virtual void reset()=0;

      /** Initialized = All static parameters are set and data structures
       * are up to date. Ready to contribute to a controller. */
      virtual sBool hasBeenInit() { return has_been_init_;  }

      /* **************************************************************
       *                   Runtime Enable/Disable Functions
       * ************************************************************** */
      /** Activated = All dynamic parameters and data structures are
       * up to date and task is actively contributing to a controller.
       *
       * Set to true/false during runtime to activate/deactivate task.
       *
       * Returns : success/failure */
      virtual sBool setActivated(sBool arg_activate)
      {
        STaskBase* t_ds = getTaskData();
        if(S_NULL == t_ds) { return false; }        //Can't access task data struct.
        if(!t_ds->has_been_init_) { return false; } //Task data struct not initialized
        t_ds->has_been_activated_=arg_activate;
        return true;
      }

      /** Activated = All dynamic parameters and data structures are
       * up to date and task is actively contributing to a controller. */
      virtual sBool hasBeenActivated()
      {
        STaskBase* t_ds = getTaskData();
        if(S_NULL == t_ds) { return false; } //Can't access task data.
#ifdef DEBUG
        if(!t_ds->has_been_init_)//If it is not initailized, it shouldn't be activated.
        { assert(!t_ds->has_been_activated_);  }
#endif
        return t_ds->has_been_activated_;
      }

    protected:
      /** Initialized = All static parameters are set and data structures
       * are up to date.
       *
       * Set to true in init() */
      sBool has_been_init_;

      /** A Dynamics model required to compute the task's dynamics */
      CDynamicsBase* dynamics_;
    };
  }
}

#endif /* SCL_CONTROL_TASKS_CTASKBASE_HPP_ */
