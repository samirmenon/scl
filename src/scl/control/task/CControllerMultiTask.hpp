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
/* \file CControllerMultiTask.hpp
 *
 *  Created on: Jul 21, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CCONTROLLERTASK_HPP_
#define CCONTROLLERTASK_HPP_

#include <scl/control/CControllerBase.hpp>
#include <scl/control/task/data_structs/SControllerMultiTask.hpp>

#include <scl/control/task/CTaskBase.hpp>
#include <scl/control/task/CNonControlTaskBase.hpp>
#include <scl/control/task/CServo.hpp>

#include <sutil/CMappedMultiLevelList.hpp>

#include <string>
#include <vector>

namespace scl
{
  //Forward declare the dynamics API.
  class CDynamicsBase;

  /** A generic task space controller:
   *
   * Contains:
   *
   * 1. A servo which computes torques for a
   *    set of tasks, and applies the torques
   *    after filtering them through
   *    the range spaces. (Uses the task-level null space
   *    decomposition for a multi-level task hierarchy [cite])
   *
   * 2. Contains a set of tasks and their priorities.
   *    Tasks contain:
   *    (a) A task servo to compute torques
   *    (b) A task model to compute dynamics
   *
   * 3. Contains a Joint space model for computing the robot's
   *    joint space dynamics
   *
   * Suggested usage :
   *
   * 1000Hz : computeControlForces //This is a servo tick.
   *          {//Executes:
   *            computeControlForces();
   *            computeTaskTorques();
   *          }
   *
   * 50-100 Hz : computeDynamics   //This is a model update
   *          {//Executes:
   *            computeModel();
   *            computeTaskModels();
   *          }
   *
   * 50-100Hz : computeNonControlTasks   //This is a non-control update
   *          {//Executes:
   *            computeNonControlTasks();
   *          }
   */
  class CControllerMultiTask : public CControllerBase
  {
  public:
    /**********************************************
     *               CControllerBase API
     * ********************************************/
    /** Default constructor : just sets pointers to NULL */
    CControllerMultiTask();

    /** Default destructor : does nothing */
    virtual ~CControllerMultiTask(){}

    /** Equal to task forces or generalized coordinate forces
     * depending on the type of controller that implements this API */
    virtual sBool computeControlForces();

    /** Computes the dynamic model   : Mass, MassInv, centrifugal/coriolis, gravity */
    virtual sBool computeDynamics();

    /** Support for non control computations in the controller.
     * Iterates over the mapped list of non-control tasks and executes them one by one.
     *
     * Such tasks typically include detailed error checks, logging, communication etc.
     * Add anything that doesn't require hard real-time and high-performance
     * constratints. */
    virtual sBool computeNonControlOperations();

    /** Returns the current control forces */
    virtual const Eigen::VectorXd* getControlForces()
    { return static_cast<const Eigen::VectorXd*>(&(data_->servo_.force_gc_));  }

    /** Whether the controller has been initialized to a particular robot */
    virtual sBool init(SControllerBase* arg_data,
            scl::CDynamicsBase* arg_dynamics);

    /** Resets to default. Can then re-initialize and reuse. */
    virtual sBool reset();


    /**********************************************
     *     Task-controller specific functions
     *     Use these for finer grained control!
     ***********************************************/
    /** Adds a task to the controller with a priority level.
     * Priority levels start at 0 (highest priority) > 1 > 2 ...
     *
     * If a higher level than max is supplied, new levels
     * are created. */
    bool addTask(const std::string &arg_task_name,
        CTaskBase* arg_task, const sUInt arg_level);

    /** Removes a task from the controller.
     * NOTE : This only removes the task from the controller. The data
     * structure is still conserved in the Database (for possible use later). */
    bool removeTask(const std::string &arg_task_name);

    /** Returns the task by this name */
    CTaskBase * getTask(const std::string& arg_name);

    /** Returns the number of tasks that this controller
     * executes simultaneously */
    sUInt getNumTasks() const { return data_->tasks_.size(); }

    /** Returns the number of tasks of a given type
     * that this controller executes simultaneously */
    sUInt getNumTasks(const std::string& arg_type) const;

    /** Enables a task within the controller */
    sBool activateTask(const std::string& arg_task_name);

    /** Disables a control task within the controller */
    sBool deactivateTask(const std::string& arg_task_name);

    /**********************************************
     *     Non Control Task specific functions
     *     Use these for finer grained control!
     ***********************************************/
    /** Adds a task to the controller */
    bool addNonControlTask(const std::string &arg_task_name,
        CNonControlTaskBase* arg_task);

    /** Removes a task from the controller.
     * NOTE : This only removes the task from the controller. The data
     * structure is still conserved in the Database (for possible use later). */
    bool removeNonControlTask(const std::string &arg_task_name);

    /** Returns the task by this name */
    CNonControlTaskBase* getNonControlTask(const std::string& arg_name);

    /** Returns the number of tasks that this controller
     * executes simultaneously */
    sUInt getNumNonControlTasks() const { return tasks_non_ctrl_.size(); }

    /** Activates a non control task */
    sBool activateNonControlTask(const std::string& arg_type);

    /** Deactivates a task within the controller */
    sBool deactivateNonControlTask(const std::string& arg_type);

  protected:
    /** Computes range spaces for all its tasks according to
     * their priorities. Starts with task level i and goes
     * down.
     * NOTE : It must be called after tasks at level i-1 undergo
     * a model update (because the null-spaces change). */
    bool computeRangeSpaces();

    /** All the data for this task-space controller */
    SControllerMultiTask * data_;

    /** The servo reads the task data and computes gc torques to be applied */
    CServo servo_;

    /** The list of tasks that this controller will execute
     * Outer vector : Specifies all tasks at a priority level.
     * Inner vector : Contains tasks */
    sutil::CMappedMultiLevelList<std::string, CTaskBase*> tasks_;

    /** The number of tasks */
    sUInt task_count_;

    /** The list of tasks that this controller will execute */
    sutil::CMappedList<std::string, CNonControlTaskBase*> tasks_non_ctrl_;

    /** The number of tasks */
    sUInt task_non_ctrl_count_;

  public:
    /** When only one task is to be executed
     * Speeds up this special (but fairly common) case.
     * Default behavior : Set to first added task. */
    CTaskBase * active_task_;
  };

}

#endif /* CCONTROLLERTASK_HPP_ */
