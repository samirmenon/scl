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
/* \file STaskBase.hpp
 *
 *  Created on: Dec 25, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef STASKBASE_HPP_
#define STASKBASE_HPP_

#include <string>

#include <scl/DataTypes.hpp>

#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/data_structs/SRobotIOData.hpp>

#include <scl/control/data_structs/SGcModel.hpp>

namespace scl
{
  //Forward declaration allows pointers to the parent controller
  class STaskController;

  /**
   * Contains all the data required to compute a task
   *
   * A task computes forces/torques in any arbitrary
   * space.
   *
   * For instance an Euclidean task would do so in Euclidean
   * space. A joint space task would do so in joint space.
   * Any arbitrary space works as long as it can be mapped to
   * the output joint space..
   *
   * NOTE : YOU CAN NOT USE THIS TASK DATA STRUCTURE DIRECTLY.
   * You must subclass it, re-implement the function:
   *    virtual bool initTaskParams()
   * to parse all the custom parameters (from the xml file)
   * stored in
   *    task_nonstd_params_.
   * The function must then return true
   */
  class STaskBase : public SObject
  {
  public:
    /** The parent controller */
    const STaskController* parent_controller_;

    /** The type of the task */
    std::string type_task_;

    /** Whether the task is active or inactive
     * Default = false.
     * True after init() */
    scl::sBool has_been_activated_;

    /** Is control task. This variable controls whether the controller
     * computes remaining null spaces for lower level tasks or not. If
     * it is true, the lower level task null spaces are not computed,
     * potentially improving performance substantially.
     *
     * Settings:
     * True  : Task dof < Robot dof       [Default]
     * False : Task dof == Robot dof      [Must enable manually in init] */
    scl::sBool has_control_null_space_;

    /** The task's priority. Important to determine its range space.
     * Lower is better. 0 is best. -1 is NULL. */
    sUInt priority_;

    /** Task space degrees of freedom */
    scl::sUInt dof_task_;

    /** Robot model. Not a const because it could use the
     * branching representation's iterator */
    SRobotParsedData* robot_;

    /** Robot generalized coordinate model */
    const SGcModel* gc_model_;

    /** The computed task space Jacobian matrices.
     * Used for velocities. Note that we compute the generalized
     * inverse (the Jacobian is usually not full rank)
     * without inverting the jacobian itself. */
    Eigen::MatrixXd jacobian_, jacobian_dyn_inv_;

    /** The task's null space. Used to compute the
     * range space of lower priority tasks. */
    Eigen::MatrixXd null_space_;

    /** Task-space mass matrix */
    Eigen::MatrixXd lambda_, lambda_inv_;

    /** Task-space centrifugal/coriolis force vector
     * Controls cc forces in a subspace of the entire gc space so
     * that the operational point experience no cc forces. Other points,
     * however, do.*/
    Eigen::MatrixXd mu_;

    /** Task-space gravity force vector.
     * Controls gravity in a subspace of the entire gc space so
     * that the operational point is massless. Other points experience
     * gravity.*/
    Eigen::MatrixXd p_;

    /** Task space forces
     * Computed by : the task-servo
     * Used by     : the task-servo to compute the force_gc_ */
    Eigen::VectorXd force_task_;

    /** Upper and lower limits of task-space forces
     * (Why separate : Consider muscles which can only pull, not push) */
    Eigen::VectorXd force_task_max_,force_task_min_;

    /** Generalized coordinate forces (usually joint torques) :
     * Computed by : the task-servo = J' * f
     * Used by     : the robot-servo to calculate overall generalized coordinate force */
    Eigen::VectorXd force_gc_;

    /** Range space of the task :
     * Computed by : the robot-servo to calculate overall generalized coordinate force
     * Used by     : the robot-servo to calculate overall generalized coordinate force */
    Eigen::MatrixXd range_space_;

    /** Gains (scalar if same for different dimensions;
     * vector if different for different dimensions)
     * kp = position, kv = velocity, ka = acceleration
     * ki = integrated position error (to fix steady-state errors).
     *
     * Computed by : None. These are constants.
     * Used by     : The task-servo to calculate task forces */
    Eigen::VectorXd kp_, kv_, ka_, ki_;

    /** A set of additional options that may be used by users to
     * initialize this specific task. These typically go above and
     * beyond the standard options.
     *
     * Eg.The parent link and the pos in parent, which are required by op
     *    point tasks but not by gc tasks. These will be stored like:
     *    task_options_[0].data_[0] = "parent_link";
     *    task_options_[0].data_[1] = "base";
     *    task_options_[1].data_[0] = "pos_in_parent";
     *    task_options_[1].data_[1] = "0.0 0.0 0.0";
     *    task_options_[2].data_[0] = "my_new_arbitrary_option";
     *    task_options_[2].data_[1] = "8080";
     *    */
    std::vector<sString2> task_nonstd_params_;

    /* *********************************************************************
     *                            Initialization functions
     * ********************************************************************* */
    /** Constructor */
    STaskBase();

    /** Destructor : Does nothing */
    virtual ~STaskBase(){}

    /** Initialization function
     *
     * NOTE : This function also activates the task. */
    bool init(const std::string & arg_name,
        const std::string & arg_type,
        const sUInt arg_priority,
        /** 0  task dof means a gc task. Ie. full dofs */
        const scl::sUInt arg_task_dof,
        SRobotParsedData* arg_robot_ds,
        SRobotIOData* arg_io_data,
        /* The remaining variables initialize model_ and servo_ */
        const SGcModel* arg_gc_model,
        const Eigen::VectorXd & arg_kp,
        const Eigen::VectorXd & arg_kv,
        const Eigen::VectorXd & arg_ka,
        const Eigen::VectorXd & arg_ki,
        const Eigen::VectorXd & arg_ftask_max,
        const Eigen::VectorXd & arg_ftask_min,
        /** These are ignored during STaskBase initialization.
         * However, subclasses may choose to use them and/or
         * require various values */
        const std::vector<scl::sString2>& arg_nonstd_params);

    /** Sets the parent controller */
    bool setParentController(const STaskController* arg_parent);

    /** Processes the task's non standard parameters, which the
     * init() function stores in the task_nonstd_params_.
     *
     * This function is called by init() and must be implemented
     * by all subclasses. Else it will be impossible to initialize
     * the task. Ie. init() will always return false. */
    virtual bool initTaskParams()=0;
  };
}
#endif /* STASKBASE_HPP_ */
