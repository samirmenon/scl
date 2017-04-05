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

#ifndef SRC_SCL_CONTROL_TASKS_DATA_STRUCTS_STASKBASE_HPP_
#define SRC_SCL_CONTROL_TASKS_DATA_STRUCTS_STASKBASE_HPP_

#include <scl/DataTypes.hpp>

#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SRobotIO.hpp>

#include <scl/data_structs/SGcModel.hpp>

#include <sutil/CMappedList.hpp>

#include <string>


namespace scl
{
  namespace tasks
  {
    enum ERROR_STATE{ None=0,
          BadInit_Generic = 1,
          BadInitState_CouldNotDeserializeDataFromJSON=2,
          BadInitState_CouldNotInitFromMappedList=3,
          BadDynamics_Generic = 10,
          BadDynamics_CouldNotFindLinkInRBDynTree = 11,
          BadSensorMeasurements_Generic = 20,
          UnknownError=100 };

    /** Contains all the data required to compute a task
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
     * You must subclass it, then implement the function:
     *    virtual bool initTaskParams()
     * to parse all the custom parameters (from the xml file)
     * stored in
     *    task_params_
     * The function must then return true.
     */
    class STaskBase : public SObject
    {
    public:
      /** The base id (or task type) of this task.
       * E.g., for STaskOpPos, it will be TaskOpPos */
      std::string task_type_id_="";

      /** Is it a subspace task? This variable controls whether the controller
       * computes remaining null spaces for lower level tasks or not. If
       * it is true, the lower level task null spaces are not computed,
       * potentially improving performance substantially.
       *
       * Settings:
       * True  : Task dof < Robot dof       [Default]
       * False : Task dof == Robot dof      [Must enable manually in init] */
      scl::sBool has_control_null_space_=true;

      /** The task's priority. Important to determine its range space.
       * Lower is better. 0 is best. -1 is NULL. */
      sUInt priority_=-1;

      /** Task space degrees of freedom */
      scl::sUInt dof_task_=0;

      /** The computed task space Jacobian matrices.
       * Used for velocities. Note that we compute the generalized
       * inverse (the Jacobian is usually not full rank)
       * without inverting the jacobian itself. */
      Eigen::MatrixXd J_, J_6_, J_dyn_inv_;

      /** The task's null space. Used to compute the
       * range space of lower priority tasks. */
      Eigen::MatrixXd null_space_;

      /** Task-space mass matrix */
      Eigen::MatrixXd M_task_, M_task_inv_;

      /** Task-space centrifugal/coriolis force vector
       * Controls cc forces in a subspace of the entire gc space so
       * that the operational point experience no cc forces. Other points,
       * however, do.*/
      Eigen::MatrixXd force_task_cc_;

      /** Task-space gravity force vector.
       * Controls gravity in a subspace of the entire gc space so
       * that the operational point is massless. Other points experience
       * gravity.*/
      Eigen::MatrixXd force_task_grav_;

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

      /** A variable to record errors (if any).
       * It might be a good idea for controllers to assume that
       * this task's torques are faulty if the error state is
       * not None.. */
      ERROR_STATE error_state_ = ERROR_STATE::None;

      /* *********************************************************************
       *                            Initialization functions
       * ********************************************************************* */
    private:
      /** Constructor : Disallow direct init so we can ensure type info is properly set up. */
      STaskBase();

    public:
      /** Constructor for setting the type */
      STaskBase(const std::string &arg_subclass_type) : SObject(arg_subclass_type)
      { task_type_id_ = arg_subclass_type.substr(1,std::string::npos); /* Remove the first S*/ }

      /** Destructor : Does nothing */
      virtual ~STaskBase(){}

      /** Initialization function (type 1; string pairs) :
       * Arguments:
       *   - A set of key value pairs, both of which are strings.
       *   - A description of the robot's kinematics
       *   - A data structure containing the robot's dynamics (NOTE TODO : Perhaps we shouldn't require this..
       *
       * Key-value pairs required:
       *   "name" : "unique-string-name"
       *   "type" : "task-type" (for dynamic object alloc & type lookup)
       *   "priority" : "0" (0 to n; lower number => higher priority)
       *   "task_dof" : "3" (the subspace dimension)
       *   "kp" : "100" (scalar for same along all axes; or vector, one val / axis)
       *   "kv" : "20 20 17"
       *   "ka" : "10" (acceleration gain; use only if you have a good acceleration trajectory generator)
       *   "ki" : "8" (integral gain; use only if there are a lot of unmodeled nolinearities and/or slow motions)
       *   "ftask_max" : "10" (max/min forces in task space; scalar or vector as with gains)
       *   "ftask_min" : "-4 -7 -11.2"
       *
       * NOTE : This simply parses the values out of the key-value mapped list and calls the other init function
       *
       * E.g., here is some sample code for setting task parameters:
            // Reference C++ code for setting a set of default parameters
            // for the task base (and one subclass; TaskOpPos)
            sutil::CMappedList<std::string, std::string> params;
            params.create(std::string("name"),std::string("hand"));
            params.create(std::string("type"),std::string("TaskOpPos"));
            params.create(std::string("priority"),std::string("0"));
            params.create(std::string("task_dof"),std::string("3"));
            params.create(std::string("kp"),std::string("10"));
            params.create(std::string("kv"),std::string("3"));
            params.create(std::string("ka"),std::string("0"));
            params.create(std::string("ki"),std::string("0"));
            params.create(std::string("ftask_max"),std::string("10"));
            params.create(std::string("ftask_min"),std::string("-10"));

            // Params for the subclass (TaskOpPos). These must also be set for the
            // init function to work
            params.create(std::string("parent_link"),std::string("end-effector"));
            params.create(std::string("pos_in_parent"),std::string("0.01 0.00 0.00"));
            params.create(std::string("flag_compute_op_gravity"),std::string("true"));
            params.create(std::string("flag_compute_op_cc_forces"),std::string("false"));
            params.create(std::string("flag_compute_op_inertia"),std::string("true"));
       */
      bool init(const sutil::CMappedList<std::string, std::string>& arg_params,
          const SRobotParsed* arg_robot_parsed);

      /** Initialization function (type 0; direct)
       *
       * NOTE : As an alternative, you may also use the other initialization
       *        function (type 1), though it does call this function in the end.
       *
       * Sets the different parameters */
      bool init(const std::string & arg_name,
          const std::string & arg_type,
          const sUInt arg_priority,
          /** 0  task dof means a gc task. Ie. full dofs */
          const scl::sUInt arg_task_dof,
          const SRobotParsed* arg_robot_parsed,
          const Eigen::VectorXd & arg_kp,
          const Eigen::VectorXd & arg_kv,
          const Eigen::VectorXd & arg_ka,
          const Eigen::VectorXd & arg_ki,
          const Eigen::VectorXd & arg_ftask_max,
          const Eigen::VectorXd & arg_ftask_min,
          const sutil::CMappedList<std::string, std::string>& arg_params);

      /** Processes the task's non standard parameters.
       * The set of additional options that may be used by users to
       * initialize this specific task. These typically go above and
       * beyond the standard options.
       *
       * This function is called by init() and must be implemented
       * by all subclasses. Else it will be impossible to initialize
       * the task. Ie. init() will always return false.
       *
       * Eg.The parent link and the pos in parent, which are required by op
       *    point tasks but not by gc tasks. These can be accessed using the
       *    mapped list .at() function. A string is returned. E.g.,
       *    arg_params.at("parent_link") == "base"
       *    arg_params.at("pos_in_parent") == "0.0 0.0 0.0"
       *    arg_params.at("my_new_arbitrary_option") == "8080"
       * */
      virtual bool initTaskSubclass(
          const sutil::CMappedList<std::string, std::string>& arg_params)=0;
    };
  }
}
#endif /* SRC_SCL_CONTROL_TASKS_DATA_STRUCTS_STASKBASE_HPP_ */
