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
/* \file CRobot.hpp
 *
 *  Created on: Dec 27, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CROBOT_HPP_
#define CROBOT_HPP_

#include <scl/DataTypes.hpp>

#include <scl/Singletons.hpp>
#include <scl/robot/data_structs/SRobot.hpp>
#include <scl/dynamics/CDynamicsBase.hpp>
#include <scl/control/CControllerBase.hpp>

#include <sutil/CMappedList.hpp>

#include <string>
#include <vector>
#include <fstream>

namespace scl
{
  /**
   * The one-stop-do-it-all class that encapsulates dynamics,
   * and a controller for a single robot.
   *
   * Also provides an interface into the underlying shared data
   * structure, the database, which facilitates communication
   * between different modules.
   *
   * NOTE : This class manages its memory. Add pointers and forget
   *        about them!
   *
   * NOTE : You will, however, have to initialize (by calling the init
   *        function) the individual objects yourself. */
  class CRobot
  {
  public:
    // **********************************************************************
    //                    Primary Computation functions
    // **********************************************************************

    /** Computes the robot's command torques.
     * Asserts false in debug mode if something bad happens */
    void computeServo();

    /** Computes the robot's dynamic model.
     * Asserts false in debug mode if something bad happens */
    void computeDynamics();

    /** Computes the robot's non-control related operations
     * Asserts false in debug mode if something bad happens */
    void computeNonControlOperations();

    /** Integrates the robot's dynamics (physics model).
     * By default, it integrates for a time period dt specifiecd
     * in the database
     *
     * Asserts false in debug mode if something bad happens */
    void integrateDynamics();


    // **********************************************************************
    //                       Initialization helper functions
    // **********************************************************************

    /** Convenience function. Pulls all the data from the database and calls
     * the other init function. */
    sBool initFromDb(std::string arg_robot_name,
        CDynamicsBase* arg_dynamics,
        CDynamicsBase* arg_integrator);

    /** The actual data required to initialize a robot.
     * Initializes the robot:
     * 1. Verifies that the robot's data in the database is consistent
     * 2. DELETES all its existing data! (NOTE this carefully!)
     * 3. Reads the list of available controllers from the database,
     *    finds the controllers that match this robot, and initializes them.
     *    (a) Creates a controller object for each new controller
     *    (b) Adds the controller data-structure to this robot's database data structure
     *    (c) Sets the last controller as the current controller (default, can be changed) */
    sBool init(std::string arg_robot_name,
        CDynamicsBase* arg_dynamics,
        CDynamicsBase* arg_integrator,
        SRobotParsed *arg_robot,
        SRobotIO *arg_io_data,
        std::vector<SControllerBase*>& arg_ctrls);

    /** Initialization state */
    sBool hasBeenInit() { return data_.has_been_init_;  }

    // **********************************************************************
    //                       Robot helper functions
    //                Runtime overrides for config file settings
    // **********************************************************************

    /** Returns a pointer to the robot's data structure */
    SRobot* getData() { return &data_;  }

    /** Turn velocity damping on or off. Turning it on will
     * make the robot lose some (1% default) velocity each
     * second */
    void setFlagApplyDamping(sBool arg_flag)
    { data_.parsed_robot_data_->flag_apply_gc_damping_ = arg_flag;  }

    /** Sets the velocity damping for each gc dof
     * WARNING: This will overwrite the values read in from the config file */
    sBool setDamping(const Eigen::VectorXd& arg_d);

    /** Turn gc dof limits on or off. Turning it on will
     * make the robot lose 99% velocity and all acceleration
     * for a gc dof if it collides with its limits */
    void setFlagApplyGcPosLimits(sBool arg_flag)
    { data_.parsed_robot_data_->flag_apply_gc_pos_limits_ = arg_flag;  }

    /** Sets the velocity damping for each gc dof
     * WARNING: This will overwrite the values read in from the config file */
    sBool setGcPosLimits(const Eigen::VectorXd& arg_max,
        const Eigen::VectorXd& arg_min);

    /** Turn the actuator limits on or off. Simulates physical
     * force limits of the actuators */
    void setFlagApplyActuatorForceLimits(sBool arg_flag)
    { data_.parsed_robot_data_->flag_apply_actuator_force_limits_ = arg_flag;  }

    /** Sets the actuator limits for each gc dof
     * WARNING: This will overwrite the values read in from the config file */
    sBool setActuatorForceLimits(const Eigen::VectorXd& arg_max,
        const Eigen::VectorXd& arg_min);

    /** Turn the controller on or off. Controller sends zero
     * command gc forces if off. */
    void setFlagControllerOn(sBool arg_flag)
    { data_.parsed_robot_data_->flag_controller_on_ = arg_flag;  }

    // **********************************************************************
    //                       Robot state helper functions
    // **********************************************************************

    const Eigen::VectorXd& getGeneralizedCoordinates()
    { return data_.io_data_->sensors_.q_; }

    const Eigen::VectorXd& getGeneralizedVelocities()
    { return data_.io_data_->sensors_.dq_; }

    const Eigen::VectorXd& getGeneralizedAccelerations()
    { return data_.io_data_->sensors_.ddq_; }

    const Eigen::VectorXd& getGeneralizedForcesMeasured()
    { return data_.io_data_->sensors_.force_gc_measured_; }

    const Eigen::VectorXd& getGeneralizedForcesCommanded()
    { return data_.io_data_->actuators_.force_gc_commanded_; }

    void setGeneralizedCoordinates(const Eigen::VectorXd& arg_q)
    {  data_.io_data_->sensors_.q_ = arg_q; }

    void setGeneralizedVelocities(const Eigen::VectorXd& arg_dq)
    {  data_.io_data_->sensors_.dq_ = arg_dq; }

    void setGeneralizedAccelerations(const Eigen::VectorXd& arg_ddq)
    {  data_.io_data_->sensors_.ddq_ = arg_ddq; }

    void setGeneralizedForcesMeasured(const Eigen::VectorXd& arg_f)
    {  data_.io_data_->sensors_.force_gc_measured_ = arg_f; }

    void setGeneralizedForcesCommanded(const Eigen::VectorXd& arg_f)
    {  data_.io_data_->actuators_.force_gc_commanded_ = arg_f; }

    void setGeneralizedCoordinatesToZero()
    {  data_.io_data_->sensors_.q_.setZero(data_.parsed_robot_data_->dof_); }

    void setGeneralizedVelocitiesToZero()
    {  data_.io_data_->sensors_.dq_.setZero(data_.parsed_robot_data_->dof_); }

    void setGeneralizedAccelerationsToZero()
    {  data_.io_data_->sensors_.ddq_.setZero(data_.parsed_robot_data_->dof_); }

    void setGeneralizedForcesMeasuredToZero()
    {  data_.io_data_->sensors_.force_gc_measured_.setZero(data_.parsed_robot_data_->dof_); }

    void setGeneralizedForcesCommandedToZero()
    {  data_.io_data_->actuators_.force_gc_commanded_.setZero(data_.parsed_robot_data_->dof_); }

    // **********************************************************************
    //                       Controller helper functions
    // **********************************************************************

    /** Adds a controller based on the initialization information passed
     * in a data structure.
     * Does type-checking (the data structure has a type) and initializes
     * an appropriate controller (controllers are derived from the CControllerBase
     * API).
     * Prints a warning if the passed data structure is invalid.
     *
     * NOTE : init() adds all the specified robot's controllers from
     * the database by default. Ie. It automatically loads controllers defined
     * in the xml file!!
     *
     * Only use this function for controllers defined "in code"
     * */
    sBool addController(SControllerBase* arg_ctrl_ds);

    /** Gets access to the current controller data structure */
    CControllerBase* getControllerCurrent();

    /** Selects the passed controller if it exists and has been initialized
     * Returns false if the controller doesn't exist*/
    sBool setControllerCurrent(std::string arg_ctrl_name);

    /** Gets access to the current controller data structure */
    SControllerBase* getControllerDataStruct(const std::string& arg_ctrl_name);

    /** Gets access to the current controller's dynamics */
    const CDynamicsBase* getControllerDynamics()
    { return static_cast<const CDynamicsBase*>(dynamics_);  }

    /** Returns the proportional gain of a given task in a controller.
     *
     * @param[out] ret_gains The variable into which the gains will be copied
     * @param[in]  ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
     * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers. */
    sBool getProportionalGain(Eigen::VectorXd& ret_gains,
        const std::string& arg_ctrl_name="",
        const std::string& arg_task_name="");

    /** Returns the derivative gain of a given task in a controller.
     *
     * @param[out] ret_gains The variable into which the gains will be copied
     * @param[in]  ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
     * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers. */
    sBool getDerivativeGain(Eigen::VectorXd& ret_gains,
        const std::string& arg_ctrl_name="",
        const std::string& arg_task_name="");

    /** Returns the integral gain of a given task in a controller.
     *
     * @param[out] ret_gains The variable into which the gains will be copied
     * @param[in]  ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
     * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers. */
    sBool getIntegralGain(Eigen::VectorXd& ret_gains,
        const std::string& arg_ctrl_name="",
        const std::string& arg_task_name="");

    /** Sets the proportional gain of a given task in a controller.
     *
     * @param[in]  arg_gains The gains to be set
     * @param[in]  ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
     * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers. */
    sBool setProportionalGain(const Eigen::VectorXd& arg_kp,
        const std::string& arg_ctrl_name="",
        const std::string& arg_task_name="");

    /** Sets the derivative gain of a given task in a controller.
     *
     * @param[in]  arg_kv The gains to be set
     * @param[in]  ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
     * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers. */
    sBool setDerivativeGain(const Eigen::VectorXd& arg_kv,
        const std::string& arg_ctrl_name="",
        const std::string& arg_task_name="");

    /** Sets the integral gain of a given task in a controller.
     *
     * @param[in]  arg_ki The gains to be set
     * @param[in]  ctrl_name The controller's name. If no controller name is specified, it picks the current controller by default.
     * @param[in]  task_name The task for which the gains will be selected. Not reqd for gen coord controllers. */
    sBool setIntegralGain(const Eigen::VectorXd& arg_ki,
        const std::string& arg_ctrl_name="",
        const std::string& arg_task_name="");

    // **********************************************************************
    //                       Logging functions
    // **********************************************************************

    /** Sets up logging to a file. The files are opened
     * in append+text mode and the state is written to them
     * every time the logState() function is called. */
    sBool setLogFile(const std::string &arg_file);

    /** Logs data to the file. Pass flags to control the data
     * written to the file. */
    sBool logState(bool arg_log_gc=true, bool arg_log_gc_matrices=true,
        bool arg_log_task_matrices=false);

    // **********************************************************************
    //                       Constructors etc.
    // **********************************************************************

    /** Constructor sets pointers to NULL */
    CRobot();

    /** Destructor does nothing */
    virtual ~CRobot();

  private:
    /** The data is available in the database for other parts of the
     * program to see.
     *
     * Programs can use it to interact with controllers that
     * don't support polling data structures in the database.
     *
     * However, the recommended way is to communicate through the
     * I/O data section of the database. */
    SRobot data_;

    SDatabase *db_;

    /** A dynamics object, which implements all the dynamics
     * matrix computation algorithms.
     *
     * NOTE : CRobot will deallocate this pointer later. */
    CDynamicsBase* dynamics_;

    /** A dynamics object, which implements a dynamics integrator.
     *
     * NOTE : CRobot will deallocate this pointer later.
     * NOTE : This may be the same as the dynamics_ object. */
    CDynamicsBase* integrator_;

    /** A mapped list of controllers that can control this robot. */
    sutil::CMappedList<std::string,CControllerBase*> ctrl_;

    /** The current controllers controlling this robot. */
    CControllerBase* ctrl_current_;

    /** For logging stuff to a file. */
    std::string log_file_name_;
    std::fstream log_file_;
    sBool logging_on_;
  };

}

#endif /* CROBOT_HPP_ */
