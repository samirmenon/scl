/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/*
 * CDynamicsSclSpatial.hpp
 *
 *  Created on: Jun 2, 2014
 *      Author: Nayan Singhal <singhalnayan91@gmail.com>
 *              Brains in Silicon Lab,
 *              Stanford University.
 */

#ifndef CDYNAMICSSCLSPATIAL_HPP_
#define CDYNAMICSSCLSPATIAL_HPP_

#include <scl/DataTypes.hpp>
#include <scl/dynamics/CDynamicsBase.hpp>

#include <Eigen/Geometry>
#include <Eigen/Core>

namespace scl_ext
{
  /** This class calculates forward and inverse dynamics using algorithms that
   * rely on spatial vectors.
   * It provides a generic way to integrate three algorithms with scl:
   * 1. CRBA
   * 2. ABA
   * 3. RNEA */
  class CDynamicsSclSpatial : public scl::CDynamicsBase
  {
  public:
    /* *******************************************************************
     *                      Dynamics Algorithms
     * ******************************************************************* */
    /** Calculate Joint Acceleration using Composite Rigid Body Algorithm */
    bool forwardDynamicsCRBA(/** Current robot state. q, dq, ddq,
            sensed generalized forces and perceived external forces.*/
        const scl::SRobotIO *arg_io_data,
        /** Individual link Jacobians, and composite inertial,
          centrifugal/coriolis gravity estimates. */
        scl::SGcModel *arg_gc_model,
        /** The returned generalized accelerations (Eg. joint accelerations) */
        Eigen::VectorXd &ret_ddq);

    /** Calculate Joint Acceleration using Articulated Body Algorithm */
    bool forwardDynamicsABA(/** Current robot state. q, dq, ddq,
            sensed generalized forces and perceived external forces.*/
        const scl::SRobotIO *arg_io_data,
        /** Individual link Jacobians, and composite inertial,
          centrifugal/coriolis gravity estimates. */
        scl::SGcModel *arg_gc_model,
        /** The returned generalized accelerations (Eg. joint accelerations) */
        Eigen::VectorXd &ret_ddq);

    /** Calculate joint Torque using Newton Euler Recursive Algorithm */
    bool inverseDynamicsNER(/** Current robot state. q, dq, ddq,
            sensed generalized forces and perceived external forces.*/
        const scl::SRobotIO *arg_io_data,
        /** Individual link Jacobians, and composite inertial,
          centrifugal/coriolis gravity estimates. */
        scl::SGcModel *arg_gc_model,
        /** The returned generalized forces (Eg. joint torques) */
        Eigen::VectorXd &ret_fgc);



    /* *******************************************************************
     *                      Integrator functions.
     * ******************************************************************* */
    /** Integrates the robot's state.
     * Uses the given applied forces, torques, positions and velocities
     * and its internal dynamic model to compute new positions and velocities.
     * Operates on the SRobotIO data structure.
     *
     * Reads from, and updates:
     *    arg_inputs_.sensors_.q_, dq_, ddq_
     *
     * Reads from:
     *    arg_inputs_.sensors_.forces_external_
     *    arg_inputs_.actuators_.force_gc_commanded_
     */
    virtual scl::sBool integrate(
        /** The existing generalized coordinates, velocities and
         * accelerations + The generalized forces + task (euclidean)
         * forces and the list of contact points and links. */
        scl::SRobotIO& arg_inputs,
        /** The time across which the system should integrate the
         * dynamics. Could take fixed steps or dynamic ones in between.
         * Up to the integrator implementation. */
        const scl::sFloat arg_time_interval)
    { return false; }

    /** Calculate joint position and velocity using Newton numerical integrator */
    virtual scl::sBool integrate(
        /** Individual link Jacobians, and composite inertial,
            centrifugal/coriolis gravity estimates. */
        scl::SGcModel &arg_gc_model,
        /** Current robot state. q, dq, ddq,
            sensed generalized forces and perceived external forces.*/
        scl::SRobotIO &arg_io_data,
        /** step dt time */
        const scl::sFloat arg_time_interval);

    /* *******************************************************************
     *                      Dynamics State functions.
     * ******************************************************************* */
    /** Gets the robot's kinetic energy */
    virtual scl::sFloat computeEnergyKinetic(
        /** The tree for which the transformation matrices are to be updated */
        sutil::CMappedTree<std::string, scl::SRigidBodyDyn> &arg_tree,
        /** The current generalized coordinates. */
        const Eigen::VectorXd& arg_q,
        /** The current generalized velocities. */
        const Eigen::VectorXd& arg_dq)
    { return false; }

    /** Gets the robot's potential energy */
    virtual scl::sFloat computeEnergyPotential(
        /** The tree for which the transformation matrices are to be updated */
        sutil::CMappedTree<std::string, scl::SRigidBodyDyn> &arg_tree,
        /** The current generalized coordinates. */
        const Eigen::VectorXd& arg_q)
    { return false; }

    /** Gets the robot's kinetic energy using spatial algorithm*/
    bool computeEnergyKinetic(
        /** Individual link Jacobians, and composite inertial,
    		          centrifugal/coriolis gravity estimates. */
        scl::SGcModel &arg_gc_model,
        /** The current generalized coordinates. */
        const Eigen::VectorXd& arg_q,
        /** The current generalized velocities. */
        const Eigen::VectorXd& arg_dq,
        /** the returned kinetic energy */
        scl::sFloat &ret_kinetic_energy);

    /** Gets the robot's potential energy using spatial algorithm*/
    bool computeEnergyPotential(
        /** Individual link Jacobians, and composite inertial,
                  centrifugal/coriolis gravity estimates. */
        scl::SGcModel &arg_gc_model,
        /** The current generalized coordinates. */
        const Eigen::VectorXd& arg_q,
        /** the returned potential energy */
        scl::sFloat &ret_potential_energy);

    /* *******************************************************************
     *                      Computational functions.
     * ******************************************************************* */
    /** Updates the generalized coordinate model matrices (Everything in SGcModel).
     *
     * This is the most efficient method to access the standard matrices.
     * Computing transformations and Jacobians individually is typically
     * wasteful.
     */
    virtual bool computeGCModel(/** Current robot state. q, dq, ddq,
                sensed generalized forces and perceived external forces.*/
        const scl::SRobotSensors * arg_sensor_data,
        /** Individual link Jacobians, and composite intertial,
                centrifugal/coriolis gravity estimates.*/
        scl::SGcModel * arg_gc_model)
    { return false; }

    /* *******************************************************************
     *                      Initialization functions.
     * ******************************************************************* */
    /** Initializes the dynamics to be computed for a specific robot.
     * This implementation is stateless. Ideally all implementations should
     * be stateless... */
    virtual bool init(const scl::SRobotParsed& arg_robot_data)
    {
      if(arg_robot_data.has_been_init_){
        robot_parsed_data_ = & arg_robot_data;
        has_been_init_ = true;
        return true;
      }
      return false;
    }

    /** Constructor */
    CDynamicsSclSpatial();

    /** The Destructor */
    virtual ~CDynamicsSclSpatial();
  };

} /* namespace scl_ext */

#endif /* CDYNAMICSSCLSPATIAL_HPP_ */
