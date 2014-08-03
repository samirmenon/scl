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
/* \file CDynamics3d.hpp
 *
 *  Created on: Mar 21, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CDYNAMICS3D_HPP_
#define CDYNAMICS3D_HPP_

#include <scl/DataTypes.hpp>
#include <scl/dynamics/CDynamicsBase.hpp>

#include <Eigen/Dense>

#include <string>

namespace scl
{
  class SGcModel;
  struct SRobotSensors;
}

//Dynamics3d forward declaration
class cDynamicBase;
class cDynamicWorld;

namespace scl_ext
{
  /** Computes the dynamics of a robot model
   * using the Dynamics3d dynamics engine.
   *
   * This is presently under development. Contact
   * the author(s) for more details. */
  class CDynamics3d : public scl::CDynamicsBase
  {
  public:
    /* *******************************************************************
     *                      Integrator functions.
     * ******************************************************************* */

    /** Calculates the collision forces for the robot to which this dynamics
     * object is assigned.
     *
     * Uses std::string based force lookup. The dynamics implementation should
     * support this (maintain a map or something).
     *
     * NOTE : The dynamics engine may delete forces from the passed mapped list
     * at will. Do not store pointers to them at random places in your code.
     */
    virtual scl::sBool computeExternalContacts(/**
            This is where the simulator will store the contact
            forces. It may use the std::string to identify when
            to remove or re-add forces.*/
        sutil::CMappedList<std::string, scl::SForce> & arg_contacts)
    {   /** NOTE TODO : Fix this: */ return false; }

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
        const Eigen::VectorXd& arg_dq);

    /** Gets the robot's potential energy */
    virtual scl::sFloat computeEnergyPotential(
        /** The tree for which the transformation matrices are to be updated */
        sutil::CMappedTree<std::string, scl::SRigidBodyDyn> &arg_tree,
        /** The current generalized coordinates. */
        const Eigen::VectorXd& arg_q);

    /* given the name of an object, computes the forces and torques around that object
     * does not work with friction.
     */
    Eigen::Vector3d computeForce(std::string name);
    Eigen::Vector3d computeTorque(std::string name, Eigen::Vector3d pos);
    scl::sBool hasContacted(std::string name);
    scl::sInt getNumContacts(std::string name);

    /* *******************************************************************
     *                      Computational functions.
     * ******************************************************************* */
    /** We don't presently support this with dynamics3d */
    virtual bool computeGCModel(
        scl::SRobotSensors const * arg_sensor_data,
        scl::SGcModel * arg_gc_model)
    { return false; }

    /** NOTE TODO : This is obsolete. WILL BE DELETED SOON!
     *
     * Gives an id for a link name.
     *
     * Enables using calculateJacobain without:
     * 1. Storing any dynamic-engine specific objects in
     *    the controller.
     * 2. Using inefficient repeated string based
     *    lookup (usually with maps)
     */
    virtual const void * getIdForLink(std::string arg_link_name);



    /* *******************************************************************
     *                      Initialization functions.
     * ******************************************************************* */
    /** Default constructor sets the initialization state to false */
    CDynamics3d();

    /** Default destructor does nothing */
    virtual ~CDynamics3d();

    /** Initializes the dynamics to be computed for a
     * specific robot.
     *
     * Returns,
     * true : Succeeds in creating a dynamics object
     *
     * false : If it can't find the robot or if the information
     * is inconsistent with what the implementation requires,
     * it returns false
     */
    virtual bool init(const scl::SRobotParsed& arg_robot_data);
    /* Dynamics3d's base node */
    cDynamicBase *c_base;

  private:
    /** The robot's name */
    std::string robot_name_;

    /** The robot's degrees of freedom */
    size_t ndof_;

    /** The gravity acting on the robot */
    Eigen::Vector3d gravity_;

  };

}

#endif /* CDYNAMICS3D_HPP_ */
