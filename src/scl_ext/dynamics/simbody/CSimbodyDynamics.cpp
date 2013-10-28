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
/* \file CSimbodyDynamics.cpp
 *
 *  Created on: Jul 24, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CSimbodyDynamics.hpp"

#include <Simbody.h>

#include <stdexcept>

namespace scl
{
  /** Constructor : Sets defaults */
  CSimbodyDynamics::CSimbodyDynamics() :
          robot_name_("(no robot yet)"),
          ndof_(0),
          simbody_matter_(simbody_system_),
          simbody_forces_(simbody_system_),
          simbody_rkm_integ_(NULL),
          simbody_ts_(NULL)
  { }

  /** Destructor : Deletes stuff */
  CSimbodyDynamics::~CSimbodyDynamics()
  {}

  /** Initialize the CSimbodyDynamics with the Simbody trees it needs in
   * order to update the kinematics and/or dynamics of a robot.
   *
   * Accesses the database.
   *
   * \return True if everything went according to plan. */
  bool CSimbodyDynamics::init(const SRobotParsed& arg_robot_data)
  {
    bool flag;

    try
    {
      //*******Step 1***********
      //Check if a valid representation was passed.
      //************************
      if (false == arg_robot_data.has_been_init_)
      { throw(std::runtime_error("Passed an uninitialized robot data structure"));  }

      //Set all the robot parameters
      robot_name_ = arg_robot_data.name_;
      gravity_ = arg_robot_data.gravity_;

      SimTK::Force::Gravity gravity(simbody_forces_, simbody_matter_,
          SimTK::Vec3(gravity_(0),gravity_(1),gravity_(2))); // up! (weird)

      //*******Step 2***********
      //Set up the root node
      //************************
      const sutil::CMappedTree<std::string, SRobotLink> & br = arg_robot_data.rb_tree_;
      const SRobotLink* tmp_root = br.getRootNodeConst();//The root node.
      if (tmp_root == NULL)
      { throw(std::runtime_error("Robot doesn't have valid root node"));}
      if(false == tmp_root->is_root_)
      { throw(std::runtime_error("Robot's branching representation returns root with SRobotLink::is_root_==false."));}

      //Set the starting body to the ground
      SimTK::MobilizedBody last = simbody_matter_.updGround();

      //NOTE TODO: We have to add a bogus fixed link with a weld joint to get this rotation.
      // Are there any other options? This sucks.
      //Set the position and orientation in the parent frame
      SimTK::Quaternion_<SimTK::Real> tmp_ori_parent_quat(
          tmp_root->ori_parent_quat_.w(),
          tmp_root->ori_parent_quat_.x(),
          tmp_root->ori_parent_quat_.y(),
          tmp_root->ori_parent_quat_.z());
      SimTK::Vec3 tmp_pos_parent(tmp_root->pos_in_parent_(0), tmp_root->pos_in_parent_(1), tmp_root->pos_in_parent_(2) );
      SimTK::Rotation_<SimTK::Real> tmp_ori_parent_rotmat(tmp_ori_parent_quat);
      SimTK::Transform_<SimTK::Real> tmp_trf_in_parent(tmp_ori_parent_rotmat,tmp_pos_parent);

      //Potentially superfluous (or atleast non-intuitive) syntax requires this. Used later
      SimTK::Body::Rigid body;
      SimTK::MobilizedBody::Weld ground(last,tmp_trf_in_parent,body, SimTK::Vec3(0));

      //*******Step 3***********
      //Traverse the robotRoot's tree and construct a Simbody tree structure
      //************************
      std::vector<SRobotLink*>::const_iterator it, ite;
      for(it= tmp_root->child_addrs_.begin(), ite = tmp_root->child_addrs_.end();
          it!=ite; ++it)
      {
        flag = addNonRootLink(ground,**it);
        if(false == flag)
        { throw(std::runtime_error("Could not add child links for robot")); }
      }

      //*******Step 4***********
      //Set up Simbody's topology
      //************************
      //Call this after changing the structure of the robot
      simbody_state_ = simbody_system_.realizeTopology();

      //*******Step 5***********
      //Set up Simbody's integrator
      //************************
      simbody_rkm_integ_ = new SimTK::RungeKuttaMersonIntegrator(simbody_system_);
      simbody_ts_ = new SimTK::TimeStepper(simbody_system_, *simbody_rkm_integ_);
      simbody_rkm_integ_->setAccuracy(1e-5); // ask for *lots* of accuracy here (default is 1e-3)
      simbody_ts_->initialize(simbody_state_);

      has_been_init_ = true;
      return true;
    }
    catch(std::exception& e)
    { std::cout<<"\nCSimbodyDynamics::init() : Error : "<<e.what();  }
    return false;
  }

  sBool CSimbodyDynamics::addNonRootLink(SimTK::MobilizedBody& arg_parent, const SRobotLink& arg_child_lnk)
  {
    sBool flag;
    try
    {
      if(arg_child_lnk.is_root_)
      { throw(std::runtime_error(std::string("Passed a root link. Can't add as non-root link: ")+arg_child_lnk.name_)); }

      if(arg_child_lnk.has_been_init_)
      { throw(std::runtime_error(std::string("Passed uninitialized non-root link: ")+arg_child_lnk.name_)); }

      //Potentially superfluous (or atleast non-intuitive) syntax requires this. Used later
      SimTK::Body::Rigid body;

      //Set the position and orientation in the parent frame
      SimTK::Quaternion_<SimTK::Real> tmp_ori_parent_quat(
          arg_child_lnk.ori_parent_quat_.w(),
          arg_child_lnk.ori_parent_quat_.x(),
          arg_child_lnk.ori_parent_quat_.y(),
          arg_child_lnk.ori_parent_quat_.z());
      SimTK::Vec3 tmp_pos_parent(arg_child_lnk.pos_in_parent_(0), arg_child_lnk.pos_in_parent_(1), arg_child_lnk.pos_in_parent_(2) );
      SimTK::Rotation_<SimTK::Real> tmp_ori_parent_rotmat(tmp_ori_parent_quat);
      SimTK::Transform_<SimTK::Real> tmp_trf_in_parent(tmp_ori_parent_rotmat,tmp_pos_parent);

      //Set the joint type
      if(scl::JOINT_TYPE_PRISMATIC_X == arg_child_lnk.joint_type_)
      {
        SimTK::MobilizedBody::Slider child_lnk(arg_parent, tmp_trf_in_parent, body, SimTK::Transform_<SimTK::Real>());
          arg_parent = child_lnk;
      }
      else if(scl::JOINT_TYPE_REVOLUTE_Z == arg_child_lnk.joint_type_)
      {
        SimTK::MobilizedBody::Pin child_lnk(arg_parent, tmp_trf_in_parent, body, SimTK::Transform_<SimTK::Real>());
        arg_parent = child_lnk;
      }
      else if(scl::JOINT_TYPE_SPHERICAL == arg_child_lnk.joint_type_)
      {
        SimTK::MobilizedBody::Ball child_lnk(arg_parent, tmp_trf_in_parent, body, SimTK::Transform_<SimTK::Real>());
        arg_parent = child_lnk;
      }
      else
      { throw(std::runtime_error(std::string("Simbody only supports Prismatic X, Revolute Z, and Spherical joints. At link: ")+arg_child_lnk.name_));  }

      //Now set the mass parameters
      SimTK::Real m(arg_child_lnk.mass_);
      SimTK::Vec<3,SimTK::Real> com(arg_child_lnk.com_(0),arg_child_lnk.com_(1),arg_child_lnk.com_(2));
      SimTK::Inertia_<SimTK::Real> inertia(arg_child_lnk.inertia_(0),arg_child_lnk.inertia_(1),arg_child_lnk.inertia_(2));
      SimTK::MassProperties_<SimTK::Real> tmp_mass(m,com,inertia);
      arg_parent.setDefaultMassProperties(tmp_mass);

      std::vector<SRobotLink*>::const_iterator it, ite;
      for(it= arg_child_lnk.child_addrs_.begin(), ite = arg_child_lnk.child_addrs_.end();
          it!=ite; ++it)
      {
        if(NULL == *it)
        { throw(std::runtime_error(std::string("Found child link with NULL pointer at parent: ")+(*it)->name_));  }

        flag = addNonRootLink(arg_parent,**it);

        if(false == flag)
        { throw(std::runtime_error(std::string("Could not add link to branching struct: ")+(*it)->name_));  }
      }

      return true;
    }
    catch(std::exception& e)
    { std::cout<<"\nCSimbodyDynamics::init() : Error : "<<e.what();  }
    return false;
  }

  bool CSimbodyDynamics::computeGCModel(/**
      This is where there current robot state is read when
      updateModelMatrices() is called. */
      SRobotSensors const * sensor_data,
      /**
      Pointer to the joint-space model structure for the
      robot. This is the place where the matrices reside
      that will get updated when calling
      updateModelMatrices(). */
      SGcModel * js_model)
  {
    //NOTE TODO : Implement this later.
    return false;
  }

  /** Gives an id for a link name.
   *
   * Enables using calculateJacobain without:
   * 1. Storing any dynamic-engine specific objects in
   *    the controller.
   * 2. Using inefficient repeated string based
   *    lookup (usually with maps)
   */
  const void* CSimbodyDynamics::getIdForLink(std::string arg_link_name)
  {
    //NOTE TODO : Implement this later
    return NULL;
  }

  /** Calculates the Transformation Matrix for the robot to which
   * this dynamics object is assigned.
   *
   * The Transformation Matrix is specified by a link and an offset
   * (in task space dimensions)from that link and is given by:
   *
   *           x_global_coords = T * x_link_coords
   *
   * Uses id based link lookup. The dynamics implementation should
   * support this (maintain a map or something).
   */
  sBool CSimbodyDynamics::computeTransform_Depracated(
      /** The link at which the transformation matrix is
       * to be calculated */
      const void* arg_link_id,
      /** The transformation matrix will be saved here. */
      Eigen::Affine3d& arg_T)
  {
    //NOTE TODO : Implement this later.
    return false;
  }

  /** Calculates the Jacobian for the robot to which this dynamics
   * object is assigned.
   *
   * The Jacobian is specified by a link and an offset (in task space
   * dimensions)from that link
   *
   * Uses id based link lookup. The dynamics implementation should
   * support this (maintain a map or something).
   */
  sBool CSimbodyDynamics::computeJacobian_Depracated(
      /** The link at which the Jacobian is to be calculated */
      const void* arg_link_id,
      /** The offset from the link's frame. */
      const Eigen::VectorXd& arg_pos_global,
      /** The Jacobain will be saved here. */
      Eigen::MatrixXd& arg_J)
  {
    //NOTE TODO : Implement this later.
    return false;
  }

  /** Integrates the robot's state.
   * Uses the given applied forces, torques, positions and velocities
   * and its internal dynamic model to compute
   * new positions and velocities.
   *
   * This version performs the entire set of operations on
   * the SRobotIO data structure.
   *
   * Reads from, and updates:
   * arg_inputs_.sensors_.q_
   * arg_inputs_.sensors_.dq_
   * arg_inputs_.sensors_.ddq_
   *
   * Reads from:
   * arg_inputs_.actuators_.forces_
   * arg_inputs_.actuators_.force_gc_commanded_
   */
  sBool CSimbodyDynamics::integrate(
      /** The existing generalized coordinates, velocities and
       * accelerations + The generalized forces + task (euclidean)
       * forces and the list of contact points and links. */
      SRobotIO& arg_inputs,
      /** The time across which the system should integrate the
       * dynamics.
       *
       * Simbody uses a forward euler integrator and uses this as the
       * dt so set it to a small value. */
      const sFloat arg_time_interval)
  {
    //Set the commanded torque:

    //Integrate
    if(arg_inputs.dof_ != simbody_state_.getNQ())
    { return false; }
    SimTK::Integrator::SuccessfulStepStatus retval;
    retval = simbody_ts_->stepTo(simbody_ts_->getTime()+arg_time_interval);
    if(SimTK::Integrator::InvalidSuccessfulStepStatus == retval)
    { return false; }

    //Copy the updated state to the sensors.
    int i=0;
    arg_inputs.sensors_.q_(i) = simbody_state_.getQ()[i]; ++i;
    arg_inputs.sensors_.q_(i) = simbody_state_.getQ()[i]; ++i;
    arg_inputs.sensors_.q_(i) = simbody_state_.getQ()[i]; ++i;
    arg_inputs.sensors_.q_(i) = simbody_state_.getQ()[i]; ++i;
    arg_inputs.sensors_.q_(i) = simbody_state_.getQ()[i]; ++i;
    arg_inputs.sensors_.q_(i) = simbody_state_.getQ()[i]; ++i;
  }

  /** Gets the robot's kinetic energy */
  sFloat CSimbodyDynamics::getKineticEnergy_Depracated()
  {
    //NOTE TODO : Implement this later.
    return false;
  }

  /** Gets the robot's potential energy */
  sFloat CSimbodyDynamics::getPotentialEnergy_Depracated()
  {
    //NOTE TODO : Implement this later.
    return false;
  }

} /* namespace scl */
