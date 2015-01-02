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
/* \file SForce.hpp
 *
 *  Created on: Mar 23, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SFORCE_HPP_
#define SFORCE_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SRigidBodyDyn.hpp>

#include <Eigen/Dense>
#include <string>

namespace scl
{
  /** Defines a task-space force acting on an articulated body.
   * This force could technically appear out of nowhere. For instance,
   * it could be a stochastic external noise force. Or a user interaction
   * virtual force.
   *
   * There are specific force sub-types derived from this basic force.
   * Contact force; collision force etc. */
  class SForce : public SObject
  {
  public:
    // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
    // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Its parent robot */
    const SRobotParsed *robot_;

    /** The link on which this force acts */
    std::string link_name_;

    /** A pointer to the dynamic engine's object for this link */
    const SRigidBodyDyn *rbd_;

    /** The force vector [fx, fy, fz, fwx, fwy, fwz] */
    Eigen::Vector6d force_;

    /** The point (wrt the link's coordinate frame) where the force acts */
    Eigen::Vector3d pos_;

    /** The direction along which the force is applied. */
    Eigen::Vector3d direction_;

    /** The Jacobian of the point. Useful for converting the applied force
     * into a set of generalized forces. Ie. J' * f
     *
     * NOTE : This must be an [ndof x 6] sized Matrix to
     * translate an [fx, fy, fz, fwx, fwy, fwz] force vector into joint torques.
     */
    Eigen::MatrixXd J_;

    /** Default constructor. Sets stuff to zero */
    SForce(): SObject("SForce"), robot_(NULL),link_name_(""),
        rbd_(NULL){}

    /** Default destructor. Does nothing */
    virtual ~SForce(){}

  protected:
    /** Typed constructor (called by subclasses). Sets stuff to zero */
    SForce(std::string arg_subclass_type): SObject(arg_subclass_type),
        robot_(NULL),link_name_(""), rbd_(NULL){}
  };

  /** In addition to a simple force, a contact force also describes potential
   * force impulses during a collision and stores references to the object that
   * is in contact.
   *
   * NOTE : In the event of a contact with the ground (an object of infinite inertia)
   * the "other" data types will be set to NULL and ignored. This is because it makes
   * no sense for the "other" object to move.
   *
   * WARNING : In case of a plastic (energy dissipating) collision with an object of
   * infinite inertia, it is not feasible to resolve the conservation of momentum
   * requirements with the information contained in this object. Instead:
   * Fixed manipulator: You may assume that the ground was part of the object itself.
   *                    As such, all collision forces are internal and things are ok.
   * Floating manipulator : This is not rigidly connected to the ground. As such it
   *                        must use some form of propulsion to stabilize its base.
   *                        The propulsion mechanism must then conserve momentum.
   */
  class SForceContact : public SForce
  {
  public:
    // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
    // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** The other object that the robot makes contact with */
    const SRobotParsed *other_robot_;

    /** The link of the other robot at which contact is make */
    std::string other_link_name_;

    /** A pointer to the dynamic engine's object for this link */
    const SRigidBodyDyn *other_rbd_;

    /** The Jacobian of the contact point for the other robot. Useful for
     * converting the applied force into a set of generalized forces. Ie. J' * f
     *
     * NOTE : This must be an [ndof x 6] sized Matrix to
     * translate an [fx, fy, fz, fwx, fwy, fwz] force vector into joint torques.
     */
    Eigen::MatrixXd other_J_;

    /** The operational point momentum of the objects before the contact/collision
     * event.
     * NOTE : Momentum is conserved during a collision. */
    Eigen::Vector6d p_pre_, other_p_pre_;

    /** The operational point momentum of the objects at the current time.
     * NOTE : Momentum is conserved during a collision. */
    Eigen::Vector6d p_curr_, other_p_curr_;

    /** The coefficient of restitution of the collision. This will determine
     * how much energy is lost.
     * 0 ==> No energy is lost (to non-conservative forces)
     * 1 ==> All energy is lost */
    sFloat coeff_restitution_;

    /** The duration of the collision impulse force (typically the time since
     * the start of the collision) */
    sFloat time_impulse_;

    /** Energy lost in the collision impulse so far */
    sFloat energy_collision_;

    /** Default constructor. Sets stuff to zero */
    SForceContact(): SForce("SForceContact"), other_robot_(NULL), other_link_name_(""),
        other_rbd_(NULL), coeff_restitution_(-1), time_impulse_(-1),
        energy_collision_(std::numeric_limits<sFloat>::quiet_NaN()){}

    /** Default destructor. Does nothing */
    virtual ~SForceContact(){}

  protected:
    /** Typed constructor (called by subclasses). Sets stuff to zero */
    SForceContact(std::string arg_subclass_type): SForce(arg_subclass_type),
    other_robot_(NULL), other_link_name_(""),
    other_rbd_(NULL), coeff_restitution_(-1), time_impulse_(-1),
    energy_collision_(std::numeric_limits<sFloat>::quiet_NaN()) {}
  };
}

#endif /* SFORCE_HPP_ */
