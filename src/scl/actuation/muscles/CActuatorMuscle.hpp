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
/*
 * \file CActuatorMuscle.hpp
 *
 *  Created on: Jul 21, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CACTUATORMUSCLE_HPP_
#define CACTUATORMUSCLE_HPP_

#include <scl/data_structs/SObject.hpp>

//Needs to access parent robot kinematics
#include <scl/data_structs/SRobotIO.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SActuatorSetMuscleParsed.hpp>
#include <scl/dynamics/CDynamicsBase.hpp>

#include <scl/actuation/muscles/data_structs/SActuatorMuscle.hpp>

#include <Eigen/Eigen>

namespace scl
{
  /** Models a muscle as a linear actuator with zero
   * force generation delay and a stiff tendon, which
   * eliminates slack.
   *
   * Muscles may be connected to one or more limbs through
   * multiple via-points.
   *
   * The muscle length is determined by the distance between
   * via-points that cross one or more generalized coordinates.
   * Since the other via-points are fixed to a surface, they
   * don't contribute to the overall length. This allows a simpler
   * formulation of the muscle length gradient (Jacobian) matrix.
   * Computing the model pair-wise between gc crossing via-points
   * reduces the worst case complexity to O(n*2*k), where k are the
   * gcs spanned by successive via-points.
   *
   * NOTE : This doesn't accommodate muscle via points that slide
   * across surfaces. While those are more biologically realistic,
   * they involve solving a shortest path formulation over a curved
   * surface (the Brachistochrone problem). The sliding point might
   * be better for large motions across complex joint complexes. */
  class CActuatorMuscle
  {
  public:
    /* *****************************************************************
     *                        Actuation Model
     * ***************************************************************** */
    /** Computes the contribution of a single muscle to the entire Jacobian.
     *
     * Positive muscle force => Muscle contraction
     * Negative muscle force => Muscle pushes (usually an error)
     *
     * Note for modeling biological musculoskeletal systems:
     * Formulation is on a gc-by-gc basis. Ie. The muscle is considered piece-
     * wise linear and each piece's contribution is measured independently.
     * This is an approximation, but more detailed continuous models are hard
     * to validate experimentally. Moreover, given the variability introduced
     * by canonical musculoskeletal scaling, the errors introduced by piece-
     * wise length gradients are probably within the envelope of experimental
     * error. This demands biological investigation on a subject-by-subject
     * basis.
     *
     * To analytically compute the Jacobian, we use the following method:
     *
     * δl = J . δq          || l = muscle len. q = gen coord. J = row in Jm
     * δli = J(i) . δq(i)   || δli = change across muscle's i'th spanned gc
     * li = |x_fixed + x_pre_gc - (x_fixed+xpost_gc)| || Part of muscle doesn't
     *                                                   move. || = L2 norm
     * δli/δq(i) = δ(|x_pre_gc - xpost_gc|)/δq(i)     || Gradient wrt. q(i)
     *   = Gradient of path length between two attachment points (p0 and p1)
     *   = δ/δq ( |p0 - p1| )
     *   = δ/δq ( sqrt( (p0-p1)' * (p0-p1) ) )
     *   = 1/(2 * sqrt( (p0-p1)' * (p0-p1) ) ) * δ/δq ( (p0-p1)' * (p0-p1) )
     *   = 1/(2 * sqrt( (p0-p1)' * (p0-p1) ) ) * 2* (p0-p1)' δ/δq (p0-p1)
     *   = 1/sqrt( (p0-p1)' * (p0-p1) ) * (p0-p1)' ( δ/δq (p0) - δ/δq (p1) )
     *   = 1/d.norm() * d' ( δ/δq (p0) - δ/δq (p1) ); where d = p0 - p1
     *
     * J_muscle_i (row-vector) = Sum_over_attachment_point_pairs_di [ |d|^-1 * d' (J_p0 - J_p1) ]
     *
     * NOTE TODO : This can be greatly improved by computing Jacobians wrt.
     * the parent link instead of wrt. the root node. Massive performance hit.
     * NOTE TODO : Need to dig into the dynamics engine implementation and
     * enable Jacobians wrt. some arbit link, and Jacobians with local xyz
     * offset wrt node. */
    virtual sBool computeJacobian(
        /** The generalized coordinates */
        const Eigen::VectorXd &arg_q,
        /** The Jacobian for this muscle
         * del.length = J * del.q
         *
         * NOTE : The Jacobian is a row vector with non-zero entries only
         * at the spanned generalized coordinates*/
        Eigen::VectorXd& ret_J);

    /* *****************************************************************
     *                           Accessors
     * ***************************************************************** */
    std::string getName() const
    { return data_.name_; }

    /* *****************************************************************
     *                        Initialization
     * ***************************************************************** */
    /** Initializes the actuator. This involves determining whether the muscle
     * matches the robot etc. It also sets up the Jacobians to be computed etc.
     */
    virtual sBool init(const std::string& arg_name,
        const SRobotParsed *arg_robot,
        const SActuatorSetMuscleParsed *arg_msys,
        const sutil::CMappedList<std::string,SRigidBodyDyn> &arg_rbdtree,
        CDynamicsBase *arg_dynamics);

    /** Has this actuator been initialized */
    virtual inline sBool hasBeenInit();

    /* *****************************************************************
     *                        Constructors
     * ***************************************************************** */
    /** Default constructor. Sets stuff to NULL. */
    CActuatorMuscle();

    /** Default destructor. Does nothing */
    virtual ~CActuatorMuscle(){}

  protected:
    /** The data struct for the muscle actuator */
    SActuatorMuscle data_;

    /** The parent robot to which this actuator is attached */
    const SRobotParsed *robot_;

    /** The parsed muscle specification */
    const SActuatorSetMuscleParsed *msys_;

    /** The muscle in the system */
    const SMuscleParsed *muscle_;

    /** Dynamics specification (to compute robot Jacobians) */
    CDynamicsBase *dynamics_;
  };

} /* namespace scl */
#endif /* CACTUATORMUSCLE_HPP_ */
