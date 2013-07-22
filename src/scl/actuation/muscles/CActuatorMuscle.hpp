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
#include <scl/data_structs/SRobotIOData.hpp>
#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/dynamics/CDynamicsBase.hpp>

#include <Eigen/Eigen>

namespace scl
{
  /** Contains the information required to implement the muscle
   * model and API   */
  class SActuatorMuscle : public SObject
  {
  public:
    class SViaPointSet{
    public:
      /** The two parent links to which the points are attached */
      std::string parent_link_0_, parent_link_1_;

      /** The two via points that span one or more generalized
       * coords */
      Eigen::Vector3d position_in_parent_0_, position_in_parent_1_;

      /** The two via points that span one or more generalized
       * coords (in global coordinates) */
      Eigen::Vector3d x_glob_0_, x_glob_1_;

      /** The instantaneous change in the two via points (in global coordinates) */
      Eigen::Vector3d dx_glob_0_, dx_glob_1_;

      /** The Jacobians at the via points */
      Eigen::MatrixXd J_0_, J_1_;

      /** The generalized coordinate spanned. This determines the columns
       * of the Jacobian that we will pay attention to. Only the actuated
       * gc columns are relevant while computing the muscle Jacobian. */
      const void* dynamics_link_id_0_, *dynamics_link_id_1_;

      /** Child link in branching representation = {0,1} */
      int child_link_id_;

      /** The generalized coordinate spanned. This determines the columns
       * of the Jacobian that we will pay attention to. Only the actuated
       * gc columns are relevant while computing the muscle Jacobian. */
      std::vector<sUInt> scl_gc_id_;

      /** Default constructor. Sets stuff to null */
      SViaPointSet();
    };

    /** The set of generalized coordinates spanned by this muscle */
    sutil::CMappedList<sUInt,SViaPointSet> via_point_set_;

    /** The generalized coordinate values */
    Eigen::VectorXd q_curr_, dq_curr_;

    /** Default constructor. Sets stuff to null. Sets SObject type. */
    SActuatorMuscle();
  };

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
    /** Some actuator sets don't directly actuate the generalized coordinates
     * and require a Jacobian to compute their contribution to the generalized
     * forces.
     *
     * Each actuator instance must implement this.
     *
     * NOTE TODO : This can be greatly improved by computing Jacobians wrt.
     * the parent link instead of wrt. the root node. Massive performance hit.
     * NOTE TODO : Need to dig into the dynamics engine implementation and
     * enable Jacobians wrt. some arbit link, and Jacobians with local xyz
     * offset wrt node. */
    virtual sBool computeJacobian(
        /** The Jacobian for this muscle
         * del.length = J * del.q
         *
         * NOTE : The Jacobian is a row vector with non-zero entries only
         * at the spanned generalized coordinates*/
        Eigen::VectorXd& ret_J);

    /* *****************************************************************
     *                        Initialization
     * ***************************************************************** */
    /** Initializes the actuator. This involves determining whether the muscle
     * matches the robot etc. It also sets up the Jacobians to be computed etc.
     */
    virtual sBool init(const std::string& arg_name,
        const SRobotParsedData *arg_robot,
        const SRobotIOData *arg_rob_io_ds,
        const SMuscleSystem *arg_msys,
        CDynamicsBase *arg_dynamics);

    /** Has this actuator been initialized */
    virtual sBool hasBeenInit();

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
    const SRobotParsedData *robot_;

    /** The parent robot to which this actuator is attached */
    const SRobotIOData *robot_io_ds_;

    /** The parsed muscle specification */
    const SMuscleSystem *msys_;

    /** The muscle in the system */
    const SMuscle *muscle_;

    /** Dynamics specification (to compute robot Jacobians) */
    CDynamicsBase *dynamics_;
  };

} /* namespace scl */
#endif /* CACTUATORMUSCLE_HPP_ */
