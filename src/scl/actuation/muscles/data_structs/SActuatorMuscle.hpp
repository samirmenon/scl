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
 * \file SActuatorMuscle.hpp
 *
 *  Created on: Aug 18, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SACTUATORMUSCLE_HPP_
#define SACTUATORMUSCLE_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>

#include <sutil/CMappedList.hpp>

#include <Eigen/Core>

namespace scl
{

  /** Contains the information required to implement the muscle
   * model and API   */
  class SActuatorMuscle : public SObject
  {
  public:
    /* ******************************************************
     *                   Helper Class
     * ****************************************************** */
    class SViaPointSet
    {
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
      const SRigidBodyDyn* rigid_body_dyn_0_, *rigid_body_dyn_1_;

      /** Is one of the links root. */
      bool is_root_0_, is_root_1_;

      /** Child link in branching representation = {0,1} */
      int child_link_id_;

      /** The generalized coordinate spanned. This determines the columns
       * of the Jacobian that we will pay attention to. Only the actuated
       * gc columns are relevant while computing the muscle Jacobian. */
      std::vector<sUInt> scl_gc_id_;

      /** Default constructor. Sets stuff to null */
      SViaPointSet() :
        parent_link_0_(""), parent_link_1_(""),
        rigid_body_dyn_0_(NULL), rigid_body_dyn_1_(NULL),
        is_root_0_(false),is_root_1_(false),
        child_link_id_(-1) {}
    };

    /* ******************************************************
     *                           Data
     * ****************************************************** */
    /** The set of generalized coordinates spanned by this muscle */
    sutil::CMappedList<sUInt,SViaPointSet> via_point_set_;

    /** The generalized coordinate values */
    Eigen::VectorXd q_curr_, dq_curr_;

    /* ******************************************************
     *                   Initialization Functions
     * ****************************************************** */
    /** Default constructor. Sets stuff to null. Sets SObject type. */
    SActuatorMuscle() : SObject("SActuatorMuscle") {}

    /** Default destructor. Does nothing. */
    virtual ~SActuatorMuscle() {}
  };

} /* namespace scl */
#endif /* SACTUATORMUSCLE_HPP_ */
