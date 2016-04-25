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
 * \file SGcModel.hpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SGCMODEL_HPP_
#define SGCMODEL_HPP_

#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SRigidBodyDyn.hpp>
#include <scl/DataTypes.hpp>

#include <Eigen/Dense>

#include <vector>

namespace scl
{
  /** A data structure to store the joint space model.
   * This model serves as the foundation for all tasks
   * to compute their own models (mass, coriolis/centrifugal
   * and gravity).
   *
   * Each controller data structure contains an object of
   * this type.   */
  class SGcModel : public SObject
  {
    /* *********************************************************************
     *                                 Data
     * ********************************************************************* */
  public:
    // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
    // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** The parent robot for this gc model */
    std::string name_robot_;

    /** M_gc_: Generalized inertia matrix */
    Eigen::MatrixXd M_gc_;

    /** M_gc_inv_ : Generalized inertia matrix inverse */
    Eigen::MatrixXd M_gc_inv_;

    /** force_gc_cc_ : Generalized coriolis+centrifugal force vector */
    Eigen::VectorXd force_gc_cc_;

    /** force_gc_grav_ : Generalized gravity force vector */
    Eigen::VectorXd force_gc_grav_;

    /** The generalized coordinates and velocity at the time
     * the model was last updated. Since the dynamics are completely
     * described by the generalized coordinates and velocities, we
     * don't need to store any higher derivatives.
     *
     * NOTE : These may not match the actual generalized coordinates
     * and velocity due to time-delay introduced while computing
     * the model. Typically, this is irrelevant, but it might start
     * to be substantial when the dynamics are computed at a lower
     * rate or with human-like delays. */
    Eigen::VectorXd q_,dq_;

    /** com : Center of mass vector */
    Eigen::Vector3d pos_com_;

    /** mass_ : Mass of the robot in Euclidean coords */
    sFloat mass_;

    /** A vector of the processing order of all the rigid bodies in the articulated body */
    std::vector<std::string> processing_order_;

    /** bool specifying whther spatial transformation 
     * and inertia is calculated or not */
    bool computed_spatial_transformation_and_inertia_; 

    /** A vector of the dynamics information for all the rigid bodies in the
     * articulated body */
    sutil::CMappedTree<std::string, SRigidBodyDyn> rbdyn_tree_;

    /** A set of vectors to be used as temp vectors during dynamics operations */
    Eigen::VectorXd vec_scratch_[5];

    /** These are used by the constrained dynamics integration */
    Eigen::MatrixXd mat_scratch_n_n[2];
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_scratch_n_n;

    /* *********************************************************************
     *                      Initialization functions
     * ********************************************************************* */
    /** Constructor does nothing */
    SGcModel();

    /** Initialization function sets up the matrix sizes */
    sBool init(const SRobotParsed& arg_robot_data);
  };
}

#endif /* SGCMODEL_HPP_ */
