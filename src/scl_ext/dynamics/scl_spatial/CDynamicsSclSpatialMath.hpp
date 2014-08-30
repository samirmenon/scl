/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/*
 * CDynamicsSclSpatialMath.hpp
 *
 *  Created on: Jun 9, 2014
 *      Author: Nayan Singhal <singhalnayan91@gmail.com>
 *              Brains in Silicon Lab,
 *              Stanford University.
 */

#ifndef CDYNAMICSSCLSPATIALMATH_HPP_
#define CDYNAMICSSCLSPATIALMATH_HPP_


#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/data_structs/SRigidBodyDyn.hpp>
#include <scl/robot/data_structs/SRobot.hpp>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <stack>
#include <iostream>


namespace scl_ext
{
  // ****************************************************
  //           Compute subspace motion and force constraint
  // ****************************************************

  /**Calculate force subspace matrix */
  bool calculateForceSubspace(/** motion subspace matrix */
      Eigen::MatrixXd &arg_subspace,
      /** return force subspace_matrix */
      Eigen::MatrixXd &ret_force_subspace);

  /** Calculate transformation matrix and motion subspace matrix */
  bool calculateTransformationAndSubspace( /** return Individual transformation matrix from one link to another*/
      Eigen::MatrixXd & ret_Xlink,
      /** return motion subspace matrix */
      Eigen::MatrixXd & ret_subspace,
      /** Individual joint type */
      scl::sInt arg_joint_type,
      /** Individiual joint postion */
      scl::sFloat arg_q);

  /** Calculate Spatial Inertia & Individual Transformation Matrix within joint */
  bool calculateTransformationAndInertia( /** Individual link Jacobians, and composite inertial,
            centrifugal/coriolis gravity estimates. */
      scl::SGcModel *arg_gc_model);

  /** Calculate tree processing order */
  bool calculateOrderOfProcessing ( /** Individual link Jacobians, and composite inertial,
            centrifugal/coriolis gravity estimates. */
      scl::SGcModel *arg_gc_model,
      /** return tree processing order */
      std::vector <std::string> &ret_processing_order );

  // ****************************************************
  //           Compute coordinate Transformation
  // ****************************************************

  /** Transformation around x axis */
  bool computeRotXAxis(/** Transformation matrix from one link to other */
      Eigen::MatrixXd & ret_transform,
      /** joint velocity of link */
      scl::sFloat arg_q);

  /** Transformation around y axis */
  bool computeRotYAxis(/** Transformation matrix from one link to other */
      Eigen::MatrixXd & ret_transform,
      /** joint velocity of link */
      scl::sFloat arg_q);

  /** Transformation around z axis */
  bool computeRotZAxis(/** transformation matrix from one link to other */
      Eigen::MatrixXd & ret_transform,
      /** joint velocity of link */
      scl::sFloat arg_q);

  /** Translation from one line position to another link */
  bool computeTranslation(/** transformation matrix from one link to other */
      Eigen::MatrixXd & ret_transform,
      /** contains the relative location of link with respect to another link */
      Eigen::Vector3d r);

  /** Transformation matrix using quaternion */
  bool computeRotFromQuaternion(/** the returned transformation matrix from quaternion */
      Eigen::MatrixXd &ret_transform,
      /** contains the orientation of link with respect to parent*/
      Eigen::Quaternion<scl::sFloat> arg_ori);

  /** Cross product of spatial velocity matrix */
  bool computeCrossForVelocity(/** the returned cross product of velocity */
      Eigen::MatrixXd &ret_vcross,
      /** spatial velocity of link */
      Eigen::MatrixXd arg_spatial_velocity);

  /** Individual spatial inertia */
  bool calculateSpatialInertia(/** the returned spatial inertia */
      scl::sSpatialXForm &spatial_inertia ,
      /** body's rotational inertia about its center of mass */
      Eigen::Matrix3d arg_inertia,
      /** vector locating the body's center of mass */
      Eigen::Vector3d arg_com,
      /** mass of the body */
      scl::sFloat arg_mass);
}

#endif /* CDYNAMICSSCLSPATIALMATH_HPP_ */
