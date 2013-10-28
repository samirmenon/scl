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
/* \file SGraphicsChaiRigidBody.hpp
 *
 *  Created on: Oct 28, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
#ifndef SGRAPHICSCHAIRIGIDBODY_HPP_
#define SGRAPHICSCHAIRIGIDBODY_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/data_structs/SRobotIO.hpp>

/**
 * NOTE : Link with the Chai3D library to get these class
 * specifications.
 */
namespace chai3d
{
  class cGenericObject;
}

namespace scl
{
  /** Scl's chai interface uses this to connect scl and chai objects.
   *
   * This represents a link upon which physics acts. */
  struct SGraphicsChaiRigidBody
  {
  public:
    const SRigidBody* robot_link_;
    chai3d::cGenericObject* graphics_obj_;
    const SRobotIO* io_data_;
    sInt io_data_idx_;

    /** For Satisfying the branching structure's constraints:
     * a) TIdx name_;
     * b) TIdx parent_name_;
     * c) TNode* parent_addr_;
     * d) std::vector<TNode*> child_addrs_; */
    std::string name_, parent_name_;
    SGraphicsChaiRigidBody* parent_addr_;
    std::vector<SGraphicsChaiRigidBody*> child_addrs_;


    SGraphicsChaiRigidBody() :
      robot_link_(S_NULL),
      graphics_obj_(S_NULL),
      io_data_(S_NULL),
      io_data_idx_ (-1),
      name_(""),
      parent_name_(""),
      parent_addr_(NULL)
    { }
    ~SGraphicsChaiRigidBody(){}
  };

  /**
   * This represents a mesh object which doesn't
   * experience any physics.
   */
  struct SGraphicsChaiMesh
  {
  public:
    // Eigen requires redefining the new operator for classes that contain fixed size Eigen member-data.
    // See http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** The rotation from the base frame (0,0,0 ; 0,0,0,0) */
    Eigen::Matrix3d rotation_;

    /** The translation from the base frame (0,0,0 ; 0,0,0,0) */
    Eigen::Vector3d translation_;

    /** The pointer to the xyz graphics library's corresponding
     * graphics object. Be sure to typecast this correctly in
     * any implementation   */
    chai3d::cGenericObject* graphics_obj_;
  };
}

#endif
