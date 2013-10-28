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
/* \file SGraphicsChai.hpp
 *
 *  Created on: Aug 6, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
#ifndef SGRAPHICSCHAI_HPP_
#define SGRAPHICSCHAI_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SRobotIO.hpp>
#include <scl/graphics/chai/data_structs/SGraphicsChaiRigidBody.hpp>
#include <scl/graphics/chai/data_structs/SGraphicsChaiMuscleSet.hpp>

#include <sutil/CMappedList.hpp>
#include <sutil/CMappedTree.hpp>

/**
 * NOTE : Link with the Chai3D library to get these class
 * specifications.
 */
namespace chai3d
{
class cWorld;
class cCamera;
class cGenericObject;
class cMesh;
struct cVector3d;
}

namespace scl
{
/** Enables passing data between a chai rendering instance and the scl control framework.
 *
 * Chai basically uses a set of <rotation matrix,translation vector> objects and
 * renders a mesh with each.
 *
 * NOTE : Chai deletes its own data. */
class SGraphicsChai : public SObject
{
public:
  /**
   * The graphics world :
   * A scenegraph of what will be rendered
   */
  chai3d::cWorld* chai_world_;

  /** A camera that looks at the world. */
  chai3d::cCamera* chai_cam_;

  /** Container for all the robots to be rendered. */
  sutil::CMappedList<std::string,
        sutil::CMappedTree<std::string, SGraphicsChaiRigidBody>
        > robots_rendered_;

  /** Container for all the mesh objects to be rendered */
  sutil::CMappedList<std::string, SGraphicsChaiMesh> meshes_rendered_;

  /** Container for all the muscle objects to be rendered */
  sutil::CMappedList<std::string, SGraphicsChaiMuscleSet> muscles_rendered_;

  /** Gui properties */
  bool mouse_mode_cam_; //manipulate the camera or forces

  sFloat gl_width_, gl_height_;
  bool mouse_button_pressed_;
  sInt mouse_x_, mouse_y_, mouse_button_;

  scl::sFloat mouse_grab_mag_; //magnitude
  Eigen::Vector3d mouse_grab_pos_;
  chai3d::cGenericObject* mouse_grab_obj_;

  /** Running or not */
  sFloat running_;

  /** Constructor : Sets stuff to NULL. */
  SGraphicsChai() : SObject(std::string("SGraphicsChai"))
  {
    chai_cam_ = S_NULL;
    chai_world_ = S_NULL;
    has_been_init_ = false;
    //DEFAULTS
    gl_width_ = 1080;
    gl_height_ = 800;
    running_ = false;

    //Mouse defaults
    mouse_mode_cam_ = true;
    mouse_button_pressed_ = false;
    mouse_x_ = 0;
    mouse_y_ = 0;
    mouse_button_ = 0;

    //For drag and click
    mouse_grab_mag_ = 0.0;
    mouse_grab_pos_ = Eigen::Vector3d::Zero();
    mouse_grab_obj_ = NULL;
  }
};

}

#endif
