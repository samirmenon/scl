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
#include <scl/data_structs/SGraphicsParsed.hpp>
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

/** Simple container object for associating static and dynamic graphics data
 * NOTE TODO: Consider moving this inside SGraphicsChai */
struct SRobotRenderObj
{
public:
  sutil::CMappedTree<std::string, SGraphicsChaiRigidBody> gr_tree_;
  const SRobotIO *io_;
};

/** Enables passing data between a chai rendering instance and the scl control framework.
 *
 * Chai basically uses a set of <rotation matrix,translation vector> objects and
 * renders a mesh with each.
 *
 * NOTE : Chai deletes its own data. */
class SGraphicsChai : public SObject
{
public:
  /** An initialization function. Note, calling this will reset the whole state by default..
   *
   * Q) Why do we support resets without erasing chai state?
   * A) We do support reinitializing this without resetting all the chai objects. This
   * is because we might want to serialize/deserialize scl graphics configurations and reset
   * the views without wanting to directly edit chai variables.
   * In this case, it is useful to simply call init again on the SGraphicsChai object without
   * having to reinitialize the entire chai data (simply an update).
   * This is useful because the parsed graphics data is basically a scene specification. And
   * it doesn't contain misc. added robots, meshes etc. Resetting the chai data would mean that
   * we'd have to re-init all the added robots etc.---a waste of time.
   * */
  bool init(/** The graphics data structure loaded from the xml file. Specifies camera etc. information. */
      const SGraphicsParsed* arg_gr_ds,
      const bool arg_reset_chai_world=true);

  /** Default constructor */
  SGraphicsChai();

  /** Virtual destructor */
  virtual ~SGraphicsChai(){}

  /**
   * The graphics world :
   * A scenegraph of what will be rendered
   */
  chai3d::cWorld* chai_world_ = S_NULL;

  /** A camera that looks at the world. */
  chai3d::cCamera* chai_cam_ = S_NULL;

  /** Container for all the robots to be rendered. */
  sutil::CMappedList<std::string, SRobotRenderObj> robots_rendered_;

  /** Container for all the mesh objects to be rendered */
  sutil::CMappedList<std::string, SGraphicsChaiMesh> meshes_rendered_;

  /** Container for all the muscle objects to be rendered */
  sutil::CMappedList<std::string, SGraphicsChaiMuscleSet> muscles_rendered_;

  /** Gui properties */
  bool mouse_mode_cam_ = true; /// Manipulate the camera or forces
  bool mouse_mode_move_scene_ = false; /// Translate the camera or forces

  sFloat gl_width_ = 1080, gl_height_ = 800; ///The width and height of the default gl instance
  bool mouse_button_pressed_=false; /// Default mouse button press state
  sInt mouse_x_=0, mouse_y_=0, mouse_button_=0; /// Mouse button defaults

  scl::sFloat mouse_grab_mag_=0.0; /// Mouse grab scaling factor
  Eigen::Vector3d mouse_grab_pos_; /// The position at which the mouse grab started
  chai3d::cGenericObject* mouse_grab_obj_=NULL; /// The object grabbed

  /** Running or not */
  sFloat running_=false;
};

}

#endif
