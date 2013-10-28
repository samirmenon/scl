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
/* \file CGraphicsChai.hpp
 *
 *  Created on: Aug 26, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
#ifndef CGRAPHICSCHAI_HPP_
#define CGRAPHICSCHAI_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SRobotParsed.hpp>

#include <scl/data_structs/SGraphicsParsed.hpp>
#include <scl/graphics/chai/data_structs/SGraphicsChai.hpp>
#include <scl/graphics/CGraphicsBase.hpp>

#include <string>

namespace scl {

/** This is the interface for using chai graphics.
 *
 * Enables rendering arbitrary scl robots. */
class CGraphicsChai : public CGraphicsBase
{
public:
  /********************************************************
   *   Initialization Functions: Sets up a scenegraph, camera
   *   etc..
   *********************************************************/
  /** Initializes a graphics world using the camera information
   * in the database.
   *
   * Returns true  : Successfully created a world
   * Returns false : Failed (or the database's world was invalid). */
  virtual sBool initGraphics(
      /**The name of a graphics instance initialized in the
       * database (parser data)*/
      const std::string & arg_graphics_name);

  /** Adds a robot's meshes to the graphics rendering environment.
   *
   * A robot is defined as:
   * 1. Anything whose dynamics are integrated by the physics simulator
   * 2. Any real world entity subject to the laws of physics
   *
   * * Tree structure built by this function:
   *  chai scenegraph root
   *  (Each child is a scl_util::CBranchingRepresentation<string, SGraphicsPhysicalLink>)
   *         |
   *     Robot Root
   *       /     \
   *   Links..   Links...
   *
   * This tree structure allows selectively accessing the branching representations
   * for different robots, and allows turning them on or off one-by-one.
   *
   * Also, since we maintain different trees (chai, dynamics, parser-data etc..), we
   * need some way to connect nodes between them. Each "SGraphicsChai data_" stores
   * a pilemap of such trees in
   * sutil::CMappedList< string, sutil::CMappedTree<string, SGraphicsPhysicalLink> >
   * where the SGraphicsPhysicalLink object allows accessing links in all the different
   * trees at one place.   */
  virtual sBool addRobotToRender(const std::string& arg_robot);

  /** Removes a robot's meshes from the graphics rendering environment.
   *
   * A robot is defined as:
   * 1. Anything whose dynamics are integrated by the physics simulator
   * 2. Any real world entity subject to the laws of physics */
  virtual sBool removeRobotFromRender(const std::string& arg_robot);

  /** Recursively adds links to the chai scenegraph (used by addRobotToRender() ).
   *
   * 1. Creates a graphics object for a passed SGraphicsPhysicalLink* provided the
   * parser data pointer is already set.
   * 2. Creates child objects for the passed SGraphicsPhysicalLink, sets their parser
   * data pointers, and recursively sets them by calling itself. */
  sBool addRobotLink(SGraphicsChaiRigidBody* arg_robot_link);

  /** Adds a static mesh to render. Indexed by its name.
   *
   * A mesh is defined as anything that DOESN'T obey the laws of
   * physics. It is merely rendered (possibly with collision etc).   */
  virtual sBool addMeshToRender(const std::string& arg_mesh_name,
      const std::string& arg_mesh_file, const Eigen::Vector3d& arg_pos,
      const Eigen::Matrix3d& arg_rot);

  /** Adds a static mesh to render. Indexed by its name.
   *
   * A mesh is defined as anything that DOESN'T obey the laws of
   * physics. It is merely rendered (possibly with collision etc).
   *
   * The parent mesh MUST exist in the graphics world already. */
  virtual sBool addMeshToParentInRender(const std::string& arg_mesh_name,
      const std::string& arg_parent_name,
      const std::string& arg_mesh_file, const Eigen::Vector3d& arg_pos,
      const Eigen::Matrix3d& arg_rot);

  /** Removes a static mesh from the rendered scene. Indexed by its name.
   *
   * A mesh is defined as anything that DOESN"T obey the laws of
   * physics. It is merely rendered (possibly with collision etc). */
  virtual sBool removeMeshFromRender(const std::string& arg_mesh_name);

  /** Scales a mesh along the x, y and z axes if it has been added
   * to the rendering environment */
  sBool scaleMesh(const std::string& arg_mesh_name,
      sFloat arg_x, sFloat arg_y, sFloat arg_z);

  /** Adds a muscle system to the graphics rendering environment
   *
   * A muscle system contains:
   * 1. A set of muscles, each with a set of connection points to certain links.
   * 2. A parent robot to whose links the muscles attach. */
  virtual sBool addMusclesToRender(
      const std::string& arg_robot,
      const std::string& arg_msys,
      const sBool add_musc_via_points);

  /** Adds a muscle system to the graphics rendering environment
   *
   * A muscle system contains:
   * 1. A set of muscles, each with a set of connection points to certain links.
   * 2. A parent robot to whose links the muscles attach. */
  virtual sBool addMusclesToRender(
      const std::string& arg_robot,
      const SMuscleSetParsed& arg_msys,
      const sBool add_musc_via_points);

  /** Removes a muscle system from the graphics rendering environment
   *
   * A muscle system is defined as:
   * 1. A set of line segments, with connection points on a robot links certain links.
   * 2. A parent robot to whose links the muscles attach.
   *
   * NOTE TODO: Implement this.*/
  virtual sBool removeMusclesFromRender(
      const std::string& arg_robot,
      const std::string& arg_msys)
  { return false; }

  /** Adds a sphere to a link on a robot in the rendering environment */
  sBool addSphereToRender(
      const std::string& arg_robot,
      const std::string& arg_link,
      const Eigen::Vector3d& arg_pos,
      const sFloat arg_size=0.01,
      /** Get the object for your use */
      chai3d::cGenericObject** arg_ret_ptr=S_NULL);

  /** Adds a sphere to the rendering environment wrt the global frame */
  sBool addSphereToRender(
      const Eigen::Vector3d& arg_pos,
      /** Reference to a pointer, pass a pointer and get the object for your use */
      chai3d::cGenericObject*& arg_ret_ptr,
      const sFloat arg_size=0.01);

  /** Adds a belted ellipsoid (representing the inertia at an operational point)
   * to the rendering environment wrt the global frame */
  sBool addBeltedEllipsoidToRender(
      const Eigen::Vector3d& arg_pos,
      /** Reference to a pointer, pass a pointer and get the object for your use */
      chai3d::cGenericObject*& arg_ret_ptr,
      const sFloat arg_size=0.01);

  /** Deallocates the chai world. */
  virtual sBool destroyGraphics();

  /********************************************************
   *   Convenience Functions: Will call other functions to
   *   do the real work
   *********************************************************/
  /** Updates the rotation and translation transformations
   * for all the robots' and meshes' chai objects.
   * ie. Updates all the CGenericObjects.. */
  virtual sBool updateGraphics();

  /** Updates the rotation and translation transformations
   * for robots' associated CGenerericObject branching
   * chai representation. */
  virtual sBool updateGraphicsForRobots();

  /** Updates the rotation and translation transformations
   * for meshes' associated CGenerericObject branching
   * chai representation.
   * NOTE : Since meshes don't move themselves, this function
   * does nothing. The meshes might move during user interaction
   * sessions, but other code must update their frame transforms. */
  virtual sBool updateGraphicsForMeshes(){return true;}

  /** Updates the rotation and translation transformations
   * for muscle systems' associated CGenerericObject branching
   * chai representation. */
  virtual sBool updateGraphicsForMuscles();

  /** Default constructor. Sets stuff to NULL. */
  CGraphicsChai() : CGraphicsBase()
  { data_ = S_NULL; data_parsed_ = S_NULL; }

  /** Default destructor. Does nothing */
  virtual ~CGraphicsChai(){}

  SGraphicsChai* getChaiData()
  { return data_; }

  SGraphicsParsed* getParsedData()
  { return data_parsed_;  }

protected:
  SGraphicsChai* data_;
  SGraphicsParsed* data_parsed_;
};

}

#endif /* CGRAPHICS_HPP_ */
