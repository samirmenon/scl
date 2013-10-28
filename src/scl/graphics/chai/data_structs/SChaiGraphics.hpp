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
/* \file SChaiGraphics.hpp
 *
 *  Created on: Aug 6, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//NOTE TODO : TMP. Remove this #define later
#define SCL_USE_CHAI_GRAPHICS 1
//END TODO

#ifdef SCL_USE_CHAI_GRAPHICS

#ifndef SCHAIGRAPHICS_HPP_
#define SCHAIGRAPHICS_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/data_structs/SRobotIOData.hpp>
#include <scl/actuation/muscles/data_structs/SActuatorSetMuscle.hpp>

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

/** Wbc's chai interface uses this to connect scl and chai objects.
 *
 * This represents a link upon which physics acts. */
struct SGraphicsPhysicalLink
{
public:
  const SRigidBody* robot_link_;
  chai3d::cGenericObject* graphics_obj_;
  const SRobotIOData* io_data_;
  sInt io_data_idx_;

  /** For Satisfying the branching structure's constraints:
   * a) TIdx name_;
   * b) TIdx parent_name_;
   * c) TNode* parent_addr_;
   * d) std::vector<TNode*> child_addrs_; */
  std::string name_, parent_name_;
  SGraphicsPhysicalLink* parent_addr_;
  std::vector<SGraphicsPhysicalLink*> child_addrs_;


  SGraphicsPhysicalLink() :
    robot_link_(S_NULL),
    graphics_obj_(S_NULL),
    io_data_(S_NULL),
    io_data_idx_ (-1),
    name_(""),
    parent_name_(""),
    parent_addr_(NULL)
  { }
  ~SGraphicsPhysicalLink(){}
};

/**
 * This represents a mesh object which doesn't
 * experience any physics.
 */
struct SGraphicsMesh
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

/** The graphics representation for rendered muscles */
struct SGraphicsMsys : public SObject
{
  /** The graphics representation for one muscle */
  struct SGraphicsMuscle
  {
    /** The graphics representation for one muscle point */
    struct SGraphicsMusclePoint
    {//Chai's GPL license might now allow us to directly link. Hence have to separate stuff.
      //Hence use forward decls and pointers. Should have used static members instead
      chai3d::cVector3d* pos_;
      SGraphicsPhysicalLink* graphics_parent_;//Access the chai and scl objects

      chai3d::cVector3d* pos_next_;
      SGraphicsPhysicalLink* graphics_parent_next_;//Access the chai and scl objects

      chai3d::cGenericObject* graphics_via_point_;
      chai3d::cGenericObject* graphics_via_line_;

      SGraphicsMusclePoint()
      {
        pos_ = S_NULL; pos_next_ = S_NULL;
        graphics_parent_ = S_NULL; graphics_parent_next_ = S_NULL;
        graphics_via_point_ = S_NULL; graphics_via_line_ = S_NULL;
      }
      ~SGraphicsMusclePoint()
      {//NOTE TODO : Possible memory leak.
        // if(S_NULL!=pos_) { delete pos_;  }
      }
    }; //End of SGraphicsMusclePoint

    /** A set of muscle points to be rendered */
    std::vector<SGraphicsMusclePoint> mpt_;

    /** A pointer to the parsed muscle object */
    const SMuscleParsed * m_parsed_;
  }; // End of : SGraphicsMuscle

  /** A set of muscles to be rendered */
  std::vector<SGraphicsMuscle> msys_;

  /** A link to the muscle system's actuator set. For rendering
   * muscle activation for different motions */
  const SActuatorSetMuscle * muscle_actuator_set_;

  /** Constructor specifies type */
  SGraphicsMsys() : SObject("SGraphicsMsys"),
      muscle_actuator_set_(NULL){}
};

/** Enables passing data between a chai rendering instance and the scl control framework.
 *
 * Chai basically uses a set of <rotation matrix,translation vector> objects and
 * renders a mesh with each.
 *
 * NOTE : Chai deletes its own data. */
class SChaiGraphics : public SObject
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
        sutil::CMappedTree<std::string, SGraphicsPhysicalLink>
        > robots_rendered_;

  /** Container for all the mesh objects to be rendered */
  sutil::CMappedList<std::string, SGraphicsMesh> meshes_rendered_;

  /** Container for all the muscle objects to be rendered */
  sutil::CMappedList<std::string, SGraphicsMsys> muscles_rendered_;

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
  SChaiGraphics() : SObject(std::string("SChaiGraphics"))
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

#endif /* SCHAIGRAPHICS_HPP_ */
#endif
