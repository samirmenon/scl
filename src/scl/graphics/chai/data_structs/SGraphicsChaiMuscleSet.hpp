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
/* \file SGraphicsChaiMuscleSet.hpp
 *
 *  Created on: Oct 28, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
#ifndef SGRAPHICSCHAIMUSCLESET_HPP_
#define SGRAPHICSCHAIMUSCLESET_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/actuation/muscles/data_structs/SActuatorSetMuscle.hpp>

/**
 * NOTE : Link with the Chai3D library to get these class
 * specifications.
 */
namespace chai3d
{
  class cGenericObject;
  struct cVector3d;
}

namespace scl
{
  /** The graphics representation for rendered muscles */
  struct SGraphicsChaiMuscleSet : public SObject
  {
    /** The graphics representation for one muscle */
    struct SGraphicsChaiMuscle;

    /** A set of muscles to be rendered */
    std::vector<SGraphicsChaiMuscle> muscle_graphics_set_;

    /** A link to the muscle system's actuator set. For rendering
     * muscle activation for different motions */
    const SActuatorSetMuscle * muscle_actuator_set_parsed_;

    /** Constructor specifies type */
    SGraphicsChaiMuscleSet() : SObject("SGraphicsMsys"),
        muscle_actuator_set_parsed_(NULL){}
  };

  /** The graphics representation for one muscle */
  struct SGraphicsChaiMuscleSet::SGraphicsChaiMuscle
  {
    /** The graphics representation for one muscle point */
    struct SGraphicsChaiMusclePoint;

    /** A set of muscle points to be rendered */
    std::vector<SGraphicsChaiMusclePoint> muscle_graphics_pt_;

    /** A pointer to the parsed muscle object */
    const SMuscleParsed * muscle_parsed_;
  }; // End of : SGraphicsMuscle

  /** The graphics representation for one muscle point */
  struct  SGraphicsChaiMuscleSet::SGraphicsChaiMuscle::SGraphicsChaiMusclePoint
  {//Chai's GPL license might now allow us to directly link. Hence have to separate stuff.
    //Hence use forward decls and pointers. Should have used static members instead
    chai3d::cVector3d* pos_;
    SGraphicsChaiRigidBody* graphics_parent_;//Access the chai and scl objects

    chai3d::cVector3d* pos_next_;
    SGraphicsChaiRigidBody* graphics_parent_next_;//Access the chai and scl objects

    chai3d::cGenericObject* graphics_via_point_;
    chai3d::cGenericObject* graphics_via_line_;

    SGraphicsChaiMusclePoint()
    {
      pos_ = S_NULL; pos_next_ = S_NULL;
      graphics_parent_ = S_NULL; graphics_parent_next_ = S_NULL;
      graphics_via_point_ = S_NULL; graphics_via_line_ = S_NULL;
    }
    ~SGraphicsChaiMusclePoint()
    {//NOTE TODO : Possible memory leak.
      // if(S_NULL!=pos_) { delete pos_;  }
    }
  }; //End of SGraphicsMusclePoint
}

#endif /* SGRAPHICSCHAIMUSCLESET_HPP_ */
