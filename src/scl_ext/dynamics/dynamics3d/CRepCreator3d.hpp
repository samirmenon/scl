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
/* \file CRepCreator3d.hpp
 *
 *  Created on: Jun 21, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CREPCREATOR3D_HPP_
#define CREPCREATOR3D_HPP_

#include <scl/DataTypes.hpp>

#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SRigidBody.hpp>

class cDynamicBase;
class cDynamicLink;
class cDynamicWorld;

namespace scl_ext {

  /** This class translates a scl database entry for a robot
   * into a 3d tree. It does not store any data.
   *
   * This class will ideally be used once at the start of the program
   * to initialize the 3d branching representation.   */
  class CRepCreator3d
  {
  public:
      /** Constructor : Does nothing. Use static function above. */
      CRepCreator3d(){}
      CRepCreator3d(const CRepCreator3d& arg){}
    /** Destructor : Does nothing */
    ~CRepCreator3d(){}

    /** Creates a 3d root structure out of a set of robot definitions.
     * Does not manage the returned memory.
     *
     * input : Robot name
     * output : TODORoot*
     *
     * Returns,
     * NULL : failure
     * Valid TODORoot pointer : success
     *
     * NOTE : Remember to dereference the vectors in TODORoot
     */
    cDynamicBase* c3dRootRepCreator(const scl::SRobotParsed& arg_robot);

    cDynamicWorld * cdw;


  private:


    /** Creates a child 3d node given a robot link with all
     * the required information to initialize the node. */
    bool createChild3dNodes(const scl::SRigidBody* arg_link,
        cDynamicLink* arg_parent_node,
        cDynamicBase* arg_base);

    /**Creates a 3d node for a non-root link given an initialized home frame
     * The node is added to the SRigidBody* structure. */
    cDynamicLink* create3dNonRootNode(const scl::SRigidBody* arg_link,
        cDynamicLink* arg_parent_node,
        cDynamicBase *base_obj);

  };

}

#endif //CREPCREATOR3D_HPP_
