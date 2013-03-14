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
/* \file CTaoRepCreator.hpp
 *
 *  Created on: May 11, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 

#ifndef CTAOREPCREATOR_HPP_
#define CTAOREPCREATOR_HPP_

#include <scl/DataTypes.hpp>

#include <scl/parser/CParserBase.hpp>
#include <scl/data_structs/SRobotLink.hpp>

#include <scl/dynamics/tao/tao/dynamics/taoNode.h>

namespace scl {

  /** This class translates a scl database entry for a robot
   * into a tao tree. It does not store any data.
   *
   * This class will ideally be used once at the start of the program
   * to initialize the tao branching representation.   */
  class CTaoRepCreator
  {
    /** Constructor : Does nothing */
    CTaoRepCreator(){}
  public:
    /** Destructor : Does nothing */
    ~CTaoRepCreator(){}

    /** Creates a tao root structure out of a set of robot definitions.
     * Does not manage the returned memory.
     *
     * input : Robot name
     * output : taoNodeRoot*
     *
     * Returns,
     * NULL : failure
     * Valid taoNodeRoot pointer : success
     *
     * NOTE : Remember to dereference the vectors in taoNodeRoot
     */
    static taoNodeRoot* taoRootRepCreator(const SRobotParsedData& arg_robot);

  private:

    /** Creates a child tao node given a robot link with all
     * the required information to initialize the node. */
    static bool createChildTaoNodes(const SRigidBody* arg_link,
        taoNode* arg_parent);

    /**Creates a tao node for a non-root link given an initialized home frame
     * The node is added to the SRobotLink* structure. */
    static taoNode* createTaoNonRootNode(const SRigidBody* arg_link,
        const taoNode* arg_parent_node);
  };

}

#endif //CTAOREPCREATOR_HPP_
