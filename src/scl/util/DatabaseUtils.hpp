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
/* \file DatabaseUtils.hpp
 *
 *  Created on: Aug 4, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef DATABASEUTILS_HPP_
#define DATABASEUTILS_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/control/task/data_structs/SControllerMultiTask.hpp>

#include <string>

namespace scl_util
{

  /** Prints a robot link tree with a passed link as its root. */
  void printRobotLinkTree(const scl::SRigidBody &link, scl::sInt depth);

  /** In addition to the default initMultiTaskCtrlDsFromParsedTasks(), this function
   * also parses some extra options if they exist.
   *
   * It also helps split the code for the task parsing from the param parsing (potentially
   * easier to read for some).
   *
   * Sometimes parameters may be provided as strings and might require dynamic type
   * resolution. As such, we will parse them here... */
  int initMultiTaskCtrlDsFromParsedTasks(
        const std::vector<scl::STaskBase*> &arg_taskvec,
        const std::vector<scl::SNonControlTaskBase*> &arg_taskvec_nc,
        const std::vector<scl::sString2> &arg_ctrl_params,
        scl::SControllerMultiTask& ret_ctrl);

  /** Checks whether dynamic type information is available. If so, it parses
   * tasks into the control data structure
   *
   * NOTE : This would have been part of the core data structure except for the
   * need for dynamic typing, which is useful for parsing user specified tasks. */
  int initMultiTaskCtrlDsFromParsedTasks(
      const std::vector<scl::STaskBase*> &arg_taskvec,
      const std::vector<scl::SNonControlTaskBase*> &arg_taskvec_nc,
      scl::SControllerMultiTask& arg_ctrl);

  /** Initializes a dynamic tree given a static tree for a robot. */
  bool initDynRobotFromParsedRobot(sutil::CMappedTree<std::string, scl::SRigidBodyDyn>& arg_rbd_tree,
      const sutil::CMappedTree<std::string, scl::SRigidBody>& arg_rb_tree);

  /** Checks if a muscle system is compatible with a given robot. Looks for
   * both in the database */
  scl::sBool isMuscleCompatWithRobot(const std::string& arg_msys,
      const std::string& arg_robot);
}

#endif /* DATABASEUTILS_HPP_ */
