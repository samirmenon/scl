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
/* \file test_robot_controller.hpp
 *
 *  Created on: Jul 25, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef TEST_ROBOT_CONTROLLER_HPP_
#define TEST_ROBOT_CONTROLLER_HPP_

#include <string>

namespace scl_test
{
  /**
   * Tests the performance of the task controller
   * on the given robot specification:
   */
  void test_task_controller(int id, int argc, char** argv,
      const std::string & arg_file,
      const std::string & arg_robot_name,
      const std::string & arg_controller_name,
      const std::string & arg_op_link_name1,
      const std::string & arg_op_link_name2,
      const double arg_sim_dt=0.0001,
      const bool arg_damping = false);
}
#endif /* TEST_CONTROLLER_HPP_ */
