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
/* \file test_controller.hpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef TEST_CONTROLLER_HPP_
#define TEST_CONTROLLER_HPP_

#include <string>

namespace scl_test
{
  /**
   * Tests the performance of the control servo loop:
   *
   * Constructs a dummy 6dof robot with k tasks
   */
  void test_control_servo(int id);

  /**
   * Tests the performance of the controller's dynamics
   * engine:
   *
   * Reads in a toy robot specification and controls
   * it with full dynamics.
   */
  void test_controller_dynamics(int id, const std::string &file_name);
}
#endif /* TEST_CONTROLLER_HPP_ */
