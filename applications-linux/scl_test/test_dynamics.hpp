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
/* \file test_dynamics.hpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef TEST_DYNAMICS_HPP_
#define TEST_DYNAMICS_HPP_


#include <string>

namespace scl_test
{
  /**
   * Tests the performance of the tao dynamics engine:
   *
   * Reads in a toy robot specification and lets it fall under gravity
   * with full dynamics.
   */
  void test_dynamics(int id, const std::string &file_name);

  /** Tests the performance of analytical dynamics implementations in scl for
   * RPP bot and compares them with the tao dynamics implementation.
   *
   * Tests tao's accuracy. */
  void test_dynamics_tao_vs_analytic_rpp(int id);

  /** Tests the performance of analytical dynamics implementations in scl for
   * RPP bot and compares them with the scl dynamics implementation. */
  void test_dynamics_scl_vs_analytic_rpp(int id);
}
#endif /* TEST_DYNAMICS_HPP_ */

