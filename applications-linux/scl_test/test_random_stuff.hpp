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
/* \file test_random_stuff.hpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef TEST_RANDOM_STUFF_HPP_
#define TEST_RANDOM_STUFF_HPP_

namespace scl_test
{
  /**
   * Tests the robot parser with the sample Scl format
   *
   * Some interesting facts:
   *  1. function call = ~3e-9 sec (ie. ~3 clock cycles for a GHz proc)
   *  2. Virtual and non-virtual functions take pretty much the same time
   *     (In fact, virtual functions are sometimes faster).
   *  3. Pointer redirects take almost 0 time.
   */
  void test_random_stuff(int id);
}

#endif /* TEST_RANDOM_STUFF_HPP_ */
