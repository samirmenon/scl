/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
