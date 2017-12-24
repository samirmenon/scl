/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
