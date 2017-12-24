/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
