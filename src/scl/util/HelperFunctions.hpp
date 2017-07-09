/* This file is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

This file is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
this file. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file HelperFunctions.hpp
 *
 *  Created on: Dec 26, 2010
 *
 *  Copyright (C) 2010, Samir Menon <smenon@stanford.edu>
 */

#ifndef HELPERFUNCTIONS_HPP_
#define HELPERFUNCTIONS_HPP_

#include <string>
#include <vector>

/** A random collection of string manipulation, os-specific
 * etc. functions.
 *
 * NOTE TODO : Feel free to add other useful functions...
 */

namespace scl_util
{
  /** A function that counts the number of words in
   * a string.
   * Input : A const pointer to a string (say a.c_str() for a std::string)
   * Output : -1 (error) 0,1... (the number of words in a string)
   *   E.g. "1 1.2 1" -> 3
   *        "0" -> 1
   *        "bobo is a clown" -> 4
   */
  unsigned int countNumbersInString(const char* arg_str);

  /** Tests if a given string is in a given vector */
  bool isStringInVector(const std::string& arg_str,
      const std::vector<std::string>& arg_vec);

  /** Does what it says */
  bool getCurrentDir(std::string& arg_cwd);

  /** Splits a string at character provided. Returns a vector */
  bool splitString(const std::string &arg_str, const char arg_split_char,
      std::vector<std::string> &ret_vector_splits);
}

#endif /* RANDOMFUNCTIONS_HPP_ */
