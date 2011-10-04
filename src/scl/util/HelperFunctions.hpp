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

#ifndef RANDOMFUNCTIONS_HPP_
#define RANDOMFUNCTIONS_HPP_

#include <string>
#include <vector>

/** A random collection of string manipulation, os-specific
 * etc. functions.
 *
 * NOTE TODO : Feel free to add other useful functions...
 */

namespace scl_util
{
  unsigned int countNumbersInString(const char* str);

  bool isStringInVector(const std::string& arg_str,
      const std::vector<std::string>& arg_vec);

  bool getCurrentDir(std::string& arg_cwd);
}

#endif /* RANDOMFUNCTIONS_HPP_ */
