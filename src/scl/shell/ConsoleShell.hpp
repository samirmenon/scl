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
/* \file ConsoleShell.hpp
 *
 *  Created on: Aug 21, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SRC_SCL_ROBOT_CONSOLESHELL_HPP_
#define SRC_SCL_ROBOT_CONSOLESHELL_HPP_

#include <scl/data_structs/SDatabase.hpp>
#include <atomic>

namespace scl
{
  namespace shell
  {
    /** Runs a console shell that enables you to modify
     * the app through your commands (provided you've
     * registered them with the callback registry).
     *
     * NOTE : The console runs in two modes:
     *
     * Mode 1 : String lookup:
     * function name  = string
     * arguments      = vector<string>
     *
     * Mode 2 : Char lookup:
     * function name  = char
     * is_caps?       = bool
     *
     * Default starts in string lookup. Press 'x' to switch between
     * modes.
     *
     * (See the callback registry sutil/CCallbackRegistry.hpp for
     * more details)
     *
     * Note that this will run forever.
     * To terminate the shell, make sure the database->running flag
     * is set to false.
     */
    void runConsoleShell(scl::SDatabase &arg_db);

    /** Tokenizes the char arr into a vector of strings
     * I.e., Splits up the words and places them in a vector.
     */
    void tokenizeCharArr(const char* arg_arr, std::vector<std::string> &ret_vec);
  }

} /* namespace scl */

#endif /* SRC_SCL_ROBOT_CONSOLESHELL_HPP_ */
