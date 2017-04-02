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
/* \file CmdLineArgReader.hpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CMDLINEARGREADER_HPP_
#define CMDLINEARGREADER_HPP_

#include <scl/data_structs/SCmdLineOptions.hpp>

namespace scl
{
  /** Reads the command line arguments into a cmd arg structure.
   * There are many common arguments that we will populate into a
   * data structure.
   */
  bool cmdLineArgReaderOneRobot(int argc, char** argv, SCmdLineOptions_OneRobot &ret_cmd_ds);

}

#endif /* CMDLINEARGREADER_HPP_ */
