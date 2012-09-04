/* Copyright (C) 2011  Samir Menon, Stanford University

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 * SHapticTask.hpp
 *
 *  Created on: Sep 1, 2011
 *      Author: Samir Menon
 */

#ifndef SHAPTICTASK_HPP_
#define SHAPTICTASK_HPP_

#include <scl/control/task/data_structs/STaskBase.hpp>

namespace scl_app
{

  class SHapticTask : public scl::STaskBase
  {
  public:
    SHapticTask(){}
    virtual ~SHapticTask(){}

    /** Processes the task's non standard parameters, which the
     * init() function stores in the task_nonstd_params_.
     *
     * NOTE : This is a sample task and hence doesn't have any non
     * standard params. You may extend it to add yours if you like. */
    virtual bool initTaskParams()
    { return true;  }
  };

} /* namespace scl */
#endif /* SHAPTICTASK_HPP_ */
