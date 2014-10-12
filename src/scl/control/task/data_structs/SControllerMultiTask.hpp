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
/* \file SControllerMultiTask.hpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCONTROLLERMULTITASK_HPP_
#define SCONTROLLERMULTITASK_HPP_

#include <vector>
#include <list>
#include <string>

#include <scl/data_structs/SObject.hpp>

#include <scl/control/data_structs/SControllerBase.hpp>
#include <scl/control/task/data_structs/SServo.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>
#include <scl/control/task/data_structs/SNonControlTaskBase.hpp>

#include <sutil/CMappedList.hpp>
#include <sutil/CMappedMultiLevelList.hpp>

namespace scl
{

  /** Controller robot data structure. One such object
   * is stored in the database singleton for every controller.  */
  class SControllerMultiTask : public SControllerBase
  {
  public:
    /** Pointer to the Servo DS. */
    SServo servo_;

    /** Pointers to the Task data structures. Organized
     * in the priority order (the outer vector) of the tasks.
     *
     * Tasks can be accessed either by name (map access), pointer (iterator_)
     * in the multi level pilemap or via the vector (std::vector access)*/
    sutil::CMappedMultiLevelList<std::string, STaskBase*> tasks_;

    /** Pointers to the non-control Task data structures.
     *
     * Tasks can be accessed either by name (map access) or pointer (iterator_) */
    sutil::CMappedList<std::string, SNonControlTaskBase*> tasks_non_ctrl_;

    /** The servo to model update rate.
     * NOTE : This is a recommended value since the servo rate and model rate are decided
     * by the application that calls the controller */
    sUInt servo_to_model_rate_;

    /** Inherited stuff:
    std::string robot_name_;
    const SRobotParsed* robot_;
    SRobotIO* io_data_;
    SGcModel* gc_model_;
    std::string name_;
    sBool has_been_init_; */

    /** Constructor sets the initialization state to false */
    SControllerMultiTask();

    /** Destructor does nothing.
     * NOTE : Someone else should delete the tasks. */
    virtual ~SControllerMultiTask();

    /** Initializes the data structure */
    virtual sBool init(const std::string & arg_ctrl_name,
        const SRobotParsed* arg_robot_ds,
        SRobotIO* arg_io_data,
        SGcModel* arg_gc_model=S_NULL,
        const std::string &arg_must_use_robot="",
        /** The ratio of (recommended) servo updates to model updates */
        const sUInt arg_servo_to_model_rate=1
        );
  };
}

#endif /* SCONTROLLERMULTITASK_HPP_ */
