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
/* \file SNonControlTaskBase.hpp
 *
 *  Created on: Jun 30, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SNONCONTROLTASKBASE_HPP_
#define SNONCONTROLTASKBASE_HPP_

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SObject.hpp>
#include <string>
#include <vector>

namespace scl
{
  //Forward declaration allows pointers to the parent controller
  class STaskController;

  /**
   * Contains all the data required to compute some (arbitrary)
   * non-control related computation.
   *
   * For instance a logging task could log a ton of data to
   * a stream. Or an I/O task could send the latest model
   * details to a remote location that can render them.
   *
   * Any arbitrary task works as long as it doesn't require some
   * sort of interference with the controller..
   *
   * NOTE : YOU CAN NOT USE THIS TASK DATA STRUCTURE DIRECTLY.
   * You must subclass it, re-implement the function:
   *    virtual bool initTaskParams()
   * to parse all the custom parameters (from the xml file)
   * stored in
   *    task_nonstd_params_.
   * The function must then return true.
   *
   * NOTE TODO : ^^^ Why in the world wouldn't we set the func to 0
   * then
   */
  class SNonControlTaskBase : public SObject
  {
  public:
    /** The parent controller */
    const STaskController* parent_controller_;

    /** The type of the task */
    std::string type_task_;

    /** Whether the task is active or inactive
     * Default = false.
     * True after init() */
    scl::sBool has_been_activated_;

    /** A set of additional options that may be used by users to
     * initialize this specific task. These typically go above and
     * beyond the standard options.
     *
     * Eg.The parent link and the pos in parent, which are required by op
     *    point tasks but not by gc tasks. These will be stored like:
     *    task_options_[0].data_[0] = "parent_link";
     *    task_options_[0].data_[1] = "base";
     *    task_options_[1].data_[0] = "pos_in_parent";
     *    task_options_[1].data_[1] = "0.0 0.0 0.0";
     *    */
    std::vector<sString2> task_nonstd_params_;

    /** Constructor */
    SNonControlTaskBase();

    /** Initialization function.
     *
     * NOTE : This function also activates the task. */
    bool init(const std::string & arg_name,
        const std::string & arg_type,
        /** These are ignored during SNonControlTaskBase initialization.
         * However, subclasses may choose to use them and/or
         * require various values */
        const std::vector<scl::sString2>& arg_nonstd_params);

    /** Sets the parent controller */
    bool setParentController(const STaskController* arg_parent);

    /** Processes the task's non standard parameters, which the
     * init() function stores in the task_nonstd_params_.
     *
     * NOTE: This function is called by init() and must be implemented
     * by all subclasses. Else it will be impossible to initialize
     * the task. Ie. init() will always return false. */
    virtual bool initTaskParams()
    { return false; }
  };

}
#endif /* SNONCONTROLTASKBASE_HPP_ */
