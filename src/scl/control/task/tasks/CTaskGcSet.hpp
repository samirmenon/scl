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
/* \file CTaskGcSet.hpp
 *
 *  Created on: Aug 29, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTASKGCSET_HPP_
#define CTASKGCSET_HPP_

#include <scl/DataTypes.hpp>

#include <scl/control/task/CTaskBase.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcSet.hpp>

#include <Eigen/Core>

#include <vector>
#include <string>

namespace scl
{
  /** This task controls a set of generalized coordinates
   * in Gc space. It uses up the range space corresponding
   * to those set of Gcs.
   *
   * It uses the general (TaskGc) data structure but
   * only uses the portions specific to the given Gcs.
   *
   * Possible uses:
   * 1. Set a posture at a specific set of Gcs
   * 2. For a complex robot, prevent a few Gcs from
   *    moving. Eg. Keep one arm/leg still for a humanoid.
   */
  class CTaskGcSet: public scl::CTaskBase
  {
  public:
    /********************************
     * CTaskBase API
     *********************************/
    /** Initializes the task object. Required to set output
     * gc force dofs */
    virtual bool init(STaskBase* arg_task_data,
        CDynamicsBase* arg_dynamics);

    /** Return this task controller's task data structure.*/
    virtual STaskBase* getTaskData();

    /** Resets the task by removing its data.
     * NOTE : Does not deallocate its data structure*/
    virtual void reset();

    /** Computes the gc forces that resist gc velocity */
    virtual bool computeServo(const SRobotSensorData* arg_sensors);

    /** Sets the null space for the next level to zero. Ie.
     * any task below this one in the hierarchy is ignored. */
    virtual bool computeModel();

    /********************************
     * CTaskGcSet specific functions
     *********************************/
    /** Default constructor : Does nothing   */
    CTaskGcSet();

    /** Default destructor : Does nothing.   */
    virtual ~CTaskGcSet(){}

  private:
    STaskGcSet* data_;
    /** These are the generalized coordinates selected on the
     * basis of their name */
    Eigen::VectorXd selected_gcs_;
  };

} /* namespace scl */
#endif /* CTASKGCSET_HPP_ */
