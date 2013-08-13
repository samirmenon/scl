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
/* \file CTaskNullSpaceDamping.hpp
 *
 *  Created on: Apr 4, 2011
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTASKNULLSPACEDAMPING_HPP_
#define CTASKNULLSPACEDAMPING_HPP_

#include <scl/control/task/CTaskBase.hpp>

#include <scl/control/task/tasks/data_structs/STaskNullSpaceDamping.hpp>

namespace scl
{
  /** This class simply introduces a velocity damping (friction) term
   * into the generalized coordinates */
  class CTaskNullSpaceDamping : public scl::CTaskBase
  {
  public:
    /********************************
     * CTaskBase API
     *********************************/
    /** Computes the gc forces that resist gc velocity */
    virtual bool computeServo(const SRobotSensorData* arg_sensors);

    /** Sets the null space for the next level to zero. Ie.
     * any task below this one in the hierarchy is ignored. */
    virtual bool computeModel();

    /* **************************************************************
     *                   Status Get/Set Functions
     * ************************************************************** */
    /** Return this task controller's task data structure.*/
    virtual STaskBase* getTaskData();

    /********************************
     * Initialization specific functions
     *********************************/
    /** Default constructor : Does nothing   */
    CTaskNullSpaceDamping();

    /** Default destructor : Does nothing.   */
    virtual ~CTaskNullSpaceDamping(){}

    /** Initializes the task object. Required to set output
     * gc force dofs */
    virtual bool init(STaskBase* arg_task_data,
        CDynamicsBase* arg_dynamics);

    /** Resets the task by removing its data.
     * NOTE : Does not deallocate its data structure*/
    virtual void reset();

  private:
    STaskNullSpaceDamping* data_;
  };

}

#endif /* CTASKNULLSPACEDAMPING_HPP_ */
