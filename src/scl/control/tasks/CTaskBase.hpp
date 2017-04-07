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
/* \file CTaskBase.hpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCL_CONTROL_TASKS_CTASKBASE_HPP_
#define SCL_CONTROL_TASKS_CTASKBASE_HPP_

#include <scl/DataTypes.hpp>

#include "data_structs/STaskBase.hpp"

#include <scl/data_structs/SRobotIO.hpp>

#include <scl/dynamics/CDynamicsBase.hpp>

namespace scl
{
  namespace tasks{
    /** Container class to encapsulate a task model and a task servo.
     *
     * NOTE : Virtual class. Subclass and implement functions
     * that compute the task forces.
     */
    class CTaskBase : public SObject {
    public:
      /* **************************************************************
       *                   Computation Functions
       * ************************************************************** */
      /** Computes the task torques :
       *
       * This is essentially a function of the type:
       *   Fgc_task = f(q,dq,other-sensors)
       *
       * Fgc_task is stored locally in this object's data structure. */
      virtual bool computeControl(
          const SRobotSensors &arg_sensors,
          const SGcModel &arg_gcm,
          const CDynamicsBase &arg_dyn)=0;

      /** Computes the dynamics (task model)
       *
       * This is essentially a set of functions of the type:
       *  Mtask = f(q,A)
       *  gtask = f(q)
       *  etc.
       *
       * The model matrices etc. are stored locally in this object's
       * data structure
       * */
      virtual bool computeModel(
          const SRobotSensors &arg_sensors,
          const SGcModel &arg_gcm,
          const CDynamicsBase &arg_dyn)=0;

      /* **************************************************************
       *                   Status Get/Set Functions
       * ************************************************************** */
      /** Return this task controller's task data structure.
       *   Use it responsibly!
       * NOTE : Use dynamic casts whenever you downcast the data.
       *        And try not to downcast very often. Perf hit. */
      virtual STaskBase* getData()=0;

      /** Return this task controller's task data structure (const).*/
      virtual const STaskBase* getDataConst() const=0;

      /* **************************************************************
       *                   Initialization Functions
       * ************************************************************** */
      /** Constructor does nothing */
      CTaskBase(const std::string &arg_type):
        SObject(arg_type){}

      /** Destructor does nothing */
      virtual ~CTaskBase(){}

      /** Initializes the task object using a set of key:value string pairs */
      virtual bool init(
          const sutil::CMappedList<std::string, std::string>& arg_params,
          const SRobotParsed &arg_rds)=0;

      /** Initializes the task object. Copies values from passed object. */
      virtual bool init(const STaskBase &arg_task_obj)=0;

      /** Initializes the task object. Required to set output
       * gc force dofs
       *
       *  Input : A JSON string that contains all the data required to
       *          initialize the data structure (STaskOpPos). */
      virtual bool init(const std::string &arg_json_ds_string)=0;

      /** Resets the task by removing its data and zeroing all
       * matrices etc. */
      virtual void reset()=0;

    private:
      /** Private default constructor enforces setting
       * type at construction (via the SObjct superclass) */
      CTaskBase();

      /** Private default constructor enforces setting
       * type at construction (via the SObjct superclass) */
      CTaskBase(const CTaskBase & obj);
    };
  }
}

#endif /* SCL_CONTROL_TASKS_CTASKBASE_HPP_ */
