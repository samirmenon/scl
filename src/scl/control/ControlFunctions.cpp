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
/*
 * \file CControlFunctions.cpp
 *
 *  Created on: Apr 6, 2017
 *
 *  Copyright (C) 2017
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "ControlFunctions.hpp"

namespace scl
{
  namespace control
  {
    /** This function computes the model for all
     * the objects required to do operational space
     * control.
     */
    bool computeOSCModel(
        sutil::CMappedMultiLevelList<std::string, scl::tasks::CTaskBase*> & arg_taskmllist,
        SGcModel &arg_rgcm,
        const SRobotIO &rio,
        const scl::CDynamicsBase &arg_dyn)
    {
      return true;
    }


    /** Compute the operational space control model for a set of tasks.
     */
    bool computeTaskModel(sutil::CMappedMultiLevelList<std::string, scl::tasks::CTaskBase*> & arg_taskmllist,
        const SGcModel &arg_rgcm,
        const SRobotIO &arg_rio,
        const scl::CDynamicsBase &arg_dyn)
    {// We'll iterate over all the tasks in the multi-level mapped list, and get their force contributions.
      for(auto & t : arg_taskmllist)//Compute the control for each task and return false if we fail.
        if(false == t->computeModel(arg_rio.sensors_,arg_rgcm,arg_dyn))
          return false;
      return true;
    }


    /** Compute the operational space control forces for a set of
     * objects that are associated with a controller.
     */
    bool computeOSCControl(sutil::CMappedMultiLevelList<std::string, scl::tasks::CTaskBase*> & arg_taskmllist,
        SGcModel &arg_rgcm,
        const SRobotIO &arg_rio,
        const scl::CDynamicsBase &arg_dyn)
    {
      return true;
    }


    /** Compute the operational space control forces for a set of
     * objects that are associated with a controller.
     */
    bool computeTaskControl(sutil::CMappedMultiLevelList<std::string, scl::tasks::CTaskBase*> & arg_taskmllist,
        const SGcModel &arg_rgcm,
        const SRobotIO &arg_rio,
        const scl::CDynamicsBase &arg_dyn)
    {// We'll iterate over all the tasks in the multi-level mapped list, and get their force contributions.
      for(auto & t : arg_taskmllist)//Compute the control for each task and return false if we fail.
        if(false == t->computeControl(arg_rio.sensors_,arg_rgcm,arg_dyn))
          return false;
      return true;
    }
  } /* namespace control */
} /* namespace scl */
