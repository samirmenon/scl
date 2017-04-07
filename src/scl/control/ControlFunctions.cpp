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
        const SRobotIO &arg_rio,
        const scl::CDynamicsBase &arg_dyn)
    {
      std::vector<Eigen::MatrixXd> tmp; tmp.resize(arg_taskmllist.getNumPriorityLevels());
      if(arg_dyn.computeGCModel(&arg_rio.sensors_,&arg_rgcm))
        if(computeTaskModel(arg_taskmllist,arg_rgcm,arg_rio,arg_dyn))
          if(computeTaskRangeProjectionsQR(tmp,arg_taskmllist,arg_rgcm,arg_dyn))
            return true;
      return false;
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

    /** Compute the operational space control model for a set of tasks.
     */
    bool computeTaskRangeProjectionsQR(
        /** A vector of range spaces associated with each task. We'll assume it's been initialized.
         * NOTE TODO : Merge this into a data struct that also contains the task level at which
         * range space reaches zero. This will help optimize code (can ignore further levels).  */
        std::vector<Eigen::MatrixXd> arg_range_spaces,
        /** A multi-level mapped list containing all the tasks. This contains enough information to prioritize tasks. */
        const sutil::CMappedMultiLevelList<std::string, scl::tasks::CTaskBase*> & arg_taskmllist,
        const SGcModel &arg_rgcm,
        const scl::CDynamicsBase &arg_dyn)
    {
      // We expect the range space vector to be initialized.
      if(arg_range_spaces.size() != arg_taskmllist.getNumPriorityLevels()) return false;

      Eigen::MatrixXd tmp;
      // The first level has full range.
      tmp.setIdentity(arg_rgcm.dof_robot_,arg_rgcm.dof_robot_);

      // Find the range space available to each successive layer
      sUInt i;
      for(i=0; i<arg_taskmllist.getNumPriorityLevels(); ++i)
      {
        arg_range_spaces[i] = tmp; // This level's range is decided already.

        // Now let's decide the range for the next level.
        // Essentially we'll first find the orthonormal basis for the above
        // levels + this level. Then run a full QR to get the available range
        // space for the next level (using Gram-schmidt)

        // Iterate over the columns of JT for each task at this level.
        // If they aren't represented in the basis set, then add them.
        for(auto &t : *arg_taskmllist.getSinglePriorityLevelConst(i))
        {
          const scl::tasks::CTaskBase &tsk = **t;
          tsk.getDataConst()->J_;
        }
      }
      i=i+1;
      while (i<arg_taskmllist.getNumPriorityLevels())//Range exhausted. Remaining tasks have none.
      { arg_range_spaces[i].setZero(arg_rgcm.dof_robot_,arg_rgcm.dof_robot_); }

      return false;
    }
  } /* namespace control */
} /* namespace scl */
