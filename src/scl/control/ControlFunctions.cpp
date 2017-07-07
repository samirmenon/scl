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
#include <cassert>

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

    /** Compute the operational space control model for a set of tasks:
     * Gram-Schmidt orthonormalization
     */
    bool computeTaskRangeProjectionsGS(
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
      sUInt i;//We use i's value after the loop...
      for(i=0; i<arg_taskmllist.getNumPriorityLevels(); ++i)
      {
        arg_range_spaces[i] = tmp; // This level's range is decided already.
        if(i==arg_taskmllist.getNumPriorityLevels()) break; // We're done.

        // Now let's decide the range for the next level (there is one more level).
        // Essentially we'll first find the orthonormal basis for the above
        // levels + this level.

        // ********* STEP 1 : Construct the J' column space so far + next level.
        // Get size of col space for all Jacobians at this level
        cols_in_level = 0;
        for(auto &t : *arg_taskmllist.getSinglePriorityLevelConst(i))
        { cols_in_level += (*t)->getDataConst()->J_.rows(); }

        // The max range space of J' is going to be dofxdof. But we'll
        // need to take all J cols into account so we'll create a larger
        // space here (and shrink it to a suitable size w a tolerance).
        Eigen::MatrixXd Rs(arg_rgcm.dof_robot_,cols_in_level);

        // Get all the cols at this level.
        sUInt cols_set=0; // Value carries over to next loop as well.
        for(auto &t : *arg_taskmllist.getSinglePriorityLevelConst(i))
        {
          Rs.block(0,cols_set,arg_rgcm.dof_robot_,(*t)->getDataConst()->J_.rows()) = (*t)->getDataConst()->J_.transpose();
          cols_set += (*t)->getDataConst()->J_.rows();
        }

        // Get all the cols at the next level.
        for(auto &t : *arg_taskmllist.getSinglePriorityLevelConst(i+1))
        {
          Rs.block(0,cols_set,arg_rgcm.dof_robot_,(*t)->getDataConst()->J_.rows()) = (*t)->getDataConst()->J_.transpose();
          cols_set += (*t)->getDataConst()->J_.rows();
        }

        assert(cols_set == cols_in_level+cols_in_next);

        // ********* STEP 1 : Orthogonalize the J' column space so far + drop cols if required.

      }// We might have exited on finding a gc task. In that case, all subsequent range spaces are zero.
      i=i+1;
      while (i<arg_taskmllist.getNumPriorityLevels())//Range exhausted. Remaining tasks have none.
      { arg_range_spaces[i].setZero(arg_rgcm.dof_robot_,arg_rgcm.dof_robot_); }

      return false;
    }
  } /* namespace control */
} /* namespace scl */


/**
 * // ********* STEP 1 : Construct the J' column space so far + next level.
        // Get size of col space for all Jacobians at this level
        for(auto &t : *arg_taskmllist.getSinglePriorityLevelConst(i))
        { cols_so_far += (*t)->getDataConst()->J_.rows(); }

        // Get size of col space for all Jacobians at next level
        sUInt cols_in_next=0;
        for(auto &t : *arg_taskmllist.getSinglePriorityLevelConst(i+1))
        { cols_in_next += (*t)->getDataConst()->J_.rows(); }

        // The max range space of J' is going to be dofxdof. But we'll
        // need to take all J cols into account so we'll create a larger
        // space here (and shrink it to a suitable size w a tolerance).
        Eigen::MatrixXd Rs(arg_rgcm.dof_robot_,cols_so_far+cols_in_next);

        // Get all the cols at this level.
        sUInt cols_set=0; // Value carries over to next loop as well.
        for(auto &t : *arg_taskmllist.getSinglePriorityLevelConst(i))
        {
          Rs.block(0,cols_set,arg_rgcm.dof_robot_,(*t)->getDataConst()->J_.rows()) = (*t)->getDataConst()->J_.transpose();
          cols_set += (*t)->getDataConst()->J_.rows();
        }

        // Get all the cols at the next level.
        for(auto &t : *arg_taskmllist.getSinglePriorityLevelConst(i+1))
        {
          Rs.block(0,cols_set,arg_rgcm.dof_robot_,(*t)->getDataConst()->J_.rows()) = (*t)->getDataConst()->J_.transpose();
          cols_set += (*t)->getDataConst()->J_.rows();
        }

        assert(cols_set == cols_so_far+cols_in_next);
 */
