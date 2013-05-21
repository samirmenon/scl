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
/* \file CTrajectoryGenerator.hpp
 *
 *  Created on: Apr 12, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CTRAJECTORYGENERATOR_HPP_
#define CTRAJECTORYGENERATOR_HPP_

#include <scl/DataTypes.hpp>
#include <scl/util/sutil::CMappedList.hpp>
#include <scl/util/FileFunctions.hpp>

#include <scl/control/trajectory/OTG/TypeIOTG.h>

#include <Eigen/Core>
#include <string>

namespace scl
{
  /** A base class for trajectory tracking tasks.
   * Reads a trajectory from file or generates
   * trajectories on the fly.
   *
   * Use this to build trajectory controllers.*/
  template <sUInt task_dof_>
  class CTrajectoryGenerator
  {
  public:
    CTrajectoryGenerator();
    virtual ~CTrajectoryGenerator();

    /** Initializes the trajectory related parameters */
    virtual sBool initTraj(
        /** The cycle time of the control loop */
        const sFloat arg_cycle_time,
        /** The current position along the task's directions */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_max_pos,
        /** The maximum velocity along the task's directions */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_max_vel,
        /** The maximum acceleration along the task's directions */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_max_acc,
        /** The minimum position along the task's directions */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_min_pos,
        /** The minimum velocity along the task's directions */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_min_vel,
        /** The minimum acceleration along the task's directions */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_min_acc);

    /** Reads a trajectory and checks the trajectory for
     * errors:
     * 1. Whether the trajectory dimensions match the task's
     * dofs
     * 2. Whether the trajectory has a time field
     *
     * Assumes : A trajectory in a text file. One time
     * slice per line.
     */
    virtual sBool setTrajFromTxtFile(
        /** The file to read in a trajectory from */
        const std::string& arg_file,
        /** The number of time slices (waypoints) along the trajectory */
        const sLongLong arg_traj_slices,
        /** Is the first collumn of the trajectory time? */
        const sBool arg_has_time=true);

    /** Generates an arbitrary function based trajectory
     * along all the dofs.
     *
     * Verifies whether the trajectory is achievable.
     * Returns false if the trajectory is impossible to achieve. */
    virtual sBool setTrajFromFunc(sFloat (*arg_func) (sFloat),
        /** The number of time slices of the required trajectory */
        const sLongLong arg_traj_slices,
        /** The time in which each trajectory slice should be achieved */
        const sFloat arg_time_per_slice,
        /** The starting position along the task's directions */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_pos_start,
        /** Scales each successive dof by this value.
         * Ie. dof_i = sin(t)*arg_scale(i); */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_scale,
        /** Staggers each successive dof by this value.
         * Ie. dof_i = sin(t)+arg_stagger(i); */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_stagger);

    /** Generates an arbitrary function based trajectory
     * along all the dofs.
     *
     * Verifies whether the trajectory is achievable.
     * Returns false if there is no current goal (ie. traj is over). */
    virtual sBool getCurrGoal(Eigen::Matrix<sFloat,task_dof_,1>& arg_pos_curr,
        Eigen::Matrix<sFloat,task_dof_,1>& arg_vel_curr,
        Eigen::Matrix<sFloat,task_dof_,1>& arg_acc_curr);

    /** Logs a step in the trajectory. Takes the passed configuration
     * and stores it in the achieved trajectory, indexed by a timestamp. */
    virtual sBool saveCurrState(
        /** The present config measured by the sensors */
        const Eigen::Matrix<sFloat,task_dof_,1>& arg_state,
        /** The present time */
        const sFloat arg_time);

    /** Returns the stored trajectory so far in a mapped list indexed
     * by the time at which the config was logged.
     * Returns false if nothing has been logged yet. */
    virtual sBool getLoggedPositions(
        sutil::CMappedList<sFloat, Eigen::Matrix<sFloat,task_dof_,1> >& ret_traj);

  protected:
    /** The desired trajectory, indexed by time or numerical index */
    sutil::CMappedList<sFloat, Eigen::Matrix<sFloat,task_dof_,1> > traj_desired_;

    /** The achieved trajectory, indexed by time */
    sutil::CMappedList<sFloat, Eigen::Matrix<sFloat,task_dof_,1> > traj_achieved_;

    /** The current trajectory position. Index into mapped list. */
    Eigen::Matrix<sFloat,task_dof_,1>* traj_curr_pos_;
    /** The current trajectory velocity. Computed on the fly. */
    Eigen::Matrix<sFloat,task_dof_,1> traj_curr_vel_;
    /** The current trajectory acceleration. Computed on the fly. */
    Eigen::Matrix<sFloat,task_dof_,1> traj_curr_acc_;

    /** The current trajectory position. Index into mapped list. */
    Eigen::Matrix<sFloat,task_dof_,1>* traj_curr_pos_;
    /** The current trajectory velocity. Computed on the fly. */
    Eigen::Matrix<sFloat,task_dof_,1> traj_curr_vel_;
    /** The current trajectory acceleration. Computed on the fly. */
    Eigen::Matrix<sFloat,task_dof_,1> traj_curr_acc_;

    /** The current trajectory time */
    sFloat traj_curr_time_;
    /** The current trajectory index */
    sLongLong traj_curr_idx_;

    Eigen::Matrix<sFloat,task_dof_,1> max_pos_, max_vel_, max_acc_,
    min_pos_, min_vel_, min_acc_;

    TypeIOTG otg_;

    sBool traj_has_been_init_;

  public:
    //Contains fixed sized eigen matrices
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


  //NOTE TODO : Don't really like initializing stuff in the constructor.
  //Ask Torsten to update his OTG API and move stuff to an init function.
  //Else hack it yourself.
  template <sUInt task_dof_>
  CTrajectoryGenerator<task_dof_>::CTrajectoryGenerator() :
  otg_(task_dof_)
  { traj_has_been_init_ = false;  }

  template <sUInt task_dof_>
  CTrajectoryGenerator<task_dof_>::~CTrajectoryGenerator()
  { }

  template <sUInt task_dof_>
  sBool CTrajectoryGenerator<task_dof_>::initTraj(
      /** The cycle time of the control loop */
      const sFloat arg_cycle_time,
      /** The current position along the task's directions */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_max_pos,
      /** The maximum velocity along the task's directions */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_max_vel,
      /** The maximum acceleration along the task's directions */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_max_acc,
      /** The minimum position along the task's directions */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_min_pos,
      /** The minimum velocity along the task's directions */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_min_vel,
      /** The minimum acceleration along the task's directions */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_min_acc)
  {
    try
    {
      //Some error checks.
      if(0 >= arg_cycle_time)
      { throw(std::runtime_error("Can't generate a trajectory for negative or zero servo cycle time.")); }
      if(0 >= task_dof_)
      { throw(std::runtime_error("Can't generate a trajectory for zero task dofs.")); }
      if(arg_max_pos.size() != static_cast<sInt>(task_dof_))
      { throw(std::runtime_error("Max position vector doesn't match task dimensions.")); }
      if(arg_max_vel.size() != static_cast<sInt>(task_dof_))
      { throw(std::runtime_error("Max velocity vector doesn't match task dimensions.")); }
      if(arg_max_acc.size() != static_cast<sInt>(task_dof_))
      { throw(std::runtime_error("Max acceleration vector doesn't match task dimensions.")); }
      if(arg_min_pos.size() != static_cast<sInt>(task_dof_))
      { throw(std::runtime_error("Min position vector doesn't match task dimensions.")); }
      if(arg_min_vel.size() != static_cast<sInt>(task_dof_))
      { throw(std::runtime_error("Min velocity vector doesn't match task dimensions.")); }
      if(arg_min_acc.size() != static_cast<sInt>(task_dof_))
      { throw(std::runtime_error("Min acceleration vector doesn't match task dimensions.")); }

      otg_.SetCycleTimeInSeconds(arg_cycle_time);

      max_pos_ = arg_max_pos;
      max_vel_ = arg_max_vel;
      max_acc_ = arg_max_acc;
      min_pos_ = arg_min_pos;
      min_vel_ = arg_min_vel;
      min_acc_ = arg_min_acc;

      traj_has_been_init_ = true;
    }
    catch(std::exception& e)
    { std::cout<<"\nCTrajectoryGenerator::initTraj() Error : "<<e.what(); }

    return traj_has_been_init_;
  }

  template <sUInt task_dof_>
  sBool CTrajectoryGenerator<task_dof_>::setTrajFromTxtFile(
      /** The file to read in a trajectory from */
      const std::string& arg_file,
      /** The number of time slices (waypoints) along the trajectory */
      const sLongLong arg_traj_slices,
      /** Is the first collumn of the trajectory time? */
      const sBool arg_has_time)
  {
    bool flag;
    try
    {
      Eigen::MatrixXd traj;

      if(arg_has_time)
      {
        traj.resize(arg_traj_slices,task_dof_+1);
        flag = scl_util::readEigenMatFromFile(traj,
            arg_traj_slices,task_dof_+1,arg_file);
        if(false==flag)
        { throw(std::runtime_error("Could not read trajectory and time from the file."));  }

        //Load the trajectory into the pilemap
        for(sUInt i = 0; i< arg_traj_slices; ++i)
        {
          Eigen::Matrix<sFloat,task_dof_,1> t = traj.row(i);
          throw(std::runtime_error("Traj row needs to go from 1 to end"));
          Eigen::Matrix<sFloat,task_dof_,1> * slice = traj_desired_.create(traj(i,0), t);
          if(S_NULL == slice)
          {
            traj_desired_.clear();
            std::stringstream ss;
            ss<<i;
            std::string s("Could not allocate trajectory's slice on the pilemap. At slice : ");
            s = s + ss.str();
            throw(std::runtime_error(s.c_str()));
          }
        }
      }
      else
      {
        traj.resize(arg_traj_slices,task_dof_);
        flag = scl_util::readEigenMatFromFile(traj,
            arg_traj_slices,task_dof_,arg_file);
        if(false==flag)
        { throw(std::runtime_error("Could not read trajectory from the file."));  }

        //Load the trajectory into the pilemap
        for(sUInt i = 0; i< arg_traj_slices; ++i)
        {
          Eigen::Matrix<sFloat,task_dof_,1> * slice = traj_desired_.create(i, traj.row(i));
          if(S_NULL == slice)
          {
            traj_desired_.clear();
            throw(std::runtime_error("Could not allocate trajectory's slice on the pilemap."));
          }
        }
      }

      return true;
    }
    catch(std::exception& e)
    {
      std::cout<<"\nCTrajectoryGenerator::readTrajFromTxtFile() Error : "<<e.what();
      return false;
    }
  }

  template <sUInt task_dof_>
  sBool CTrajectoryGenerator<task_dof_>::setTrajFromFunc(sFloat (*arg_func) (sFloat),
      /** The number of time slices of the required trajectory */
      const sLongLong arg_traj_slices,
      /** The time in which each trajectory slice should be achieved */
      const sFloat arg_time_per_slice,
      /** The starting position along the task's directions */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_pos_start,
      /** Scales each successive dof by this value.
       * Ie. dof_i = sin(t)*arg_scale(i); */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_scale,
      /** Staggers each successive dof by this value.
       * Ie. dof_i = sin(t)+arg_stagger(i); */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_stagger)
  {
    try
    {
      if(arg_pos_start.size() != static_cast<sInt>(task_dof_))
      { throw(std::runtime_error("Start position vector doesn't match task dimensions.")); }
      if(arg_scale.size() != static_cast<sInt>(task_dof_))
      { throw(std::runtime_error("Scale vector doesn't match task dimensions.")); }
      if(arg_stagger.size() != static_cast<sInt>(task_dof_))
      { throw(std::runtime_error("Stagger vector doesn't match task dimensions.")); }

      sFloat time=0.0;

      Eigen::Matrix<sFloat,task_dof_,1> slice;
      slice = arg_pos_start;

      for(sUInt i=0; i<arg_traj_slices;++i)
      {
        //Compute the next step according to the given function
        sFloat x = arg_func(time);
        slice = arg_pos_start + arg_scale*x + arg_stagger;

        //Store the trajectory in the pilemap.
        Eigen::Matrix<sFloat,task_dof_,1> * vec = S_NULL;
        vec = traj_desired_.create(time,slice);
        if(S_NULL==vec)
        { throw(std::runtime_error("Could not create trajectory slice on the pile.")); }

        time+=arg_time_per_slice;//Increment the time
      }
      return true;
    }
    catch(std::exception& e)
    {
      std::cout<<"\nCTrajectoryGenerator::setTrajFromFunc() Error : "<<e.what();
      return false;
    }
  }


  template <sUInt task_dof_>
  sBool CTrajectoryGenerator<task_dof_>::getCurrGoal(
      Eigen::Matrix<sFloat,task_dof_,1>& arg_pos_curr,
      Eigen::Matrix<sFloat,task_dof_,1>& arg_vel_curr,
      Eigen::Matrix<sFloat,task_dof_,1>& arg_acc_curr)
  {
//    for(int i=0;i<task_dof_;++i)
//    {
//      otg_.GetNextMotionState_Position();
//    }
  }

  template <sUInt task_dof_>
  sBool CTrajectoryGenerator<task_dof_>::saveCurrState(
      /** The present config measured by the sensors */
      const Eigen::Matrix<sFloat,task_dof_,1>& arg_state,
      /** The present time */
      const sFloat arg_time)
  {
    Eigen::Matrix<sFloat,task_dof_,1> * vec = S_NULL;
    //Adds an entry to the trajectory logs
    vec = traj_achieved_.create(arg_time,arg_state);
    if(S_NULL == vec)
    { return false; }
    else
    { return true;  }
  }

}

#endif /* CTrajectoryGenerator_HPP_ */
