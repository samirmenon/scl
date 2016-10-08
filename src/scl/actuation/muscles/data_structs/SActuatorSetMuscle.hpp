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
/* \file  SActuatorSetMuscle.hpp
 *
 *  Created on: Aug 18, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SACTUATORSETMUSCLE_HPP_
#define SACTUATORSETMUSCLE_HPP_

#include <scl/data_structs/SObject.hpp>

//Needs to access parent robot kinematics
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SActuatorSetMuscleParsed.hpp>
#include <scl/dynamics/CDynamicsBase.hpp>

namespace scl
{
  class SActuatorSetMuscle : public SActuatorSetBase
  {
  public:
    /* ******************************************************
     *                        Data
     * ****************************************************** */
    /** The parsed muscle specification */
    const SActuatorSetMuscleParsed *mset_parsed_;

    /* ******************************************************
     *                        Initialization
     * ****************************************************** */
    bool init(const SActuatorSetMuscleParsed * arg_mset_parsed)
    {
      if(NULL == arg_mset_parsed){ return false; }
      if(false == arg_mset_parsed->hasBeenInit()){ return false; }

      // Save a pointer to the parsed data.
      // NOTE TODO : Perhaps reconsider this? Might want to pass both around, where needed.
      mset_parsed_ = arg_mset_parsed;

      // Initialize the data to zero.
      force_actuator_.setZero(arg_mset_parsed->muscles_.size());
      force_gc_des_.setZero(arg_mset_parsed->robot_->dof_);
      J_.setZero(arg_mset_parsed->muscles_.size(),arg_mset_parsed->robot_->dof_);

      has_been_init_ = true;
      return true;
    }

    /** Default constructor. Sets stuff to NULL. */
    SActuatorSetMuscle() : SActuatorSetBase("SActuatorSetMuscle"),
        mset_parsed_(NULL){}

    /** Default destructor. Does nothing. */
    virtual ~SActuatorSetMuscle(){}
  };

} /* namespace scl */
#endif /* SACTUATORSETMUSCLE_HPP_ */
