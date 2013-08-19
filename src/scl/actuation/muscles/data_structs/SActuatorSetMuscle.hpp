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
#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/dynamics/CDynamicsBase.hpp>

namespace scl
{
  class SActuatorSetMuscle : public SActuatorSetBase
  {
  public:
    /* ******************************************************
     *                        Data
     * ****************************************************** */
    /** The parent robot to which this actuator is attached */
    const SRobotParsedData *robot_;

    /** The parsed muscle specification */
    const SMuscleSystem *msys_;

    /* ******************************************************
     *                        Initialization
     * ****************************************************** */
    /** Default constructor. Sets stuff to NULL. */
    SActuatorSetMuscle() : SActuatorSetBase("SActuatorSetMuscle"),
        robot_(NULL), msys_(NULL){}

    /** Default destructor. Does nothing. */
    virtual ~SActuatorSetMuscle(){}
  };

} /* namespace scl */
#endif /* SACTUATORSETMUSCLE_HPP_ */
