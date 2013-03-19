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
/* \file STaskGcLimitCentering.cpp
 *
 *  Created on: Mar 15, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "STaskGcLimitCentering.hpp"

#include <stdexcept>
#include <iostream>

namespace scl
{

//5mm or 0.005 rad resolution
#define SCL_GCTASK_SPATIAL_RESOLUTION 0.005

  STaskGcLimitCentering::STaskGcLimitCentering():
    spatial_resolution_(SCL_GCTASK_SPATIAL_RESOLUTION)
  { }

  STaskGcLimitCentering::~STaskGcLimitCentering()
  { }

  bool STaskGcLimitCentering::initTaskParams()
  {
    try
    {
      // To test whether the goal position has been achieved.
      spatial_resolution_ = SCL_GCTASK_SPATIAL_RESOLUTION;

      // Initialize the gc vectors to the correct sizes
      q_.setZero(dof_task_);
      dq_.setZero(dof_task_);

      q_goal_.setZero(dof_task_);
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSTaskGcLimitCentering::init() : "<<e.what();
      return false;
    }
    return true;
  }
}
