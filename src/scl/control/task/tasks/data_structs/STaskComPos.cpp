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
/* \file STaskComPos.cpp
 *
 *  Created on: Sep 2, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "STaskComPos.hpp"

#include <stdexcept>
#include <iostream>

namespace scl
{
  //0.5cm spatial resolution
#define SCL_COMPOSTASK_SPATIAL_RESOLUTION 0.005

  STaskComPos::STaskComPos() :  STaskBase("STaskComPos"),
    spatial_resolution_(SCL_COMPOSTASK_SPATIAL_RESOLUTION)
  { }

  STaskComPos::~STaskComPos()
  { }


  bool STaskComPos::initTaskParams()
  {
    try
    {
      if(3!=dof_task_)//This is a position based op point task
      { throw(std::runtime_error("Com pos tasks MUST have 3 dofs (xyz translation at a point)."));  }

      //Initalize the task data structure.
      pos_in_parent_ = Eigen::Vector3d::Zero();

      //Set task space vector sizes stuff to zero
      x_ = Eigen::Vector3d::Zero();
      dx_ = Eigen::Vector3d::Zero();
      ddx_ = Eigen::Vector3d::Zero();

      x_goal_ = Eigen::Vector3d::Zero();
      dx_goal_ = Eigen::Vector3d::Zero();
      ddx_goal_ = Eigen::Vector3d::Zero();

      force_task_ = Eigen::Vector3d::Zero();
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSTaskComPos::init() : "<<e.what();
      return false;
    }
    return true;
  }
}
