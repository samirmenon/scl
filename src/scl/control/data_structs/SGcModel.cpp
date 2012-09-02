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
 * \file SGcModel.cpp
 *
 *  Created on: May 5, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/control/data_structs/SGcModel.hpp>
#include <stdexcept>
#include <iostream>

namespace scl
{
  SGcModel::SGcModel()
  { }

  sBool SGcModel::init(const sUInt arg_robot_dof)
  {
    try
    {
      if(0==arg_robot_dof)
      { throw(std::runtime_error("Can not initialize for a parent-robot with 0 dof")); }

      A_.setIdentity(arg_robot_dof,arg_robot_dof);
      Ainv_.setIdentity(arg_robot_dof,arg_robot_dof);
      b_.setZero(arg_robot_dof);
      g_.setZero(arg_robot_dof);

      for(int i=0;i<arg_robot_dof;++i)
      {
        SCOMInfo com;
        com.J_com_.setZero(6, arg_robot_dof);
        com.pos_com_.setZero(3);
        com.name_ = "";
        coms_.push_back(com);
      }

      return true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSGcModel::init() : "<<e.what();
      return false;
    }
  }
}
