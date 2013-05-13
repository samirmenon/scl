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
/* \file SNonControlTaskBase.cpp
 *
 *  Created on: Jun 30, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "SNonControlTaskBase.hpp"

#include <stdexcept>
#include <iostream>
#include <vector>


namespace scl
{

  SNonControlTaskBase::SNonControlTaskBase() : SObject("SNonControlTaskBase")
  {
    parent_controller_ = NULL;
    name_ = "";
    has_been_init_ = false;
    has_been_activated_ = false;
  }

  bool SNonControlTaskBase::init(const std::string & arg_name,
      const std::string & arg_type,
      /** These are ignored during SNonControlTaskBase initialization.
       * However, subclasses may choose to use them and/or
       * require various values to be set. */
      const std::vector<scl::sString2>& arg_nonstd_params)
  {
    bool flag;
    try
    {
      if(1>arg_name.size())
      { throw(std::runtime_error("Task name is too short")); }
      if(1>arg_type.size())
      { throw(std::runtime_error("Task type is too short")); }

      name_ = arg_name;
      type_task_ = arg_type;

      //Store the nonstandard params
      task_nonstd_params_ = arg_nonstd_params;

      flag = initTaskParams();
      if(false == flag)
      { throw(std::runtime_error("Could not initialize the non standard task parameters. \nTODO : Subclass SNonControlTaskBase, implement your task data structure, and make the function return true.")); }

      has_been_init_ = true;
      has_been_activated_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nSNonControlTaskBase::init() : "<<e.what();

      //Even if it was initialized earlier, it might be in an invalid state now.
      //We will, however. keep the past values for error handling / debugging.
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  bool SNonControlTaskBase::setParentController(const STaskController* arg_parent)
  {
    if(NULL == arg_parent)
    { return false; }
    parent_controller_ = arg_parent;
    return true;
  }
}
