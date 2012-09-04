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
/* \file HapticCallbacks.hpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef HAPTICCALLBACKS_HPP_
#define HAPTICCALLBACKS_HPP_

#include <scl/Singletons.hpp>

#include <sutil/CRegisteredPrintables.hpp>

#include <scl/robot/GenericCallbacks.hpp>
#include <scl/robot/GenericPrintables.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>

namespace scl_app
{
  /** This function registers all the console io callbacks */
  bool registerCallbacks()
  {
    bool flag;
    try
    {
      /** ************************************************************************
       * Keyboard keys for the command line shell:
       *
       * Associate keyboard keys as the key handlers for
       * ui_point_1's x, y and z values. ui_point_1 can
       * then be used in any part of the program for other
       * stuff (like controlling an operational point)
       * ************************************************************************ */
      flag = sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
          'w', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](0)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
          's', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](0)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
          'a', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](1)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
          'd', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](1)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
          'q', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](2)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
          'e', &(scl::CDatabase::getData()->s_gui_.ui_point_[0](2)) );

      flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
          'u', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](0)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
          'j', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](0)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
          'h', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](1)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
          'k', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](1)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackDecrement,char,bool,double>(
          'y', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](2)) );
      flag = flag && sutil::callbacks::add<scl::CCallbackIncrement,char,bool,double>(
          'i', &(scl::CDatabase::getData()->s_gui_.ui_point_[1](2)) );
      if(false == flag){throw(std::runtime_error("Could not add a keyboard callback"));  }

      /** ************************************************************************
       * Add a help function to the command line shell
       * *************************************************************************/
      flag = sutil::callbacks::add<scl::CCallbackHelp, std::string, std::vector<std::string> >(
          std::string("help") );
      if(false == flag){throw(std::runtime_error("Could not add a help callback"));  }

      /** ************************************************************************
       * Add an echo function to the command line shell
       * *************************************************************************/
      flag = sutil::callbacks::add<scl::CCallbackEcho, std::string, std::vector<std::string> >(
          std::string("echo") );
      if(false == flag){throw(std::runtime_error("Could not add an echo callback"));  }

      /** ************************************************************************
       * Add a print callback. NOTE : You also need to add printables to print
       * *************************************************************************/
      flag = sutil::callbacks::add<scl::CCallbackPrint, std::string, std::vector<std::string> >(
          std::string("print") );
      if(false == flag){throw(std::runtime_error("Could not add a print callback"));  }

      flag = scl::addRobotPrintables();
      if(false == flag){throw(std::runtime_error("Could not add callbacks to print robot info"));  }
    }
    catch(std::exception &e)
    {
      std::cout<<"\nregisterCallbacks() : Error: "<<e.what();
      return false;
    }
    return true;
  }
}

#endif /* HAPTICCALLBACKS_HPP_ */
