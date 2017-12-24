/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/* \file ExampleCallbacks.hpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef EXAMPLECALLBACKS_HPP_
#define EXAMPLECALLBACKS_HPP_

#include <scl/Singletons.hpp>

#include <scl/callbacks/GenericCallbacks.hpp>
#include <scl/callbacks/PrintablesJSON.hpp>

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

      /** ************************************************************************
       * Add a print callback hooks for everything in the database.
       * NOTE : This only works if you use the database. So it won't work in apps
       * that don't use the database. In that case, you'll have to manually add
       * hooks. Look at the function below and extract the appropriate code.
       * *************************************************************************/
      flag = scl::printableAddObject<scl::SDatabase>(*scl::CDatabase::getData());
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

#endif /* EXAMPLECALLBACKS_HPP_ */
