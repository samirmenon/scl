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
/* \file GenericCallbacks.hpp
 *
 *  Created on: Aug 20, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "GenericCallbacks.hpp"

#include <sutil/CRegisteredCallbacks.hpp>
#include <sutil/CRegisteredPrintables.hpp>

#include <string>
#include <iostream>


namespace scl
{
  /** A set of generic callbacks (to be used with the CRobotApp console).
   *
   * Please read the documentation for the CRobotApp console
   * to learn more. */

  /** An echo function call. Echoes a string back on to the console.
   * (An example for you) */
  void CCallbackEcho::call(std::vector<std::string>& arg)
  {//Print out all the arguments (except the first, which is the command)
    if(1 >= arg.size()) {}
    else if("--help" == arg[1])
    { std::cout<<" >>echo\n Echoes a string back on to the screen"; }
    else
    {
      std::vector<std::string>::iterator it, ite;
      for(it = arg.begin()+1, ite = arg.end(); it != ite; ++it)
      { std::cout<<*it<<" "; }
    }
  }

  CCallbackEcho::base* CCallbackEcho::createObject()
  { return dynamic_cast<CCallbackEcho::base*>(new CCallbackEcho()); }

  /** Print out a specific data structure.
   * Requires the data structure to overload the << operator.
   *
   * To use this class, you must register your object
   * with sutil::printables. (Open the file to see an
   * example of how)*/
  void CCallbackPrint::call(std::vector<std::string>& arg)
  {//Print out all the arguments (except the first, which is the command)
    if(1 >= arg.size())
    { std::cout<<"Object not found";  }
    else if("--help" == arg[1])
    { std::cout<<" >>print <object_name>\n Finds the object in the print registry and prints its contents"; }
    else
    {//Prints the contents of the object
      std::vector<std::string>::iterator it, ite;
      for(it = arg.begin()+1, ite = arg.end(); it != ite; ++it)
      {
        const sutil::SPrintableBase* obj = sutil::printables::get(*it);
        if(NULL == obj)
        { std::cout<<"Object not found";  }
        else
        { std::cout<<(*obj);  }
      }
    }
  }

  CCallbackPrint::base* CCallbackPrint::createObject()
  { return dynamic_cast<CCallbackPrint::base*>(new CCallbackPrint()); }

  /** Set values for a specific data structure.
   * Requires the data structure to overload the << operator.
   *
   * To use this class, you must register your object
   * with scl::CObjectMap. (Open scl/Singletons.hpp to see the
   * class)*/
  void CCallbackSet::call(std::vector<std::string>& arg)
  {//Set out all the arguments (except the first, which is the command)
    if(1 >= arg.size())
    { std::cout<<"Object not found";  }
    else if("--help" == arg[1])
    { std::cout<<" >>set <object_name> <arg_str1> <arg_str2> ... <arg_strN>"
      <<"\n Sets the contents of an object in the database"; }
    else
    {//Sets the contents of the object
      //const scl::SObject** obj = scl::CObjectMap::getData()->at_const(*(arg.begin()+1));
      //if(NULL == obj) { std::cout<<"Object not found";  }
      //else
      //{ std::cout<<(*obj);  }
    }
  }

  CCallbackSet::base* CCallbackSet::createObject()
  { return dynamic_cast<CCallbackSet::base*>(new CCallbackSet()); }

  /** Prints out a generic help message and also lists all the
   * other available commands */
  void CCallbackHelp::call(std::vector<std::string>& arg)
  {//Help out all the arguments (except the first, which is the command)
    std::vector<std::string> idxlist;
    //Get a list of all the callbacks in the registry
    if(false == sutil::callbacks::list(idxlist))
    { std::cout<<"Callback registry is empty";  }
    else
    {
      if(1 >= idxlist.size())
      { std::cout<<"Available commands:"; }
      std::vector<std::string>::iterator it,ite;
      for(it = idxlist.begin(), ite = idxlist.end(); it!=ite; ++it)
      { std::cout<<" "<<(*it); } //Print all the callbacks
      std::cout<<"\nFor details, type: <command> --help\nTo switch to char mode, type: x\nTo exit, type: exit";
    }
  }

  CCallbackHelp::base* CCallbackHelp::createObject()
  { return dynamic_cast<base*>(new CCallbackHelp()); }


  /** Key hander callbacks for decrementing a double data member
   * (5x data change if key is caps) */
  void CCallbackDecrement::call(bool& arg)
  {
    if(arg)
      (*data_)-=0.1;
    else
      (*data_)-=0.02;
  }

  CCallbackDecrement::base* CCallbackDecrement::createObject()
  {
    return dynamic_cast<CCallbackDecrement::base*>(new CCallbackDecrement());
  }


  /** Key hander callbacks for incrementing a double data member
   * (5x data change if key is caps) */
  void CCallbackIncrement::call(bool& arg)
  {
    if(arg)
      (*data_)+=0.1;
    else
      (*data_)+=0.02;
  }

  CCallbackIncrement::base* CCallbackIncrement::createObject()
  {
    return dynamic_cast<CCallbackIncrement::base*>(new CCallbackIncrement());
  }
}

