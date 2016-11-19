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
 *  Created on: Sep 18, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef GENERICCALLBACKS_HPP_
#define GENERICCALLBACKS_HPP_

#include <sutil/CRegisteredCallbacks.hpp>

#include <string>


namespace scl
{
  /** A set of generic callbacks (to be used with the CRobotApp console).
   *
   * Please read the documentation for the CRobotApp console
   * to learn more. */

  /** An echo function call. Echoes a string back on to the console.
   * (An example for you) */
  class CCallbackEcho : public sutil::CCallbackBase<std::string, std::vector<std::string> >
  {
  public:
    typedef sutil::CCallbackBase<std::string, std::vector<std::string> > base;

    virtual void call(std::vector<std::string>& arg);

    virtual base* createObject();
  };

  /** Print out a specific data structure.
   * Requires the data structure to overload the << operator.
   *
   * To use this class, you must register your object
   * with sutil::printables. (Open the file to see an
   * example of how)*/
  class CCallbackPrint : public sutil::CCallbackBase<std::string, std::vector<std::string> >
  {
  public:
    typedef sutil::CCallbackBase<std::string, std::vector<std::string> > base;

    virtual void call(std::vector<std::string>& arg);

    virtual base* createObject();
  };


  /** Set values for a specific data structure.
   * Requires the data structure to overload the << operator.
   *
   * To use this class, you must register your object
   * with scl::CObjectMap. (Open scl/Singletons.hpp to see the
   * class)*/
  class CCallbackSet : public sutil::CCallbackBase<std::string, std::vector<std::string> >
  {
  public:
    typedef sutil::CCallbackBase<std::string, std::vector<std::string> > base;

    virtual void call(std::vector<std::string>& arg);

    virtual base* createObject();
  };

  /** Prints out a generic help message and also lists all the
   * other available commands */
  class CCallbackHelp : public sutil::CCallbackBase<std::string, std::vector<std::string> >
  {
  public:
    typedef sutil::CCallbackBase<std::string, std::vector<std::string> > base;

    virtual void call(std::vector<std::string>& arg);

    virtual base* createObject();
  };

  /** Key hander callbacks for decrementing a double data member
   * (5x data change if key is caps) */
  class CCallbackDecrement : public sutil::CCallbackBase<char, bool, double>
  {
  public:
    typedef sutil::CCallbackBase<char,bool,double> base;
    CCallbackDecrement(){}
    virtual void call(bool& arg);
    virtual base* createObject();
  };

  /** Key hander callbacks for incrementing a double data member
   * (5x data change if key is caps) */
  class CCallbackIncrement : public sutil::CCallbackBase<char, bool, double>
  {
  public:
    typedef sutil::CCallbackBase<char,bool,double> base;
    CCallbackIncrement() {}
    virtual void call(bool& arg);
    virtual base* createObject();
  };
}

#endif /* GENERICCALLBACKS_HPP_ */
