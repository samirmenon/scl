/* This file is part of Busylizzy, a control and simulation library
for robots and biomechanical models.

Busylizzy is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

Busylizzy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
Busylizzy. If not, see <http://www.gnu.org/licenses/>.
*/
/* \file Singletons.hpp
 *
 *  Created on: May 19, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

/** \file Singletons.hpp */

#ifndef SINGLETONS_HPP_
#define SINGLETONS_HPP_

#include <scl/data_structs/SDatabase.hpp>
#include <scl/data_structs/SObject.hpp>

#include <sutil/CSingleton.hpp>

namespace scl
{
  /** Data structure wrapper which will contain all the data
   * required to simulate and/or control a set of robots.
   *
   * These classes will provide direct memory access to all the
   * data stored within it.
   *
   * TODO : Thread safety : 1-writer + n-readers for different subcomponents*/
  typedef sutil::CSingleton<SDatabase> CDatabase;

  /** Data structure wrapper which will contain all the data
   * that a user might query during run time. This data will
   * typically be accessed through callbacks. */
  typedef sutil::CSingleton<sutil::CMappedList<std::string,SObject*> > CObjectMap;
}

#endif /* SINGLETONS_HPP_ */
