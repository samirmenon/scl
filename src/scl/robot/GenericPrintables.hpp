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
/* \file GenericPrintables.hpp
 *
 *  Created on: Sep 19, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef GENERICPRINTABLES_HPP_
#define GENERICPRINTABLES_HPP_

#include <scl/data_structs/SRobotIO.hpp>
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/control/data_structs/SControllerBase.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskGc.hpp>
#include <scl/control/task/data_structs/SControllerMultiTask.hpp>

#include <scl/Singletons.hpp>

#include <sutil/CRegisteredPrintables.hpp>

#include <stdexcept>
#include <iomanip>

#ifdef DEBUG
#include <cassert>
#endif

/* **************************************************************************
 *                         Starting namespace sutil
 *
 * These are a set of printable object definitions for objects in the database
 * Creating separate printable objects enables accessing the objects's values
 * through callbacks
 * Directly accessing the objects might be hard for certain situations
 * (Like from another process or a gui) and callbacks are good there.
 * ************************************************************************* */
namespace sutil
{
  template <>
  void printToStream<scl::SRobotParsed>(
      std::ostream& ostr,
      const scl::SRobotParsed& arg_data
  );

  template <>
  void printToStream<scl::SRobotIO>(
      std::ostream& ostr,
      const scl::SRobotIO& arg_data
  );

  template <>
  void printToStream<scl::SRigidBody>(
      std::ostream& ostr,
      const scl::SRigidBody& arg_data
  );

  template <>
  void printToStream<scl::STaskBase>(
      std::ostream& ostr,
      const scl::STaskBase& arg_data
  );

  template <>
  void printToStream<scl::STaskOpPos>(
      std::ostream& ostr,
      const scl::STaskOpPos& arg_data
  );

  template <>
  void printToStream<scl::STaskGc>(
      std::ostream& ostr,
      const scl::STaskGc& arg_data
  );

  template <>
  void printToStream<scl::SControllerMultiTask>(
      std::ostream& ostr,
      const scl::SControllerMultiTask& arg_data
  );

  template <>
  void printToStream<scl::SGcModel>(
      std::ostream& ostr,
      const scl::SGcModel& arg_data
  );

  template <>
  void printToStream<Eigen::Vector3d >(
      std::ostream& ostr,
      const Eigen::Vector3d& arg_eigvec
  );
}

/* **************************************************************************
 *                         Starting namespace scl
 *
 * Here we define callback adders for generic objects (like robots)
 * ************************************************************************* */
namespace scl
{
  /** This adds an object to the printable registry. That allows pretty-printing
   * this object from the command line shell...
   *
   * NOTE : This function may throw an exception (not typical SCL) */
  template <class T>
  bool printableAddObject(const T & arg_obj);

  /** This is to just add an object and member objects
   * NOTE : This function may throw an exception (not typical SCL) */
  template <>
  bool printableAddObject<scl::SRobotParsed>(const scl::SRobotParsed & arg_obj);

  /** This is to just add an object and member objects */
  template <>
  bool printableAddObject<scl::SRobotIO>(const scl::SRobotIO & arg_obj);

  /** This is to just add a gc model object */
  template <>
  bool printableAddObject<scl::SGcModel>(const scl::SGcModel & arg_obj);

  bool addRobotPrintables();
}
#endif /* GENERICPRINTABLES_HPP_ */
