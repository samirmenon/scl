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
 * \file CSensorSetBase.hpp
 *
 *  Created on: Aug 24, 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */


#ifndef CSENSORSETBASE_HPP_
#define CSENSORSETBASE_HPP_

#include <scl/sensing/CSensorBase.hpp>
#include <scl/data_structs/SObject.hpp>
#include <sutil/CMappedList.hpp>
#include <string>

namespace scl
{
  /** An sensor set combines a variety of potentially different
   * sensors into a common interface that a controller can interact
   * with. The number of sensors is not necessarily related to the
   * degrees of freedom of the system (could be more, equal or less,
   * or could even be video feeds etc.).
   *
   * Subclasses of this class will simply organize sensors into a
   * coherent interface, abstracting all physical details and exposing
   * a data source for the controller.
   *
   * NOTE for "Real Robots": While working with a real robot, subclass
   *  this base class and implement your driver in it.
   */
  class CSensorSetBase
  {
  public:
    /* *****************************************************************
     *                        Sensor Modeling
     * ***************************************************************** */
    /** Reads the sensors. These will be translated into individual
     * sensor commands. */
    virtual sBool sense()=0;

    /** Gets the output of the sensor. Returned in the passed variable. */
    virtual sBool getSensorOutputs(Eigen::VectorXd& ret_output)=0;

    /* *****************************************************************
     *                        Initialization
     * ***************************************************************** */
    /** Initialize the sensor set */
    virtual sBool init()=0;

    /** Has this sensor been initialized */
    virtual sBool hasBeenInit()
    { return has_been_init_; }

    /** Default constructor. Sets stuff to NULL */
    CSensorSetBase(): has_been_init_(false) {}

    /** Default destructor. Does nothing. */
    virtual ~CSensorSetBase() {}

  protected:
    /** A collection of sensors that a controller can send commands to */
    sutil::CMappedPointerList<std::string, CSensorBase, true> sensors_;

    /** Initialization state */
    sBool has_been_init_;
  };

} /* namespace scl */
#endif /* CSENSORSETBASE_HPP_ */
