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
/* \file CNonControlTaskBase.hpp
 *  Encapsulates a task servo and a task model
 *
 *  Created on: Jul 30, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CNONCONTROLTASKBASE_HPP_
#define CNONCONTROLTASKBASE_HPP_

#include <scl/DataTypes.hpp>

namespace scl {

/**
 * Container class to encapsulate a non-control task that
 * a controller might have to do (like log stuff etc.).
 *
 * NOTE : Virtual class. Subclass and implement functions
 * that compute the task forces.
 */
class CNonControlTaskBase {
public:
  // ***************** Initialization functions *****************
  /** Computes the task's stuff (whatever that might be) */
  virtual bool computeTask()=0;

  /** Initializes the task object. Create a subclass of
   * STaskBase if your task requires more data than the defaults
   * in STaskBase provide.
   * This function should set has_been_init_ to true
   *
   * NOTE : This is typically a one time operation, unlike activation. */
  virtual bool init()=0;

  /** Resets the task to pre-init. */
  virtual void reset()=0;

  // ***************** Accessor functions *****************
  /** NOTE : This is typically a dynamic operation that happens multiple times during runtime. */
  virtual void setActivation(sBool arg_act) { has_been_activated_ = arg_act; }

  /** Present initialization state */
  virtual sBool hasBeenInit() { return has_been_init_;  }

  /** Present activation state */
  virtual bool hasBeenActivated() { return has_been_activated_; }

  // ***************** Constructor/Destructor functions *****************
  /** Constructor does nothing */
  CNonControlTaskBase(): has_been_init_(false), has_been_activated_(false), name_(""){}

  /** Destructor does nothing */
  virtual ~CNonControlTaskBase(){}

protected:
  sBool has_been_init_;
  sBool has_been_activated_;
  std::string name_;
};

}

#endif /* CNONCONTROLTASKBASE_HPP_ */
