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
/* \file SActuatorSetParsed.hpp
 *
 *  Created on: Aug 19, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SACTUATORSETPARSED_HPP_
#define SACTUATORSETPARSED_HPP_

#include <scl/data_structs/SObject.hpp>

#include <Eigen/Core>

namespace scl
{
  class SRobotParsed;

  /** A base class for different actuator sets */
  class SActuatorSetParsed : public SObject
  {
  public:
    /* ******************************************************
     *                        Initialization
     * ****************************************************** */
    /** The robot to which this actuator set is attached */
    const SRobotParsed *robot_;

  protected:
    /** Must have a dynamic type object as well. For instance, a muscle
     * actuator set parsed ("SActuatorSetMuscleParsed") will have a dyn type
     * "SActuatorSetMuscle" */
    std::string type_dyn_obj_;

  public:
    /* ******************************************************
     *                        Initialization
     * ****************************************************** */
    explicit SActuatorSetParsed(const std::string& subclass_type_name,
        const std::string& subclass_type_dyn_name) :
      SObject(subclass_type_name),  robot_(NULL), type_dyn_obj_(subclass_type_dyn_name){}

    explicit SActuatorSetParsed(const char* subclass_type_name,
        const std::string& subclass_type_dyn_name) :
      SObject(subclass_type_name), robot_(NULL), type_dyn_obj_(subclass_type_dyn_name){}

    virtual ~SActuatorSetParsed(){}

    /** Returns the type of the dynamic object associated with this object */
    const std::string & getTypeDyn(){return type_dyn_obj_; }

  private:
    /** This is private to force subclasses to expose their type */
    SActuatorSetParsed();
  };

} /* namespace scl */
#endif /* SACTUATORSETPARSED_HPP_ */
