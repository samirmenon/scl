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
/* \file SActuatorSetBase.hpp
 *
 *  Created on: Aug 19, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SACTUATORSETBASE_HPP_
#define SACTUATORSETBASE_HPP_

#include <scl/data_structs/SActuatorSetParsed.hpp>

#include <Eigen/Core>

namespace scl
{
  /** A base class for different actuator sets */
  class SActuatorSetBase : public SObject
  {
  public:
    /* ******************************************************
     *                        Data
     * ****************************************************** */
    /** The actuator set forces in the actuator coordinates */
    Eigen::VectorXd force_actuator_;

    /** The desired gc forces that will be mapped to actuator forces */
    Eigen::VectorXd force_gc_des_;

    /** The Jacobian that maps actuator forces into gc forces
     *  => force_gc_ = J_ * force_actuator_;
     */
    Eigen::MatrixXd J_;

    /* ******************************************************
     *                        Initialization
     * ****************************************************** */
    explicit SActuatorSetBase(const std::string& subclass_type_name) :
      SObject(subclass_type_name){}

    explicit SActuatorSetBase(const char* subclass_type_name) :
      SObject(subclass_type_name){}

    virtual ~SActuatorSetBase(){}

    virtual bool init(const SActuatorSetParsed * arg_actuator_set)=0;

  private:
    /** This is private to force subclasses to expose their type */
    SActuatorSetBase();
  };

} /* namespace scl */
#endif /* SACTUATORSETBASE_HPP_ */
