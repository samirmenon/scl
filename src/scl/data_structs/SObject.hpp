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
/* \file SObject.hpp
 *
 *  Created on: Jul 1, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SOBJECT_HPP_
#define SOBJECT_HPP_

#include <scl/DataTypes.hpp>

#include <string>

namespace scl
{
  /** A generic object in the world. All database objects should
   * inherit from this. */
  class SObject
  {
  public:
    /** Constructor sets the type of the object.
     * All subclasses should call this and set their type. */
    explicit SObject(const std::string &arg_type) : type_(arg_type)
    {}

    explicit SObject(const char* arg_type) : type_(arg_type)
    {}

    /** Default destructor. Does nothing. */
    virtual ~SObject(){}

    /** Get the object's type */
    virtual const std::string& getType() const {  return type_; }

    /** Get the object's type */
    virtual const std::string& getName() const {  return name_; }

    /** Get the object's type */
    virtual bool hasBeenInit() const {  return has_been_init_; }

    /** The object's name */
    std::string name_="";

    /** Whether the object is ready for use */
    sBool has_been_init_=false;

  protected:
    /** The object's type. Should only be set by the constructor */
    std::string type_;

  private:
    /** Disabled default constructors */
    SObject();
    //SObject& operator = (const SObject&);
  };

}

#endif /* SOBJECT_HPP_ */
