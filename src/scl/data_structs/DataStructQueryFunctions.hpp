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
/* \file DataStructHelperFunctions.hpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef DATASTRUCTHELPERFUNCTIONS_HPP_
#define DATASTRUCTHELPERFUNCTIONS_HPP_

#include <scl/data_structs/SActuatorSetMuscleParsed.hpp>

#include <iostream>

namespace scl
{
  namespace dsquery
  {
    /** Does what the name says. Else returns NULL */
    scl::SActuatorSetMuscleParsed* getFromActuatorSetFirstMuscleSet(scl::SActuatorSetParsed& arg_aset) const
    {
      scl::SActuatorSetMuscleParsed* ret_mset = NULL;
      // Let's find the parsed actuator set and make sure it has a type "muscle"
      sutil::CMappedPointerList<std::string, scl::SActuatorSetParsed, true>::iterator it,ite;
      for (it = arg_aset.begin(), ite = arg_aset.end(); it!=ite;++it)
      {
        scl::SActuatorSetParsed* tmp_aset = *it;
        // Since there could be different actuator types, the code casts their objects to a
        // simpler type and so we have to cast them to the correct type.
        if("SActuatorSetMuscleParsed" == tmp_aset->getType())
        {
          rob_mset_ds = dynamic_cast<scl::SActuatorSetMuscleParsed*>(tmp_aset);
          if(NULL==rob_mset_ds)
          {
            std::cout<<"\n WARNING : getFromActuatorSetFirstMuscleSet() : Actuator set type mismatch. This should be a muscle set:"
            <<tmp_aset->name_;
          }
          else  { break;  }//We found a suitable muscle actuator set. Won't look for more and so will exit the loop.
        }
      }
      // Return whatever we have. If no muscles were found, this will be NULL.
      return ret_mset;
    }
  }
}
#endif /* DATASTRUCTHELPERFUNCTIONS_HPP_ */
