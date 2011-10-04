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
/* \file SMtrackData.hpp
 *
 *  Created on: May 14, 2011
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SMTRACKDATA_HPP_
#define SMTRACKDATA_HPP_

#include <scl/data_structs/SObject.hpp>

#include <scl/DataTypes.hpp>

#include <Eigen/Dense>

namespace scl
{

  class SMtrackData : public scl::SObject
  {
  public:
    SMtrackData();
    virtual ~SMtrackData();
  };

}

#endif /* SMTRACKDATA_HPP_ */
