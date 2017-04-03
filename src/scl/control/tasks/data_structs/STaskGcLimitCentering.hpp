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
/* \file STaskGcLimitCentering.hpp
 *
 *  Created on: Mar 15, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef STASKGCLIMITCENTERING_HPP_
#define STASKGCLIMITCENTERING_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>

#include <Eigen/Dense>

namespace scl
{

  class STaskGcLimitCentering : public scl::STaskBase
  {
  public:
    Eigen::VectorXd q_;             //Goal Position in the global frame
    Eigen::VectorXd dq_;            //Goal Velocity in the global frame

    Eigen::VectorXd q_goal_;        //Goal Position in the global frame

    sFloat spatial_resolution_;     //Meters (acceptable error).

    /** Default constructor sets stuff to S_NULL */
    STaskGcLimitCentering();

    /** Default destructor does nothing */
    virtual ~STaskGcLimitCentering();

    /** 1. Initializes the task specific data members.
     *
     * 2. Parses non standard task parameters,
     * which are stored in STaskBase::task_nonstd_params_ */
    virtual bool initTaskParams();
  };

}

#endif /* STASKGCLIMITCENTERING_HPP_ */
