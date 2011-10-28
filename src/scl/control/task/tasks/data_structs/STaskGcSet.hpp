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
/* \file STaskGcSet.hpp
 *
 *  Created on: Aug 29, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef STASKGCSET_HPP_
#define STASKGCSET_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>

#include <Eigen/Dense>

namespace scl
{

  class STaskGcSet : public scl::STaskBase
  {
  public:
    std::vector<std::string> q_sel_names_; //The selected generalized coordinates (joint names)

    Eigen::VectorXd q_sel_;         //The selected generalized coordinates to be controlled
    Eigen::VectorXd q_goal_;        //Goal Position in the global frame
    Eigen::VectorXd dq_goal_;       //Goal Velocity in the global frame
    Eigen::VectorXd ddq_goal_;      //Goal Acceleration in the global frame

    sFloat spatial_resolution_;     //Meters

    /** Default constructor sets stuff to S_NULL */
    STaskGcSet();

    /** Default destructor does nothing */
    virtual ~STaskGcSet();

    /** 1. Initializes the task specific data members.
     *
     * 2. Parses non standard task parameters,
     * which are stored in STaskBase::task_nonstd_params_ */
    virtual bool initTaskParams();
  };

} /* namespace scl */
#endif /* STASKGCSET_HPP_ */
