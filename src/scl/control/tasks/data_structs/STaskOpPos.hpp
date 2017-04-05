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
/* \file STaskOpPos.hpp
 *
 *  Created on: Jan 1, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SRC_SCL_CONTROL_TASKS_DATA_STRUCTS_STASKOPPOS_HPP_
#define SRC_SCL_CONTROL_TASKS_DATA_STRUCTS_STASKOPPOS_HPP_

#include <scl/DataTypes.hpp>
#include "STaskBase.hpp"

#include <Eigen/Dense>

namespace scl
{
  namespace tasks
  {
    class STaskOpPos : public scl::tasks::STaskBase
    {
    private:
      //0.5cm spatial resolution
      const scl::sUInt const_expected_task_dof_=3;
    public:
      //Computed attributes (last measured, in x dimensional task-space)
      Eigen::VectorXd x_;             //Position in the global frame
      Eigen::VectorXd dx_;            //Velocity in the global frame
      Eigen::VectorXd ddx_;           //Acceleration in the global frame

      Eigen::VectorXd x_goal_;        //Goal Position in the global frame
      Eigen::VectorXd dx_goal_;       //Goal Velocity in the global frame
      Eigen::VectorXd ddx_goal_;      //Goal Acceleration in the global frame

      Eigen::Vector3d pos_in_parent_; //Position in the parent link's local frame (x,y,z)
      std::string link_name_="";      //The parent link

      sFloat spatial_resolution_=0.005;//Meters

      const  sBool flag_defaults_[3] = {true, false, true}; ///< Well we'll try to not be too verbose..
      sBool flag_compute_op_gravity_=flag_defaults_[0];  ///< Use operational point gravity? Default = true
      sBool flag_compute_op_cc_forces_=flag_defaults_[1];///< Use operational centrifugal/coriolis forces? Default = false
      sBool flag_compute_op_inertia_=flag_defaults_[2];  ///< Use operational inertia? If true, set to identity. Default = true

      /** Default constructor sets stuff to S_NULL */
      STaskOpPos() : STaskBase("STaskOpPos"){}

      /** Default destructor does nothing */
      virtual ~STaskOpPos(){}

      /** Processes the task's non standard parameters.
       * This function is called by init() and must be implemented
       * by all subclasses. Else it will be impossible to initialize
       * the task. Ie. init() will always return false.
       *
       * Eg.The parent link and the pos in parent, which are required by op
       *    point tasks but not by gc tasks. These can be accessed using the
       *    mapped list .at() function. A string is returned. E.g.,
       *    arg_params.at("parent_link") == "base"
       *    arg_params.at("pos_in_parent") == "0.0 0.0 0.0"
       *    arg_params.at("my_new_arbitrary_option") == "8080"*/
      virtual bool initTaskSubclass(
          const sutil::CMappedList<std::string, std::string>& arg_params);
    };

  }
}
#endif /* SRC_SCL_CONTROL_TASKS_DATA_STRUCTS_STASKOPPOS_HPP_ */
