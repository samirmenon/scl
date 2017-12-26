/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

This file is released under the MIT license.
See COPYING.MIT in the scl base directory.
*/
/* \file STaskCustom.hpp
 *
 *  Created on: Oct 20, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef STASKCUSTOM_HPP_
#define STASKCUSTOM_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>

#include <Eigen/Dense>

namespace scl_app
{

  class STaskCustom : public scl::STaskBase
  {
  public:
    //Computed attributes (last measured, in x dimensional task-space)
    Eigen::VectorXd x_;             //Position in the global frame
    Eigen::VectorXd dx_;            //Velocity in the global frame
    Eigen::VectorXd ddx_;           //Acceleration in the global frame

    Eigen::VectorXd x_goal_;        //Goal Position in the global frame
    Eigen::VectorXd dx_goal_;       //Goal Velocity in the global frame
    Eigen::VectorXd ddx_goal_;      //Goal Acceleration in the global frame

    Eigen::Vector3d pos_in_parent_; //Position in the parent link's local frame (x,y,z)
    std::string link_name_;         //The parent link
    const scl::SRigidBody *link_ds_;     //The parent link's parsed data structure

    scl::sFloat spatial_resolution_;     //Meters

    const scl::SRigidBodyDyn *rbd_;   //For quickly obtaining a task Jacobian

    bool enable_wonky_behavior_;

    /** Default constructor sets stuff to S_NULL */
    STaskCustom();

    /** Default destructor does nothing */
    virtual ~STaskCustom();

    /** 1. Initializes the task specific data members.
     *
     * 2. Parses non standard task parameters,
     * which are stored in STaskBase::task_nonstd_params_.
     * Namely:
     *  (a) parent link name
     *  (b) pos in parent.*/
    virtual bool initTaskParams();
  };

}

#endif /* STASKCUSTOM_HPP_ */
