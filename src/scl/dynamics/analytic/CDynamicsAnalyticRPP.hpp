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
/*
 * CDynamicsAnalyticRPP.hpp
 *
 *  Created on: Sep 11, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CDYNAMICSANALYTICRPP_HPP_
#define CDYNAMICSANALYTICRPP_HPP_

#include <scl/DataTypes.hpp>
#include <scl/dynamics/CDynamicsAnalyticBase.hpp>

namespace scl
{
  /** Implements analytical dynamics for the RPP bot.
   *
   * For the details of the math used here, please see:
   * Mathematica : doc/MathTutorial_01_RPPBot.nb
   */
  class CDynamicsAnalyticRPP : public CDynamicsAnalyticBase
  {
  public:
    /* **************************************************************
     *             CDynamicsAnalyticBase API functions
     * ************************************************************** */
    /** Calculates the Transformation Matrix for the robot to which
     * this dynamics object is assigned.
     *
     * The Transformation Matrix is specified by a link and an offset
     * (in task space dimensions)from that link and is given by:
     *
     *           x_ancestor_frame_coords = T * x_link_coords
     *
     * Uses id based link lookup. The dynamics implementation should
     * support this (maintain a map or something).
     */
    virtual sBool computeTransformationMatrix(
        /** The generalized coordinates */
        const Eigen::VectorXd &arg_q,
        /** The link at which the transformation matrix is to be calculated */
        sInt arg_link_id,
        /** The link up to which the transformation matrix is to be calculated */
        sInt arg_ancestor_link_id,
        /** The transformation matrix will be saved here. */
        Eigen::Affine3d& arg_T);

    /** Calculates the Jacobian for the robot to which this dynamics
     * object is assigned.
     *
     * The Jacobian is specified by a link and an offset (in task space
     * dimensions)from that link
     *
     * Uses id based link lookup. The dynamics implementation should
     * support this (maintain a map or something).
     */
    virtual sBool computeJacobian(
        /** The generalized coordinates */
        const Eigen::VectorXd &arg_q,
        /** The link at which the Jacobian is to be calculated */
        sInt arg_link_id,
        /** The offset from the link's frame (in local coordinates). */
        const Eigen::VectorXd& arg_pos_local,
        /** The Jacobian will be saved here. */
        Eigen::MatrixXd& arg_J)
    { return false; }

    /** Calculates the Jacobian for the robot to which this dynamics
     * object is assigned.
     *
     * The Jacobian is specified by a link and an offset (in task space
     * dimensions)from that link
     *
     * Uses id based link lookup. The dynamics implementation should
     * support this (maintain a map or something).
     */
    virtual sBool computeGCModel(
        /** The generalized coordinates */
        const Eigen::VectorXd &arg_q,
        /** All individual dynamics matrices will be saved here. */
        SGcModel& arg_gc_model)
    { return false; }

    /* **************************************************************
     *               Analytic Computation functions
     * ************************************************************** */

    /** NOTE : The affine offset includes the offset of the global origin to
     * the robot's origin. This is hard wired for now. */
    void computeTOrg_0(const Eigen::VectorXd &arg_q,
          Eigen::Affine3d& arg_T);

    void computeT0_1(const Eigen::VectorXd &arg_q,
          Eigen::Affine3d& arg_T);

    void computeT1_2(const Eigen::VectorXd &arg_q,
          Eigen::Affine3d& arg_T);

    /** Compute the Jacobian that maps center of mass velocities in
     * origin (global) coordinates to generalized velocities */
    bool computeJcom(const Eigen::VectorXd &arg_q,
        unsigned int arg_link_id, Eigen::MatrixXd& arg_J);

    /** Compute the generalized inertia matrix */
    bool computeMgc(const Eigen::VectorXd &arg_q,
        Eigen::MatrixXd& arg_Mgc);

    /* **************************************************************
     *                   Data access functions
     * ************************************************************** */
    /** Gives an id for a link name.
     *
     * Useful because:
     * 1. Allows storing any dynamic-engine specific objects in the controller.
     * 2. Avoids using inefficient repeated string based lookup (usually with maps)
     */
    virtual sUInt getIdForLink(std::string arg_link_name)
    {
      if("link0" == arg_link_name)
      { return 0; }
      else if("link1" == arg_link_name)
      { return 1; }
      else if("link2" == arg_link_name)
      { return 2; }
      else
      { return -1; }
    }

    /* **************************************************************
     *                   Initialization functions
     * ************************************************************** */
    /** Default constructor sets the initialization state to false */
    CDynamicsAnalyticRPP() : CDynamicsAnalyticBase() {}

    /** Default destructor does nothing */
    virtual ~CDynamicsAnalyticRPP(){}

    /** The only state here is the position of the robot's ground
     * wrt. the global origin.
     * There is nothing to initialize since the entire dynamics engine
     * is hard-coded for a robot and is otherwise stateless.
     *
     * Returns,
     * true  : success
     * false : failure
     */
    virtual sBool init(const SRobotParsed& arg_robot_data);

    /** Initialization state */
    virtual sBool hasBeenInit() {  return has_been_init_;  }

    /* **************************************************************
     *                            Data
     * ************************************************************** */
  protected:
    Eigen::Vector3d pos_root_in_global_org_;
  };

} /* namespace scl */
#endif /* CDYNAMICSANALYTICRPP_HPP_ */
