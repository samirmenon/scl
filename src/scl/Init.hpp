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
/* \file Init.hpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCL_INIT_HPP_
#define SCL_INIT_HPP_

#include <scl/DataTypes.hpp>

#include <scl/AllHeaders.hpp>
// We'll keep this till we depracate the older tasks
#include <scl/control/tasks/AllHeaders.hpp>

namespace scl
{
  namespace init
  {

    /** ***********************************************************
                        DYNAMIC TYPES
     *********************************************************** **/
    /** Dynamic typing helper functions for different scl types
     *  Registers the native dynamic types */
    scl::sBool registerNativeDynamicTypes();


    /** ***********************************************************
                        PARSE AND INIT
     *********************************************************** **/
    /** An acme (swiss knife/random util/highly customized junk function) that
     * consolidates a lot of init code.
     * This just happens to be what is required for a generic control app.
     * It's not general at all. If you write your own app, I strongly recommend
     * that you copy the function's contents and insert them into your main
     * file. I just don't want to keep repeating this code all over the
     * place so it's consolidated here.
     */
    bool parseAndInitRobotAndController(
        scl::CParserScl &arg_p /** Parser */,
        const scl::SCmdLineOptions_OneRobot arg_rcmd /** The command line options */,
        scl::SRobotParsed &arg_rds /**Robot data structure*/,
        scl::SRobotIO &arg_rio /**I/O data structure. */,
        scl::SGcModel &arg_rgcm /**Robot data structure with dynamic quantities...*/,
        scl::CDynamicsScl &arg_dyn_scl /**Robot kinematics and dynamics computation object...*/,
        scl::SControllerMultiTask &arg_rctr_ds /**A multi-task controller data structure*/,
        scl::CControllerMultiTask &arg_rctr /**A multi-task controller*/,
        std::vector<scl::STaskBase*> &arg_rtasks /**A set of executable tasks*/,
        std::vector<scl::SNonControlTaskBase*> &arg_rtasks_nc /**A set of non-control tasks*/,
        std::vector<scl::sString2> &arg_ctrl_params /**Used to parse extra xml tags)*/
    );

    /** An acme (swiss knife/random util/highly customized junk function) that
     * consolidates a lot of init code.
     * This just happens to be what is required for a generic control app.
     * It's not general at all. If you write your own app, I strongly recommend
     * that you copy the function's contents and insert them into your main
     * file. I just don't want to keep repeating this code all over the
     * place so it's consolidated here.
     */
    bool parseAndInitRobotAndDynamics(
        scl::CParserScl &arg_p /** Parser */,
        scl::SCmdLineOptions_OneRobot arg_rcmd /* For parsing command line options */,
        scl::SRobotParsed &arg_rds /**Robot data structure*/,
        scl::SRobotIO &arg_rio /**I/O data structure. */,
        scl::SGcModel &arg_rgcm /**Robot data structure with dynamic quantities...*/,
        scl::CDynamicsScl &arg_dyn_scl /**Robot kinematics and dynamics computation object...*/
    );

    /** ***********************************************************
                            DATA STRUCT INIT
     * Some commonly used init functions; not general enough to be
     * in the core lib, but common enough to warrant some helper
     * functions.
     *********************************************************** **/
    /** In many places we assign a subset of tasks to 3d vector control points.
     * Note that the controller ds (arg_rctr_ds) is not a const because the ui vector
     * stores direct pointers into its members (that are modified later)....
     *
     * This is a very common operation FOR SOME APPS so we'll add some code here to avoid repetition
     * It's not general enough to warrant inclusion in any of the core functions.
     */
    bool initUI3dPointVectorFromParsedTasksAndCmdLineArgs(
        const scl::SCmdLineOptions_OneRobot &arg_rcmd /** For parsing command line options */,
        scl::SControllerMultiTask &arg_rctr_ds /** The initialized controller multi-task data struct */,
        std::vector<scl::STaskOpPos*> &arg_rtask_ui_3d_ds /** This will contain the final op pos tasks */
    );

    /** Checks whether dynamic type information is available. If so, it parses
     * tasks into the control data structure
     *
     * NOTE : This would have been part of the core data structure except for the
     * need for dynamic typing, which is useful for parsing user specified tasks. */
    int initMultiTaskCtrlDsFromParsedTasks(
        const std::vector<scl::STaskBase*> &arg_taskvec,
        const std::vector<scl::SNonControlTaskBase*> &arg_taskvec_nc,
        scl::SControllerMultiTask& arg_ctrl);

    /** In addition to the default initMultiTaskCtrlDsFromParsedTasks(), this function
     * also parses some extra options if they exist.
     *
     * It also helps split the code for the task parsing from the param parsing (potentially
     * easier to read for some).
     *
     * Sometimes parameters may be provided as strings and might require dynamic type
     * resolution. As such, we will parse them here... */
    int initMultiTaskCtrlDsFromParsedTasks(
        const std::vector<scl::STaskBase*> &arg_taskvec,
        const std::vector<scl::SNonControlTaskBase*> &arg_taskvec_nc,
        const std::vector<scl::sString2> &arg_ctrl_params,
        scl::SControllerMultiTask& ret_ctrl);

    /** Takes a list of tasks as an input and organizes them into
     * a mapped multi-level list.
     *
     * Returns : The number of tasks parsed (should be equal to tasks passed) */
    int initMappedMultiLevelListFromFromTasks(
        const SRobotParsed &arg_rds,
        const std::vector<scl::tasks::STaskBase*> &arg_taskvec,
        /** Pointers to the Task data structures. Organized
         * in the priority order (the outer vector) of the tasks.
         *
         * Tasks can be accessed either by name (map access), pointer (iterator_)
         * in the multi level pilemap or via the vector (std::vector access)*/
        sutil::CMappedMultiLevelList<std::string, scl::tasks::CTaskBase*> &ret_tasks);

    /** Initializes a dynamic tree given a static tree for a robot. */
    bool initDynRobotFromParsedRobot(sutil::CMappedTree<std::string, scl::SRigidBodyDyn>& arg_rbd_tree,
        const sutil::CMappedTree<std::string, scl::SRigidBody>& arg_rb_tree);


    ///////////////////////////////////////////////////////////////
    ///////////////////////////THE END/////////////////////////////
  }
}

#endif /* SCL_INIT_HPP_ */
