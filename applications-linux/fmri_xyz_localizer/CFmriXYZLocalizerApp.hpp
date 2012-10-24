/* \file CFmriXYZLocalizerApp.hpp
 *
 *  Created on: Sep 3, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CHAPTICAPP_HPP_
#define CHAPTICAPP_HPP_

#include <scl/robot/CRobotApp.hpp>

#include <scl/control/task/CTaskController.hpp>
#include <scl/control/task/tasks/COpPointTask.hpp>
#include <scl/control/task/tasks/CComPosTask.hpp>

#include <scl/graphics/chai/data_structs/SChaiGraphics.hpp>

#include <scl/haptics/chai/CChaiHaptics.hpp>

namespace scl_app
{
  class CFmriXYZLocalizerApp : public scl::CRobotApp
  {
  public:
    // ****************************************************
    //                 The main functions
    // ****************************************************
    /** Runs the task controller. */
    virtual void stepMySimulation();

    // ****************************************************
    //           The initialization functions
    // ****************************************************
    /** Default constructor. Sets stuff to zero. */
    CFmriXYZLocalizerApp();

    /** Default destructor. Does nothing. */
    virtual ~CFmriXYZLocalizerApp(){}

    /** Sets up the task controller. */
    virtual scl::sBool initMyController(const std::vector<std::string>& argv,
        scl::sUInt args_parsed);

    /** Register any custom dynamic types that you have. */
    virtual scl::sBool registerCustomDynamicTypes();

    /** Sets all the ui points to their current position and
     * run the dynamics once to flush the state. */
    void setInitialStateForUIAndDynamics();

  private:
    // ****************************************************
    //                      The data
    // ****************************************************
    scl::CTaskController* ctrl_;           //Use a task controller

    /** This is an internal class for organizing the control-task
     * to ui-point connection through the keyboard or an external
     * haptic device */
    class SOpPointUiLinkData
    {
    public:
      std::string name_;
      scl::COpPointTask* task_;
      scl::SOpPointTask* task_ds_;
      scl::sInt ui_pt_;
      scl::sBool has_been_init_;
      cGenericObject *chai_pos_,*chai_pos_des_;
      SOpPointUiLinkData() :
        name_(""),task_(NULL), task_ds_(NULL),ui_pt_(-1),
        has_been_init_(false), chai_pos_(NULL), chai_pos_des_(NULL){}
    };
    /** The operational points that will be linked to keyboard handlers */
    std::vector<SOpPointUiLinkData> taskvec_op_point_;

    //For controlling the com task with a ui point
    std::string name_com_task_;
    scl::CComPosTask* task_com_;
    scl::SComPosTask* task_ds_com_;
    scl::sInt ui_pt_com_;
    scl::sBool has_been_init_com_task_;
    cGenericObject *chai_com_pos_,*chai_com_pos_des_;

    //For controlling op points with haptics
    //The app will support dual-mode control, with the haptics controlling op points.
    scl::CChaiHaptics haptics_;
    scl::sUInt num_haptic_devices_to_use_; //These will directly control ui-points.
    scl::sBool has_been_init_haptics_;
    std::vector<Eigen::VectorXd> haptic_pos_;
    std::vector<Eigen::VectorXd> haptic_base_pos_;
  };

}

#endif /* CHAPTICAPP_HPP_ */
