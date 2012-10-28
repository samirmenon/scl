/* \file fmri_xyz_localizer_main.cpp
 *
 *  Created on: Oct 23, 2012
 *
 *  Copyright (C) 2012
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/graphics/chai/CChaiGraphics.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/haptics/chai/CChaiHaptics.hpp>
#include <scl/parser/lotusparser/CLotusParser.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <scl/util/FileFunctions.hpp>

//sUtil lib
#include <sutil/CSystemClock.hpp>

//Chai
#include <chai3d.h>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Openmp
#include <omp.h>

//rs232 (Serial port communication)
#include <rs232.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

//Standard includes
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cassert>
#include <ctime>

/** A sample application to control a haptic fmri experiment */
int main(int argc, char** argv)
{
  bool flag;
  const double NaN = 0.0/0.0;
  if((argc != 2)&&(argc != 3))
  {
    std::cout<<"\nfmri_xyz_localizer demo application allows controlling a haptic device."
        <<"\n Optional : It can also trigger an fmri scan by setting : trigger_fmri_scan = 1 "
        <<"\nThe command line input is: ./<executable> <config_file_name.xml> <optional : trigger_fmri_scan> \n";
    return 0;
  }
  else
  {
    try
    {
      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start())
      { throw(std::runtime_error("Could not start clock"));  }

      scl::SDatabase* db = scl::CDatabase::getData(); //Sanity Check
      if(S_NULL==db) { throw(std::runtime_error("Database not initialized"));  }

      db->dir_specs_ = db->cwd_ + "../../specs/"; //Set the specs dir so scl knows where the graphics are.

      //Get going..
      std::cout<<"\nInitialized clock and database. Start Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

      std::string tmp_infile(argv[1]);
      std::cout<<"Running scl benchmarks for input file: "<<tmp_infile;

      /******************************File Parsing************************************/
      scl_parser::CLotusParser tmp_lparser;//Use the lotus tinyxml parser

      std::vector<std::string> graphics_names;
      flag = tmp_lparser.listGraphicsInFile(tmp_infile,graphics_names);
      if(false == flag) { throw(std::runtime_error("Could not list graphics names from the file"));  }

      if(S_NULL == scl_registry::parseGraphics(tmp_infile, graphics_names[0], &tmp_lparser))
      { throw(std::runtime_error("Could not register graphics with the database"));  }

      /******************************ChaiGlut Graphics************************************/
      glutInit(&argc, argv);

      scl::CChaiGraphics chai_gr;
      flag = chai_gr.initGraphics(graphics_names[0]);
      if(false==flag) { throw(std::runtime_error("Couldn't initialize chai graphics")); }

      if(false == scl_chai_glut_interface::initializeGlutForChai(graphics_names[0], &chai_gr))
      { throw(std::runtime_error("Glut initialization error")); }

      glutFullScreen();

      /*****************************Chai Control Points************************************/
      cGenericObject *chai_haptic_pos = S_NULL,*chai_haptic_pos_des = S_NULL;
      cGenericObject *chai_haptic_box_des_red = S_NULL, *chai_haptic_box_des_green = S_NULL,
          *chai_haptic_box_des_yellow = S_NULL;

      /** Render a sphere at the haptic point's position */
      flag = chai_gr.addSphereToRender(Eigen::Vector3d::Zero(), chai_haptic_pos, 0.02);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at com pos"));  }
      if(NULL == chai_haptic_pos){ throw(std::runtime_error("Could not add sphere at com pos"));  }

      /** If the box is not centered */
      // For now, the box's dimensions are 50x50x50 x1000 (mm units). Convert to m units.
      //  = 0.05 x 0.05 x 0.05 m *2 = 0.1 x 0.1 x 0.1 m
      const Eigen::Vector3d box_xyz_translate_center(0.05,0.05,0.05);
      const double box_scaling = 0.002;

      /** Render a box at the haptic point's desired position */
      flag = chai_gr.addMeshToRender("haptic_box_red","./graphics/haptic_des_box_red.obj",
          Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
      if(false == flag) { throw(std::runtime_error("Could not add mesh box at com desired pos"));  }
      chai_haptic_box_des_red = chai_gr.getChaiData()->meshes_rendered_.at("haptic_box_red")->graphics_obj_;
      if(NULL == chai_haptic_box_des_red){ throw(std::runtime_error("Could not add red mesh box at com desired pos"));  }
      // For now, the box's dimensions are 50x50x50 x1000 (mm units). Convert to m units.
      chai_haptic_box_des_red->scale(box_scaling,true);
      chai_haptic_box_des_red->setLocalPos(-box_xyz_translate_center);
      chai_haptic_box_des_red->setEnabled(false,true);

      /** Render a box at the haptic point's desired position */
      flag = chai_gr.addMeshToRender("haptic_box_green","./graphics/haptic_des_box_green.obj",
          Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
      if(false == flag) { throw(std::runtime_error("Could not add green mesh box at com desired pos"));  }
      chai_haptic_box_des_green = chai_gr.getChaiData()->meshes_rendered_.at("haptic_box_green")->graphics_obj_;
      if(NULL == chai_haptic_box_des_green){ throw(std::runtime_error("Could not add green mesh box at com desired pos"));  }
      // For now, the box's dimensions are x1000 (mm units). Convert to m units.
      chai_haptic_box_des_green->scale(box_scaling,true);
      chai_haptic_box_des_green->setEnabled(false,true);

      /** Render a box at the haptic point's desired position */
      flag = chai_gr.addMeshToRender("haptic_box_yellow","./graphics/haptic_des_box_yellow.obj",
          Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
      if(false == flag) { throw(std::runtime_error("Could not add yellow mesh box at com desired pos"));  }
      chai_haptic_box_des_yellow = chai_gr.getChaiData()->meshes_rendered_.at("haptic_box_yellow")->graphics_obj_;
      if(NULL == chai_haptic_box_des_yellow){ throw(std::runtime_error("Could not add yellow mesh box at com desired pos"));  }
      // For now, the box's dimensions are x1000 (mm units). Convert to m units.
      chai_haptic_box_des_yellow->scale(box_scaling,true);
      chai_haptic_box_des_yellow->setEnabled(false,true);

      /** Render a sphere at the haptic point's desired position */
      flag = chai_gr.addSphereToRender(Eigen::Vector3d::Zero(), chai_haptic_pos_des, 0.01);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at com desired pos"));  }
      if(NULL == chai_haptic_pos_des){ throw(std::runtime_error("Could not add sphere at com desired pos"));  }
      chai_haptic_pos_des->setEnabled(false,true);

//      //Make desired position red/orange
//      cMaterial mat_red; mat_red.setRedFireBrick();
//      chai_haptic_pos_des->setMaterial(mat_red, false);

      /*****************************fMRI State Machine************************************/
      /** The first task ids are for the actual states */
      enum fMRIState {FMRI_RESTING, FMRI_CENTER, FMRI_PLANNING, FMRI_EXECUTING,
        /** The remaining enum ids are for the state initialization code */
        FMRI_INCREMENT_TASK_STATE,
        FMRI_REST_INIT,
        FMRI_CENTER_INIT,
        FMRI_PLAN_INIT,
        FMRI_EXECUTE_INIT,
        FMRI_EXIT};
      fMRIState state=FMRI_REST_INIT; //Always start off at the rest state

      // ALL THE STATE VARIABLES :
      int state_task_row_in_matrix=0;
      double state_t_curr = 0.0;
      double state_t_curr_task_max=-1.0;
      Eigen::Vector3d state_x_des_curr_task(0,0,0);
      Eigen::MatrixXd state_task_selection_matrix;

      /** Read the states for this trial run.
       * The state machine will accept inputs like:
       *
       * <task-id> <time-at-completion> <time-duration> <x-des> <y-des> <z-des> <optional-args>
       *
       * NOTES:
       * 1. The time is relative time wrt. the start of the fMRI machine.
       * 2. xyz desired are only valid for planning/executing tasks (they are ignored for others).
       * 3. Optional args may trigger some other behavior for future tasks.
       * 4. The time-duration is for your knowledge. It is ignored by the program.
       * 5. Only time-at-completion is used for the timer (this simplifies code to prevent time-drift)
       * 6. Workspace : Roughly 24" (along bore) x 12" (width) x 8" (height)
       *                ==      0.6 x 0.3 x 0.2 m (xyz in chai)
       *                ==      0.3 x 0.15 x 0. m (xyz displacements to reach cuboid edges from center)
       */
      flag = scl_util::readEigenMatFromFile(state_task_selection_matrix, 10, 6, "task.txt");
      if(false == flag)
      { throw(std::runtime_error("Could not open task input file"));  }
      std::cout<<"\nRunning task matrix:"
          <<"\n<task-id> <time-at-completion> <time-duration> <x-des> <y-des> <z-des>"
          <<"\n"<<state_task_selection_matrix<<"\n";

      if(0!=state_task_selection_matrix(0,0))
      { throw(std::runtime_error("Simulation must start at rest state"));  }

      if(FMRI_REST_INIT!=state)
      { throw(std::runtime_error("Simulation state machine must start at rest init state"));  }

      /*****************************Chai Background Text************************************/
      cFont *text_font = NEW_CFONTCALIBRI20();
      cLabel *text_label;
      // Initialize the text label
      text_label = new cLabel(text_font);
      chai_gr.getChaiData()->chai_cam_->m_frontLayer->addChild(text_label);
      text_label->m_fontColor.setBlue();
      text_label->setFontScale(2);

      /******************************Chai Haptics************************************/
      //For controlling op points with haptics
      //The app will support dual-mode control, with the haptics controlling op points.
      scl::CChaiHaptics haptics_;
      scl::sBool has_been_init_haptics;
      Eigen::VectorXd haptic_pos_;
      Eigen::VectorXd haptic_base_pos_;

      //First set up the haptics
      flag = haptics_.connectToDevices();
      if(false == flag)
      {
        std::cout<<"\nWARNING : Could not connect to haptic device. Proceeding in kbd mode. \n\tDid you run as sudo?";
      }

      //Set up haptic device
      if(0 == haptics_.getNumDevicesConnected())
      {
        std::cout<<"WARNING: No haptic devices connected. Proceeding in keyboard mode";
        has_been_init_haptics = false;
      }
      else
      { has_been_init_haptics = true; }

      /****************************** Logging ************************************/
      FILE * fp;
      std::string tmp_outfile = tmp_infile + std::string(".log");
      fp = fopen(tmp_outfile.c_str(),"a");
      if(NULL == fp)
      { throw(std::runtime_error(std::string("Could not open log file : ") + tmp_outfile));  }

      time_t t = time(0);   // get time now
      struct tm * now = localtime( & t );
      int year = now->tm_year + 1900;
      int mon = now->tm_mon + 1;
      int day = now->tm_mday;

      double last_log_time = sutil::CSystemClock::getSysTime();

      //Log initial state:
      fprintf(fp,"\n*******************************************************************");
      fprintf(fp,"\n************************fMRI Localizer*****************************");
      fprintf(fp,"\n Real World Start Date : %d-%d-%d", mon, day, year);
      fprintf(fp,"\n Real World Start Time : %lf", sutil::CSystemClock::getSysTime());
      fprintf(fp,"\n Config File : %s", tmp_infile.c_str());
      fprintf(fp,"\n Haptics Used : %s", has_been_init_haptics?"true":"false");
      fprintf(fp,"\n Data format : time x_des y_des z_des x_curr y_curr z_curr");
      fprintf(fp,"\n Data accuracy : time = ms. pos = mm");
      fprintf(fp,"\n*******************************************************************");
      fprintf(fp,"\n*******************************************************************");


      /***************************************************************************/
      /***************************************************************************/
      /******************************Main Loop************************************/
      /***************************************************************************/
      /***************************************************************************/
      std::cout<<std::flush;
      scl::sLongLong gr_ctr=0;//Graphics computation counter
      scl::sLongLong haptics_ctr=0;//Graphics computation counter
      scl::sFloat t_start, t_end;

      omp_set_num_threads(2);
      int thread_id;

      /******************************fMRI Scan Trigger************************************/
      //Trigger the fmri scan with the serial pulse here.
      if(argc == 3)
      {
        std::string tmp_trigger(argv[2]);
        if(tmp_trigger == std::string("1"))
        {
          char ch;
          if(false == has_been_init_haptics)
          {
            std::cout<<"\n WARNING : Could not intialize haptics. Trigger scan? y/n \n>>";
            std::cin>>ch;
          }
          if(ch == 'y')
          {
            std::cout<<"\n ********** Triggering scan ********** \n";
            int err;
            err = OpenComport(22,57600);
            if(0 == err)
            { std::cout<<"\nOpened com port /dev/ttyACM0 at a baud rate of 57600"; }
            else
            { throw(std::runtime_error("Could not connect to fMRI serial port: Is /dev/ttyACM0 connected? $ sudo chown user:user /dev/ttyACM0 ? Else fix code or try manual trigger."));  }

            err = 0;
            err = err+SendByte(22, '[');
            err = err+SendByte(22, 't');
            err = err+SendByte(22, ']');
            err = err+SendByte(22, '\n');
            if(0 == err)
            {
              t_start = sutil::CSystemClock::getSysTime();
              std::cout<<"\nSent scan trigger '[t]\n' over the usb-serial port";
              std::cout<<"\nStarted experiment clock";
            }
            { throw(std::runtime_error("Could not trigger fMRI scan: Is /dev/ttyACM0 connected? $ sudo chown user:user /dev/ttyACM0 ? Else fix code or try manual trigger."));  }

            CloseComport(22);
            std::cout<<"\nClosed com port. Continuing to the experiment.";
            std::cout<<"\n ========= Fmri scan triggered. With haptics. ========";
          }
        }
      }
      else
      {
        std::cout<<"\n ========= Skipping fmri scan trigger. No haptics ========";
        t_start = sutil::CSystemClock::getSysTime();
      }

#ifndef DEBUG
#pragma omp parallel private(thread_id)
      {
        thread_id = omp_get_thread_num();
        if(thread_id==1)
        {
          std::cout<<"\nI am the haptics and logging thread. Id = "<<thread_id<<std::flush;
#endif
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            // Get latest haptic device position and update the pointer on the
            // screen
            Eigen::Vector3d& hpos = db->s_gui_.ui_point_[0];
            if(has_been_init_haptics)
            {
              haptics_.getHapticDevicePosition(0,haptic_pos_);
              hpos = haptic_pos_;
            }
            // SET THE CURRENT POSITION OF THE DEVICE!!
            chai_haptic_pos->setLocalPos(hpos);

            //Set up the state variables for the state machine
            // Current time
            state_t_curr = sutil::CSystemClock::getSysTime() - t_start;
            // Max time in this state
            state_t_curr_task_max = state_task_selection_matrix(state_task_row_in_matrix,1);
            // Display target's position
            state_x_des_curr_task<<state_task_selection_matrix(state_task_row_in_matrix,3),
                state_task_selection_matrix(state_task_row_in_matrix,4),
                state_task_selection_matrix(state_task_row_in_matrix,5);

            // ************** STATE MACHINE ******************
            switch(state)
            {
              case FMRI_RESTING:
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_CENTER:
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_PLANNING:
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_EXECUTING:
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_INCREMENT_TASK_STATE:  // Set state_x_des_curr_task
                //Transition to next state:
                state_task_row_in_matrix++;
                if(state_task_row_in_matrix >= state_task_selection_matrix.rows())
                { state = FMRI_EXIT;  }
                else
                {
                  switch(static_cast<int>(state_task_selection_matrix(state_task_row_in_matrix,0)))
                  {
                    case 0:
                      state = FMRI_REST_INIT;
                      break;
                    case 1:
                      state = FMRI_CENTER_INIT;
                      break;
                    case 2:
                      state = FMRI_PLAN_INIT;
                      break;
                    case 3:
                      state = FMRI_EXECUTE_INIT;
                      break;
                  }
                }
                break;

              case FMRI_REST_INIT:
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(false,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                //Goal point
                chai_haptic_pos_des->setEnabled(false,true);
                //Transition to next state:
                std::cout<<"\nSwitching to : FMRI_RESTING"<<std::flush;
                state = FMRI_INCREMENT_TASK_STATE;
                break;

              case FMRI_CENTER_INIT:  // R : yes, G : no, Y : no
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(true,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(0,0,0);
                //Transition to next state:
                std::cout<<"\nSwitching to : FMRI_CENTER"<<std::flush;
                state = FMRI_CENTER;
                break;

              case FMRI_PLAN_INIT:    // R : yes, G : no, Y : yes
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(true,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(true,true);
                //Goal box pos
                chai_haptic_box_des_yellow->setLocalPos(state_x_des_curr_task-box_xyz_translate_center);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(state_x_des_curr_task);
                //Switch tasks
                std::cout<<"\nSwitching to : FMRI_PLANNING"<<std::flush;
                state = FMRI_PLANNING;
                break;

              case FMRI_EXECUTE_INIT:    // R : no, G : yes, Y : no
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(false,true);
                chai_haptic_box_des_green->setEnabled(true,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                //Goal box pos
                chai_haptic_box_des_green->setLocalPos(state_x_des_curr_task-box_xyz_translate_center);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(state_x_des_curr_task);
                //Switch tasks
                std::cout<<"\nSwitching to : FMRI_EXECUTING"<<std::flush;
                state = FMRI_EXECUTING;
                break;

              case FMRI_EXIT:
                //Exit the simulation at the next loop
                scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
                break;
            }

            haptics_ctr++;

            //Log the data
            double curr_time = sutil::CSystemClock::getSysTime();
            if(0.001 < curr_time - last_log_time)
            {
              std::stringstream ss;
              ss <<cStr(curr_time - t_start, 1)<<" : ["
                  <<cStr(state_x_des_curr_task(0), 2)<<", "
                  <<cStr(state_x_des_curr_task(1), 2)<<", "
                  <<cStr(state_x_des_curr_task(2), 2)<<"] ["
                  <<cStr(hpos(0), 2)<<", "
                  <<cStr(hpos(1), 2)<<", "
                  <<cStr(hpos(2), 2)<<"]";
              std::string s;
              s = ss.str();
              // update haptic rate label
              //text_label->setString ("Pos: "+cStr(sutil::CSystemClock::getSysTime(), 1) + " [sec]");
              text_label->setString(s);
              int px = (int)(0.5 * (1024 - text_label->getWidth()));
              text_label->setLocalPos(px, 15);

              fprintf(fp,"\n%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",curr_time - t_start,
                  state_x_des_curr_task(0), state_x_des_curr_task(1), state_x_des_curr_task(2), hpos(0), hpos(1), hpos(2) );
              last_log_time = curr_time;
            }
#ifndef DEBUG
          }
        }
        else
        {
          std::cout<<"\nI am the graphics thread. Id = "<<thread_id<<std::flush;
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
#endif
            glutMainLoopEvent();
            gr_ctr++;
#ifndef DEBUG
            const timespec ts = {0, 5000000};//Sleep for 5ms
            nanosleep(&ts,NULL);
#endif
          }
#ifndef DEBUG
        }
      }
#endif

      t_end = sutil::CSystemClock::getSysTime();

      /****************************Print Collected Statistics*****************************/
      std::cout<<"\nSimulation Took Time : "<<t_end-t_start <<" sec";
      std::cout<<"\nReal World End Time  : "<<sutil::CSystemClock::getSysTime() <<" sec \n";
      std::cout<<"\nTotal Graphics Updates  : "<<gr_ctr;
      std::cout<<"\nTotal Haptics Updates   : "<<haptics_ctr;

      fprintf(fp,"\n Simulation Took Time : %lf", t_end-t_start);
      fprintf(fp,"\n Real World End Time  : %lf", sutil::CSystemClock::getSysTime());
      fprintf(fp,"\n Tot Graphics Updates : %lld", gr_ctr);
      fprintf(fp,"\n Tot Haptics Updates  : %lld", haptics_ctr);
      fprintf(fp,"\n*******************************************************************");
      fprintf(fp,"\n*******************************************************************");

      /****************************Deallocate Memory And Exit*****************************/
      flag = chai_gr.destroyGraphics();
      if(false == flag) { throw(std::runtime_error("Error deallocating graphics pointers")); } //Sanity check.

      std::cout<<"\nExecuted Successfully";
      std::cout<<"\n*************************\n"<<std::flush;
      return 0;
    }
    catch(std::exception & e)
    {
      std::cout<<"\nEnd Time:"<<sutil::CSystemClock::getSysTime()<<"\n";

      std::cout<<"\nFailed: "<< e.what();
      std::cout<<"\n*************************\n";
      return 1;
    }
  }
}
