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

//sUtil lib
#include <sutil/CSystemClock.hpp>

//Chai
#include <chai3d.h>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Openmp
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

//Standard includes
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cassert>

/** A sample application to control a haptic fmri experiment */
int main(int argc, char** argv)
{
  bool flag;
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

      /*****************************Chai Control Points************************************/
      cGenericObject *chai_haptic_pos_,*chai_haptic_pos_des_;

      /******************************Chai Haptics************************************/
      //For controlling op points with haptics
      //The app will support dual-mode control, with the haptics controlling op points.
      scl::CChaiHaptics haptics_;
      scl::sBool has_been_init_haptics_;
      Eigen::VectorXd haptic_pos_;
      Eigen::VectorXd haptic_base_pos_;

      //First set up the haptics
      flag = haptics_.connectToDevices();
      if(false == flag)
      { throw(std::runtime_error("Could not get connect to haptic device"));  }

      //Set up haptic device
      if(0 == haptics_.getNumDevicesConnected())
      {
        std::cout<<"WARNING: No haptic devices connected. Proceeding in keyboard mode";
        has_been_init_haptics_ = false;
      }
      else
      { has_been_init_haptics_ = true; }


      /******************************Main Loop************************************/
      std::cout<<std::flush;
      scl::sLongLong gr_ctr=0;//Graphics computation counter
      scl::sLongLong haptics_ctr=0;//Graphics computation counter
      scl::sFloat t_start, t_end;

      omp_set_num_threads(2);
      int thread_id;

      //Trigger the fmri scan with the serial pulse here.
      if(argc == 3)
      {
        std::string tmp_trigger(argv[2]);
        if(tmp_trigger == std::string("1"))
        {
          std::cout<<"\n ********** Triggering scan ********** ";
          //NOTE TODO : Call the usb-serial code
        }
      }
      t_start = sutil::CSystemClock::getSysTime();

#pragma omp parallel private(thread_id)
      {
        thread_id = omp_get_thread_num();
        if(thread_id==1)
        {
          std::cout<<"\nI am the haptics thread. Id = "<<thread_id<<std::flush;
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            // Get latest haptic device position and update the pointer on the
            // screen
            if(has_been_init_haptics_)
            {
              haptics_.getHapticDevicePositions(haptic_pos_);
              db->s_gui_.ui_point_[0] = haptic_pos_;
            }
            chai_haptic_pos_->setLocalPos(db->s_gui_.ui_point_[0]);

            // Get the desired position where the haptic point should go
            chai_haptic_pos_des_->setLocalPos(db->s_gui_.ui_point_[1]);

            // NOTE TODO : If reached, do something else

            haptics_ctr++;
          }
        }
        else
        {
          std::cout<<"\nI am the graphics thread. Id = "<<thread_id<<std::flush;
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            glutMainLoopEvent();
            gr_ctr++;
            const timespec ts = {0, 15000000};//Sleep for 15ms
            nanosleep(&ts,NULL);
          }
        }
      }

      t_end = sutil::CSystemClock::getSysTime();

      /****************************Print Collected Statistics*****************************/
      std::cout<<"\nSimulation Took Time : "<<t_end-t_start <<" sec";
      std::cout<<"\nReal World End Time  : "<<sutil::CSystemClock::getSysTime() <<" sec \n";
      std::cout<<"\nTotal Graphics Updates  : "<<gr_ctr;
      std::cout<<"\nTotal Haptics Updates   : "<<haptics_ctr;

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
