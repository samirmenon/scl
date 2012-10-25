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

      /*****************************Chai Control Points************************************/
      cGenericObject *chai_haptic_pos = S_NULL,*chai_haptic_pos_des = S_NULL;

      /** Render a sphere at the haptic point's position */
      flag = chai_gr.addSphereToRender(Eigen::Vector3d::Zero(), chai_haptic_pos, 0.02);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at com pos"));  }
      if(NULL == chai_haptic_pos){ throw(std::runtime_error("Could not add sphere at com pos"));  }

      /** Render a sphere at the haptic point's desired position */
      flag = chai_gr.addSphereToRender(Eigen::Vector3d::Zero(), chai_haptic_pos_des, 0.02);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at com desired pos"));  }
      if(NULL == chai_haptic_pos_des){ throw(std::runtime_error("Could not add sphere at com desired pos"));  }

      //Make desired position red/orange
      cMaterial mat_red; mat_red.setRedFireBrick();
      chai_haptic_pos_des->setMaterial(mat_red, false);

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
          char ch;
          if(false == has_been_init_haptics)
          {
            std::cout<<"\n WARNING : Could not intialize haptics. Trigger scan? y/n \n>>";
            std::cin>>ch;
          }
          if(ch == 'y')
          {
            std::cout<<"\n ********** Triggering scan ********** ";
            //NOTE TODO : Call the usb-serial code
          }
        }
      }
      else
      { std::cout<<"\n ========= Skipping fmri scan trigger. No haptics ========";  }
      t_start = sutil::CSystemClock::getSysTime();

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
            chai_haptic_pos->setLocalPos(hpos);

            // Get the desired position where the haptic point should go
            Eigen::Vector3d& hpos_des = db->s_gui_.ui_point_[1];
            chai_haptic_pos_des->setLocalPos(hpos_des);

            // NOTE TODO : If reached, do something else

            haptics_ctr++;

            //Log the data
            double curr_time = sutil::CSystemClock::getSysTime();
            if(0.001 < curr_time - last_log_time)
            {
              fprintf(fp,"\n%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",curr_time - t_start,
                  hpos_des(0), hpos_des(1), hpos_des(2), hpos(0), hpos(1), hpos(2) );
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
