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
#include <scl/parser/sclparser/CSclParser.hpp>
#include <scl/util/DatabaseUtils.hpp>
#include <scl/util/FileFunctions.hpp>

//sUtil lib
#include <sutil/CSystemClock.hpp>

//Chai
#include <chai3d.h>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Sensoray
#include <sensoray/CBfrDriver.hpp>

//Openmp
#include <omp.h>

//rs232 (Serial port communication)
#include <rs232.h>

//Freeglut windowing environment
#include <GL/freeglut.h>

//Standard includes
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cassert>
#include <ctime>
#include <string>

void applyForcesREPLACE_ME_WITH_THE_REAL_DEAL_LATER(double time, double x, double y, double z)
{
  std::cout<<"\nI need to apply this force at ("<<time<<") : "<<x<<", "<<y<<", "<<z;
}

/** A sample application to control a haptic fmri experiment */
int main(int argc, char** argv)
{
  bool flag;
  bool flag_trigger_scan = false;
  bool flag_display_timer = false;
  int file_rows = 10;
  int file_cols = 7;
  const double DIST_AT_GOAL_ACHIEVED = 0.025;
  if(argc < 2)
  {
    std::cout<<"\nfmri_xyz_localizer demo application allows controlling a haptic device."
        <<"\n Optional : It can also trigger an fmri scan by setting : trigger_fmri_scan = 1 "
        <<"\nThe command line input is: ./<executable> <config_file_name.xml> <optional : \"-trig\" to trigger_fmri_scan>"
        <<"\n\t<optional : \"-frows\" to specify file rows = 10> <optional : \"-fcols\" to specify file cols = 7>"
        <<" <optional : \"-timer\" to display a text timer> \n";
    return 0;
  }
  else
  {
    try
    {
      /******************************CommandLineFlags************************************/
      for(int i=0; i<argc; ++i)
      {
        std::stringstream s;
        s<<argv[i];
        std::string str;
        str = s.str();
        if(str == "-trig")
        { flag_trigger_scan = true; }
        if(str == "-timer")
        { flag_display_timer = true; }
        if(str == "-frows")
        {
          if(i+1>=argc)
          { throw("Argument '-frows' requires the number of rows to be entered");}
          std::stringstream ss;
          ss<<argv[i+1];
          ss>>file_rows;
          i = i+1;
        }
        if(str == "-fcols")
        {
          if(i+1>=argc)
          { throw("Argument '-fcols' requires the number of cols to be entered");}
          std::stringstream ss;
          ss<<argv[i+1];
          ss>>file_cols;
          i = i+1;
        }
      }

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
      scl_parser::CSclParser tmp_lparser;//Use the scl tinyxml parser

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
      cGenericObject *chai_haptic_cylinder = S_NULL, *chai_haptic_plane = S_NULL, *chai_haptic_sphere_red = S_NULL,
          *chai_haptic_sphere_green = S_NULL;

      Eigen::Vector3d hpos_center_offset;
      hpos_center_offset<<0.0,0.0,0.0;


      /** If the box is not centered */
      // For now, the box's dimensions are 50x50x50 x1000 (mm units). Convert to m units.
      //  = 0.05 x 0.05 x 0.05 m *2 = 0.1 x 0.1 x 0.1 m
      const Eigen::Vector3d box_xyz_translate_center(0.05,0.05,0.05);
      Eigen::Vector3d cylinder_translate_center[3];
      const double box_scaling = 0.002;

      cylinder_translate_center[0]<<0.1, -0.07171, -0.07171;
      cylinder_translate_center[1]<<-0.07171,-0.1,0.07171;
      cylinder_translate_center[2]<<-0.07171,0.07171,-0.10;

      /** Render a sphere at the haptic point's position*/
      flag = chai_gr.addSphereToRender(Eigen::Vector3d::Zero(), chai_haptic_pos, 0.02);
      if(false == flag) { throw(std::runtime_error("Could not add sphere at com pos"));  }
      if(NULL == chai_haptic_pos){ throw(std::runtime_error("Could not add sphere at com pos"));  }


      /** Render a sphere obj at the haptic point's desired position
      flag = chai_gr.addMeshToRender("haptic_pos","./graphics/sphere.obj",
          Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
      if(false == flag) { throw(std::runtime_error("Could not add mesh green sphere at com desired pos"));  }
      chai_haptic_pos = chai_gr.getChaiData()->meshes_rendered_.at("haptic_pos")->graphics_obj_;
      if(NULL == chai_haptic_pos){ throw(std::runtime_error("Could not add mesh sphere at com desired pos"));  }
      chai_haptic_pos->scale(.03,true);
      //chai_haptic_pos->setTransparencyLevel(.88,true,false);
      chai_haptic_pos->setEnabled(true,true);
      */

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

      /** Render a cylinder at the haptic point's desired position */
      flag = chai_gr.addMeshToRender("haptic_cylinder","./graphics/cylinder100mm.obj",
          Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
      if(false == flag) { throw(std::runtime_error("Could not add mesh cylinder at com desired pos"));  }
      chai_haptic_cylinder = chai_gr.getChaiData()->meshes_rendered_.at("haptic_cylinder")->graphics_obj_;
      if(NULL == chai_haptic_cylinder){ throw(std::runtime_error("Could not add mesh cylinder at com desired pos"));  }
      // Cylinder dimensions are 71mm(inner diameter) x 100mm(height) Convert to m units.
      chai_haptic_cylinder->scale(box_scaling,true);
      chai_haptic_cylinder->setTransparencyLevel(0.1,true,false);
      chai_haptic_cylinder->setEnabled(false,true);

      /** Render a plane */
      flag = chai_gr.addMeshToRender("haptic_plane","./graphics/plane.obj",
          Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
      if(false == flag) { throw(std::runtime_error("Could not add plane at com desired pos"));  }
      chai_haptic_plane = chai_gr.getChaiData()->meshes_rendered_.at("haptic_plane")->graphics_obj_;
      if(NULL == chai_haptic_plane){ throw(std::runtime_error("Could not add plane at com desired pos"));  }
      chai_haptic_plane->setEnabled(true,true);
      chai_haptic_plane->setLocalPos(0,0,-0.25);
      //change color of plane? Lighting?

      //chai_haptic_cylinder->setColor(198,242,255,0.5);

      // Set up some materials
      cMaterial mat_green; mat_green.setGreenForest();
      cMaterial mat_red; mat_red.setRedFireBrick();

      /*****************************fMRI State Machine************************************/
      /** The first task ids are for the actual states */
      enum fMRIState {FMRI_RESTING, FMRI_HOLD, FMRI_PLANNING, FMRI_EXECUTING, FMRI_ACTIVE_HOLD, FMRI_HOLD_VIS, FMRI_PLAN_VIS, FMRI_EXECUTE_VIS, FMRI_ACTIVE_HOLD_VIS, FMRI_REST_VIS, FMRI_CYLINDER,
        /** The remaining enum ids are for the state initialization code */
        FMRI_INCREMENT_TASK_STATE,
        FMRI_REST_INIT,
        FMRI_HOLD_INIT,
        FMRI_PLAN_INIT,
        FMRI_EXECUTE_INIT,
        FMRI_ACTIVE_HOLD_INIT,
        FMRI_HOLD_VIS_INIT,
        FMRI_PLAN_VIS_INIT,
        FMRI_EXECUTE_VIS_INIT,
        FMRI_ACTIVE_HOLD_VIS_INIT,
        FMRI_REST_VIS_INIT,
        FMRI_CYLINDER_INIT,
        //Cylinder vis? Did not add yet.
        FMRI_EXIT};
      fMRIState state=FMRI_REST_INIT; //Always start off at the rest state

      // ALL THE STATE VARIABLES :
      int state_task_row_in_matrix=0;     //current row in task file
      double state_t_curr = 0.0;          //current time in simulation
      double state_t_curr_task_max=-1.0;  //end time of current task
      int state_force_file_id = 0;        //force file # that will be executed; determined in col 7 (idx = 6) of task file
      double t_file = 0;                  //time elapsed while executing curr active hold (force file)
      double state_t_start = 0;           //start time of current task
      int t_mat_idx = 0;                  //current row in force file
      double t_cfile = 0;                 //time elapsed while executing cylinder state
      int cfile_idx = 0;                  //current row in cylinder file
      int state_cyl_file_id = 0;          //cyl file # that will be executed; col 8 of task file (idx = 7)


      Eigen::Vector3d state_x_des_curr_task(0,0,0);
      Eigen::MatrixXd state_task_selection_matrix;
      Eigen::Vector3d cylinder_pos(0,0,0);

      Eigen::Matrix3d rot_lookup[3];

      /*********************** Make rotation lookup *********************/
      // 0 = x, 1 = y, 2 = z orientations
      //x
      Eigen::Matrix3d rotation_mat_x;
      rotation_mat_x << 0, -1, 0,
          1, 0, 0,
          0, 0, 1;
      rot_lookup[0] = rotation_mat_x;

      //y
      Eigen::Matrix3d rotation_mat_y;
      rotation_mat_y << 0, 0, 1,
          0, 1, 0,
          -1, 0, 0;
      rot_lookup[1] = rotation_mat_y;

      //z
      Eigen::Matrix3d rotation_mat_z;
      rotation_mat_z << 1, 0, 0,
          0, 0, -1,
          0, 1, 0;
      rot_lookup[2] = rotation_mat_z;

      /** Read the states for this trial run.
       * The state machine will accept inputs like:
       * 1          2                    3               4        5       6       7
       * <task-id> <time-at-completion> <time-duration> <x-des> <y-des> <z-des> <force-file>
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
      flag = scl_util::readEigenMatFromFile(state_task_selection_matrix, "taskc1.txt");
      if(false == flag)
      { throw(std::runtime_error("Could not open task input file"));  }

      // Check for a consistent task input file
      if(state_task_selection_matrix.rows()<1)
      { throw(std::runtime_error("Task input file was empty"));  }

      // NOTE TODO : Add more extensive checks on input file.


      /** Read all force files
       * Force ID file:
       * 0  force_file0.txt
       * 1  force_file1.txt
       * .
       * .
       * .
       * k  force_filek.txt
       *
       *****
       *
       * force file format:
       * 0                                    1         2     3
       * <Time (ending time, not duration)> <Force_x> <F_y> <F_z>
       */

      //Read force files
      std::vector<Eigen::MatrixXd> force_files;
      std::ifstream force_IDs;
      Eigen::MatrixXd tmp_file;

      //read index-force_file name lookup file
      force_IDs.open("force_file_IDs.txt");
      string file_line;


      while(std::getline(force_IDs, file_line)){
        //getline(force_IDs, file_line);
        std::istringstream stream(file_line);
        int idx;
        std::basic_string<char, std::char_traits<char>, std::allocator<char>> filename;
        if(!(stream>>idx >>filename)) break;
        //stream >> idx >> filename;
        //read individual force files into force_files
        //TODO instead of 1 row, read actual number of rows in each force file (potentially unique)
        flag = scl_util::readEigenMatFromFile(tmp_file, filename);
        if(flag == false) {
          throw(std::runtime_error("Could not open force_file"));
        }
        else{
          std::cout << filename << " read" << std::endl;
        }
        force_files.push_back(tmp_file);
      }
      force_IDs.close();

      /************** Read cylinder files *********************/
      /** Format:
       * Cylinder ID file:
       * 0  cyl_file_x.txt
       * 1  cyl_file_y.txt
       * 2  cyl_file_z.txt
       *
       *****
       *
       * cylinder file format:
       * 0                                    1         2     3
       * <Time (ending time, not duration)> <dPosition_x> <F_y> <F_z>
       */

      //Read cylinder files
      std::vector<Eigen::MatrixXd> cyl_files;
      std::ifstream cyl_IDs;
      Eigen::MatrixXd tmp_file_c;

      //read index-force_file name lookup file
      cyl_IDs.open("cyl_file_IDs.txt");
      file_line = "";


      while(std::getline(cyl_IDs, file_line)){
        //getline(force_IDs, file_line);
        std::istringstream stream(file_line);
        int idx;
        std::basic_string<char, std::char_traits<char>, std::allocator<char>> filename;
        if(!(stream>>idx >>filename)) break;
        //stream >> idx >> filename;
        //read individual force files into force_files
        flag = scl_util::readEigenMatFromFile(tmp_file_c, filename);
        if(flag == false) {
          throw(std::runtime_error("Could not open cyl_file"));
        }
        else{
          std::cout << filename << " read" << std::endl;
        }
        cyl_files.push_back(tmp_file_c);
      }
      cyl_IDs.close();

      std::cout<<"\nRunning task matrix:"
          <<"\n<task-id> <time-at-completion> <time-duration> <x-des> <y-des> <z-des> <force_file-id>"
          <<"\n"<<state_task_selection_matrix<<"\n";

      if(0!=state_task_selection_matrix(0,0))
      { throw(std::runtime_error("Simulation must start at rest state"));  }

      if(FMRI_REST_INIT!=state)
      { throw(std::runtime_error("Simulation state machine must start at rest init state"));  }

      /*****************************Chai Background Image************************************/
      //taken from fmri_force_sensing_main.cpp
      cBackground bkg_image, bkg_noimage;
      flag = bkg_image.loadFromFile("./Screen1280x800.png");
      if(false == flag)
      { throw(std::runtime_error("Could not load background image."));  }

      cColorf bkg_grey_color(0.5f,0.5f,0.5f);
      bkg_noimage.setUniformColor(bkg_grey_color);

      // Attach the background to the camera's back layer. Chai's default
      // is to support "widgets" like this on layers (see chai code for more).
      chai_gr.getChaiData()->chai_cam_->m_backLayer->addChild(&bkg_image);
      chai_gr.getChaiData()->chai_cam_->m_backLayer->addChild(&bkg_noimage);

      // Enable the background.
      bkg_image.setEnabled(false,false);
      bkg_noimage.setEnabled(true,false);

      /*****************************Chai Background Text************************************/
      cFont *text_font = NEW_CFONTCALIBRI20();
      cLabel *text_label = NULL;
      if(flag_display_timer)
      {
        // Initialize the text label
        text_label = new cLabel(text_font);
        chai_gr.getChaiData()->chai_cam_->m_frontLayer->addChild(text_label);
        text_label->m_fontColor.setBlue();
        text_label->setFontScale(2);
      }

      /******************************Chai Haptics************************************/
      //taken from fmri_force_sensing_main.cpp
      //For controlling op points with haptics
      //The app will support dual-mode control, with the haptics controlling op points.
      bfr::CBfrDriver bfr;

      scl::sBool has_been_init_haptics;
      Eigen::Vector3d haptic_pos_; haptic_pos_<<0,0,0;
      Eigen::Vector3d haptic_base_pos_; haptic_base_pos_<<0,0,0;
      Eigen::Vector3d haptic_force_;

      /* NEW CODE -- HAVING PROBLEMS*/
      //First set up the haptics - taken from fmri_force_sensing_main.cpp
      has_been_init_haptics = bfr.init();
      if(false == has_been_init_haptics)
      { std::cout<<"\nWARNING : Could not connect to haptic device. Proceeding in kbd mode. \n\tDid you run as sudo?"; }

      bfr.getEEZeroPosition(haptic_base_pos_(0),haptic_base_pos_(1),haptic_base_pos_(2));

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
      if(flag_trigger_scan)
      {
        char ch;
        if(false == has_been_init_haptics)
        { std::cout<<"\n WARNING : Could not intialize haptics."; }
        std::cout<<"\n Trigger scan? y/n \n>>";
        std::cin>>ch;
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
          else
          { throw(std::runtime_error("Could not trigger fMRI scan: Is /dev/ttyACM0 connected? $ sudo chown user:user /dev/ttyACM0 ? Else fix code or try manual trigger."));  }

          CloseComport(22);
          std::cout<<"\nClosed com port. Continuing to the experiment.";
          std::cout<<"\n ========= Fmri scan triggered. With haptics. ========";
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
          bool flag_goal_pos_achieved=false;
          // Get latest haptic device position and update the pointer on the
          // screen
          Eigen::Vector3d& hpos = db->s_gui_.ui_point_[0];
          while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
          {
            if(has_been_init_haptics)
            {
              //haptics_.getHapticDevicePosition(0,haptic_pos_);
              //hpos = haptic_pos_;

              hpos = haptic_pos_ - haptic_base_pos_;
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
            // Set the force file id for the current task
            // Also get force_files parameters ready
            state_force_file_id = state_task_selection_matrix(state_task_row_in_matrix,6);
            state_cyl_file_id = state_task_selection_matrix(state_task_row_in_matrix,7);
            //num_forces = force_files[state_force_file_id].rows();
            state_t_start = state_task_selection_matrix(state_task_row_in_matrix,1) -
                state_task_selection_matrix(state_task_row_in_matrix,2);

            // ************** HPos Color ******************

            if( (state_x_des_curr_task + cylinder_pos - hpos).norm() <= DIST_AT_GOAL_ACHIEVED
                && (false == flag_goal_pos_achieved) )
            {
              //Make desired position green/olive
              chai_haptic_pos->setMaterial(mat_green, false);
              // NOTE TODO : This is buggy in chai
              // chai_haptic_pos->setTransparencyLevel(0.5,true,false);
              flag_goal_pos_achieved = true;
            }
            else if( (state_x_des_curr_task + cylinder_pos - hpos).norm() > DIST_AT_GOAL_ACHIEVED
                && (true == flag_goal_pos_achieved) )
            {
              //Make desired position red/orange
              chai_haptic_pos->setMaterial(mat_red, false);
              //chai_haptic_pos->setTransparencyLevel(0.5,true,false);
              // NOTE TODO : This is buggy in chai
              //chai_haptic_pos->setTransparencyLevel(0.9,1,1);
              flag_goal_pos_achieved = false;
            }

            //cylinder color?
            //chai_haptic_cylinder->setMaterial(mat_green,false);

            // ************** STATE MACHINE ******************
            switch(state)
            {
              case FMRI_RESTING:
                if(has_been_init_haptics){
                  bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                }
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_HOLD:
                if(has_been_init_haptics){
                  bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                }
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_PLANNING:
                if(has_been_init_haptics){
                  bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                }
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_EXECUTING:
                if(has_been_init_haptics){
                  bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                }
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_ACTIVE_HOLD:
                //Transition to next state if this state's time has expired:
                if(state_t_curr_task_max < state_t_curr)
                {
                  if(has_been_init_haptics){
                    //set all forces back to zero
                    bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                  }
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                else {
                  // Find out where we are in the force matrix(file).
                  t_file = state_t_curr - state_t_start;

                  while(t_file > force_files[state_force_file_id](t_mat_idx,0)) //t_mat_idx in INIT
                  { //This changes every time as you loop, so you feel a time-varying force..
                    t_mat_idx++;
                  }


                  // No more forces to apply.
                  if(t_mat_idx < force_files[state_force_file_id].rows())
                  {
                    // Now we have the right index in the force file, time to apply forces

                    //This will execute when connected to the haptic device
                    if(has_been_init_haptics){
                      haptic_force_(0) += (force_files[state_force_file_id](t_mat_idx,1) - haptic_force_(0))/5;
                      haptic_force_(1) += (force_files[state_force_file_id](t_mat_idx,2) - haptic_force_(0))/5;
                      haptic_force_(2) += (force_files[state_force_file_id](t_mat_idx,3) - haptic_force_(0))/5;
                      bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2),
                          haptic_force_(0), haptic_force_(1), haptic_force_(2));
                      /*
                      const timespec ts = {0, 25000000};//Sleep for 25ms
                      nanosleep(&ts,NULL);
                      std::cout<<"\nBFR command, sim_time: " <<state_t_curr<<" | f_time: "<<force_files[state_force_file_id](t_mat_idx,0)<<"\tt_file: "<<t_file<<"     \tPos: "<<haptic_pos_.transpose()<<"\t | force: "<<force_files[state_force_file_id](t_mat_idx,1)<<", "<<force_files[state_force_file_id](t_mat_idx,2)<<", "<<force_files[state_force_file_id](t_mat_idx,3)<<std::flush;
                      */
#ifdef DEBUG
                      const timespec ts = {0, 50000000};//Sleep for 50ms
                      nanosleep(&ts,NULL);

                      std::cout<<"\n\t\tBFR command. Pos: "<<haptic_pos_.transpose()<<". Force: "<<force_files[state_force_file_id](t_mat_idx,1)<<", "<<force_files[state_force_file_id](t_mat_idx,2)<<", "<<force_files[state_force_file_id](t_mat_idx,3)<<std::flush;
#endif

                      hpos = haptic_pos_ - haptic_base_pos_;
                    }/*
                    else {
                      const timespec ts = {0, 15000000};//Sleep for 15ms
                      nanosleep(&ts,NULL);

                    std::cout<<"\nBFR command. Time: " <<state_t_curr<<" apply force (t): "<<t_file<< "Pos: "<<haptic_pos_.transpose()<<". Force: "
                        <<force_files[state_force_file_id](t_mat_idx,1)<<", "
                        <<force_files[state_force_file_id](t_mat_idx,2)<<", "
                        <<force_files[state_force_file_id](t_mat_idx,3)<< std::flush;

                    }*/
                  }
                }

                break;

              case FMRI_CYLINDER:
                if(has_been_init_haptics){
                  bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                }
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                else {
                  t_cfile = state_t_curr - state_t_start;

                  while(t_cfile > cyl_files[state_cyl_file_id](cfile_idx,0)) //t_mat_idx in INIT
                  { //This changes every time as you loop, so you feel a time-varying force..
                    cfile_idx++;
                  }
                  state = FMRI_CYLINDER_INIT;
                }
                break;

              case FMRI_HOLD_VIS:
                if(has_been_init_haptics){
                  bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                }
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_PLAN_VIS:
                if(has_been_init_haptics){
                  bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                }
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_EXECUTE_VIS:
                if(has_been_init_haptics){
                  bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                }
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_ACTIVE_HOLD_VIS:
                //Transition to next state if this state's time has expired:
                if(state_t_curr_task_max < state_t_curr)
                {
                  if(has_been_init_haptics){
                    //set all forces back to zero
                    bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                  }
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                else {
                  // Find out where we are in the force matrix(file).
                  t_file = state_t_curr - state_t_start;

                  while(t_file > force_files[state_force_file_id](t_mat_idx,0)) //t_mat_idx in INIT
                  { //This changes every time as you loop, so you feel a time-varying force..
                    t_mat_idx++;
                  }


                  // No more forces to apply.
                  if(t_mat_idx < force_files[state_force_file_id].rows())
                  {
                    // Now we have the right index in the force file, time to apply forces

                    //This will execute when connected to the haptic device
                    if(has_been_init_haptics){
                      haptic_force_(0) += (force_files[state_force_file_id](t_mat_idx,1) - haptic_force_(0))/5;
                      haptic_force_(1) += (force_files[state_force_file_id](t_mat_idx,2) - haptic_force_(0))/5;
                      haptic_force_(2) += (force_files[state_force_file_id](t_mat_idx,3) - haptic_force_(0))/5;
                      bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2),
                          haptic_force_(0), haptic_force_(1), haptic_force_(2));
                      /*
                      const timespec ts = {0, 25000000};//Sleep for 25ms
                      nanosleep(&ts,NULL);
                      std::cout<<"\nBFR command, sim_time: " <<state_t_curr<<" | f_time: "<<force_files[state_force_file_id](t_mat_idx,0)<<"\tt_file: "<<t_file<<"     \tPos: "<<haptic_pos_.transpose()<<"\t | force: "<<force_files[state_force_file_id](t_mat_idx,1)<<", "<<force_files[state_force_file_id](t_mat_idx,2)<<", "<<force_files[state_force_file_id](t_mat_idx,3)<<std::flush;
                      */
#ifdef DEBUG
                      const timespec ts = {0, 50000000};//Sleep for 50ms
                      nanosleep(&ts,NULL);

                      std::cout<<"\n\t\tBFR command. Pos: "<<haptic_pos_.transpose()<<". Force: "<<force_files[state_force_file_id](t_mat_idx,1)<<", "<<force_files[state_force_file_id](t_mat_idx,2)<<", "<<force_files[state_force_file_id](t_mat_idx,3)<<std::flush;
#endif

                      hpos = haptic_pos_ - haptic_base_pos_;
                    }/*
                    else {
                      const timespec ts = {0, 15000000};//Sleep for 15ms
                      nanosleep(&ts,NULL);

                    std::cout<<"\nBFR command. Time: " <<state_t_curr<<" apply force (t): "<<t_file<< "Pos: "<<haptic_pos_.transpose()<<". Force: "
                        <<force_files[state_force_file_id](t_mat_idx,1)<<", "
                        <<force_files[state_force_file_id](t_mat_idx,2)<<", "
                        <<force_files[state_force_file_id](t_mat_idx,3)<< std::flush;

                    }*/
                  }
                }
                break;

              case FMRI_REST_VIS:
                if(has_been_init_haptics){
                  bfr.readEEPositionAndCommandEEForce(haptic_pos_(0),haptic_pos_(1),haptic_pos_(2), 0, 0, 0);
                }
                //Transition to next state:
                if(state_t_curr_task_max < state_t_curr)
                {
                  std::cout<<"\n"<<state_t_curr<<" : Switching to : FMRI_INCREMENT_TASK_STATE"<<std::flush;
                  state = FMRI_INCREMENT_TASK_STATE;
                }
                break;

              case FMRI_INCREMENT_TASK_STATE:  // Set state_x_des_curr_task
                //Transition to next state:
                //std::cout<<"\nSwitching states, previous state vars-- state_t_curr: "<<state_t_curr<<", state_t_curr_task_max: "<<state_t_curr_task_max<<std::flush;
                cylinder_pos.setZero();
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
                      state = FMRI_HOLD_INIT;
                      break;
                    case 2:
                      state = FMRI_PLAN_INIT;
                      break;
                    case 3:
                      state = FMRI_EXECUTE_INIT;
                      break;
                    case 4:
                      state = FMRI_ACTIVE_HOLD_INIT;
                      break;
                    case 5:
                      state = FMRI_CYLINDER_INIT;
                      std::cout<<"\nSwitching to : FMRI_EXECUTING"<<std::flush;
                      break;
                    case 6:
                      state = FMRI_HOLD_VIS_INIT;
                      break;
                    case 7:
                      state = FMRI_PLAN_VIS_INIT;
                      break;
                    case 8:
                      state = FMRI_EXECUTE_VIS_INIT;
                      break;
                    case 9:
                      state = FMRI_ACTIVE_HOLD_VIS_INIT;
                      break;
                    case 10:
                      state = FMRI_REST_VIS_INIT;
                      break;
                    case 11: //don't need this...
                      state = FMRI_EXIT;
                      break;
                  }
                }
                break;

              case FMRI_REST_INIT:
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(false,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                chai_haptic_cylinder->setEnabled(false,true);

                //Goal point
                chai_haptic_pos_des->setEnabled(false,true);
                //Transition to next state:
                std::cout<<"\nSwitching to : FMRI_RESTING"<<std::flush;
                state = FMRI_RESTING;
                break;

              case FMRI_HOLD_INIT:  // R : yes, G : no, Y : no
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(true,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                chai_haptic_cylinder->setEnabled(false,true);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_box_des_red->setLocalPos(state_x_des_curr_task-box_xyz_translate_center);
                chai_haptic_pos_des->setLocalPos(state_x_des_curr_task);
                //Transition to next state:
                std::cout<<"\nSwitching to : FMRI_HOLD"<<std::flush;
                state = FMRI_HOLD;
                break;

              case FMRI_PLAN_INIT:    // R : yes, G : no, Y : yes
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(true,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(true,true);
                chai_haptic_cylinder->setEnabled(false,true);
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
                chai_haptic_cylinder->setEnabled(false,true);
                //Goal box pos
                chai_haptic_box_des_green->setLocalPos(state_x_des_curr_task-box_xyz_translate_center);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(state_x_des_curr_task);
                //Switch tasks
                std::cout<<"\nSwitching to : FMRI_EXECUTING"<<std::flush;
                state = FMRI_EXECUTING;
                break;

              case FMRI_ACTIVE_HOLD_INIT:  // R : yes, G : no, Y : no, copied from FMRI_HOLD_INIT
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(true,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                chai_haptic_cylinder->setEnabled(false,true);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(0,0,0);

                t_mat_idx = 0;

                std::cout<<"\nSwitching to : FMRI_ACTIVE_HOLD, state_force_file_id: "<<state_force_file_id<<std::flush;
                state = FMRI_ACTIVE_HOLD;
                break;

              case FMRI_CYLINDER_INIT:  // R : yes, G : no, Y : no, copied from FMRI_EXECUTE_INIT
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(false,true);
                chai_haptic_box_des_green->setEnabled(true,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                chai_haptic_cylinder->setEnabled(true,true);
                //Goal box pos
                cylinder_pos(0) = cyl_files[state_cyl_file_id](cfile_idx,1);
                cylinder_pos(1) = cyl_files[state_cyl_file_id](cfile_idx,2);
                cylinder_pos(2) = cyl_files[state_cyl_file_id](cfile_idx,3);
                chai_haptic_box_des_green->setLocalPos(state_x_des_curr_task+cylinder_pos-box_xyz_translate_center);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(state_x_des_curr_task+cylinder_pos);
                //cylinder
                chai_haptic_cylinder->setLocalRot(rot_lookup[state_cyl_file_id]);
                chai_haptic_cylinder->setLocalPos(state_x_des_curr_task+cylinder_translate_center[state_cyl_file_id]);
                //Switch tasks
                //std::cout<<"\nSwitching to : FMRI_EXECUTING"<<std::flush;
                state = FMRI_CYLINDER;
                cfile_idx = 0;
                break;

              case FMRI_HOLD_VIS_INIT:  // R : yes, G : no, Y : no, copied from FMRI_HOLD_INIT
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(true,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                chai_haptic_cylinder->setEnabled(false,true);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(0,0,0);
                std::cout<<"\nSwitching to : FMRI_HOLD_VIS"<<std::flush;
                state = FMRI_HOLD_VIS;
                break;

              case FMRI_PLAN_VIS_INIT:     // R : yes, G : no, Y : yes, copied from FMRI_PLAN_INIT
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(true,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(true,true);
                chai_haptic_cylinder->setEnabled(false,true);
                //Goal box pos
                chai_haptic_box_des_yellow->setLocalPos(state_x_des_curr_task-box_xyz_translate_center);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(state_x_des_curr_task);
                //Switch tasks
                std::cout<<"\nSwitching to : FMRI_PLAN_VIS"<<std::flush;
                state = FMRI_PLAN_VIS;
                break;

              case FMRI_EXECUTE_VIS_INIT:     // R : no, G : yes, Y : no, copied from FMRI_EXECUTE_INIT
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(false,true);
                chai_haptic_box_des_green->setEnabled(true,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                chai_haptic_cylinder->setEnabled(false,true);
                //Goal box pos
                chai_haptic_box_des_green->setLocalPos(state_x_des_curr_task-box_xyz_translate_center);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(state_x_des_curr_task);
                //Switch tasks
                std::cout<<"\nSwitching to : FMRI_EXECUTE_VIS"<<std::flush;
                state = FMRI_EXECUTE_VIS;
                break;

              case FMRI_ACTIVE_HOLD_VIS_INIT:  // R : yes, G : no, Y : no, copied from FMRI_HOLD_INIT
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(true,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                chai_haptic_cylinder->setEnabled(false,true);
                //Goal point
                chai_haptic_pos_des->setEnabled(true,true);
                chai_haptic_pos_des->setLocalPos(0,0,0);
                //Switch tasks
                std::cout<<"\nSwitching to : FMRI_ACTIVE_HOLD_VIS"<<std::flush;
                state = FMRI_ACTIVE_HOLD_VIS;
                break;

              case FMRI_REST_VIS_INIT: //copied from FMRI_REST_INIT
                //Goal boxes
                chai_haptic_box_des_red->setEnabled(false,true);
                chai_haptic_box_des_green->setEnabled(false,true);
                chai_haptic_box_des_yellow->setEnabled(false,true);
                chai_haptic_cylinder->setEnabled(false,true);
                //Goal point
                chai_haptic_pos_des->setEnabled(false,true);
                //Switch tasks
                std::cout<<"\nSwitching to : FMRI_REST_VIS"<<std::flush;
                state = FMRI_REST_VIS;
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
              //Update haptic rate label if required
              if(NULL!=text_label)
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

                //text_label->setString ("Pos: "+cStr(sutil::CSystemClock::getSysTime(), 1) + " [sec]");
                text_label->setString(s);
                int px = (int)(0.5 * (1024 - text_label->getWidth()));
                text_label->setLocalPos(px, 15);
              }

              // Log data at 1ms resolution
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
      bfr.shutdown();
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
