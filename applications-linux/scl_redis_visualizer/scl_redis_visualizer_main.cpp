/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* \file scl_redis_visualizer_main.cpp
 *
 *  Created on: Jul 10, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/scl.hpp>

#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>

//Freeglut windowing environment
#include <GL/freeglut.h>

/** A sample application to render a physics simulation being run in scl.
 *
 * It will display the robot in an OpenGL graphics window. */
int main(int argc, char** argv)
{
  std::cout<<"\n*******************************************************\n";
  std::cout<<  "               SCL Redis Visualizer";
  std::cout<<"\n*******************************************************\n";
  std::cout<<"\n NOTE : This application assumes a default redis server is "
      <<"\n        running on the standard port (6379) and that "
      <<"\n        appropriate keys are set\n";

  bool flag;
  if(argc < 2)
  {
    std::cout<<"\n The 'scl_redis_visualizer' application can graphically render an scl physics simulation robot with redis io."
        <<"\n\n ERROR : Provided incorrect arguments. The correct input format is:"
        <<"\n\n   ./scl_redis_visualizer <file_name.xml> <options...>"
        <<"\n\n    Command Line Options : \n"
        <<"\n          -r <robot_name; default = first in xml file> "
        <<"\n          -g <graphics_name; default = first in xml file>"
        <<"\n          -muscles/-m : Activate muscle rendering; default = false"
        <<"\n          -redis-master : Sets camera and ui-pos/flag/int keys in redis; default = false\n\n";
    return 0;
  }
  else
  {
    try
    {
      /******************************Parse Command Line Args************************************/
      scl::SCmdLineOptions_OneRobot rcmd;
      flag = scl::cmdLineArgReaderOneRobot(argc,argv,rcmd);
      if(false == flag) { throw(std::runtime_error("Could not parse command line arguments"));  }

      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

      //2. Set up dynamic types
      scl::init::registerNativeDynamicTypes();

      scl::SRobotParsed rds;     //Robot data structure.
      scl::SRobotIO rio;         //I/O data structure.
      scl::SGraphicsParsed rgr;  //Robot graphics data structure.
      scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
      scl::CParserScl p;         //This time, we'll parse the tree from a file.

      bool flag_write_ui_to_redis = rcmd.flag_is_redis_master_; // Will depend on command line args. False by default.
      std::string ui_master_current;

      /******************************File Parsing************************************/
      std::cout<<"\n - Running scl_redis_sim for input file: "<<rcmd.name_file_config_;

      std::vector<std::string> str_vec;
      if(rcmd.name_robot_ == "")
      {//Use the first robot spec in the file if one isn't specified by the user.
        flag = p.listRobotsInFile(rcmd.name_file_config_,str_vec);
        if(false == flag) { throw(std::runtime_error("Could not read robot names from the file"));  }
        rcmd.name_robot_ = str_vec[0];//Use the first available robot.
      }

      if(rcmd.name_graphics_ == "")
      {
        str_vec.clear();
        flag = p.listGraphicsInFile(rcmd.name_file_config_,str_vec);
        if(false == flag) { throw(std::runtime_error("Could not read graphics xml from the file"));  }
        rcmd.name_graphics_ = str_vec[0];//Use the first available graphics
      }

      /******************************Load Robot Specification************************************/
      std::cout<<"\n - Parsing robot: "<<rcmd.name_robot_;
      bool flag = p.readRobotFromFile(rcmd.name_file_config_,"../../specs/",rcmd.name_robot_,rds);
      flag = flag && rio.init(rds);             //Set up the IO data structure
      if(false == flag) { throw(std::runtime_error("Could not read robot description from file"));  }

      /******************************Load Robot Muscle Specification************************************/
      scl::SActuatorSetMuscleParsed *rob_mset_ds = NULL;  // This is the parsed muscle specification (not modified during runtime)
      scl::SActuatorSetMuscle *rob_mset_ds_dyn = NULL;    // This is the actuator force description (stored in rio and modified in runtime)

      char rstr_musclekey[SCL_MAX_REDIS_KEY_LEN_CHARS];//For redis key formatting
      sprintf(rstr_musclekey, "scl::robot::%s::actuators::fm",rcmd.name_robot_.c_str());

      // Also render muscles..
      // NOTE : If the robot has a valid muscle spec, it will already be parsed and ready in the
      // robot data structure. We just have to get it and set the forces.
      // So let's look in the actuator sets and find the first muscle set (there should only
      // be one muscle set, but we'll be lazy for now and ignore the rest if they exist).
      if(rcmd.flag_muscles_)
      {
        // Let's find the parsed muscle actuator set
        rob_mset_ds = scl::dsquery::getFromRobotParsedFirstMuscleSet(rds);
        if(NULL == rob_mset_ds) { throw(std::runtime_error("Did not find muscle actuator set in parsed robot data structure."));  }

        //Now get the dynamic muscle set spec...
        rob_mset_ds_dyn = dynamic_cast<scl::SActuatorSetMuscle*>(*rio.actuators_.actuator_sets_.at(rob_mset_ds->name_));
        if(NULL == rob_mset_ds_dyn) { throw(std::runtime_error("Could not find muscle actuator dyn set in io data structure."));  }

        // Initialize the actuator force vector..
        rob_mset_ds_dyn->force_actuator_.setZero(rob_mset_ds->muscles_.size());
        rob_mset_ds_dyn->force_actuator_.array()+=0.01;//Just to make sure all the muscles aren't green.
      }

      /******************************ChaiGlut Graphics************************************/
      glutInit(&argc, argv); // We will use glut for the window pane (chai for the graphics).

      flag = p.readGraphicsFromFile(rcmd.name_file_config_,rcmd.name_graphics_,rgr);
      flag = flag && rchai.initGraphics(&rgr);
      flag = flag && rchai.addRobotToRender(&rds,&rio);
      flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
      if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

      /****************************** Redis Initialization ************************************/
      // ======= SET UP KEY STRINGS (so later key access is faster)
      char rstr_q_key[SCL_MAX_REDIS_KEY_LEN_CHARS]; //For redis key formatting
      sprintf(rstr_q_key, "scl::robot::%s::sensors::q",rcmd.name_robot_.c_str());

      // Add strings for the standard (cam) ui vars
      char rstr_campos_key[SCL_MAX_REDIS_KEY_LEN_CHARS],rstr_camlookat_key[SCL_MAX_REDIS_KEY_LEN_CHARS],
        rstr_camup_key[SCL_MAX_REDIS_KEY_LEN_CHARS],
        rstr_ui_master[SCL_MAX_REDIS_KEY_LEN_CHARS],
        rstr_ui_id[64]; //For ui redis key formatting
      sprintf(rstr_campos_key, "scl::robot::%s::ui::cam_pos",rcmd.name_robot_.c_str());
      sprintf(rstr_camlookat_key, "scl::robot::%s::ui::cam_lookat",rcmd.name_robot_.c_str());
      sprintf(rstr_camup_key, "scl::robot::%s::ui::cam_up",rcmd.name_robot_.c_str());
      sprintf(rstr_ui_master, "scl::robot::%s::ui::master",rcmd.name_robot_.c_str());
      sprintf(rstr_ui_id, "scl_redis_visualizer::%s", rcmd.id_time_created_str_.c_str());

      // Add strings for the special (data) ui vars
      char rstr_ui_pt[SCL_NUM_UI_POINTS][SCL_MAX_REDIS_KEY_LEN_CHARS], rstr_ui_flag[SCL_NUM_UI_FLAGS][SCL_MAX_REDIS_KEY_LEN_CHARS],
        rstr_ui_int[SCL_NUM_UI_INTS][SCL_MAX_REDIS_KEY_LEN_CHARS];
      for(int i=0; i<SCL_NUM_UI_POINTS;++i)
      { sprintf(rstr_ui_pt[i], "scl::robot::%s::ui::point::%d", rcmd.name_robot_.c_str(), i); }
      for(int i=0; i<SCL_NUM_UI_FLAGS;++i)
      { sprintf(rstr_ui_flag[i], "scl::robot::%s::ui::flag::%d", rcmd.name_robot_.c_str(), i); }
      for(int i=0; i<SCL_NUM_UI_INTS;++i)
      { sprintf(rstr_ui_int[i], "scl::robot::%s::ui::int::%d", rcmd.name_robot_.c_str(), i); }

      // ======= NOW CONNECT TO REDIS SERVER
      scl::CIORedis ioredis;
      scl::SIORedis ioredis_ds;
      flag = ioredis.connect(ioredis_ds,false);
      if(false == flag)
      { throw(std::runtime_error( std::string("Could not connect to redis server : ") + std::string(ioredis_ds.context_->errstr) ));  }

      std::cout<<"\n\n Monitoring REDIS key : "<<rstr_q_key;
      std::cout<<"\n ** To monitor Redis messages, open a redis-cli and type 'monitor' **";

      /* ********************************* Set UI keys in Redis ********************************* */
      scl_chai_glut_interface::SChaiGlobals* chai_glob_ds = scl_chai_glut_interface::CChaiGlobals::getData();
      if(S_NULL == chai_glob_ds) { throw(std::runtime_error("Chai shared data singleton not initialized")); }

      // Get a handle to the gui data structure
      scl::SGuiData &rgui = scl::CDatabase::getData()->s_gui_;

      ioredis.set(ioredis_ds,rstr_campos_key, rgr.cam_pos_);
      ioredis.set(ioredis_ds,rstr_camlookat_key, rgr.cam_lookat_);
      ioredis.set(ioredis_ds,rstr_camup_key, rgr.cam_up_);
      if(flag_write_ui_to_redis) // If command line args say become master, do so and set your key in redis.
      {
        ioredis.set(ioredis_ds,rstr_ui_master, std::string(rstr_ui_id));
        ui_master_current = rstr_ui_id;
        std::cout<<"\n UI window is redis master. Taking control of redis camera keys!";
        std::cout<<"\n If you'd like to take control of redis, change the master key to something else: "<< rstr_ui_master;
        std::cout<<"\n If you'd like to relinquish control of redis later, del the master key: "<< rstr_ui_master<<std::endl;
      }
      else
      { std::cout<<"\n UI window is redis slave. Will never try to take control. \n Will simply read camera keys from redis!" << std::endl; }

      /******************************Graphics Rendering************************************/
      while(scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        // NOTE TODO : Consider having separate update rates for the ui graphics and the glut loop..
        // Update graphics
        glutMainLoopEvent();

        // **************************************************************************************
        //                                User Interface IO
        // **************************************************************************************
        if (rcmd.flag_is_redis_master_)// Will only try to assume control if it had an original command to do so
        {
          // Figure out if the ui is still master
          flag = ioredis.get(ioredis_ds,rstr_ui_master, ui_master_current);
          if(false == flag){
            std::cout<<"\n EVENT : key("<<rstr_ui_master<<") not set."
                <<"\n Someone took ownership and left ownership."
                <<"\n *** Visualizer is taking control of redis keys again ***"<<std::flush;
            ui_master_current=rstr_ui_id;
            ioredis.set(ioredis_ds,rstr_ui_master, ui_master_current);
            flag_write_ui_to_redis = true;
          }
          else{
            if(ui_master_current == std::string(rstr_ui_id))
            { flag_write_ui_to_redis = true;  }// still the master
            else{ flag_write_ui_to_redis = false; }
          }
        }

        if(flag_write_ui_to_redis && rcmd.flag_is_redis_master_)
        {
          // Set the camera position.
          ioredis.set(ioredis_ds,rstr_campos_key, rgr.cam_pos_);
          ioredis.set(ioredis_ds,rstr_camlookat_key, rgr.cam_lookat_);
          ioredis.set(ioredis_ds,rstr_camup_key, rgr.cam_up_);

          // Also set the ui points etc.
          for(int i=0; i<SCL_NUM_UI_POINTS;++i)
          { ioredis.set(ioredis_ds, rstr_ui_pt[i], rgui.ui_point_[i]);  }
          for(int i=0; i<SCL_NUM_UI_FLAGS;++i)
          { ioredis.set(ioredis_ds, rstr_ui_flag[i], rgui.ui_flag_[i]);  }
          for(int i=0; i<SCL_NUM_UI_INTS;++i)
          { ioredis.set(ioredis_ds, rstr_ui_int[i], rgui.ui_int_[i]);  }
        }
        else
        {
          // Update the camera position.
          ioredis.get(ioredis_ds,rstr_campos_key, rgr.cam_pos_);
          ioredis.get(ioredis_ds,rstr_camlookat_key, rgr.cam_lookat_);
          ioredis.get(ioredis_ds,rstr_camup_key, rgr.cam_up_);

          // Set new position to camera
          chai_glob_ds->cam_lookat_x_ = rgr.cam_lookat_(0);
          chai_glob_ds->cam_lookat_y_ = rgr.cam_lookat_(1);
          chai_glob_ds->cam_lookat_z_ = rgr.cam_lookat_(2);

          // Move to spherical coordinates so the mouse rotation still works.
          chai_glob_ds->cam_sph_x_=(rgr.cam_pos_ - rgr.cam_lookat_).norm();
          chai_glob_ds->cam_sph_v_= 180/M_PI*(std::asin(rgr.cam_pos_[2]/chai_glob_ds->cam_sph_x_));
          chai_glob_ds->cam_sph_h_= 180/M_PI*(std::asin(rgr.cam_pos_[1]/
              (chai_glob_ds->cam_sph_x_*chai3d::cCosDeg(chai_glob_ds->cam_sph_v_))));

          scl_chai_glut_interface::updateCameraPosition();

          // Update the ui points
          for(int i=0; i<SCL_NUM_UI_POINTS;++i)
          { ioredis.get(ioredis_ds, rstr_ui_pt[i], rgui.ui_point_[i]);  }
          for(int i=0; i<SCL_NUM_UI_FLAGS;++i)
          { ioredis.get(ioredis_ds, rstr_ui_flag[i], rgui.ui_flag_[i]);  }
          for(int i=0; i<SCL_NUM_UI_INTS;++i)
          { ioredis.get(ioredis_ds, rstr_ui_int[i], rgui.ui_int_[i]);  }
        }

        // **************************************************************************************
        //                                ROBOT STATE IO
        // **************************************************************************************
        // REDIS IO : Get q key (we assume it will exist else would have thrown an error earlier)
        flag = ioredis.get(ioredis_ds,rstr_q_key,rio.sensors_.q_);
        if(false == flag || rio.sensors_.q_.rows() != rio.dof_)
        {
          rio.sensors_.q_.setZero(rio.dof_); // We'll set the key to zero just to be safe..
          std::cout<<"\n WARNING : Missing redis key: "<<rstr_q_key<<" size("<< rio.dof_<<"). Will wait for it...";
          const timespec ts = {0, 200000000};/*200ms sleep */ nanosleep(&ts,NULL); continue;
        }

        if(rcmd.flag_muscles_)
        {
          flag = ioredis.get(ioredis_ds,rstr_musclekey,rob_mset_ds_dyn->force_actuator_);
          if(false == flag)
          {
            std::cout<<"\n WARNING : Missing redis key: "<<rstr_musclekey<<" size("<< rob_mset_ds->muscles_.size()
                <<"). Will wait for it...";
            const timespec ts = {0, 200000000};/*200ms sleep */ nanosleep(&ts,NULL); continue;
          }

        }
        // **************************************************************************************

        const timespec ts = {0, 25000000};/*25ms sleep : ~40Hz update*/ nanosleep(&ts,NULL);
      }

      /* ************** Clean up UI keys *************************** */
      ioredis.del(ioredis_ds,rstr_campos_key);
      ioredis.del(ioredis_ds,rstr_camlookat_key);
      ioredis.del(ioredis_ds,rstr_camup_key);
      if(flag_write_ui_to_redis) // If command line args say become master, clear keys
      {// If someone else has assumed master's role, do not clear the keys
        ioredis.del(ioredis_ds,rstr_ui_master);

        for(int i=0; i<SCL_NUM_UI_POINTS;++i)
        { ioredis.del(ioredis_ds, rstr_ui_pt[i]);  }
        for(int i=0; i<SCL_NUM_UI_FLAGS;++i)
        { ioredis.del(ioredis_ds, rstr_ui_flag[i]);  }
        for(int i=0; i<SCL_NUM_UI_INTS;++i)
        { ioredis.del(ioredis_ds, rstr_ui_int[i]);  }
      }

      /******************************Exit Gracefully************************************/
      std::cout<<"\n Executed Successfully";
      std::cout<<"\n**********************************\n"<<std::flush;

      return 0;
    }
    catch(std::exception & e)
    {
      std::cout<<"\n SCL Failed: "<< e.what();
      std::cout<<"\n*************************\n";
      return 1;
    }
  }
}
