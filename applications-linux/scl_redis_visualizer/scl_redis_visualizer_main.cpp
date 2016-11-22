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
      <<"\n        appropriate keys are set";

  bool flag;
  if(argc < 2)
  {
    std::cout<<"\n The 'scl_redis_visualizer' application can graphically render an scl physics simulation robot with redis io."
        <<"\n ERROR : Provided incorrect arguments. The correct input format is:"
        <<"\n   ./scl_redis_visualizer <file_name.xml> <optional: -r robot_name> <optional: -g graphics_name> <optional : -muscles/-m>"
        <<"\n If a robot or graphics name isn't provided, the first one from the xml file will be used. Muscles are not rendered by default.\n";
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

      /******************************File Parsing************************************/
      std::cout<<"\nRunning scl_redis_sim for input file: "<<rcmd.name_file_config_;

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
      std::cout<<"\nParsing robot: "<<rcmd.name_robot_;
      bool flag = p.readRobotFromFile(rcmd.name_file_config_,"../../specs/",rcmd.name_robot_,rds);
      flag = flag && rio.init(rds);             //Set up the IO data structure
      if(false == flag) { throw(std::runtime_error("Could not read robot description from file"));  }

      /******************************Load Robot Muscle Specification************************************/
      scl::SActuatorSetMuscleParsed *rob_mset_ds = NULL;  // This is the parsed muscle specification (not modified during runtime)
      scl::SActuatorSetMuscle *rob_mset_ds_dyn = NULL;    // This is the actuator force description (stored in rio and modified in runtime)

      char rstr_musclekey[1024];//For redis key formatting
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

      /******************************Redis Initialization************************************/
      char rstr_q_key[1024]; //For redis key formatting
      sprintf(rstr_q_key, "scl::robot::%s::sensors::q",rcmd.name_robot_.c_str());

      char rstr_campos_key[1024],rstr_camlookat_key[1024], rstr_camup_key[1024]; //For ui redis key formatting
      sprintf(rstr_campos_key, "scl::robot::%s::ui::cam_pos",rcmd.name_robot_.c_str());
      sprintf(rstr_camlookat_key, "scl::robot::%s::ui::cam_lookat",rcmd.name_robot_.c_str());
      sprintf(rstr_camup_key, "scl::robot::%s::ui::cam_up",rcmd.name_robot_.c_str());

      scl::CIORedis ioredis;
      scl::SIORedis ioredis_ds;
      flag = ioredis.connect(ioredis_ds,false);
      if(false == flag)
      { throw(std::runtime_error( std::string("Could not connect to redis server : ") + std::string(ioredis_ds.context_->errstr) ));  }

      std::cout<<"\n Monitoring REDIS key : "<<rstr_q_key;
      std::cout<<"\n ** To monitor Redis messages, open a redis-cli and type 'monitor' **";

      /* ************** Set up UI keys *************************** */
      scl_chai_glut_interface::SChaiGlobals* chai_glob_ds = scl_chai_glut_interface::CChaiGlobals::getData();
      if(S_NULL == chai_glob_ds) { throw(std::runtime_error("Chai shared data singleton not initialized")); }

      ioredis.set(ioredis_ds,rstr_campos_key, rgr.cam_pos_);
      ioredis.set(ioredis_ds,rstr_camlookat_key, rgr.cam_lookat_);
      ioredis.set(ioredis_ds,rstr_camup_key, rgr.cam_up_);

      /******************************Graphics Rendering************************************/
      while(scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        // Update graphics
        glutMainLoopEvent();

        // **************************************************************************************
        //                                User Interface IO
        // **************************************************************************************
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
          { std::cout<<"\n WARNING : Missing redis key: "<<rstr_musclekey<<" size("<< rob_mset_ds->muscles_.size()<<"). Will wait for it..."; }
        }
        // **************************************************************************************

        const timespec ts = {0, 25000000};/*25ms sleep : ~40Hz update*/ nanosleep(&ts,NULL);
      }

      /* ************** Clean up UI keys *************************** */
      ioredis.del(ioredis_ds,rstr_campos_key);
      ioredis.del(ioredis_ds,rstr_camlookat_key);
      ioredis.del(ioredis_ds,rstr_camup_key);


      /******************************Exit Gracefully************************************/
      std::cout<<"\n\nExecuted Successfully";
      std::cout<<"\n**********************************\n"<<std::flush;

      return 0;
    }
    catch(std::exception & e)
    {
      std::cout<<"\nSCL Failed: "<< e.what();
      std::cout<<"\n*************************\n";
      return 1;
    }
  }
}
