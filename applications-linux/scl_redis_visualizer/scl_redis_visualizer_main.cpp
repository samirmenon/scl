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
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>

#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>

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
  if((argc < 2)||(argc > 4))
  {
    std::cout<<"\n The 'scl_redis_visualizer' application can graphically render an scl physics simulation robot with redis io."
        <<"\n ERROR : Provided incorrect arguments. The correct input format is:"
        <<"\n   ./scl_redis_visualizer <file_name.xml> <optional: robot_name> <optional: graphics_name>"
        <<"\n If a robot name isn't provided, the first one from the xml file will be used.\n";
    return 0;
  }
  else
  {
    try
    {
      /******************************Initialization************************************/
      //1. Initialize the database and clock.
      if(false == sutil::CSystemClock::start()) { throw(std::runtime_error("Could not start clock"));  }

      scl::SRobotParsed rds;     //Robot data structure.
      scl::SRobotIO rio;         //I/O data structure.
      scl::SGraphicsParsed rgr;  //Robot graphics data structure.
      scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
      scl::CParserScl p;         //This time, we'll parse the tree from a file.

      /******************************File Parsing************************************/
      std::string tmp_infile(argv[1]);
      std::cout<<"\nRunning scl_redis_sim for input file: "<<tmp_infile;

      std::string robot_name, gr_name;

      std::vector<std::string> str_vec;
      if(argc<3)
      {//Use the first robot spec in the file if one isn't specified by the user.
        flag = p.listRobotsInFile(tmp_infile,str_vec);
        if(false == flag) { throw(std::runtime_error("Could not read robot names from the file"));  }
        robot_name = str_vec[0];//Use the first available robot.
      }
      else { robot_name = argv[2];}

      if(argc<4)
      {
        str_vec.clear();
        flag = p.listGraphicsInFile(tmp_infile,str_vec);
        if(false == flag) { throw(std::runtime_error("Could not read graphics xml from the file"));  }
        gr_name = str_vec[0];//Use the first available graphics
      }
      else { gr_name = argv[3];}

      std::cout<<"\nParsing robot: "<<robot_name;
      if(false == flag) { throw(std::runtime_error("Could not read robot description from file"));  }

      /******************************Load Robot Specification************************************/
      //We will use a slightly more complex xml spec than the first few tutorials
      bool flag = p.readRobotFromFile(tmp_infile,"../../specs/",robot_name,rds);
      flag = flag && rio.init(rds);             //Set up the IO data structure
      if(false == flag){ return 1; }            //Error check.

      /******************************ChaiGlut Graphics************************************/
      glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

      flag = p.readGraphicsFromFile(tmp_infile,gr_name,rgr);
      flag = flag && rchai.initGraphics(&rgr);
      flag = flag && rchai.addRobotToRender(&rds,&rio);
      flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
      if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

      /******************************Graphics Rendering************************************/
      while(scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
        glutMainLoopEvent();

        const timespec ts = {0, 20000000};/*20ms sleep : 50Hz update*/
        nanosleep(&ts,NULL);
      }

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
