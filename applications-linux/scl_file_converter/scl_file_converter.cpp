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
 * scl_file_converter.cpp
 *
 * Application to convert different file types to
 * busylizzy's file format
 *
 *  Created on: Nov 22, 2010
 *      Author: Samir Menon (smenon@stanford.edu)
 */

#include <sstream>

//Standard includes
#include <iostream>
#include <stdexcept>

//Eigen 3rd party lib
#include <Eigen/Dense>

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>

#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/parser/saiparser/CParserSai.hpp>
#include <scl/parser/osimparser/CParserOsim.hpp>
#include <scl/parser/osimparser/CParserOsimForOldFiles.hpp>

#include <scl/util/DatabaseUtils.hpp>


/**
 * A sample application to convert supported files into a scl file format.
 */
int main(int argc, char** argv)
{
  bool flag;
  if(argc != 3)
  {
    std::cout<<"\nThis is a sample application to convert supported files (sai.xml, .osim) into a scl file format.";
    std::cout<<"\nThe command line input is: ./<executable> <input_file_name> <output_file_name>\n";
    std::cout<<"\nNOTE : \n1. Link names must be unique!\n2. Numbers in xml tags must be separated by spaces (not commas!)"
        <<"\n3. A zero rotation in quaternions is {0 0 0 1} NOT {0 0 0 0}"
        <<std::endl<<std::flush;
    return 0;
  }
  else
  {
    try
    {
      /******************************Initialization************************************/
      //1. Set filenames and initialize the database
      std::string tmp_infile(argv[1]);
      std::string tmp_outfile(argv[2]);

      /******************************File Parsing************************************/
      //2. Find out the filetype
      std::string ftype;
      if(static_cast<char>(*(argv[1] + strlen(argv[1])-5))=='.')
      { ftype = static_cast<char*>(argv[1] + strlen(argv[1])-5);  }
      if(static_cast<char>(*(argv[1] + strlen(argv[1])-4))=='.')
      { ftype = static_cast<char*>(argv[1] + strlen(argv[1])-4);  }
      std::cout<<"\nInput File Type: "<<ftype;

      //3. Create a parser that can parse the specific filetype (Parsers inherit from CParserBase)
      scl::SRobotParsed tmp_robot; //Will parse a robot into this data structure.
      tmp_robot.rb_tree_.clear();
      scl::SMuscleSetParsed tmp_msys;
      tmp_msys.muscles_.clear();

      if(ftype == ".xml") //SAI Xml format
      {
        std::cout<<"\nWARNING : SAI support is only for reading ONE robot per file. Will convert the first robot to scl xml.";
        std::cout<<"\nWARNING : SAI support does NOT permit using commas in the xml (,). ONLY use spaces.";
        std::cout<<"\nWARNING : SAI support REQUIRES unique link and joint names for all links.";
        scl::CParserSai tmp_sai_parser;
        flag = tmp_sai_parser.readRobotFromFile(tmp_infile, "I_Am_Superfluous", tmp_robot);
        if(false == flag) { throw(std::runtime_error("Could not read SAI xml file."));  }
      }
      else if(ftype == ".osim")
      {
        std::cout<<"\nWARNING : OSIM support is only for reading ONE humanoid per file. Will convert the first humanoid model.";
        std::cout<<"\nWARNING : OSIM presently loads muscles and rigid bodies. It does not add inter-joint constraints.";
        std::cout<<"\nWARNING : OSIM support REQUIRES unique link and joint names for all links.";
        std::cout<<"\nWARNING : OSIM support REQUIRES a root link named --ground--.";
        scl::CParserOsim tmp_osim_parser;
        flag = tmp_osim_parser.readOsimBiomechFromFile(tmp_infile, tmp_robot, tmp_msys);
        if(false == flag)
        {
          std::cout<<"\nCould not read OSIM xml file. Trying parser for older file types.";
          scl::CParserOsimForOldFiles tmp_osim_parser_old;
          //Clear out stuff that has already been read in.
          tmp_robot.rb_tree_.clear();
          tmp_msys.muscles_.clear();
          //Try the older parser.
          flag = tmp_osim_parser_old.readOsimBiomechFromFile(tmp_infile, tmp_robot, tmp_msys);
          if(false == flag)
          {
            throw(std::runtime_error("Tried 2 file formats but could not read OSIM xml file.\nCall up the osim guys and ask them to de-fragment their file formats! Thanks."));
          }
        }
      }
      else
      { throw(std::runtime_error("Unrecognized input file type."));  }

      //4. Write the file into a scl output file
      scl::CParserScl tmp_scl_parser;
      flag = tmp_scl_parser.saveRobotToFile(tmp_robot,tmp_outfile);
      if(false == flag) { throw(std::runtime_error("Could not write to Scl xml file."));  }

      /****************************Deallocate Memory And Exit*****************************/
      std::cout<<"\nSCL File Converter Executed Successfully";
      std::cout<<"\n*************************\n"<<std::flush;
    }
    catch(std::exception & e)
    {
      std::cout<<"\nSCL File Converter Failed: "<< e.what();
      std::cout<<"\n*************************\n"<<std::flush;
    }
    return 0;
  }
}
