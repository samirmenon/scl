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
/* \file CmdLineArgReader.cpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CmdLineArgReader.hpp"

#include <stdexcept>
#include <iostream>

namespace scl
{
  /** Reads the command line arguments into a cmd arg structure.
   * There are many common arguments that we will populate into a
   * data structure.
   */
  bool cmdLineArgReaderOneRobot(int argc, char** argv, SCmdLineOptions_OneRobot &ret_cmd_ds)
  {
    /******************************Parse Arguments************************************/
    try
    {
      //The index a ctr value refers to in in array of args_parsed = (args_parsed - 1)
      //So ctr for un-parsed arg = (args_parsed - 1) + 1
      int args_ctr = 1;

      // Check that we haven't finished parsing everything
      while(args_ctr < argc)
      {

        if(std::string(argv[args_ctr]) == "-f")
        {
          if(args_ctr+1 >= argc) {  throw(std::runtime_error("Specified -f but did not specify config file name"));  }
          ret_cmd_ds.name_file_config_ = argv[args_ctr+1];
          args_ctr++;
        }
        else if(std::string(argv[args_ctr])== "-r")
        {
          if(args_ctr+1 >= argc) {  throw(std::runtime_error("Specified -r but did not specify robot name"));  }
          ret_cmd_ds.name_robot_ = argv[args_ctr+1];
          args_ctr++;
        }
        else if(std::string(argv[args_ctr]) == "-c")
        {
          if(args_ctr+1 >= argc) {  throw(std::runtime_error("Specified -c but did not specify controller name"));  }
          ret_cmd_ds.name_ctrl_ = argv[args_ctr+1];
          args_ctr++;
        }
        else if(std::string(argv[args_ctr]) == "-t" || std::string(argv[args_ctr]) == "-op" || std::string(argv[args_ctr]) == "-ui")
        {

          if(args_ctr+1 >= argc) { throw(std::runtime_error("Specified -op/-t/-ui but did not specify a control task's name"));  }
          ret_cmd_ds.name_tasks_.push_back(argv[args_ctr+1]);
          args_ctr++;
        }
        else if(std::string(argv[args_ctr]) == "-g")
        {
          if(args_ctr+1 >= argc) {  throw(std::runtime_error("Specified -g but did not specify graphics name"));  }
          ret_cmd_ds.name_graphics_ = argv[args_ctr+1];
          args_ctr++;
        }
        else if (std::string(argv[args_ctr]) == "-p")
        {
          ret_cmd_ds.flag_pause_at_start_ = true;
        }
        else if (std::string(argv[args_ctr]) == "-redis-master")
        {
          ret_cmd_ds.flag_is_redis_master_ = true;
        }
        else if (std::string(argv[args_ctr]) == "-muscles" || std::string(argv[args_ctr]) == "-m" )
        {
          ret_cmd_ds.flag_muscles_ = true;
        }
        else if (std::string(argv[args_ctr]) == "-actuatorset" || std::string(argv[args_ctr]) == "-aset" )
        {
          ret_cmd_ds.name_actuator_set_ = argv[args_ctr+1];
          args_ctr++;
        }

        //Defaults : Assume
        // $ ./<executable> <cfg_file_name> <robot_name> <controller>
        else if(args_ctr == 1)//If none of the options are provided, we'll assume the first option is a file name
        { ret_cmd_ds.name_file_config_ = argv[args_ctr];  }
        else if(args_ctr == 2)//If none of the options are provided, we'll assume the second option is a robot name
        { ret_cmd_ds.name_robot_ = argv[args_ctr];  }
        else if(args_ctr == 3)//If none of the options are provided, we'll assume the third option is a controller name
        { ret_cmd_ds.name_ctrl_ = argv[args_ctr];  }
        else
        { throw(std::runtime_error("Malformed task arguments"));  }

        args_ctr++;
      }

      return true;
    }
    catch(std::exception &e)
    { std::cout<<"\ncommandLineArgumentReader() : "<<e.what(); }
    return false;
  }

}
