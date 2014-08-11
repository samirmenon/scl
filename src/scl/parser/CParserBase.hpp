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
/* \file CParserBase.hpp
 *
 *  Created on: May, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CPARSERBASE_HPP_
#define CPARSERBASE_HPP_

//Data structures passed to the branching template
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SGraphicsParsed.hpp>
#include <scl/data_structs/SMuscleSetParsed.hpp>
#include <scl/data_structs/SUIParsed.hpp>

#include <scl/control/gc/data_structs/SControllerGc.hpp>
#include <scl/control/task/data_structs/SControllerMultiTask.hpp>

#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <scl/DataTypes.hpp>

namespace scl
{
  /** This class is a base for reading a robot definition into an SRobotParsed
	 * data structure.
	 *
	 * All file-format specific parsers will subclass this API class. */
	class CParserBase
	{
	public:
		/** Default constructor. Does nothing. */
	  CParserBase() {}

	  /** Default destructor. Does nothing. */
		virtual ~CParserBase(){}

    /** Returns a list of robots and the files in which their
     * specifications reside. */
    virtual bool listRobotsInFile(const std::string& arg_file,
        std::vector<std::string>& arg_robot_names)=0;

		/** Required : Reads in a robot from the given file.
		 *
		 * This function must be implemented by parsers who inherit from
		 * CParserBase. Its implementation will depend on the file format
		 * supported.
		 *
		 * NOTE : Subclass implementation details for the readRobotFromFile() function:
     *   1) Add a bunch of SRigidBody links to the SRobotParsed's branching representation,
     *      "robot_.robot_br_rep_", after reading in the values from a file
     *   2) When all the links have been read in, call "robot_.robot_br_rep_.linkNodes()"
     *
     * If you want to implement a new parser for a new filetype, please see how you
     * should populate the SRobotParsed data structure from an existing parser.
     * Eg. The Scl parser implements the full API
		 */
    virtual bool readRobotFromFile(const std::string& arg_file,
        /** The spec which will be instantiated in the object */
        const std::string& arg_robot_name,
        /** The robot's data will be filled into
         * this data structure */
        scl::SRobotParsed& arg_robot)=0;

    /** Saves a robot definition to file.
     * Takes the name of the robot and a file name as aguments.
     *
     * Looks up the robot in the database and writes it to a file
     * if it exists. */
    virtual bool saveRobotToFile(scl::SRobotParsed& arg_robot,
        const std::string &arg_file)
    { return false; }

    /** Optional: Returns a list of muscle systems in a file.
     * Required : If readMuscleSysFromFile() is implemented */
    virtual bool listMuscleSysInFile(const std::string& arg_file,
        std::vector<std::string>& arg_msys_names)
    { return S_NULL;  }

    /** Reads a muscle model from a file. */
    virtual bool readMuscleSysFromFile(const std::string& arg_file,
                const std::string& arg_msys_name,
                scl::SMuscleSetParsed& arg_msys)
    { return false; }

    /** Optional: Returns a list of graphics views in a file.
     * Required : If readGraphicsFromFile() is implemented */
    virtual bool listGraphicsInFile(const std::string& arg_file,
            std::vector<std::string>& arg_graphics_names)
		{ return false;  }

		/** Optional: If required, the subclass may support parsing graphics
     * specifications. */
    virtual bool readGraphicsFromFile(const std::string &arg_file,
        const std::string &arg_graphics_name,
        /** The robot's data will be filled into
         * this data structure */
        scl::SGraphicsParsed& arg_graphics)
    { return false; }

    /** Optional: If required, the subclass may support parsing user interface
     * specifications. */
    virtual bool listUISpecsInFile(const std::string& arg_file,
              std::vector<std::string>& arg_ui_spec_names)
    { return false; }

    /** Optional: If required, the subclass may support parsing user interface
     * specifications. */
    virtual bool readUISpecFromFile(const std::string &arg_file,
        const std::string &arg_ui_spec_name,
        scl::SUIParsed& arg_ui_spec)
    { return false; }

    /** Optional: The subclass may support read out the controller
     * specifications in a file. Each controller is to be stored as
     * a <name, type> pair */
    virtual bool listControllersInFile(const std::string &arg_file,
          std::vector<std::pair<std::string,std::string> > &arg_ctrl_name_and_type)
    { return false; }

    /** Optional: The subclass may support parsing gc controller
     * specifications (There are 2 different controller reading
     * functions because the data structures are totally incompatible) */
    virtual bool readGcControllerFromFile(const std::string &arg_file,
        const std::string &arg_ctrl_name,
        std::string &ret_must_use_robot,
        scl::SControllerGc& arg_ctrl)
    { return false; }

    /** Optional: The subclass may support parsing task controller
     * specifications (There are 2 different controller reading
     * functions because the data structures are totally incompatible) */
    virtual bool readTaskControllerFromFile(const std::string &arg_file,
        const std::string &arg_ctrl_name,
        std::string &ret_must_use_robot,
        /** Returns a vector of tasks that this task controller can execute.
         * Typically, the task controller will execute all of them simultaneously
         * with a priority order.*/
        std::vector<scl::STaskBase*> &ret_taskvec,
        std::vector<scl::SNonControlTaskBase*> &ret_task_non_ctrl_vec)
    { return false; }
};

}//End of namespace scl_parser

#endif /*CPARSERBASE_HPP_*/
