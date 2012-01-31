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
/* \file COsimParser.hpp
 *
 *  Created on: May, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef COSIMPARSER_HPP_
#define COSIMPARSER_HPP_

#include <scl/DataTypes.hpp>
#include <scl/parser/CParserBase.hpp>

//The tinyxml parser implementation for sai xml files
#include <scl/parser/scl_tinyxml/scl_tinyxml.h>


namespace scl_parser
{

  /** This class implements a limited subset of the CParserBase API
   * for the "Osim" file format. */
  class COsimParser: public CParserBase
  {
  public:
    /** Only support reading from Osim files. Use the file converter
     * to convert them into Lotus files to also add controllers.
     * (applications/scl_file_converter)
     *
     * Tags not supported:
     * 1. MuscleViaPoint :
     *    A class implementing a conditional path point, which is a point that
     *    is active only for a specified range of a coordinate.
     *    We will NOT SUPPORT this point-off mechanism.
     *
     * 2. MovingMusclePoint :
     *    These seem redundant. We will split osim nodes into n links,
     *    one for each coordinate. The via point will be attached to the last link in a
     *    series of links that correspond to a node's coordinates.
     *    Eg. node_name : coord_tx, coord_ty, coord_rz will create three links:
     *        node_name_coord_tx, node_name_coord_ty, node_name_coord_rz and the
     *        moving muscle point will be attached to the last one.
     *
     * Notes:
     * 1. Throws errors when required tags are missing
     * 2. Prints warnings when non-required but important (dynamics info) tags are missing
     * 3. Does nothing when default values can usually replace missing tags. Eg. joint limits
     * 4. Osim seems to support w,x,y,z quaternions in its orientation tag. Eigen supports x,y,z,w.
     */
    virtual bool readOsimBiomechFromFile(
        const std::string& arg_file,
        scl::SRobotParsedData& arg_biomech,
        scl::SMuscleSystem& arg_msys);

    /* Since the Osim format only has one robot in a file, the "arg_msys_name"
     * argument is not used.
     *
     * Model
     * - Actuator Set
     *   - Objects
     *     -Thelen2003Muscle (reads contents of this)
     *     -Schutte1993Muscle (reads contents of this) */
    virtual bool readMuscleSysFromFile(
        const std::string& arg_file,
        const std::string& arg_msys_name,
        scl::SMuscleSystem& arg_msys);

    /* Since the Osim format only has one robot in a file, the "arg_robot_name"
     * argument is not used. */
    virtual bool readRobotFromFile(
        const std::string& arg_file,
        const std::string& arg_robot_name,
        scl::SRobotParsedData& arg_robot);

    virtual bool listRobotsInFile(const std::string& arg_file,
        std::vector<std::string>& arg_robot_names)
    { return false; }

    COsimParser() : root_link_name_("ground") {}
    virtual ~COsimParser() {}

  private:
    /** Osim link specification reader:
     * In case a body has multiple coordinates, it creates a series
     * of joints/links for all the coordinates. */
    bool readBody(
        const scl_tinyxml::TiXmlHandle& arg_tiHndl_link,
        scl::SRobotParsedData& arg_robot,
        const std::string& arg_joint_type);

    /** To read in the opensim full body format */
    struct SOsimJoint{
      struct SOsimCoordinate{
        scl::sFloat default_pos_,min_,max_;
        std::string coord_name_;
        scl::sBool is_rot_;
      };
      struct SOsimTransformAxis{
        std::string name_, coord_name_;
        Eigen::Vector3d axis_;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };
      sutil::CMappedList<std::string, SOsimCoordinate> coordinates_;
      sutil::CMappedList<std::string, SOsimTransformAxis> trf_axes_;
      std::string name_, parent_name_;
      Eigen::Vector3d pos_in_parent_;
      Eigen::Quaternion<scl::sFloat> ori_in_parent_;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /** Reads a joint for a body.
     * Will be converted into multiple joints/links for scl. */
    bool readJoint(
        const scl_tinyxml::TiXmlHandle& arg_tiHndl_jnt,
        SOsimJoint& arg_joint);

    const std::string root_link_name_;
  };

}

#endif /*COSIMPARSER_HPP_*/
