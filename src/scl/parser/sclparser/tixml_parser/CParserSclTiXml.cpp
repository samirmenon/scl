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
/* \file CParserSclTiXml.cpp
 *
 *  Created on: May, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CParserSclTiXml.hpp"

#include <sstream>
#include <iostream>

#include <stdexcept>

#include <scl/Singletons.hpp>


using namespace scl;
using namespace scl_tinyxml;

namespace scl {

  bool CParserSclTiXml::readLink(
      const TiXmlHandle & arg_link_txml,
      scl::SRigidBody& arg_link_ds,
      bool arg_is_root)
  {
    try
    {
      //Read in the information into arg_link_ds
      TiXmlElement* link_data, *obj_data;

      //Link name
      link_data = arg_link_txml.FirstChildElement( "link_name" ).Element();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>arg_link_ds.name_;
      }
      else
      { throw(std::runtime_error("Error reading link name")); }

#ifdef DEBUG
      std::cout<<"\nParsing link : "<<arg_link_ds.name_;//Remaining messages are for this link name.
#endif

      //Pos in parent
      link_data = arg_link_txml.FirstChildElement( "position_in_parent" ).Element();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>arg_link_ds.pos_in_parent_[0];
        ss>>arg_link_ds.pos_in_parent_[1];
        ss>>arg_link_ds.pos_in_parent_[2];
      }
      else
      { throw(std::runtime_error("No position in parent information.")); }

      //Orientation in parent.
      link_data = arg_link_txml.FirstChildElement( "orientation_in_parent" ).Element();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>arg_link_ds.ori_parent_quat_.x();
        ss>>arg_link_ds.ori_parent_quat_.y();
        ss>>arg_link_ds.ori_parent_quat_.z();
        ss>>arg_link_ds.ori_parent_quat_.w();
        if((0==arg_link_ds.ori_parent_quat_.x()) &&
           (0==arg_link_ds.ori_parent_quat_.y()) &&
           (0==arg_link_ds.ori_parent_quat_.z()) &&
           (0==arg_link_ds.ori_parent_quat_.w()))
        { throw(std::runtime_error("Specified quaternion (0,0,0,0) is invalid. Specify (0,0,0,1) instead.")); }
      }
      else
      { throw(std::runtime_error("No orientation in parent information.")); }

      //******************************** Child link specific stuff ******************************
      if(!arg_is_root)
      {
        //Mass
        link_data = arg_link_txml.FirstChildElement( "mass" ).Element();
        if( link_data )
        {
          std::stringstream ss(link_data->FirstChild()->Value());
          ss>>arg_link_ds.mass_;
        }
        else
        {throw(std::runtime_error("Error reading mass of link"));}

        //Inertia
        link_data = arg_link_txml.FirstChildElement( "inertia" ).Element();
        if ( link_data )
        {
          arg_link_ds.inertia_ == Eigen::Matrix3d::Identity();
          std::stringstream ss(link_data->FirstChild()->Value());
          ss>>arg_link_ds.inertia_(0,0);
          ss>>arg_link_ds.inertia_(1,1);
          ss>>arg_link_ds.inertia_(2,2);
          //Odd syntax?
          //Reason: Sets the var and implicitly sets the state of the stringstream
          //If the stringstream is empty, it sets the last valid var to every succeeding var.
          //Ie. If this following line fails:
          //       (arg_link_ds.inertia_(0,1) ==  arg_link_ds.inertia_(2,2))
          if(ss>>arg_link_ds.inertia_(0,1))
          {
          ss>>arg_link_ds.inertia_(0,2);
          ss>>arg_link_ds.inertia_(1,2);
          }
          else
          {//If the prev "if" failed, we have to reset these three.
            arg_link_ds.inertia_(0,1) = 0.0;
            arg_link_ds.inertia_(0,2) = 0.0;
            arg_link_ds.inertia_(1,2) = 0.0;
#ifdef DEBUG
            std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                    <<"]. Only three inertia values specified at link : "
                <<arg_link_ds.name_<<". \nConsider specifying all 6 : {Ixx, Iyy, Izz, Ixy, Ixz, Iyz}";
#endif
          }
          //The inertia matrix is symmetric
          arg_link_ds.inertia_(1,0) = arg_link_ds.inertia_(0,1);
          arg_link_ds.inertia_(2,0) = arg_link_ds.inertia_(0,2);
          arg_link_ds.inertia_(2,1) = arg_link_ds.inertia_(1,2);
        }
        else
        {throw(std::runtime_error("Error reading inertia"));}
        //Com
        link_data = arg_link_txml.FirstChildElement( "center_of_mass" ).Element();
        if ( link_data )
        {
          std::stringstream ss(link_data->FirstChild()->Value());
          ss>>arg_link_ds.com_[0];
          ss>>arg_link_ds.com_[1];
          ss>>arg_link_ds.com_[2];
        }
        else
        {throw(std::runtime_error("Error reading com"));}

        //******************************** Joint specific stuff ******************************
        //Joint name
        link_data = arg_link_txml.FirstChildElement( "joint_name" ).Element();
        if ( link_data )
        {
          std::stringstream ss(link_data->FirstChild()->Value());
          ss>>arg_link_ds.joint_name_;
        }
        else
        {
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
          <<"]. Error reading joint name";
        }

        //Parent link name
        link_data = arg_link_txml.FirstChildElement( "parent_link_name" ).Element();
        if ( link_data )
        {
          std::stringstream ss(link_data->FirstChild()->Value());
          ss>>arg_link_ds.parent_name_;
        }
        else  {
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Error reading parent link name";
        }

        //Lower Joint Limit
        link_data = arg_link_txml.FirstChildElement( "joint_limits" ).Element();
        if ( link_data )
        {
          std::stringstream ss(link_data->FirstChild()->Value());
          ss>>arg_link_ds.joint_limit_lower_;
          ss>>arg_link_ds.joint_limit_upper_;
        }
        else  {
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Error reading joint limits";
        }

        //Default Joint Pos
        link_data = arg_link_txml.FirstChildElement( "default_joint_position" ).Element();
        if ( link_data )
        {
          std::stringstream ss(link_data->FirstChild()->Value());
          ss>>arg_link_ds.joint_default_pos_;
        }
        else  {
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Error reading default joint position";
        }

        //Joint type
        link_data = arg_link_txml.FirstChildElement( "joint_type" ).Element();
        if ( link_data )
        {
          std::string jtype(link_data->FirstChild()->Value());

          if(jtype == "px"){ arg_link_ds.joint_type_ = JOINT_TYPE_PRISMATIC_X;  }
          if(jtype == "py"){ arg_link_ds.joint_type_ = JOINT_TYPE_PRISMATIC_Y;  }
          if(jtype == "pz"){ arg_link_ds.joint_type_ = JOINT_TYPE_PRISMATIC_Z;  }
          if(jtype == "rx"){ arg_link_ds.joint_type_ = JOINT_TYPE_REVOLUTE_X;  }
          if(jtype == "ry"){ arg_link_ds.joint_type_ = JOINT_TYPE_REVOLUTE_Y;  }
          if(jtype == "rz"){ arg_link_ds.joint_type_ = JOINT_TYPE_REVOLUTE_Z;  }
          if(jtype == "sp") { arg_link_ds.joint_type_ = JOINT_TYPE_SPHERICAL;  }
          if(jtype == "sl"){ arg_link_ds.joint_type_ = JOINT_TYPE_SPLINE;  }
        }
        else {throw(std::runtime_error("Error reading joint type"));}

        //NOTE TODO : If there is any special processing for spherical joints, it will take place here

        //************** END OF Joint specific stuff
      } //************** END OF Child link specific stuff


      //******************************** Graphics Stuff ******************************
      bool flag_graphics_tag_found,flag_graphics_element_found=false;
      flag_graphics_tag_found = (NULL != arg_link_txml.FirstChild( "graphics" ).ToElement());

      //**** Graphics : obj file
      link_data = arg_link_txml.FirstChild( "graphics" ).FirstChild("obj_file").ToElement();

      for(; link_data; link_data=link_data->NextSiblingElement("obj_file") )
      {
        std::string ss;

        //Obj file paths are always relative to the specs directory.
        if("" == scl::CDatabase::getData()->dir_specs_)
        {
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : At link ["<<arg_link_ds.name_
                  <<"]. Specs directory not set in the database. Can't read link's obj file.";
          continue;
        }

        //Read all the obj_files
        SRigidBodyGraphics tgr;
        tgr.class_ = SRigidBodyGraphics::GRAPHIC_TYPE_FILE_OBJ;
        ss = link_data->FirstChildElement("name")->FirstChild()->Value();
        ss = scl::CDatabase::getData()->dir_specs_ + ss;
        tgr.file_name_ = ss;

        obj_data = link_data->FirstChildElement( "position_in_parent" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.pos_in_parent_(0) = tmpvar;
          ss>>tmpvar;
          tgr.pos_in_parent_(1) = tmpvar;
          ss>>tmpvar;
          tgr.pos_in_parent_(2) = tmpvar;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Position in parent not found for obj file : "<<tgr.file_name_;
#endif
        }

        obj_data = link_data->FirstChildElement( "orientation_in_parent" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.ori_parent_quat_(0) = tmpvar;
          ss>>tmpvar;
          tgr.ori_parent_quat_(1) = tmpvar;
          ss>>tmpvar;
          tgr.ori_parent_quat_(2) = tmpvar;
          ss>>tmpvar;
          tgr.ori_parent_quat_(3) = tmpvar;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Orientation in parent not found for obj file : "<<tgr.file_name_;
#endif
        }

        obj_data = link_data->FirstChildElement( "scaling" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.scaling_(0) = tmpvar; //x
          ss>>tmpvar;
          tgr.scaling_(1) = tmpvar; //y
          ss>>tmpvar;
          tgr.scaling_(2) = tmpvar; //z
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Graphics scaling not found for obj file : "<<tgr.file_name_;
#endif
        }

        obj_data = link_data->FirstChildElement( "collision_type" );
        if ( obj_data )
        {
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tgr.collision_type_;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Collision type not found for obj file : "<<tgr.file_name_;
#endif
        }

        arg_link_ds.graphics_obj_vec_.push_back(tgr);
        flag_graphics_element_found = true;
      }// End of obj file

      //**** Graphics : sphere
      link_data = arg_link_txml.FirstChild( "graphics" ).FirstChild("sphere").ToElement();

      for(; link_data; link_data=link_data->NextSiblingElement("sphere") )
      {
        std::string ss;

        //Read all the obj_files
        SRigidBodyGraphics tgr;
        tgr.class_ = SRigidBodyGraphics::GRAPHIC_TYPE_SPHERE;

        obj_data = link_data->FirstChildElement( "position_in_parent" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.pos_in_parent_(0) = tmpvar;
          ss>>tmpvar;
          tgr.pos_in_parent_(1) = tmpvar;
          ss>>tmpvar;
          tgr.pos_in_parent_(2) = tmpvar;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Position in parent not found for sphere";
#endif
        }

        obj_data = link_data->FirstChildElement( "radius" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.scaling_(0) = tmpvar; //radius
          tgr.scaling_(1) = tmpvar; //radius
          tgr.scaling_(2) = tmpvar; //radius
        }
        else
        { throw(std::runtime_error("Radius not found for sphere")); }

        obj_data = link_data->FirstChildElement( "color" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.color_[0] = tmpvar;
          ss>>tmpvar;
          tgr.color_[1] = tmpvar;
          ss>>tmpvar;
          tgr.color_[2] = tmpvar;
        }
        else{ throw(std::runtime_error("Color not found for sphere")); }

        obj_data = link_data->FirstChildElement( "collision_type" );
        if ( obj_data )
        {
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tgr.collision_type_;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Collision type not found for sphere";
#endif
        }

        arg_link_ds.graphics_obj_vec_.push_back(tgr);
        flag_graphics_element_found = true;
      }// End of sphere


      //**** Graphics : cuboid
      link_data = arg_link_txml.FirstChild( "graphics" ).FirstChild("cuboid").ToElement();

      for(; link_data; link_data=link_data->NextSiblingElement("cuboid") )
      {
        std::string ss;

        //Read all the obj_files
        SRigidBodyGraphics tgr;
        tgr.class_ = SRigidBodyGraphics::GRAPHIC_TYPE_CUBOID;

        obj_data = link_data->FirstChildElement( "position_in_parent" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.pos_in_parent_(0) = tmpvar;
          ss>>tmpvar;
          tgr.pos_in_parent_(1) = tmpvar;
          ss>>tmpvar;
          tgr.pos_in_parent_(2) = tmpvar;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Position in parent not found for cuboid";
#endif
        }

        obj_data = link_data->FirstChildElement( "orientation_in_parent" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.ori_parent_quat_(0) = tmpvar;
          ss>>tmpvar;
          tgr.ori_parent_quat_(1) = tmpvar;
          ss>>tmpvar;
          tgr.ori_parent_quat_(2) = tmpvar;
          ss>>tmpvar;
          tgr.ori_parent_quat_(3) = tmpvar;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Warning : Orientation in parent not found for cuboid";
#endif
        }

        //SI units. Default is 1 1 1 (massive!)
        obj_data = link_data->FirstChildElement( "scaling" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.scaling_(0) = tmpvar; //x
          ss>>tmpvar;
          tgr.scaling_(1) = tmpvar; //y
          ss>>tmpvar;
          tgr.scaling_(2) = tmpvar; //z
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Warning : Graphics scaling not found for cuboid";
#endif
        }

        obj_data = link_data->FirstChildElement( "color" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.color_[0] = tmpvar;
          ss>>tmpvar;
          tgr.color_[1] = tmpvar;
          ss>>tmpvar;
          tgr.color_[2] = tmpvar;
        }
        else{
#ifdef DEBUG
std::cerr<< "\nCParserSclTiXml::readLink() : Warning : Color not found for cuboid";
#endif
        }

        obj_data = link_data->FirstChildElement( "collision_type" );
        if ( obj_data )
        {
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tgr.collision_type_;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Collision type not found for cuboid element";
#endif
        }

        arg_link_ds.graphics_obj_vec_.push_back(tgr);
        flag_graphics_element_found = true;
      }// End of cuboid


      //**** Graphics : cylinder
      link_data = arg_link_txml.FirstChild( "graphics" ).FirstChild("cylinder").ToElement();

      for(; link_data; link_data=link_data->NextSiblingElement("cylinder") )
      {
        std::string ss;

        //Read all the obj_files
        SRigidBodyGraphics tgr;
        tgr.class_ = SRigidBodyGraphics::GRAPHIC_TYPE_CYLINDER;

        obj_data = link_data->FirstChildElement( "position_in_parent" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.pos_in_parent_(0) = tmpvar;
          ss>>tmpvar;
          tgr.pos_in_parent_(1) = tmpvar;
          ss>>tmpvar;
          tgr.pos_in_parent_(2) = tmpvar;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Position in parent not found for cylinder";
#endif
        }

        obj_data = link_data->FirstChildElement( "orientation_in_parent" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.ori_parent_quat_(0) = tmpvar;
          ss>>tmpvar;
          tgr.ori_parent_quat_(1) = tmpvar;
          ss>>tmpvar;
          tgr.ori_parent_quat_(2) = tmpvar;
          ss>>tmpvar;
          tgr.ori_parent_quat_(3) = tmpvar;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Orientation in parent not found for cylinder";
#endif
        }

        //SI units. Default is 1 1 1 (massive!)
        obj_data = link_data->FirstChildElement( "baseR_topR_height" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.scaling_(0) = tmpvar; //x
          ss>>tmpvar;
          tgr.scaling_(1) = tmpvar; //y
          ss>>tmpvar;
          tgr.scaling_(2) = tmpvar; //z
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
              <<"]. Graphics scaling not found for cylinder";
#endif
        }

        obj_data = link_data->FirstChildElement( "color" );
        if ( obj_data )
        {
          sFloat tmpvar;
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tmpvar;
          tgr.color_[0] = tmpvar;
          ss>>tmpvar;
          tgr.color_[1] = tmpvar;
          ss>>tmpvar;
          tgr.color_[2] = tmpvar;
        }
        else{
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
              <<"]. Color not found for cylinder";
#endif
        }

        obj_data = link_data->FirstChildElement( "collision_type" );
        if ( obj_data )
        {
          std::stringstream ss(obj_data->FirstChild()->Value());
          ss>>tgr.collision_type_;
        }
        else  {
#ifdef DEBUG
          std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                  <<"]. Collision type not found for cylinder";
#endif
        }

        arg_link_ds.graphics_obj_vec_.push_back(tgr);
        flag_graphics_element_found = true;
      }// End of cuboid

      //Graphics : collision type //NOTE TODO : Old style (Depracated)
      link_data = arg_link_txml.FirstChildElement( "graphics" ).FirstChildElement("collision_type").Element();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>arg_link_ds.collision_type_;
        flag_graphics_element_found = true;
        std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
                <<"]. <graphics> <collision_type> tags are depracated. Use these in the graphics elements.";
      }

      if(flag_graphics_tag_found && (false == flag_graphics_element_found))
      {
        std::cerr<< "\nCParserSclTiXml::readLink() : WARNING : In link ["<<arg_link_ds.name_
        <<"]. Found <graphics> tag but no sub-elements.";
      }
      //******************************** End of Graphics Stuff ******************************
    }
    catch(std::exception& e)
    {
      std::cout<<"\nCParserSclTiXml::readLink() : "<<e.what();
      return false;
    }
    return true;
  }



  /** Reads single links */
  bool CParserSclTiXml::readMuscle(const scl_tinyxml::TiXmlHandle& arg_musc_txml,
      scl::SMuscleParsed& arg_muscle_ds, bool arg_is_root)
  {
    try
    {
      //Read in the information into arg_glob_ds
      TiXmlElement* muscle_data, *muscle_datal2;

      //muscle name.
      muscle_data = arg_musc_txml.Element();
      if ( muscle_data )
      {
        std::stringstream ss(muscle_data->Attribute("type"));
        ss>>arg_muscle_ds.muscle_type_;
      }
      else
      {throw(std::runtime_error("Error reading muscle type"));}

      //NOTE : Should read multiple cameras. But reads only one.
      //Camera orientation and position
      muscle_data = arg_musc_txml.FirstChildElement("name").Element();
      if ( muscle_data )
      {
        std::stringstream ss(muscle_data->FirstChild()->Value());
        ss>>arg_muscle_ds.name_;
      }
      else
      {throw(std::runtime_error("Error reading muscle name"));}

      /** ************** HILL TYPE MODEL PROPERTIES ************* */
      muscle_data = arg_musc_txml.FirstChildElement("max_isometric_force").Element();
      if ( muscle_data )
      {
        std::stringstream ss(muscle_data->FirstChild()->Value());
        ss>>arg_muscle_ds.max_isometric_force_;
      }
      else
      {throw(std::runtime_error("Error reading max isometric force"));}

      muscle_data = arg_musc_txml.FirstChildElement("stiffness").Element();
      if ( muscle_data )
      {
        std::stringstream ss(muscle_data->FirstChild()->Value());
        ss>>arg_muscle_ds.stiffness_;
      }
      else
      {throw(std::runtime_error("Error reading stiffness"));}

      muscle_data = arg_musc_txml.FirstChildElement("damping").Element();
      if ( muscle_data )
      {
        std::stringstream ss(muscle_data->FirstChild()->Value());
        ss>>arg_muscle_ds.damping_;
      }
      else
      {throw(std::runtime_error("Error reading damping"));}

      muscle_data = arg_musc_txml.FirstChildElement("tendon_stiffness").Element();
      if ( muscle_data )
      {
        std::stringstream ss(muscle_data->FirstChild()->Value());
        ss>>arg_muscle_ds.stiffness_tendon_;
      }
      else
      {throw(std::runtime_error("Error reading tendon stiffness"));}

      /** ********************** Now read muscle attachment points ***************************** */
      // Count all the muscle attachment points.
      int n_musc_points=0;
      muscle_data = arg_musc_txml.FirstChild( "muscle_points" ).FirstChild("point").ToElement();
      for(; muscle_data; muscle_data=muscle_data->NextSiblingElement("point") )
      { n_musc_points++; }

      arg_muscle_ds.points_.resize(n_musc_points,SMusclePointParsed());

      // Iterate over all the muscle attachment points.
      muscle_data = arg_musc_txml.FirstChild( "muscle_points" ).FirstChild("point").ToElement();
      for(; muscle_data; muscle_data=muscle_data->NextSiblingElement("point"))
      {// For each point
        int musc_order=-1;

        if(NULL == muscle_data->Attribute("order"))
        { throw(std::runtime_error("No order attribute at muscle point (must be 0 to num_points-1)")); }

        std::stringstream ss(muscle_data->Attribute("order"));
        ss>>musc_order;
        if(musc_order<0 || musc_order>=n_musc_points)
        { throw(std::runtime_error("Negative muscle point order (must be 0 to num_points-1)")); }

        //Store a reference to the current point (for simplicity)
        SMusclePointParsed &tmp_musc_pt = arg_muscle_ds.points_[musc_order];
        tmp_musc_pt.position_on_muscle_ = static_cast<sUInt>(musc_order);

        // Read in the parent link
        muscle_datal2 = muscle_data->FirstChildElement("parent_link_name");
        if ( muscle_datal2 )
        {
          std::stringstream ss(muscle_datal2->FirstChild()->Value());
          ss>>tmp_musc_pt.parent_link_;
        }
        else
        {throw(std::runtime_error("Error reading parent link name"));}

        // Read in the pos in parent
        muscle_datal2 = muscle_data->FirstChildElement("position_in_parent");
        if ( muscle_datal2 )
        {
          std::stringstream ss(muscle_datal2->FirstChild()->Value());
          ss>>tmp_musc_pt.pos_in_parent_(0);
          ss>>tmp_musc_pt.pos_in_parent_(1);
          ss>>tmp_musc_pt.pos_in_parent_(2);
        }
        else
        {throw(std::runtime_error("Error reading parent link name"));}
      }

      return true;
    }
    catch(std::exception& e)
    { std::cout<<"\nCParserSclTiXml::readMuscle() : "<< e.what(); }
    return false;
  }


  bool CParserSclTiXml::readGraphics(
        const TiXmlHandle &arg_graphics_data_txml,
        scl::SGraphicsParsed& arg_graphics_ds)
    {
      try
      {
        //Read in the information into arg_glob_ds
        TiXmlElement* graphics_data;

        //Graphics name.
        graphics_data = arg_graphics_data_txml.Element();
        if ( graphics_data )
        {
          std::stringstream ss(graphics_data->Attribute("name"));
          ss>>arg_graphics_ds.name_;
        }
        else
        {throw(std::runtime_error("Error reading graphics name"));}

        //NOTE : Should read multiple cameras. But reads only one.
        //Camera orientation and position
        graphics_data = arg_graphics_data_txml.FirstChildElement( "camera" ).
            FirstChildElement( "pos" ).Element();
        if ( graphics_data )
        {
          std::stringstream ss(graphics_data->FirstChild()->Value());
          ss>>arg_graphics_ds.cam_pos_[0];
          ss>>arg_graphics_ds.cam_pos_[1];
          ss>>arg_graphics_ds.cam_pos_[2];
        }
        else{
#ifdef DEBUG
          std::cerr<< "\nWarning: Couldn't find camera pos. Proceeding.";
#endif
        }

        graphics_data = arg_graphics_data_txml.FirstChildElement( "camera" ).
            FirstChildElement( "lookat" ).Element();
        if ( graphics_data )
        {
          std::stringstream ss(graphics_data->FirstChild()->Value());
          ss>>arg_graphics_ds.cam_lookat_[0];
          ss>>arg_graphics_ds.cam_lookat_[1];
          ss>>arg_graphics_ds.cam_lookat_[2];
        }
        else{
#ifdef DEBUG
          std::cerr<< "\nWarning. Couldn't find camera lookat. Proceeding.";
#endif
          }

        graphics_data = arg_graphics_data_txml.FirstChildElement( "camera" ).
            FirstChildElement( "up" ).Element();
        if ( graphics_data )
        {
          std::stringstream ss(graphics_data->FirstChild()->Value());
          ss>>arg_graphics_ds.cam_up_[0];
          ss>>arg_graphics_ds.cam_up_[1];
          ss>>arg_graphics_ds.cam_up_[2];
        }
        else{
#ifdef DEBUG
          std::cerr<< "\nWarning. Couldn't find camera up. Proceeding";
#endif
          }

        return true;
      }
      catch(std::exception& e)
      { std::cout<<"\nCParserSclTiXml::readGraphicsData() : "<< e.what(); }
      return false;
    }
}
