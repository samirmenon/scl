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
/* \file CSaiParser.cpp
 *
 *  Created on: Jan, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <sstream>
#include <stdexcept>

//The Class definition.
#include <scl/parser/saiparser/CSaiParser.hpp>

//The required data structures
#include <scl/Singletons.hpp>
#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/data_structs/SRigidBody.hpp>

using namespace scl;
using namespace scl_tinyxml; //Tinyxml parser implementation is in a separate namespace

namespace scl_parser {

bool CSaiParser::readRobotFromFile(const std::string& arg_file,
    const std::string& arg_robot_name,
    scl::SRobotParsedData& arg_robot)
{
  bool flag;
  SRigidBody* tmp_link_ds=S_NULL;
  try
  {
    //Set up the parser.
    TiXmlHandle tiHndl_file_handle(NULL), tiHndl_world(NULL), tiHndl_link(NULL);
    TiXmlDocument tiDoc_file(arg_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw(std::runtime_error("Could not open xml file to read robot definition.")); }


    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChild( "dynworld" );
    tiHndl_link = tiHndl_world.FirstChild("baseNode");

    //Read in the root link. (baseNode)
    tmp_link_ds = arg_robot.rb_tree_.create(root_link_name_, true); //Add the root link
    tmp_link_ds->init(); //Default params. Change it after converting to scl.
    tmp_link_ds->is_root_ = true;
    tmp_link_ds->name_ = root_link_name_;
    tmp_link_ds->link_id_ = -1;

    //Read in the other links
    flag = readLink(tiHndl_link, true, tmp_link_ds->name_, arg_robot );
    if(false == flag )
    { throw(std::runtime_error("Could not read robot's links.")); }

    arg_robot.dof_ = arg_robot.rb_tree_.size() - 1;//The root node is stationary
    flag = arg_robot.rb_tree_.linkNodes();
    if(false == flag)
    { throw(std::runtime_error("Could not link robot's branching representation nodes.")); }

    return true;
  }
  catch(std::exception& e)
  {
    std::cerr<<"\nCSaiParser::readRobotFromFile("<<arg_file<<") : "<<e.what();
    return false;
  }
}


bool CSaiParser::readLink(const TiXmlHandle& arg_tiHndl_link, const bool arg_is_root,
    const std::string& arg_parent_lnk_name, scl::SRobotParsedData& arg_robot)
{
  //NOTE : This function will recurse at the end.
  TiXmlElement* link_data;
  std::string lnk_name;
  bool flag;
  SRigidBody* tmp_link_ds=S_NULL;

  try
  {
    if(true == arg_is_root) //Root link is initialized outside this function.
    {
      lnk_name = root_link_name_;
      tmp_link_ds = arg_robot.rb_tree_.at(lnk_name);
      if(S_NULL == tmp_link_ds)
      { throw(std::runtime_error("Could not find root node in the branching representation"));  }
    }
    else
    {
      link_data = arg_tiHndl_link.FirstChild( "linkName" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>lnk_name;
      }
      else { throw(std::runtime_error("Could not read a link name.")); }

      //Create the link on the pile
      tmp_link_ds = arg_robot.rb_tree_.create(lnk_name,false);
      if(S_NULL == tmp_link_ds)
      {
        std::string msg;
        msg = ". Could not allocate node on the pile.";
        msg = lnk_name + msg;
        throw(std::runtime_error(msg.c_str()));
      }
      tmp_link_ds->name_ = lnk_name;
      tmp_link_ds->parent_name_ = arg_parent_lnk_name;

      link_data = arg_tiHndl_link.FirstChild( "jointName" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp_link_ds->joint_name_;
      }
      else { throw(std::runtime_error("Could not read a joint name.")); }

      link_data = arg_tiHndl_link.FirstChild( "lowerJointLimit" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp_link_ds->joint_limit_lower_;
      }

      link_data = arg_tiHndl_link.FirstChild( "upperJointLimit" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp_link_ds->joint_limit_lower_;
      }

      link_data = arg_tiHndl_link.FirstChild( "defaultJointPosition" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp_link_ds->joint_default_pos_;
      }

      link_data = arg_tiHndl_link.FirstChild( "ID" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp_link_ds->link_id_;
      }

      char axis[3];
      link_data = arg_tiHndl_link.FirstChild( "type" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>axis[0];
      }
      link_data = arg_tiHndl_link.FirstChild( "axis" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>axis[1];
      }
      axis[2] = '\0';
      std::string jtype; jtype = axis;
      if(jtype == "px"){ tmp_link_ds->joint_type_ = JOINT_TYPE_PRISMATIC_X;  }
      else if(jtype == "py"){ tmp_link_ds->joint_type_ = JOINT_TYPE_PRISMATIC_Y;  }
      else if(jtype == "pz"){ tmp_link_ds->joint_type_ = JOINT_TYPE_PRISMATIC_Z;  }
      else if(jtype == "rx"){ tmp_link_ds->joint_type_ = JOINT_TYPE_REVOLUTE_X;  }
      else if(jtype == "ry"){ tmp_link_ds->joint_type_ = JOINT_TYPE_REVOLUTE_Y;  }
      else if(jtype == "rz"){ tmp_link_ds->joint_type_ = JOINT_TYPE_REVOLUTE_Z;  }
      else { std::cerr<<"\nWarning : Link \'"<<tmp_link_ds->name_<<"\' has invalid joint type information : " <<jtype; }

      link_data = arg_tiHndl_link.FirstChild( "rot" ).ToElement();
      if ( link_data )
      {
        int tmp[4];
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp[0];
        ss>>tmp[1];
        ss>>tmp[2];
        ss>>tmp[3];

        //PS : Blame these on people hacking the legacy SAI xml format.
        //SUPER HACK! If sai uses axis angle uncomment this.
        Eigen::AngleAxisd tmp_aa;
        tmp_aa.axis()(0) = tmp[0];
        tmp_aa.axis()(1) = tmp[1];
        tmp_aa.axis()(2) = tmp[2];
        tmp_aa.angle() = tmp[3];
        tmp_link_ds->ori_parent_quat_ = tmp_aa;

        //SUPER HACK! If sai uses quaternions uncomment this.
//        tmp_link_ds->ori_parent_quat_.w() = tmp[0];
//        tmp_link_ds->ori_parent_quat_.x() = tmp[1];
//        tmp_link_ds->ori_parent_quat_.y() = tmp[2];
//        tmp_link_ds->ori_parent_quat_.z() = tmp[3];
      }
      else { std::cerr<<"\nWarning : Link \'"<<tmp_link_ds->name_<<"\' has no position information."; }

      link_data = arg_tiHndl_link.FirstChild( "pos" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp_link_ds->pos_in_parent_[0];
        ss>>tmp_link_ds->pos_in_parent_[1];
        ss>>tmp_link_ds->pos_in_parent_[2];
      }
      else { std::cerr<<"\nWarning : Link \'"<<tmp_link_ds->name_<<"\' has no position information."; }

      link_data = arg_tiHndl_link.FirstChild( "mass" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp_link_ds->mass_;
      }
      else
      { std::cerr<<"\nWarning : Link \'"<<tmp_link_ds->name_<<"\' has no mass information. Default is 1.";  }

      link_data = arg_tiHndl_link.FirstChild( "inertia" ).ToElement();
      if ( link_data )
      {
        tmp_link_ds->inertia_ = Eigen::Matrix3d::Identity(); //Default
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp_link_ds->inertia_(0,0);
        ss>>tmp_link_ds->inertia_(1,1);
        ss>>tmp_link_ds->inertia_(2,2);
        //Odd syntax?
        //Reason: Sets the var and implicitly sets the state of the stringstream
        //If the stringstream is empty, it sets the last valid var to every succeeding var.
        //Ie. If this following line fails:
        //       (tmp_link_ds->inertia_(0,1) ==  tmp_link_ds->inertia_(2,2))
        if(ss>>tmp_link_ds->inertia_(0,1))
        {
          ss>>tmp_link_ds->inertia_(0,2);
          ss>>tmp_link_ds->inertia_(1,2);
        }
        else
        {//If the prev "if" failed, we have to reset these three.
          tmp_link_ds->inertia_(0,1) = 0.0;
          tmp_link_ds->inertia_(0,2) = 0.0;
          tmp_link_ds->inertia_(1,2) = 0.0;
#ifdef DEBUG
          std::cout<<"\nCSclTiXmlParser::readLink() : WARNING : Only three inertia values specified at link : "
              <<tmp_link_ds->name_<<". \nConsider specifying all 6 : {Ixx, Iyy, Izz, Ixy, Ixz, Iyz}";
#endif
        }
        //The inertia matrix is symmetric
        tmp_link_ds->inertia_(1,0) = tmp_link_ds->inertia_(0,1);
        tmp_link_ds->inertia_(2,0) = tmp_link_ds->inertia_(0,2);
        tmp_link_ds->inertia_(2,1) = tmp_link_ds->inertia_(1,2);
      }
      else
      { std::cerr<<"\nWarning : Link \'"<<tmp_link_ds->name_<<"\' has no inertia information. Default is (1,1,1)."; }

      link_data = arg_tiHndl_link.FirstChild( "com" ).ToElement();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>tmp_link_ds->com_[0];
        ss>>tmp_link_ds->com_[1];
        ss>>tmp_link_ds->com_[2];
      }
      else
      { std::cerr<<"\nWarning : Link \'"<<tmp_link_ds->name_<<"\' has no com information. Default is (1,1,1)."; }

      //HACK : Support vince's imagePos tag! If it isn't there, do the standard jig.
      if(S_NULL==arg_tiHndl_link.FirstChild( "imagePos" ).ToElement())
      {
        link_data = arg_tiHndl_link.FirstChild( "url" ).ToElement();
        while ( link_data )
        {
          std::string obj;
          std::stringstream ss(link_data->FirstChild()->Value());
          ss>>obj;

          SRigidBodyGraphics tmp_lnk;
          tmp_lnk.file_name_ = obj;
          tmp_link_ds->graphics_obj_vec_.push_back(tmp_lnk);

          link_data = link_data->NextSiblingElement("url");
        }
      }
      else
      {//HACK : Parse Vince's hacked sai format.
        //If there is imagePos data do something more.
        TiXmlElement* url_ele, * img_ele;
        img_ele = arg_tiHndl_link.FirstChild( "imagePos" ).ToElement();
        url_ele = arg_tiHndl_link.FirstChild( "url" ).ToElement();
        while(img_ele && url_ele)
        {
          //Does not support the deparacted obj behavior. New style only.
          SRigidBodyGraphics tmp;

          std::stringstream ss(img_ele->FirstChild()->Value());
          ss>>tmp.pos_in_parent_[0];
          ss>>tmp.pos_in_parent_[1];
          ss>>tmp.pos_in_parent_[2];

          std::stringstream ss2(url_ele->FirstChild()->Value());
          ss2>>tmp.file_name_;

          tmp_link_ds->graphics_obj_vec_.push_back(tmp);

          img_ele = img_ele->NextSiblingElement("imagePos");
          url_ele = url_ele->NextSiblingElement("url");
        }
      }
    }

    for(TiXmlElement* tiElem_link = arg_tiHndl_link.FirstChild("jointNode").ToElement();
        tiElem_link; tiElem_link=tiElem_link->NextSiblingElement("jointNode"))
    {
      TiXmlHandle tmp(tiElem_link);
      flag = readLink(tmp , false, tmp_link_ds->name_, arg_robot);
      if(false == flag)
      {
        std::string msg;
        msg = ". Could not read link.";
        msg = tmp_link_ds->name_ + msg;
        throw(std::runtime_error(msg.c_str()));
      }
    }//END Loop over jointNodes

    return true;
  }
  catch(std::exception& e)
  {
    std::cerr<<"\nCSaiParser::readLink() : "<<e.what();
    return false;
  }
}
}
