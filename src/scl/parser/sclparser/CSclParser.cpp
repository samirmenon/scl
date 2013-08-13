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
/* \file CSclParser.cpp
 *
 *  Created on: May, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */
//The Class definition.
#include <scl/parser/sclparser/CSclParser.hpp>

//The required data structures
#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/data_structs/SRobotParsedData.hpp>
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/util/HelperFunctions.hpp>

//The tinyxml parser implementation for scl xml files
#include <scl/parser/scl_tinyxml/scl_tinyxml.h>
#include <scl/parser/sclparser/tixml_parser/CSclTiXmlParser.hpp>

//The Standard cpp headers
#include <sstream>
#include <stdexcept>
#include <utility>


using namespace scl_tinyxml; //Tinyxml parser implementation is in a separate namespace
using namespace scl;


namespace scl{
  /** A basic container to parse generic task related information.
   *
   * This helps avoid using dynamic typing within the parser, which
   * is a design decision.
   *
   * Avoiding dynamic typing in the parser keeps things simple, removes
   * a dependence on sutil. One could consider adding it to the parser,
   * but since its primary use will only be for control tasks, the increase
   * in complexity might not be worth it.
   *
   * If you want a parser that handles dynamic typing, consider writing
   * another one.
   */
  class STaskParsedData : public STaskBase
  {
  public:
    virtual bool initTaskParams() { return false;  }
  };

  /** This class is used by the parser to load in the generic
   * options for each class.
   *
   * Also read the comments above for STaskParsedData
   */
  class SNonControlTaskParsedData : public SNonControlTaskBase
  {
  public:
    virtual bool initTaskParams() {return false;}
  };
}

namespace scl_parser {

bool CSclParser::listRobotsInFile(const std::string& arg_file,
    std::vector<std::string>& arg_robot_names)
{
  bool flag;
  try
  {
    //Set up the parser.
    TiXmlElement* tiElem_robot;
    TiXmlHandle tiHndl_glob_settings(NULL), tiHndl_file_handle(NULL), tiHndl_world(NULL);

    TiXmlDocument tiDoc_file(arg_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw std::runtime_error("Could not open xml file to read robots."); }

    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChildElement( "scl" );

    //Read in the robots.
    tiElem_robot = tiHndl_world.FirstChildElement( "robot" ).ToElement();

    //Iterating with TiXmlElement is faster than TiXmlHandle
    for(; tiElem_robot; tiElem_robot=tiElem_robot->NextSiblingElement("robot") )
    {
      if(NULL == tiElem_robot->Attribute("name"))
      {
#ifdef DEBUG
        throw(std::runtime_error("File contains a robot without a name"));
#endif
      }
      else
      {
        std::string tmp_name= tiElem_robot->Attribute("name");
        arg_robot_names.push_back(tmp_name);
      }
    }
    if(arg_robot_names.size()<=0)
    { throw(std::runtime_error("Couldn't find any robot specifications in the file."));}
  }
  catch(std::exception& e)
  {
    std::cerr<<"\nCSclParser::listRobotsInFile() :"<<e.what();
    arg_robot_names.clear();
    return false;
  }
  return true; //Success.
}

bool CSclParser::readRobotFromFile(const std::string& arg_file,
    const std::string& arg_robot_name,
    scl::SRobotParsedData& arg_robot)
{
  bool flag;
  SRigidBody* tmp_link_ds=S_NULL;

  try
  {
    //Set up the parser.
    TiXmlElement* tiElem_robot;
    TiXmlHandle tiHndl_file_handle(NULL), tiHndl_world(NULL);
    TiXmlDocument tiDoc_file(arg_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw std::runtime_error("Could not open xml file to read robot definition."); }

    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChildElement( "scl" );

    // *****************************************************************
    //                        Find the robot's xml
    // *****************************************************************

    //Read in the links.
    tiElem_robot = tiHndl_world.FirstChildElement( "robot" ).ToElement();

    sUInt robot_ctr=0;

    // *****************************************************************
    //          Loop over the robot xml for each robot
    // We loop to just double check that no two robots have the same name
    // *****************************************************************
    //Iterating with TiXmlElement is faster than TiXmlHandle
    for(; tiElem_robot; tiElem_robot=tiElem_robot->NextSiblingElement("robot") )
    {
      TiXmlHandle _robot_handle(tiElem_robot); //Back to handles

      //Read robot name
      if(NULL == tiElem_robot->Attribute("name")) //Unnamed robot
      { continue; }
      else
      {
        if(arg_robot_name != tiElem_robot->Attribute("name")) //Names don't match
        { continue; }
      }

      arg_robot.name_ = arg_robot_name;

      robot_ctr++; //Found a candidate
      if(robot_ctr>1) //There should only be one robot with this name.
      {
#ifdef DEBUG
        //Stronger error checking in debug mode. Break if robot names aren't unique.
        throw(std::runtime_error("Multiple robots have the same name"));
#endif
        break; //In release mode, we ignore the extra robots. Move on.
      }

      // *****************************************************************
      //                        Start parsing the robot
      // *****************************************************************
      //Read in the gravity that will apply to the robot
      TiXmlElement* grav_data;
      grav_data = _robot_handle.FirstChildElement( "gravity" ).Element();
      if ( grav_data )
      {
        std::stringstream ss(grav_data->FirstChild()->Value());
        ss>>arg_robot.gravity_(0);
        ss>>arg_robot.gravity_(1);
        ss>>arg_robot.gravity_(2);
      }
      else
      {throw(std::runtime_error("Error reading joint gravity"));}

      // *****************************************************************
      //                        Now parse the optional flags
      // *****************************************************************
      //Read in the gravity that will apply to the robot
      //All flags are optional
      TiXmlElement* xmlflags;
      xmlflags = _robot_handle.FirstChildElement( "flag_gc_damping" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        std::string sss;
        ss>>sss;
        if("true" == sss || "1" == sss)
        { arg_robot.flag_apply_gc_damping_ = true;  }
        else
        { arg_robot.flag_apply_gc_damping_ = false;  }
      }

      xmlflags = _robot_handle.FirstChildElement( "flag_gc_limits" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        std::string sss;
        ss>>sss;
        if("true" == sss || "1" == sss)
        { arg_robot.flag_apply_gc_pos_limits_ = true;  }
        else
        { arg_robot.flag_apply_gc_pos_limits_ = false;  }
      }

      xmlflags = _robot_handle.FirstChildElement( "flag_actuator_force_limits" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        std::string sss;
        ss>>sss;
        if("true" == sss || "1" == sss)
        { arg_robot.flag_apply_actuator_force_limits_ = true;  }
        else
        { arg_robot.flag_apply_actuator_force_limits_ = false;  }
      }

      xmlflags = _robot_handle.FirstChildElement( "flag_actuator_vel_limits" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        std::string sss;
        ss>>sss;
        if("true" == sss || "1" == sss)
        { arg_robot.flag_apply_actuator_vel_limits_ = true;  }
        else
        { arg_robot.flag_apply_actuator_vel_limits_ = false;  }
      }

      xmlflags = _robot_handle.FirstChildElement( "flag_actuator_acc_limits" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        std::string sss;
        ss>>sss;
        if("true" == sss || "1" == sss)
        { arg_robot.flag_apply_actuator_acc_limits_ = true;  }
        else
        { arg_robot.flag_apply_actuator_acc_limits_ = false;  }
      }

      xmlflags = _robot_handle.FirstChildElement( "flag_controller_on" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        std::string sss;
        ss>>sss;
        if("true" == sss || "1" == sss)
        { arg_robot.flag_controller_on_ = true;  }
        else
        { arg_robot.flag_controller_on_ = false;  }
      }

      xmlflags = _robot_handle.FirstChildElement( "flag_logging_on" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        std::string sss;
        ss>>sss;
        if("true" == sss || "1" == sss)
        { arg_robot.flag_logging_on_ = true;  }
        else
        { arg_robot.flag_logging_on_ = false;  }
      }

      xmlflags = _robot_handle.FirstChildElement( "flag_wireframe_on" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        std::string sss;
        ss>>sss;
        if("true" == sss || "1" == sss)
        { arg_robot.flag_wireframe_on_ = true;  }
        else
        { arg_robot.flag_wireframe_on_ = false;  }
      }

      xmlflags = _robot_handle.FirstChildElement( "option_axis_frame_size" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        ss>>arg_robot.option_axis_frame_size_;
      }

      xmlflags = _robot_handle.FirstChildElement( "option_muscle_via_pt_sz" ).Element();
      if ( xmlflags )
      {
        std::stringstream ss(xmlflags->FirstChild()->Value());
        ss>>arg_robot.option_muscle_via_pt_sz_;
        arg_robot.muscle_system_.render_muscle_via_pt_sz_ = arg_robot.option_muscle_via_pt_sz_;
      }

      // *****************************************************************
      //                        Now parse the links
      // *****************************************************************
      TiXmlElement* link_data;
      std::string spec(""), spec_muscle(""), spec_file("");

      //Spec name
      link_data = _robot_handle.FirstChildElement( "spec" ).Element();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>spec;
      }
      else
      { throw(std::runtime_error("\nError reading robot spec name")); }

      //Muscle spec name
      link_data = _robot_handle.FirstChildElement( "option_muscle_spec" ).Element();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>spec_muscle;
      }

      //File name
      link_data = _robot_handle.FirstChildElement( "file" ).Element();
      if ( link_data )
      {
        std::stringstream ss(link_data->FirstChild()->Value());
        ss>>spec_file;
      }
      else
      { throw(std::runtime_error("\nError reading robot spec file name")); }

      //Add the root node
      tmp_link_ds = new SRigidBody();
      if(S_NULL == tmp_link_ds) { throw(std::runtime_error("Couldn't allocate a root link")); }
      tmp_link_ds->init();

      TiXmlElement *tmp_ele = _robot_handle.FirstChildElement("root_link").ToElement();
      if(S_NULL == tmp_ele)
      { throw(std::runtime_error("Couldn't find a root link with the robot specification"));  }

      TiXmlHandle root_link_handle(tmp_ele); //Back to handles
      flag = CSclTiXmlParser::readLink(root_link_handle, *tmp_link_ds, true);

      SRigidBody* tmp_root = arg_robot.robot_br_rep_.create(tmp_link_ds->name_,*tmp_link_ds,true); //Add the root link
      if(S_NULL == tmp_root)
      { throw(std::runtime_error("Couldn't create the root link in the branching representation"));  }
      tmp_root->is_root_=true;
      tmp_root->robot_name_ = arg_robot_name;
      tmp_root->link_id_ = -1;
      delete tmp_link_ds; tmp_link_ds = S_NULL;

      //Now read the generic robot specification from the spec file.
      flag = readRobotSpecFromFile(spec_file, spec, arg_robot);
      if(false == flag)
      {
        std::string err;
        err ="Could not read the robot's spec (" + spec +") from its file : "+ spec_file;
        throw(std::runtime_error(err));
      }

      //Now read the generic muscle specification from the spec file.
      if("" != spec_muscle)
      {
        flag = readMuscleSpecFromFile(spec_file, spec_muscle, /*ret val */arg_robot);
        if(false == flag)
        {
          std::string err;
          err ="Could not read the robot's muscle spec (" + spec_muscle +") from its file : "+ spec_file;
          throw(std::runtime_error(err));
        }
      }

      arg_robot.dof_ = arg_robot.robot_br_rep_.size() - 1;//The root node is stationary
      flag = arg_robot.robot_br_rep_.linkNodes();
      if(false == flag)
      { throw(std::runtime_error("Could not link robot's nodes.")); }

      //Set up the remaining robot variables
      arg_robot.gc_pos_limit_max_.setZero(arg_robot.dof_);
      arg_robot.gc_pos_limit_min_.setZero(arg_robot.dof_);
      arg_robot.gc_pos_default_.setZero(arg_robot.dof_);

      sutil::CMappedTree<std::basic_string<char>, scl::SRigidBody>::iterator it, ite;
      for(it = arg_robot.robot_br_rep_.begin(), ite = arg_robot.robot_br_rep_.end();
          it!=ite; ++it)
      {
        const SRigidBody& lnk = *it;
        if(lnk.link_id_ >= 0)
        {
          arg_robot.gc_pos_limit_max_(lnk.link_id_) = lnk.joint_limit_upper_;
          arg_robot.gc_pos_limit_min_(lnk.link_id_) = lnk.joint_limit_lower_;
          arg_robot.gc_pos_default_(lnk.link_id_) = lnk.joint_default_pos_;
        }
      }

    }//End of loop over robots in the xml file.

    if(robot_ctr<1)
    { throw(std::runtime_error("Didn't find any robots with the given name.")); }

    return true;
  }
  catch(std::exception& e)
  {
    if(S_NULL!=tmp_link_ds) { delete tmp_link_ds; } //Clear out memory
    std::cerr<<"\nCSclParser::readRobotFromFile("<<arg_file<<", "
        <<arg_robot_name<<") : "<<e.what();
  }
  return false;
}


bool CSclParser::readRobotSpecFromFile(const std::string& arg_spec_file,
    const std::string& arg_robot_spec_name,
    scl::SRobotParsedData& arg_robot)
{
  bool flag;
  SRigidBody* tmp_link_ds=S_NULL;
  try
  {
    //Set up the parser.
    TiXmlElement* tiElem_robot;
    TiXmlHandle tiHndl_file_handle(NULL), tiHndl_world(NULL);
    TiXmlDocument tiDoc_file(arg_spec_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw std::runtime_error("Could not open xml file to read robot spec"); }


    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChildElement( "scl" );

    //Read in the links.
    tiElem_robot = tiHndl_world.FirstChildElement( "robot" ).ToElement();
    if(NULL == tiElem_robot) //Unnamed robot
    { throw std::runtime_error("No robot specs found in spec file"); }

    sUInt robot_ctr=0;

    //Iterating with TiXmlElement is faster than TiXmlHandle
    for(; tiElem_robot; tiElem_robot=tiElem_robot->NextSiblingElement("robot") )
    {
      TiXmlHandle _robot_handle(tiElem_robot); //Back to handles

      //Read robot name
      if(NULL == tiElem_robot->Attribute("spec_name")) //Unnamed robot
      {
#ifdef DEBUG
        std::cout<<"Warning : Found unnamed spec.";
#endif
        continue;
      }

      if(arg_robot_spec_name != tiElem_robot->Attribute("spec_name")) //Names don't match
      {
#ifdef DEBUG
        std::cout<<"Warning : Wanted spec: "<<arg_robot_spec_name<<". Got: "<<tiElem_robot->Attribute("spec_name");
#endif
        continue;
      }

      robot_ctr++; //Found a candidate
      if(robot_ctr>1) //There should only be one robot with this name.
      {
#ifdef DEBUG
        //Stronger error checking in debug mode. Break if robot names aren't unique.
        throw(std::runtime_error("Multiple robots have the same name"));
#endif
        break; //In release mode, we ignore the extra robots. Move on.
      }

      //Now actually read in the robot's links

      //Now parse the child nodes
      sInt id = 0;
      TiXmlElement* _child_link_element = _robot_handle.FirstChildElement( "link" ).ToElement();
      for(; _child_link_element; _child_link_element=_child_link_element->NextSiblingElement("link") )
      {
        TiXmlHandle _child_link_handle(_child_link_element); //Back to handles
        tmp_link_ds = new SRigidBody();
        if(S_NULL == tmp_link_ds) { throw(std::runtime_error("Couldn't allocate a root link")); }
        tmp_link_ds->init();
        tmp_link_ds->link_id_ = id; id++;
        flag = CSclTiXmlParser::readLink(_child_link_handle, *tmp_link_ds, false);
        if(false == flag)
        { throw(std::runtime_error("Couldn't read a link")); }
        //Add the root node to the robdef
        SRigidBody* tmp_link_child_ds = arg_robot.robot_br_rep_.create(tmp_link_ds->name_,*tmp_link_ds, false);
        tmp_link_child_ds->robot_name_ = arg_robot.name_;
        delete tmp_link_ds; tmp_link_ds = S_NULL;
      }

    }//End of loop over robots in the xml file.

    if(robot_ctr<1)
    {
      std::string errstr = "Didn't find any robot spec called: "+arg_robot_spec_name;
      throw(std::runtime_error(errstr.c_str()));
    }

    return true;
  }
  catch(std::exception& e)
  {
    if(S_NULL!=tmp_link_ds) { delete tmp_link_ds; } //Clear out memory
    std::cerr<<"\nCSclParser::readRobotSpecFromFile("
        <<arg_spec_file<<"): "<<e.what();
  }
  return false;
}

bool CSclParser::readMuscleSpecFromFile(const std::string& arg_spec_file,
    const std::string& arg_muscle_spec_name,
    scl::SRobotParsedData& arg_robot)
{
  bool flag;
  try
  {
    //Set up the parser.
    TiXmlElement* tiElem_muscle;
    TiXmlHandle tiHndl_file_handle(NULL), tiHndl_world(NULL);
    TiXmlDocument tiDoc_file(arg_spec_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw std::runtime_error("Could not open xml file to read muscle spec"); }


    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChildElement( "scl" );

    //Read in the links.
    tiElem_muscle = tiHndl_world.FirstChildElement( "muscle_system" ).ToElement();
    if(NULL == tiElem_muscle) //Unnamed muscle
    { throw std::runtime_error("No muscle system specs found in spec file"); }

    sUInt muscle_spec=0;

    //Iterating with TiXmlElement is faster than TiXmlHandle
    for(; tiElem_muscle; tiElem_muscle=tiElem_muscle->NextSiblingElement("muscle_system") )
    {
      TiXmlHandle _muscle_handle(tiElem_muscle); //Back to handles

      //Read muscle name
      if(NULL == tiElem_muscle->Attribute("spec_name")) //Unnamed muscle
      {
#ifdef DEBUG
        std::cout<<"Warning : Found unnamed spec.";
#endif
        continue;
      }

      if(arg_muscle_spec_name != tiElem_muscle->Attribute("spec_name")) //Names don't match
      {
#ifdef DEBUG
        std::cout<<"Warning : Wanted spec: "<<arg_muscle_spec_name<<". Got: "<<tiElem_muscle->Attribute("spec_name");
#endif
        continue;
      }

      //Set the muscle name
      arg_robot.muscle_system_.name_ = arg_muscle_spec_name;

      muscle_spec++; //Found a candidate
      if(muscle_spec>1) //There should only be one muscle with this name.
      {
#ifdef DEBUG
        //Stronger error checking in debug mode. Break if muscle names aren't unique.
        throw(std::runtime_error("Multiple muscle_system specs have the same name"));
#endif
        break; //In release mode, we ignore the extra muscle_system specs. Move on.
      }

      //Now actually read in the muscle_system by parsing all the muscles
      TiXmlElement* _child_link_element = _muscle_handle.FirstChildElement( "muscle" ).ToElement();
      for(; _child_link_element; _child_link_element=_child_link_element->NextSiblingElement("muscle") )
      {
        //Allocate an object for the muscle to be parsed
        SMuscle tmp_musc_ds;

        //Read in the xml data into the muscle object
        TiXmlHandle _child_link_handle(_child_link_element); //Back to handles
        flag = CSclTiXmlParser::readMuscle(_child_link_handle, tmp_musc_ds, false);
        if(false == flag)
        { throw(std::runtime_error("Couldn't parse a muscle:")); }

        //Add the root node to the robdef
        SMuscle *tmp_musc_ds2 = arg_robot.muscle_system_.muscles_.create(tmp_musc_ds.name_,tmp_musc_ds);
        if(S_NULL == tmp_musc_ds2)
        { throw(std::runtime_error(std::string("Couldn't allocate a muscle (id taken already?):") + tmp_musc_ds.name_)); }

        tmp_musc_ds2 = S_NULL;
      }

    }//End of loop over robots in the xml file.

    if(muscle_spec<1)
    { throw(std::runtime_error(std::string("Didn't find any muscle spec called: ")+arg_muscle_spec_name)); }

    arg_robot.muscle_system_.has_been_init_ = true;
    return true;
  }
  catch(std::exception& e)
  { std::cerr<<"\nCSclParser::readMuscleSpecFromFile("<<arg_spec_file<<"): "<<e.what(); }
  return false;
}




bool CSclParser::saveRobotToFile(scl::SRobotParsedData& arg_robot,
    const std::string &arg_file)
{
  FILE* fp=S_NULL;
  try
  {
    const SRigidBody* tmp_lnk = arg_robot.robot_br_rep_.getRootNode();
    if(S_NULL == tmp_lnk)
    { throw(std::runtime_error("The robot doesn't have a root")); }

    std::stringstream tmp_sstr;

    //Open file for writing.
    fp = fopen(arg_file.c_str(),"w");
    if(S_NULL==fp){ throw(std::runtime_error( "Couldn't open file for writing")); }
    else{ std::cout<<"\nWriting robot <"<<arg_robot.name_<<"> to file : "<<arg_file;  }

    //Else save xml info.
    fprintf(fp, "<?xml version=\"1.0\"?>\n<!DOCTYPE LOTUS SYSTEM \"scl.dtd\">" );
    fprintf(fp, "\n<scl>");
    fprintf(fp, "\n<robot spec_name=\"%s\">",arg_robot.name_.c_str());

    /**
     * Root link
     */
    fprintf(fp, "\n\t<root_link>");
    fprintf(fp, "\n\t<!-- NOTE : Typically, remove the root_link and add it to a RobotCfg.xml file (see specs for examples) -->");
    fprintf(fp, "\n\t\t<link_name>%s</link_name>",tmp_lnk->name_.c_str());
    fprintf(fp, "\n\t\t<position_in_parent>%lf %lf %lf</position_in_parent>",
        tmp_lnk->pos_in_parent_(0),tmp_lnk->pos_in_parent_(1),
        tmp_lnk->pos_in_parent_(2));
    tmp_sstr.clear();
    tmp_sstr<<tmp_lnk->ori_parent_quat_.coeffs().transpose();
    fprintf(fp, "\n\t\t<orientation_in_parent>%s</orientation_in_parent>",
        tmp_sstr.str().c_str());
    fprintf(fp, "\n\t\t<graphics>"); // SRigidBodyGraphics
    for(sUInt i=0;i<tmp_lnk->graphics_obj_vec_.size();++i)
    {
      fprintf(fp, "\n\t\t\t<obj_file>");
      fprintf(fp, "\n\t\t\t\t<name>%s</name>",
          tmp_lnk->graphics_obj_vec_[i].file_name_.c_str());
      fprintf(fp, "\n\t\t\t\t<position_in_parent>%lf %lf %lf</position_in_parent>",
                tmp_lnk->graphics_obj_vec_[i].pos_in_parent_(0),tmp_lnk->graphics_obj_vec_[i].pos_in_parent_(1),
                tmp_lnk->graphics_obj_vec_[i].pos_in_parent_(2));
      fprintf(fp, "\n\t\t\t\t<orientation_in_parent>%lf %lf %lf %lf</orientation_in_parent>",
                  tmp_lnk->graphics_obj_vec_[i].ori_parent_quat_(0), tmp_lnk->graphics_obj_vec_[i].ori_parent_quat_(1),
                  tmp_lnk->graphics_obj_vec_[i].ori_parent_quat_(2), tmp_lnk->graphics_obj_vec_[i].ori_parent_quat_(3));
      fprintf(fp, "\n\t\t\t\t<scaling>%lf %lf %lf</scaling>",
              tmp_lnk->graphics_obj_vec_[i].scaling_(0),tmp_lnk->graphics_obj_vec_[i].scaling_(1),
              tmp_lnk->graphics_obj_vec_[i].scaling_(2));
      fprintf(fp, "\n\t\t\t\t<collision_type>%d</collision_type>",
              tmp_lnk->graphics_obj_vec_[i].collision_type_);
      fprintf(fp, "\n\t\t\t</obj_file>");
    }
    fprintf(fp, "\n\t\t</graphics>");
    fprintf(fp, "\n\t</root_link>");

    /**
     * Loop over other links.
     */
    sutil::CMappedTree<std::basic_string<char>, scl::SRigidBody>::iterator it, ite;
    for(it = arg_robot.robot_br_rep_.begin(), ite = arg_robot.robot_br_rep_.end();
        it!=ite; ++it)
    {
      static sUInt link_ctr = 0;
      link_ctr++;
      if(link_ctr > arg_robot.robot_br_rep_.size())
      { throw(std::runtime_error(
          "Corrupt robot branching representation. Parsed more links than the representation contains.")); }

      scl::SRigidBody& tmp_lnk = *it;

      if(tmp_lnk.is_root_ == true)
      {//There should be only one root in the robot
        static int i=0; i++;
#ifdef DEBUG
        std::cout<<"\nFound root link in robot : "<<tmp_lnk.name_;
        if(2<=i)//This error is more of a warning but we'll raise it anyway in debug mode.
        {throw(std::runtime_error("There are multiple roots in the robot"));  }
#endif
        //Already wrote the root node earlier
        continue;
      }

      //Write the data (non-root node)
      fprintf(fp, "\n\t<link>");
      fprintf(fp, "\n\t\t<link_name>%s</link_name>",tmp_lnk.name_.c_str());
      fprintf(fp, "\n\t\t<position_in_parent>%lf %lf %lf</position_in_parent>",
          tmp_lnk.pos_in_parent_(0),tmp_lnk.pos_in_parent_(1),
          tmp_lnk.pos_in_parent_(2));
      std::stringstream tmp_sstr2;
      tmp_sstr2.clear();
      tmp_sstr2<<tmp_lnk.ori_parent_quat_.coeffs().transpose();
      fprintf(fp, "\n\t\t<orientation_in_parent>%s</orientation_in_parent>",
          tmp_sstr2.str().c_str());

      //Non-root node specific stuff
      fprintf(fp, "\n\t\t<mass>%lf</mass>",tmp_lnk.mass_);
      fprintf(fp, "\n\t\t<inertia>%lf %lf %lf %lf %lf %lf</inertia>",tmp_lnk.inertia_(0,0),
          tmp_lnk.inertia_(1,1),tmp_lnk.inertia_(2,2), tmp_lnk.inertia_(0,1),
          tmp_lnk.inertia_(0,2), tmp_lnk.inertia_(1,2));
      fprintf(fp, "\n\t\t<center_of_mass>%lf %lf %lf</center_of_mass>",tmp_lnk.com_(0),
          tmp_lnk.com_(1),tmp_lnk.com_(2));

      fprintf(fp, "\n\t\t<joint_name>%s</joint_name>",tmp_lnk.joint_name_.c_str());
      fprintf(fp, "\n\t\t<parent_link_name>%s</parent_link_name>",tmp_lnk.parent_name_.c_str());

      std::string jtype;
      switch(tmp_lnk.joint_type_)
      {
        case JOINT_TYPE_PRISMATIC_X:
          jtype = "px"; break;
        case JOINT_TYPE_PRISMATIC_Y:
          jtype = "py"; break;
        case JOINT_TYPE_PRISMATIC_Z:
          jtype = "pz"; break;
        case JOINT_TYPE_REVOLUTE_X:
          jtype = "rx"; break;
        case JOINT_TYPE_REVOLUTE_Y:
          jtype = "ry"; break;
        case JOINT_TYPE_REVOLUTE_Z:
          jtype = "rz"; break;
        case JOINT_TYPE_SPHERICAL:
          jtype = "sp"; break;
        case JOINT_TYPE_SPLINE:
          jtype = "sl"; break;
        case JOINT_TYPE_NOTASSIGNED:
          jtype = "ERROR"; break;
      }
      fprintf(fp, "\n\t\t<joint_type>%s</joint_type>",jtype.c_str());

      fprintf(fp, "\n\t\t<joint_limits>%lf %lf</joint_limits>",
          tmp_lnk.joint_limit_lower_, tmp_lnk.joint_limit_upper_);
      fprintf(fp, "\n\t\t<default_joint_position>%lf</default_joint_position>",
          tmp_lnk.joint_default_pos_);

      fprintf(fp, "\n\t\t<graphics>");
      for(sUInt i=0;i<tmp_lnk.graphics_obj_vec_.size();++i)
      {
        fprintf(fp, "\n\t\t\t<obj_file>");
        fprintf(fp, "\n\t\t\t\t<name>%s</name>",
            tmp_lnk.graphics_obj_vec_[i].file_name_.c_str());
        fprintf(fp, "\n\t\t\t\t<position_in_parent>%lf %lf %lf</position_in_parent>",
            tmp_lnk.graphics_obj_vec_[i].pos_in_parent_(0),tmp_lnk.graphics_obj_vec_[i].pos_in_parent_(1),
            tmp_lnk.graphics_obj_vec_[i].pos_in_parent_(2));
        fprintf(fp, "\n\t\t\t\t<orientation_in_parent>%lf %lf %lf %lf</orientation_in_parent>",
            tmp_lnk.graphics_obj_vec_[i].ori_parent_quat_(0), tmp_lnk.graphics_obj_vec_[i].ori_parent_quat_(1),
            tmp_lnk.graphics_obj_vec_[i].ori_parent_quat_(2), tmp_lnk.graphics_obj_vec_[i].ori_parent_quat_(3));
        fprintf(fp, "\n\t\t\t\t<scaling>%lf %lf %lf</scaling>",
            tmp_lnk.graphics_obj_vec_[i].scaling_(0),tmp_lnk.graphics_obj_vec_[i].scaling_(1),
            tmp_lnk.graphics_obj_vec_[i].scaling_(2));
        fprintf(fp, "\n\t\t\t\t<collision_type>%d</collision_type>",
            tmp_lnk.graphics_obj_vec_[i].collision_type_);
        fprintf(fp, "\n\t\t\t</obj_file>");
      }
      fprintf(fp, "\n\t\t</graphics>");
      fprintf(fp, "\n\t</link>");
    }
    //Finish
    fprintf(fp, "\n</robot>");
    fprintf(fp, "\n</scl>");
  }
  catch(std::exception& e)
  {
    std::cout<<"\nsaveRobotToFile() : Failed. "<<e.what();
    if(S_NULL!=fp){ fclose(fp); }
    return false;
  }
  if(S_NULL!=fp){ fclose(fp); }
  return true;
}


bool CSclParser::readGraphicsFromFile(const std::string &arg_file,
    const std::string &arg_graphics_name, scl::SGraphicsParsedData& arg_graphics)
{
  bool flag;
  try
  {
    //Set up the parser.
    TiXmlElement* tiElem_graphics, *tiElem_lights, *tiElem_bkg_color;
    TiXmlHandle tiHndl_glob_settings(NULL), tiHndl_file_handle(NULL), tiHndl_world(NULL);

    TiXmlDocument tiDoc_file(arg_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw std::runtime_error("Could not open xml file to read graphics definition."); }

    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChildElement( "scl" );

    //2. Read in the links.
    tiElem_graphics = tiHndl_world.FirstChildElement( "graphics" ).ToElement();
    flag = false;
    //Iterating with TiXmlElement is faster than TiXmlHandle
    for(; tiElem_graphics; tiElem_graphics=tiElem_graphics->NextSiblingElement("graphics") )
    {
      std::string name = tiElem_graphics->Attribute("name");
      if(name!=arg_graphics_name)
      {
        arg_graphics.name_ = "NotInitialized";
        continue;
      }

      arg_graphics.name_ = tiElem_graphics->Attribute("name");

      TiXmlHandle _gr_handle(tiElem_graphics); //Back to handles

      TiXmlElement* gr_data = _gr_handle.FirstChildElement("camera").FirstChildElement("pos").Element();
      if ( gr_data )
      {
        std::stringstream ss(gr_data->FirstChild()->Value());
        ss>>arg_graphics.cam_pos_[0];
        ss>>arg_graphics.cam_pos_[1];
        ss>>arg_graphics.cam_pos_[2];
      }
      else
      { throw(std::runtime_error("No position information."));  }

      gr_data = _gr_handle.FirstChildElement("camera").FirstChildElement("lookat").Element();
      if ( gr_data )
      {
        std::stringstream ss(gr_data->FirstChild()->Value());
        ss>>arg_graphics.cam_lookat_[0];
        ss>>arg_graphics.cam_lookat_[1];
        ss>>arg_graphics.cam_lookat_[2];
      }
      else
      { throw(std::runtime_error("No look-at information."));  }

      gr_data = _gr_handle.FirstChildElement("camera").FirstChildElement("up").Element();
      if ( gr_data )
      {
        std::stringstream ss(gr_data->FirstChild()->Value());
        ss>>arg_graphics.cam_up_[0];
        ss>>arg_graphics.cam_up_[1];
        ss>>arg_graphics.cam_up_[2];
      }
      else
      { throw(std::runtime_error("No up-direction information."));  }

      gr_data = _gr_handle.FirstChildElement("camera").FirstChildElement("clip").Element();
      if ( gr_data )
      {
        std::stringstream ss(gr_data->FirstChild()->Value());
        ss>>arg_graphics.cam_clipping_dist_[0];
        ss>>arg_graphics.cam_clipping_dist_[1];
      }
      else
      { throw(std::runtime_error("No clipping information."));  }

      gr_data = _gr_handle.FirstChildElement("camera").FirstChildElement("background").Element();
      if ( gr_data )
      {
        std::stringstream ss(gr_data->FirstChild()->Value());
        ss>>arg_graphics.background_color_[0];
        ss>>arg_graphics.background_color_[1];
        ss>>arg_graphics.background_color_[2];
      }
      else
      { throw(std::runtime_error("No background information."));  }

      //Lights
      tiElem_lights = _gr_handle.FirstChildElement( "light" ).ToElement();

      for(; tiElem_lights; tiElem_lights=tiElem_lights->NextSiblingElement("light") )
      {
        TiXmlHandle _light_handle(tiElem_lights); //Back to handles

        scl::SGraphicsParsedData::SLight tmp_light;
        gr_data = _light_handle.FirstChildElement("pos").Element();
        if ( gr_data )
        {
          std::stringstream ss(gr_data->FirstChild()->Value());
          ss>>tmp_light.pos_[0];
          ss>>tmp_light.pos_[1];
          ss>>tmp_light.pos_[2];
        }
        else
        { throw(std::runtime_error("No light-pos information."));  }

        gr_data = _light_handle.FirstChildElement("lookat").Element();
        if ( gr_data )
        {
          std::stringstream ss(gr_data->FirstChild()->Value());
          ss>>tmp_light.lookat_[0];
          ss>>tmp_light.lookat_[1];
          ss>>tmp_light.lookat_[2];
        }
        else
        { throw(std::runtime_error("No light-lookat information."));  }

        arg_graphics.lights_.push_back(tmp_light);
      }

      //Read in the background color
      tiElem_bkg_color = _gr_handle.FirstChildElement( "background_color" ).ToElement();
      if(NULL!=tiElem_bkg_color)
      {
        std::stringstream ss(tiElem_bkg_color->FirstChild()->Value());
        ss>>arg_graphics.background_color_[0];
        ss>>arg_graphics.background_color_[1];
        ss>>arg_graphics.background_color_[2];
      }
      else
      {
        std::cout<<"\nCSclParser::readGraphicsFromFile() : No background color. Setting to black {0.0, 0.0, 0.0}";
        arg_graphics.background_color_[0] = 0.0;
        arg_graphics.background_color_[1] = 0.0;
        arg_graphics.background_color_[2] = 0.0;
      }

      flag = true;

    }//End of loop over graphics objects in the xml file.

    if(false == flag)
    { throw(std::runtime_error("Did not find specified graphics specification in the file")); }

    return true;
  }
  catch(std::exception& e)
  { std::cerr<<"\nCSclParser::readGraphicsFromFile(): "<<e.what();  }
  return false;
}

bool CSclParser::listGraphicsInFile(const std::string& arg_file,
    std::vector<std::string>& arg_graphics_names)
{
  bool flag;
  try
  {
    //Set up the parser.
    TiXmlElement* tiElem_robot;
    TiXmlHandle tiHndl_glob_settings(NULL), tiHndl_file_handle(NULL), tiHndl_world(NULL);

    TiXmlDocument tiDoc_file(arg_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw std::runtime_error("Could not open xml file to read robots."); }

    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChildElement( "scl" );

    //2. Read in the robots.
    tiElem_robot = tiHndl_world.FirstChildElement( "graphics" ).ToElement();

    //Iterating with TiXmlElement is faster than TiXmlHandle
    for(; tiElem_robot; tiElem_robot=tiElem_robot->NextSiblingElement("graphics") )
    {
      TiXmlHandle _robot_handle(tiElem_robot); //Back to handles

      //Read robot name
      if(NULL == tiElem_robot->Attribute("name"))
      {//Unique robot name generated by timestamp
        std::stringstream ss;
        static int ilist=0;
        ss.clear();
        ss<<ilist;
        ilist++;
        std::string tmp = "Unnamed_Graphics_" + ss.str();
        arg_graphics_names.push_back(tmp);
      }
      else
      {
        std::string tmp= tiElem_robot->Attribute("name");
        arg_graphics_names.push_back(tmp);
      }
    }

    if(arg_graphics_names.size()<=0)
    { throw("Couldn't find any graphics specification.");}
  }
  catch(std::exception& e)
  {
    std::cerr<<"\nCSclParser::listRobotsInFile() :"<<e.what();
    arg_graphics_names.clear();
    return false;
  }
  return true; //Success.
}

bool CSclParser::listControllersInFile(const std::string &arg_file,
      std::vector<std::pair<std::string,std::string> > &arg_ctrl_name_and_type)
{
  bool flag;
  try
  {
    //Set up the parser.
    TiXmlElement* tiElem_ctrl;
    TiXmlHandle tiHndl_glob_settings(NULL), tiHndl_file_handle(NULL), tiHndl_world(NULL);

    TiXmlDocument tiDoc_file(arg_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw std::runtime_error("Could not open xml file to read controllers."); }

    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChildElement( "scl" );

    //Read in the robots.
    tiElem_ctrl = tiHndl_world.FirstChildElement( "controller" ).ToElement();

    //Iterating with TiXmlElement is faster than TiXmlHandle
    for(; tiElem_ctrl; tiElem_ctrl=tiElem_ctrl->NextSiblingElement("controller") )
    {
      if(NULL == tiElem_ctrl->Attribute("name"))
      {
#ifdef DEBUG
        throw(std::runtime_error("File contains a controller without a name"));
#endif
      }
      else
      {
        std::string tmp_name= tiElem_ctrl->Attribute("name");
        std::string tmp_type= tiElem_ctrl->FirstChildElement("type")->FirstChild()->Value();
        std::pair<std::string,std::string> tmp_pair(tmp_name,tmp_type);
        arg_ctrl_name_and_type.push_back(tmp_pair);
      }
    }
    if(arg_ctrl_name_and_type.size()<=0)
    { throw(std::runtime_error("Couldn't find any controller specifications in the file."));}
  }
  catch(std::exception& e)
  {
    std::cerr<<"\nCSclParser::listControllersInFile() :"<<e.what();
    return false;
  }
  return true; //Success.
}

bool CSclParser::readGcControllerFromFile(const std::string &arg_file,
      const std::string &arg_ctrl_name,
      std::string &arg_must_use_robot,
      scl::SGcController& arg_ctrl)
{
  bool flag;
  try
  {
    //Set up the parser.
    TiXmlElement* tiElem_gc_ctrl;
    TiXmlHandle tiHndl_glob_settings(NULL), tiHndl_file_handle(NULL), tiHndl_world(NULL);

    TiXmlDocument tiDoc_file(arg_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw std::runtime_error("Could not open xml file to read gc-controller definition."); }

    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChildElement( "scl" );

    //2. Read in the controller.
    tiElem_gc_ctrl = tiHndl_world.FirstChildElement( "controller" ).ToElement();
    flag = false;
    //Iterating with TiXmlElement is faster than TiXmlHandle
    for(; tiElem_gc_ctrl; tiElem_gc_ctrl=tiElem_gc_ctrl->NextSiblingElement("controller") )
    {
      std::string name = tiElem_gc_ctrl->Attribute("name");
      if(name!=arg_ctrl_name)
      {
        arg_ctrl.name_ = "NotInitialized";
        continue;
      }

      arg_ctrl.name_ = tiElem_gc_ctrl->Attribute("name");

      TiXmlHandle _cr_handle(tiElem_gc_ctrl); //Back to handles

      TiXmlElement* cr_data = _cr_handle.FirstChildElement("type").Element();
      if ( cr_data )
      {
        std::string type(cr_data->FirstChild()->Value());
        if(type!="gc")
        {
          arg_ctrl.name_ = "NotInitialized";
          continue;
        }
      }
      else
      { throw(std::runtime_error("No controller type."));  }

      cr_data = _cr_handle.FirstChildElement("must_use_robot").Element();
      if ( cr_data )
      {
        std::string tmp(cr_data->FirstChild()->Value());
        arg_must_use_robot = tmp;
      }
      else
      { arg_must_use_robot=""; }

      cr_data = _cr_handle.FirstChildElement("kp").Element();
      if ( cr_data )
      {
        unsigned int sz=0;
        //Count how many numbers the string has
        const char* ss = cr_data->FirstChild()->Value();
        if(ss==NULL)
        { throw(std::runtime_error("No kp information."));  }

        sz = scl_util::countNumbersInString(ss);
        if(0==sz)
        { throw(std::runtime_error("No kp information."));  }

        //Now save them to the kp vector
        arg_ctrl.kp_.setZero(sz);
        std::stringstream sstr(cr_data->FirstChild()->Value());
        for(unsigned int i=0;i<sz;i++)
        { sstr>>arg_ctrl.kp_(i);  }
      }
      else
      { throw(std::runtime_error("No kp information."));  }

      cr_data = _cr_handle.FirstChildElement("kv").Element();
      if ( cr_data )
      {
        unsigned int sz=0;
        //Count how many numbers the string has
        const char* ss = cr_data->FirstChild()->Value();
        if(ss==NULL)
        { throw(std::runtime_error("No kv information."));  }

        sz = scl_util::countNumbersInString(ss);
        if(0==sz)
        { throw(std::runtime_error("No kv information."));  }

        //Now save them to the kv vector
        arg_ctrl.kv_.setZero(sz);
        std::stringstream sstr(cr_data->FirstChild()->Value());
        for(unsigned int i=0;i<sz;i++)
        { sstr>>arg_ctrl.kv_(i);  }
      }
      else
      { throw(std::runtime_error("No kv information."));  }

      cr_data = _cr_handle.FirstChildElement("ka").Element();
      if ( cr_data )
      {
        unsigned int sz=0;
        //Count how many numbers the string has
        const char* ss = cr_data->FirstChild()->Value();
        if(ss==NULL)
        { throw(std::runtime_error("No ka information."));  }

        sz = scl_util::countNumbersInString(ss);
        if(0==sz)
        { throw(std::runtime_error("No ka information."));  }

        //Now save them to the ka vector
        arg_ctrl.ka_.setZero(sz);
        std::stringstream sstr(cr_data->FirstChild()->Value());
        for(unsigned int i=0;i<sz;i++)
        { sstr>>arg_ctrl.ka_(i);  }
      }
      else
      { throw(std::runtime_error("No ka information."));  }

      cr_data = _cr_handle.FirstChildElement("ki").Element();
      if ( cr_data )
      {
        unsigned int sz=0;
        //Count how many numbers the string has
        const char* ss = cr_data->FirstChild()->Value();
        if(ss==NULL)
        { throw(std::runtime_error("No ki information."));  }

        sz = scl_util::countNumbersInString(ss);
        if(0==sz)
        { throw(std::runtime_error("No ki information."));  }

        //Now save them to the ki vector
        arg_ctrl.ki_.setZero(sz);
        std::stringstream sstr(cr_data->FirstChild()->Value());
        for(unsigned int i=0;i<sz;i++)
        { sstr>>arg_ctrl.ki_(i);  }
      }
      else
      {
        arg_ctrl.ki_.setZero(1);
        std::cerr<<"\nCSclParser::readGcControllerFromFile() : WARNING : In Controller ["<<arg_ctrl.name_
            <<"]No ki information. Setting to zero.";
      }

      cr_data = _cr_handle.FirstChildElement("force_max").Element();
      if ( cr_data )
      {
        unsigned int sz=0;
        //Count how many numbers the string has
        const char* ss = cr_data->FirstChild()->Value();
        if(ss==NULL)
        { throw(std::runtime_error("No max force information."));  }

        sz = scl_util::countNumbersInString(ss);

        //Now save them to the force max
        arg_ctrl.force_gc_max_.setZero(sz);
        std::stringstream sstr(cr_data->FirstChild()->Value());
        for(unsigned int i=0;i<sz;i++)
        { sstr>>arg_ctrl.force_gc_max_(i);  }
      }
      else
      { throw(std::runtime_error("No max force information."));  }

      cr_data = _cr_handle.FirstChildElement("force_min").Element();
      if ( cr_data )
      {
        unsigned int sz=0;
        //Count how many numbers the string has
        const char* ss = cr_data->FirstChild()->Value();
        if(ss==NULL)
        { throw(std::runtime_error("No min force information."));  }

        sz = scl_util::countNumbersInString(ss);

        //Now save them to the force min
        arg_ctrl.force_gc_min_.setZero(sz);
        std::stringstream sstr(cr_data->FirstChild()->Value());
        for(unsigned int i=0;i<sz;i++)
        { sstr>>arg_ctrl.force_gc_min_(i);  }
      }
      else
      { throw(std::runtime_error("No min force information."));  }

      flag = true;
    }//End of loop over controllers in the xml file.
    if(false == flag)
    { throw(std::runtime_error("Controller was not found in the file."));  }
  }
  catch(std::exception& e)
  {
    std::cerr<<"\nCSclParser::readGcControllerFromFile() : "<<e.what();
    return false;
  }
  return true; //Success.
}

bool CSclParser::readTaskControllerFromFile(const std::string &arg_file,
      const std::string &arg_ctrl_name,
      std::string &arg_must_use_robot,
      scl::STaskController& arg_ctrl,
      std::vector<STaskBase*> &arg_taskvec,
      std::vector<SNonControlTaskBase*> &arg_task_non_ctrl_vec)
{
  bool flag;
  try
  {
    //Set up the parser.
    TiXmlElement* tiElem_tctrl_ctrl, * tiElem_task_ctrl;
    TiXmlHandle tiHndl_glob_settings(NULL), tiHndl_file_handle(NULL), tiHndl_world(NULL);

    TiXmlDocument tiDoc_file(arg_file.c_str());

    //Check if file opened properly
    flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
    if(false == flag)
    { throw(std::runtime_error("Could not open xml file to read task-controller definition.")); }

    //Get handles to the tinyxml loaded ds
    tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
    tiHndl_world = tiHndl_file_handle.FirstChildElement( "scl" );

    //2. Read in the links.
    tiElem_tctrl_ctrl = tiHndl_world.FirstChildElement( "controller" ).ToElement();

    flag = false;//Set to true if atleast one controller was found in the file.

    //Iterating with TiXmlElement is faster than TiXmlHandle
    for(; tiElem_tctrl_ctrl; tiElem_tctrl_ctrl=tiElem_tctrl_ctrl->NextSiblingElement("controller") )
    {
      std::string name = tiElem_tctrl_ctrl->Attribute("name");
      if(name!=arg_ctrl_name)
      {
        arg_ctrl.name_ = "NotInitialized";
        continue;
      }

      if(true == flag)
      { throw(std::runtime_error("Already parsed a controller with this name. Two controllers can't have the same name.")); }

      arg_ctrl.name_ = tiElem_tctrl_ctrl->Attribute("name");

      TiXmlHandle _cr_handle(tiElem_tctrl_ctrl); //Back to handles

      TiXmlElement* cr_data = _cr_handle.FirstChildElement("type").Element();
      if ( cr_data )
      {
        std::string type(cr_data->FirstChild()->Value());
        if(type!="task")
        {
          arg_ctrl.name_ = "NotInitialized";
          continue;
        }
      }
      else
      { throw(std::runtime_error("No controller type."));  }

      cr_data = _cr_handle.FirstChildElement("must_use_robot").Element();
      if ( cr_data )
      {
        std::string type(cr_data->FirstChild()->Value());
        arg_must_use_robot = type;
      }
      else
      { arg_must_use_robot=""; }

      /** ***********************************************
       *    PARSE ALL THE CONTROL TASKS HERE
       * ************************************************ */
      tiElem_task_ctrl = _cr_handle.FirstChildElement( "task" ).ToElement();
      //Iterating with TiXmlElement is faster than TiXmlHandle
      for(; tiElem_task_ctrl; tiElem_task_ctrl=tiElem_task_ctrl->NextSiblingElement("task") )
      {
        /** *********************************************
         *  PARSE ALL THE STANDARD CONTROLLER OPTIONS
         *
         *  These are contained in the STaskBase data
         *  structure
         *  *********************************************/
        STaskBase* tmp_task = new STaskParsedData();

        tmp_task->name_ = tiElem_task_ctrl->Attribute("name");

        TiXmlHandle _task_handle(tiElem_task_ctrl); //Back to handles

        cr_data = _task_handle.FirstChildElement("type").Element();
        if ( cr_data )
        {
          std::string type(cr_data->FirstChild()->Value());
          tmp_task->type_task_ = type;
        }
        else
        { throw(std::runtime_error("No task type."));  }

        cr_data = _task_handle.FirstChildElement("priority").Element();
        if ( cr_data )
        {
          std::stringstream sstr(cr_data->FirstChild()->Value());
          sstr>>tmp_task->priority_;
        }
        else
        { throw(std::runtime_error("No priority information."));  }

        cr_data = _task_handle.FirstChildElement("task_dof").Element();
        if ( cr_data )
        {
          std::stringstream sstr(cr_data->FirstChild()->Value());
          sstr>>tmp_task->dof_task_;
        }
        else
        { throw(std::runtime_error("No task dof information."));  }

        cr_data = _task_handle.FirstChildElement("kp").Element();
        if ( cr_data )
        {
          unsigned int sz=0;
          //Count how many numbers the string has
          const char* ss = cr_data->FirstChild()->Value();
          if(ss==NULL)
          { throw(std::runtime_error("No kp information."));  }

          sz = scl_util::countNumbersInString(ss);
          if(0==sz)
          { throw(std::runtime_error("No kp information."));  }

          //Now save them to the kp vector
          tmp_task->kp_.setZero(sz);
          std::stringstream sstr(cr_data->FirstChild()->Value());
          for(unsigned int i=0;i<sz;i++)
          { sstr>>tmp_task->kp_(i);  }
        }
        else
        { throw(std::runtime_error("No kp information."));  }

        cr_data = _task_handle.FirstChildElement("kv").Element();
        if ( cr_data )
        {
          unsigned int sz=0;
          //Count how many numbers the string has
          const char* ss = cr_data->FirstChild()->Value();
          if(ss==NULL)
          { throw(std::runtime_error("No kv information."));  }

          sz = scl_util::countNumbersInString(ss);
          if(0==sz)
          { throw(std::runtime_error("No kv information."));  }

          //Now save them to the kv vector
          tmp_task->kv_.setZero(sz);
          std::stringstream sstr(cr_data->FirstChild()->Value());
          for(unsigned int i=0;i<sz;i++)
          { sstr>>tmp_task->kv_(i);  }
        }
        else
        { throw(std::runtime_error("No kv information."));  }

        cr_data = _task_handle.FirstChildElement("ka").Element();
        if ( cr_data )
        {
          unsigned int sz=0;
          //Count how many numbers the string has
          const char* ss = cr_data->FirstChild()->Value();
          if(ss==NULL)
          { throw(std::runtime_error("No ka information."));  }

          sz = scl_util::countNumbersInString(ss);
          if(0==sz)
          { throw(std::runtime_error("No ka information."));  }

          //Now save them to the ka vector
          tmp_task->ka_.setZero(sz);
          std::stringstream sstr(cr_data->FirstChild()->Value());
          for(unsigned int i=0;i<sz;i++)
          { sstr>>tmp_task->ka_(i);  }
        }
        else
        { throw(std::runtime_error("No ka information."));  }

        cr_data = _task_handle.FirstChildElement("ki").Element();
        if ( cr_data )
        {
          unsigned int sz=0;
          //Count how many numbers the string has
          const char* ss = cr_data->FirstChild()->Value();
          if(ss==NULL)
          { throw(std::runtime_error("No ki information."));  }

          sz = scl_util::countNumbersInString(ss);
          if(0==sz)
          { throw(std::runtime_error("No ki information."));  }

          //Now save them to the ki vector
          tmp_task->ki_.setZero(sz);
          std::stringstream sstr(cr_data->FirstChild()->Value());
          for(unsigned int i=0;i<sz;i++)
          { sstr>>tmp_task->ki_(i);  }
        }
        else
        {
          tmp_task->ki_.setZero(1);
          std::cerr<<"\nCSclParser::readTaskControllerFromFile() : WARNING : Task ["<<tmp_task->name_
              <<"]. No ki information. Setting to zero.";
        }

        cr_data = _task_handle.FirstChildElement("force_max").Element();
        if ( cr_data )
        {
          unsigned int sz=0;
          //Count how many numbers the string has
          const char* ss = cr_data->FirstChild()->Value();
          if(ss==NULL)
          { throw(std::runtime_error("No max force information."));  }

          sz = scl_util::countNumbersInString(ss);

          //Now save them to the force max
          tmp_task->force_task_max_.setZero(sz);
          std::stringstream sstr(cr_data->FirstChild()->Value());
          for(unsigned int i=0;i<sz;i++)
          { sstr>>tmp_task->force_task_max_(i);  }
        }
        else
        { throw(std::runtime_error("No max force information."));  }

        cr_data = _task_handle.FirstChildElement("force_min").Element();
        if ( cr_data )
        {
          unsigned int sz=0;
          //Count how many numbers the string has
          const char* ss = cr_data->FirstChild()->Value();
          if(ss==NULL)
          { throw(std::runtime_error("No min force information."));  }

          sz = scl_util::countNumbersInString(ss);

          //Now save them to the force min
          tmp_task->force_task_min_.setZero(sz);
          std::stringstream sstr(cr_data->FirstChild()->Value());
          for(unsigned int i=0;i<sz;i++)
          { sstr>>tmp_task->force_task_min_(i);  }
        }
        else
        { throw(std::runtime_error("No min force information."));  }

        /** *********************************************
         *  PARSE ALL THE NON STANDARD CONTROLLER OPTIONS
         *
         *  These are not contained in the STaskBase data
         *  structure. Each task should know what to do with
         *  them (in its init function).
         *  *********************************************/

        //Each task has some non-standard arguments
        //Save these into the vector of strings and then
        //insert them into the passed task's vector at the end
        //of the for loop
        TiXmlElement* tiElem_task_options = _task_handle.FirstChildElement().ToElement();

        //Iterating with TiXmlElement is faster than TiXmlHandle (But handles are "pointer safe" so we
        //use them anyway. Thank you TiXml..)
        for(; tiElem_task_options; tiElem_task_options=tiElem_task_options->NextSiblingElement() )
        {
          if(strcmp(tiElem_task_options->Value(),"type")==0 ||
              strcmp(tiElem_task_options->Value(),"priority")==0 ||
              strcmp(tiElem_task_options->Value(),"task_dof")==0 ||
              strcmp(tiElem_task_options->Value(),"kp")==0 ||
              strcmp(tiElem_task_options->Value(),"kv")==0 ||
              strcmp(tiElem_task_options->Value(),"ka")==0 ||
              strcmp(tiElem_task_options->Value(),"ki")==0 ||
              strcmp(tiElem_task_options->Value(),"force_max")==0 ||
              strcmp(tiElem_task_options->Value(),"force_min")==0)
          {
            //Ignore the standard options.
            continue;
          }

          scl::sString2 nonstd_param;//The name of the param and its value

          if(S_NULL == tiElem_task_options->FirstChild())
          { throw(std::runtime_error(std::string(tiElem_task_options->Value()) + std::string(" -- A non-standard task parameter's value is not specified."))); }

          nonstd_param.data_[0] = tiElem_task_options->Value();
          nonstd_param.data_[1] = tiElem_task_options->FirstChild()->Value();

#ifdef DEBUG
          std::cout<<"\nCSclParser::readTaskControllerFromFile() : Read a non standard param: "<<nonstd_param.data_[0]<<" "<<nonstd_param.data_[1];
#endif

          //Store the nonstandard param
          tmp_task->task_nonstd_params_.push_back(nonstd_param);
        }

        /** *********************************************
         *  END PARSING CONTROLLER OPTIONS
         *  **********************************************/

        //Add the parsed task to the controller' task vector
        arg_taskvec.push_back(tmp_task);

#ifdef DEBUG
          std::cout<<"\nCSclParser::readTaskControllerFromFile() : Parsed task: "<<tmp_task->name_<<". Type: "<<tmp_task->type_task_;
#endif

      }//End of loop over tasks in the xml file.

      /** ***********************************************
       *    END PARSING ALL THE CONTROL TASKS HERE
       * ************************************************ */

      /** ***********************************************
       *    PARSE ALL THE NON-CONTROL TASKS HERE
       * ************************************************ */
      tiElem_task_ctrl = _cr_handle.FirstChildElement( "task_non_ctrl" ).ToElement();
      //Iterating with TiXmlElement is faster than TiXmlHandle
      for(; tiElem_task_ctrl; tiElem_task_ctrl=tiElem_task_ctrl->NextSiblingElement("task_non_ctrl") )
      {
        /** *********************************************
         *  PARSE ALL THE STANDARD CONTROLLER OPTIONS
         *
         *  These are contained in the STaskBase data
         *  structure
         *  *********************************************/
        SNonControlTaskBase* tmp_task = new SNonControlTaskParsedData();

        tmp_task->name_ = tiElem_task_ctrl->Attribute("name");

        TiXmlHandle _task_handle(tiElem_task_ctrl); //Back to handles

        cr_data = _task_handle.FirstChildElement("type").Element();
        if ( cr_data )
        {
          std::string type(cr_data->FirstChild()->Value());
          tmp_task->type_task_ = type;
        }
        else
        { throw(std::runtime_error("No task type."));  }

        /** *********************************************
         *  PARSE ALL THE NON STANDARD CONTROLLER OPTIONS
         *
         *  These are not contained in the SNonControlTaskBase data
         *  structure. Each task should know what to do with
         *  them (in its init function).
         *  *********************************************/

        //Each task has some non-standard arguments
        //Save these into the vector of strings and then
        //insert them into the passed task's vector at the end
        //of the for loop
        TiXmlElement* tiElem_task_options = _task_handle.FirstChildElement().ToElement();

        //Iterating with TiXmlElement is faster than TiXmlHandle (But handles are "pointer safe" so we
        //use them anyway. Thank you TiXml..)
        for(; tiElem_task_options; tiElem_task_options=tiElem_task_options->NextSiblingElement() )
        {
          //Ignore the standard options.
          if(strcmp(tiElem_task_options->Value(),"type")==0)
          { continue;  }

          scl::sString2 nonstd_param;//The name of the param and its value

          if(S_NULL == tiElem_task_options->FirstChild())
          { throw(std::runtime_error(std::string(tiElem_task_options->Value()) +
              std::string(" -- A non-standard task parameter's value is not specified."))); }

          nonstd_param.data_[0] = tiElem_task_options->Value();
          nonstd_param.data_[1] = tiElem_task_options->FirstChild()->Value();

#ifdef DEBUG
          std::cout<<"\nCSclParser::readTaskControllerFromFile() : Read a non standard param: "
              <<nonstd_param.data_[0]<<" "<<nonstd_param.data_[1];
#endif
          //Store the nonstandard param
          tmp_task->task_nonstd_params_.push_back(nonstd_param);
        }
        //Add the parsed task to the controller' task vector
        arg_task_non_ctrl_vec.push_back(tmp_task);

#ifdef DEBUG
        std::cout<<"\nCSclParser::readTaskControllerFromFile() : Parsed non-control task: "<<tmp_task->name_<<". Type: "<<tmp_task->type_task_;
#endif

      }//End of loop over tasks in the xml file.

      /** ***********************************************
       *    END PARSING ALL THE NON-CONTROL TASKS HERE
       * ************************************************ */


#ifdef DEBUG
          std::cout<<"\nCSclParser::readTaskControllerFromFile() : Parsed controller: "<<arg_ctrl_name<<". Type: "<<arg_ctrl.type_ctrl_ds_;
#endif

      //Successfully parsed atleast one controller from the file
      flag = true;

    }//End of loop over controllers in the xml file.

    if(false == flag)
    { throw(std::runtime_error("Controller was not found in the file."));  }
  }
  catch(std::exception& e)
  {
    std::cerr<<"\nCSclParser::readTaskControllerFromFile() : Error : "<<e.what();
    return false;
  }
  return true; //Success.
}
}
