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
/* \file CParserOsimForOldFiles.cpp
 *
 *  Created on: Jan, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//The Class definition.
#include <scl/parser/osimparser/CParserOsimForOldFiles.hpp>

//The required data structures
#include <scl/Singletons.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SRigidBody.hpp>

using namespace scl_tinyxml; //Tinyxml parser implementation is in a separate namespace
using namespace scl;

#include <sstream>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace scl_parser {
  bool CParserOsimForOldFiles::readOsimBiomechFromFile(const std::string& arg_file,
      scl::SRobotParsed& arg_biomech,
      scl::SMuscleSetParsed& arg_msys)
  {
    sBool flag;
    try
    {
      flag = readMuscleSysFromFile(arg_file,"I_AM_SUPERFLUOUS",arg_msys);
      if(false == flag)
      { std::cout<<"\nWARNING : Could not read osim muscle definitions. Will try rigid bodies."; }

      flag = readRobotFromFile(arg_file,"I_AM_SUPERFLUOUS",arg_biomech);
      if(false == flag)
      { throw(std::runtime_error("Couldn't read osim rigid body definitions.")); }

      return true;
    }
    catch(std::exception& e)
    { std::cerr<<"\nCParserOsimForOldFiles::readOsimBiomechFromFile() : "<<e.what();  }
    return false;
  }

  bool CParserOsimForOldFiles::readMuscleSysFromFile(
      const std::string& arg_file,
      const std::string& arg_msys_name,
      scl::SMuscleSetParsed& arg_msys)
  {
    sBool flag;
    try
    {
      //Set up the parser.
      TiXmlElement * tiElem_muscle;
      TiXmlHandle tiHndl_file_handle(NULL), tiHndl_model(NULL);
      TiXmlDocument tiDoc_file(arg_file.c_str());

      //Check if file opened properly
      flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
      if(false == flag)
      { throw(std::runtime_error("Could not open xml file to read robot definition.")); }

      //Get handles to the tinyxml loaded ds
      tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
      tiHndl_model = tiHndl_file_handle.FirstChild( "OpenSimDocument" ).FirstChild( "Model" );
      if(S_NULL == tiHndl_model.ToElement())
      {//Ridiculous versioning changes everywhere.
        tiHndl_model = tiHndl_file_handle.FirstChild( "Model" );
        if(S_NULL == tiHndl_model.ToElement())
        { throw(std::runtime_error("Could not find the Model or OpenSimDocument tags")); }
      }

      //Set the model's name. Must be unique.
      std::string model_name;
      if(NULL == tiHndl_model.ToElement()->Attribute("name")) //Unnamed model : Throw error
      { throw(std::runtime_error("Found an unnamed model. XML file is corrupt.")); }
      else
      { model_name = tiHndl_model.ToElement()->Attribute("name"); }

      //1. Read muscle system(s)
      arg_msys.name_ = model_name;
      arg_msys.must_use_robot_ = model_name;

      //Read in the muscles : Regardless of type.
      tiElem_muscle = tiHndl_model.FirstChild( "ActuatorSet" ).
          FirstChild( "objects" ).FirstChild().ToElement();

      //Iterating with TiXmlElement is faster than TiXmlHandle
      for(; tiElem_muscle; tiElem_muscle=tiElem_muscle->NextSiblingElement() )
      {
        TiXmlHandle _muscle_handle(tiElem_muscle); //Back to handles. Hail TiXml!

        //Read muscle name
        std::string muscle_name;
        if(NULL == tiElem_muscle->Attribute("name")) //Unnamed muscle : Throw error
        { throw(std::runtime_error("Found an unnamed muscle. XML file is corrupt.")); }
        else
        { muscle_name = tiElem_muscle->Attribute("name"); }

        SMuscleParsed* mus = arg_msys.muscles_.create(muscle_name);
        if(S_NULL == mus)
        {
          std::string s; s = "Can't allocate a muscle on the pile : " + muscle_name;
          throw(std::runtime_error(s.c_str()));
        }
        mus->name_ = muscle_name;
        mus->muscle_type_ = _muscle_handle.ToElement()->Value();

        //Load the scalar properties
        if(NULL == _muscle_handle.FirstChild("max_isometric_force").ToElement())
        {
          std::string msg = "No max_isometric_force. At muscle : " + muscle_name;
          throw(std::runtime_error(msg.c_str()));
        }
        else
        {
          std::stringstream ss(tiElem_muscle->FirstChild("max_isometric_force")->Value());
          ss>>mus->max_isometric_force_;
        }

        if(NULL == _muscle_handle.FirstChild("optimal_fiber_length").ToElement())
        {
          std::string msg = "No optimal_fiber_length. At muscle : " + muscle_name;
          throw(std::runtime_error(msg.c_str()));
        }
        else
        {
          std::stringstream ss(tiElem_muscle->FirstChild("optimal_fiber_length")->Value());
          ss>>mus->optimal_fiber_length_;
        }

        if(NULL == _muscle_handle.FirstChild("tendon_slack_length").ToElement())
        {
          std::string msg = "No tendon_slack_length. At muscle : " + muscle_name;
          throw(std::runtime_error(msg.c_str()));
        }
        else
        {
          std::stringstream ss(tiElem_muscle->FirstChild("tendon_slack_length")->Value());
          ss>>mus->tendon_slack_length_;
        }

        if(NULL == _muscle_handle.FirstChild("pennation_angle").ToElement())
        {
          std::string msg = "No pennation_angle. At muscle : " + muscle_name;
          throw(std::runtime_error(msg.c_str()));
        }
        else
        {
          std::stringstream ss(tiElem_muscle->FirstChild("pennation_angle")->Value());
          ss>>mus->pennation_angle_;
        }

        //Iterate over all the muscle points.
        TiXmlElement* tiElem_muspt;
        sUInt pos_on_mus=0;
        tiElem_muspt = _muscle_handle.FirstChild("MusclePointSet").
            FirstChild("objects").FirstChild("MusclePoint").ToElement();
        for(; tiElem_muspt; tiElem_muspt=tiElem_muspt->NextSiblingElement() )
        {
          TiXmlHandle _muspt(tiElem_muspt);
          SMusclePointParsed tmp;

          std::string element_name = _muspt.ToElement()->Value();
          if(element_name == "MusclePoint" ||
              /**NOTE : MuscleViaPoint = Simbody::ConditionalPathPoint
               * A class implementing a conditional path point, which is a point that
               * is active only for a specified range of a coordinate.
               * We will NOT support turning this point off. */
              element_name == "MuscleViaPoint" ||
              /**NOTE : Simbody::MovingPathPoint
               * A class implementing a moving muscle point, which is a muscle point that
               * moves in a body's reference frame as a function of a coordinate.
               *
               * NOTE : This presently requires picking an average point on the parent link
               * instead of an actually moving point. */
              element_name == "MovingMusclePoint")
          {
            //Read location
            if(S_NULL == _muspt.FirstChild("location").ToElement())
            {
              std::string msg = "Found muscle point with no location. (Add avg value if it is a moving muscle point). At muscle : " + muscle_name;
              throw(std::runtime_error(msg.c_str()));
            }
            std::stringstream ss(_muspt.FirstChild("location").ToElement()->FirstChild()->Value());
            ss>>tmp.pos_in_parent_(0); ss>>tmp.pos_in_parent_(1); ss>>tmp.pos_in_parent_(2);

            if(S_NULL == _muspt.FirstChild("body").ToElement())
            {
              std::string msg = "Found muscle point with no parent link. At muscle : " + muscle_name;
              throw(std::runtime_error(msg.c_str()));
            }
            std::string ss2(_muspt.FirstChild("body").ToElement()->FirstChild()->Value());
            tmp.parent_link_ = ss2;
          }

          /**NOTE : Simbody::MovingPathPoint
           * A class implementing a moving muscle point, which is a muscle point that
           * moves in a body's reference frame as a function of a coordinate. */
          if(element_name == "MovingMusclePoint")
          {
          }

          tmp.position_on_muscle_ = pos_on_mus;
          pos_on_mus++;

          //Add the muscle point to the vector
          mus->points_.push_back(tmp);

#ifdef DEBUG
          std::cout<<"\n"<<model_name <<" ("<<muscle_name<<") Pt#"<<tmp.position_on_muscle_<<" Lnk: "<<tmp.parent_link_<<" Pos: "<<tmp.pos_in_parent_.transpose();
#endif
        }

        if(2>mus->points_.size())
        {
          std::string msg = "Found muscle with only one attachment point. At muscle : " + muscle_name;
          throw(std::runtime_error(msg.c_str()));
        }

      }

      return true;
    }
    catch(std::exception& e)
    { std::cerr<<"\nCParserOsimForOldFiles::readMuscleSysFromFile() : "<<e.what();  }
    return false;
  }

  /* Since the Osim format only has one robot in a file, the "arg_robot_name"
   * argument is not used. */
  bool CParserOsimForOldFiles::readRobotFromFile(
      const std::string& arg_file,
      const std::string& arg_robot_name,
      scl::SRobotParsed& arg_robot)
  {
    bool flag;
    try
    {
      //Set up the parser.
      TiXmlElement * tiElem_body;
      TiXmlHandle tiHndl_file_handle(NULL), tiHndl_model(NULL);
      TiXmlDocument tiDoc_file(arg_file.c_str());

      //Check if file opened properly
      flag = tiDoc_file.LoadFile(scl_tinyxml::TIXML_ENCODING_UNKNOWN);
      if(false == flag)
      { throw(std::runtime_error("Could not open xml file to read robot definition.")); }

      //Get handles to the tinyxml loaded ds
      tiHndl_file_handle = TiXmlHandle( &tiDoc_file );
      tiHndl_model = tiHndl_file_handle.FirstChild( "OpenSimDocument" ).FirstChild( "Model" ).ToElement();
      if(S_NULL == tiHndl_model.ToElement())
      {//Ridiculous versioning changes everywhere.
        tiHndl_model = tiHndl_file_handle.FirstChild( "Model" );
        if(S_NULL == tiHndl_model.ToElement())
        { throw(std::runtime_error("Could not find the Model or OpenSimDocument tags")); }
      }

      //Set the model's name. Must be unique.
      std::string model_name;
      if(NULL == tiHndl_model.ToElement()->Attribute("name")) //Unnamed model : Throw error
      { throw(std::runtime_error("Found an unnamed model. XML file is corrupt.")); }
      else
      { model_name = tiHndl_model.ToElement()->Attribute("name"); }

      //1. Read rigid body dynamics system(s)
      arg_robot.name_ = model_name;

      //Read in the bodies : Regardless of type.
      std::string body_name; // To ensure SimTK's two types work : BodySet and SimbodySet
      if(S_NULL != tiHndl_model.FirstChild("DynamicsEngine").FirstChild( "SimbodyEngine" ).
          FirstChild( "BodySet" ).FirstChild("objects").FirstChild("Body").ToElement())
      { body_name = "Body"; }
      //More file format changes..
      else if(S_NULL != tiHndl_model.FirstChild("DynamicsEngine").FirstChild( "SimbodyEngine" ).
          FirstChild( "BodySet" ).FirstChild("objects").FirstChild("SimbodyBody").ToElement())
      { body_name = "SimbodyBody"; }

      /*******************OSIM FILE FORMAT HACK*******************
       *  Read in the nodes with custom joints
       ***********************************************************/
      tiElem_body = tiHndl_model.FirstChild("DynamicsEngine").FirstChild( "SimbodyEngine" ).
          FirstChild( "BodySet" ).FirstChild("objects").FirstChild(body_name.c_str()).ToElement();
      //Iterating with TiXmlElement is faster than TiXmlHandle
      for(; tiElem_body; tiElem_body=tiElem_body->NextSiblingElement() )
      {
        TiXmlHandle _body_handle(tiElem_body); //Back to handles. Hail TiXml!
        flag = readBody(_body_handle,arg_robot,"CustomJoint");
        if(false == flag)
        { throw(std::runtime_error("Could not read bodies.")); }
      }

      /*******************OSIM FILE FORMAT HACK*******************
       *  Read in the nodes with weld joints, fuse them into their
       *  parent nodes.
       ***********************************************************/
      tiElem_body = tiHndl_model.FirstChild("DynamicsEngine").FirstChild( "SimbodyEngine" ).
          FirstChild( "BodySet" ).FirstChild("objects").FirstChild(body_name.c_str()).ToElement();
      scl::SRobotParsed tmp_welded_robot;
      //Iterating with TiXmlElement is faster than TiXmlHandle
      for(; tiElem_body; tiElem_body=tiElem_body->NextSiblingElement() )
      {
        TiXmlHandle _body_handle(tiElem_body); //Back to handles. Hail TiXml!
        flag = readBody(_body_handle,tmp_welded_robot,"WeldJoint");
        if(false == flag)
        { throw(std::runtime_error("Could not read a body with weld joint(s).")); }
      }

      /*******************OSIM FILE FORMAT HACK*******************
       *  Merge weld joints into custom joints.
       *
       * Weld joints only produce transformations for graphics objects and/or
       * masses. Redundant in scl.
       *
       * This function creates a separate tree for only the weld-joint links
       * And then it adds their graphics objects to the actual parent links
       * in the robot branching structure.
       *
       * If it finds a series of weld-joint links, it goes up the tree until
       * it finds a non-weld-joint parent-link.
       ***********************************************************/

      flag = tmp_welded_robot.rb_tree_.linkNodes();
      if(false == flag) { throw(std::runtime_error("Could not link weld joints"));  }

      sutil::CMappedTree<std::basic_string<char>, scl::SRigidBody>::iterator it, ite;
      for(it = tmp_welded_robot.rb_tree_.begin(), ite = tmp_welded_robot.rb_tree_.end(); it!=ite; ++it)
      {
        SRigidBody& lnk = *it;

        SRigidBody* par;
        if(S_NULL == lnk.parent_addr_)
        {//Orphan weld joint. Its parent must be a custom joint
          par = arg_robot.rb_tree_.at(lnk.parent_name_);
          if((S_NULL == par) && (false == lnk.is_root_))
          {
            std::string s;
            s = "No parent custom joint (" + lnk.parent_name_
                + ") for an orphan weld joint (" + lnk.name_ + ")";
            throw(std::runtime_error(s.c_str()));
          }
        }
        else
        { par = lnk.parent_addr_;  }

        std::vector<SRigidBodyGraphics>::iterator it,ite;
        for(it = lnk.graphics_obj_vec_.begin(), ite = lnk.graphics_obj_vec_.end();
            it!=ite;++it)
        {
          //Transform the joint to the parent's frame
          SRigidBodyGraphics tmp(*it);

          //1. Translate
          tmp.pos_in_parent_ += par->pos_in_parent_;

          //2. Rotate
          Eigen::Quaternion<sFloat> tmp_q((*it).ori_parent_quat_(3),
              (*it).ori_parent_quat_(0),(*it).ori_parent_quat_(1),
              (*it).ori_parent_quat_(2));
          Eigen::Quaternion<sFloat> tmp_q2;
          tmp_q2 = par->ori_parent_quat_.toRotationMatrix() *
              tmp_q.toRotationMatrix();
          tmp.ori_parent_quat_(0) = tmp_q2.x();
          tmp.ori_parent_quat_(1) = tmp_q2.y();
          tmp.ori_parent_quat_(2) = tmp_q2.z();
          tmp.ori_parent_quat_(3) = tmp_q2.w();

          //3. Insert translated/rotated element into its parent
          par->graphics_obj_vec_.push_back(tmp);

#ifdef DEBUG
          std::cout<<"\nPushed: "<<std::setw(15)<<lnk.name_
              <<", To: "<<std::setw(15)<<par->name_
              <<", GrObj: "<<std::setw(22)<<tmp.file_name_<<" ("
              <<(*it).pos_in_parent_.transpose()<<") to ("
              <<tmp.pos_in_parent_.transpose()<<") and ("
              <<(*it).ori_parent_quat_.x()
              <<" "<<(*it).ori_parent_quat_.y()
              <<" "<<(*it).ori_parent_quat_.z()
              <<" "<<(*it).ori_parent_quat_.w()<<") to ("
              <<tmp.ori_parent_quat_.x()
              <<" "<<tmp.ori_parent_quat_.y()
              <<" "<<tmp.ori_parent_quat_.z()
              <<" "<<tmp.ori_parent_quat_.w()<<")";
#endif
        }
      }

      /*******************OSIM FILE FORMAT HACK*******************
       *  Also set up orphan custom joint nodes to link to their
       *  parents through the weld joints.
       ***********************************************************/

      for(it = arg_robot.rb_tree_.begin(), ite = arg_robot.rb_tree_.end(); it!=ite; ++it)
      {
        SRigidBody& lnk = *it;

        if(true == lnk.is_root_)//Nothing needed here.
        { continue; }

        if(S_NULL == arg_robot.rb_tree_.at(lnk.parent_name_))
        {
          Eigen::Vector3d pos_in_parent = lnk.pos_in_parent_;
          Eigen::Quaternion<scl::sFloat> ori_in_parent = lnk.ori_parent_quat_;

          //Its parent isn't in the custom joint links. Must be in the weld joint links.
          SRigidBody* wpar = tmp_welded_robot.rb_tree_.at(lnk.parent_name_);
          if(S_NULL == wpar)
          {
            std::string s; s = "No parent weld link (" + lnk.parent_name_
            + ") for an orphan custom-weld-custom joint (" + lnk.name_ + ").";
            throw(std::runtime_error(s.c_str()));
          }

          while(S_NULL!=wpar->parent_addr_)
          {
            pos_in_parent += wpar->pos_in_parent_;
            ori_in_parent = wpar->ori_parent_quat_.toRotationMatrix() * ori_in_parent.toRotationMatrix();
            wpar = wpar->parent_addr_;
          }//Reach the weld joint connected to a custom joint.

          if(true == wpar->is_root_)
          { throw(std::runtime_error("Found a path to ground from a weld joint node. Invalid state"));  }

          SRigidBody* cpar;//Find the custom joint parent, and link the custom joint to it.
          cpar = arg_robot.rb_tree_.at(wpar->parent_name_);
          if(S_NULL == cpar)
          {
            std::string s; s = "No parent custom joint (" + wpar->parent_name_
                + ") for an orphan custom-weld-custom joint (" + lnk.name_ + ").";
            throw(std::runtime_error(s.c_str()));
          }

          pos_in_parent += cpar->pos_in_parent_;
          ori_in_parent = cpar->ori_parent_quat_.toRotationMatrix() * ori_in_parent.toRotationMatrix();

#ifdef DEBUG
          std::cout<<"\nCust Pushed: "<<std::setw(15)<<lnk.name_
              <<", To: "<<std::setw(15)<<cpar->name_<<" ("
              <<lnk.pos_in_parent_.transpose()<<") to ("
              <<pos_in_parent.transpose()<<") and ("
              <<lnk.ori_parent_quat_.x()
              <<" "<<lnk.ori_parent_quat_.y()
              <<" "<<lnk.ori_parent_quat_.z()
              <<" "<<lnk.ori_parent_quat_.w()<<") to ("
              <<ori_in_parent.x()
              <<" "<<ori_in_parent.y()
              <<" "<<ori_in_parent.z()
              <<" "<<ori_in_parent.w()<<")";
#endif

          lnk.parent_name_ = cpar->name_;
          lnk.pos_in_parent_ = pos_in_parent;
          lnk.ori_parent_quat_ = ori_in_parent;
        }
      }

      return true;
    }
    catch(std::exception& e)
    { std::cerr<<"\nCParserOsimForOldFiles::readRobotFromFile("<<arg_file<<") : "<<e.what();  }
    return false;
  }


  /** Osim link specification reader:
   * In case a body has multiple coordinates, it creates a series
   * of joints/links for all the coordinates. */
  bool CParserOsimForOldFiles::readBody(
      const scl_tinyxml::TiXmlHandle& arg_tiHndl_body,
      scl::SRobotParsed& arg_robot,
      const std::string& arg_joint_type)
  {
    bool flag;
    std::string body_name("Not set yet");
    try
    {
      static sUInt link_id=0;

      //Read link name
      if(NULL == arg_tiHndl_body.ToElement()->Attribute("name")) //Unnamed link : Throw error
      { throw(std::runtime_error("Found an unnamed link. XML file is corrupt.")); }
      else
      { body_name = arg_tiHndl_body.ToElement()->Attribute("name"); }

      if(body_name == "ground")
      {
        //Root link is always ground in osim
        SRigidBody* lnk = arg_robot.rb_tree_.create(body_name,true);
        if(S_NULL == lnk)
        { throw(std::runtime_error("Can't allocate a link on the pile"));  }
        lnk->name_ = "ground";
        lnk->is_root_ = true;
        return true;
      }

      if(S_NULL == arg_tiHndl_body.FirstChild("Joint").FirstChild(arg_joint_type.c_str()).ToElement())
      { return true;  } //Separately parse weld and custom joints.

      SRigidBody* lnk = S_NULL;

      SOsimJoint j;
      flag = readJoint(arg_tiHndl_body.FirstChild("Joint").FirstChild(arg_joint_type.c_str()),j);
      if(false == flag)
      { throw(std::runtime_error("Could not read joint"));  }

      /*******************OSIM FILE FORMAT HACK*******************
       *                   Ignore weld joints
       ***********************************************************/
      if("WeldJoint" == arg_joint_type)
      {//Is not a joint. So don't parse the joint stuff except the parent name and pos
        lnk = arg_robot.rb_tree_.create(body_name,false);
        if(S_NULL == lnk)
        {
          std::string s; s= "Can't allocate a link on the pile : " + body_name;
          throw(std::runtime_error(s.c_str()));
        }
        lnk->name_ = body_name;
        lnk->parent_name_ = j.parent_name_;
        lnk->pos_in_parent_ = j.pos_in_parent_;
        lnk->ori_parent_quat_ = j.ori_in_parent_;
      }
      /*******************OSIM FILE FORMAT HACK*******************
       *  Read in the joint coordinates and after that create a series of
       *  links for them. For CustomJoints with multiple dofs only.
       ***********************************************************/
      else if("CustomJoint" == arg_joint_type)
      {//Parse custom joints using the transform axes.
        sutil::CMappedList<std::basic_string<char>,
        scl_parser::CParserOsimForOldFiles::SOsimJoint::SOsimTransformAxis>::iterator it, ite;

        for(it = j.trf_axes_.begin(), ite = j.trf_axes_.end(); it!=ite; ++it)
        {
          SOsimJoint::SOsimTransformAxis& tr = *it;

          SOsimJoint::SOsimCoordinate *cr = j.coordinates_.at(tr.coord_name_);
          if(S_NULL==cr){ throw(std::runtime_error("Transform axis has invalid coordinate"));  }

          std::string parent; Eigen::Vector3d pos; Eigen::Quaternion<scl::sFloat> ori;
          if(S_NULL==lnk)
          { //Set the first link's parent to the actual parent
            parent = j.parent_name_;
            pos = j.pos_in_parent_;
            ori = j.ori_in_parent_;
          }
          else
          {//Each successive link is a child of the previous one.
            parent = lnk->name_;
            pos = Eigen::Vector3d::Zero();
            // NOTE TODO : How to set ori from the axes?
          }

          std::string lnk_name;
          sutil::CMappedList<std::basic_string<char>,
          scl_parser::CParserOsimForOldFiles::SOsimJoint::SOsimTransformAxis>::iterator it2(it);
          ++it2;
          if(ite == it2)
          {//If this is the last link, name it to the actual body
            lnk_name = body_name;
          }
          else
          {//Else name it to the coordinate name
            lnk_name = cr->coord_name_;
          }

          //Allocate the link on the pile
          lnk = arg_robot.rb_tree_.create(lnk_name,false);
          if(S_NULL == lnk)
          {
            std::string s;
            s= "Can't allocate a link on the pile : " + lnk_name;
            throw(std::runtime_error(s.c_str()));
          }

          //Set the global link ids : NOTE TODO : Implement this with a singleton
          lnk->link_id_ = link_id; link_id++;
          //Set up the scl link
          lnk->name_ = lnk_name;
          lnk->joint_name_ = parent + "_to_" + lnk->name_;
          lnk->robot_name_ = arg_robot.name_;
          lnk->parent_name_ = parent;
          lnk->pos_in_parent_ = pos;
          lnk->ori_parent_quat_ = ori;
          lnk->joint_limit_lower_ = cr->min_;
          lnk->joint_limit_upper_ = cr->max_;
          lnk->joint_default_pos_ = cr->default_pos_;
          lnk->mass_ = 1.0;
          lnk->inertia_ = Eigen::Matrix3d::Identity();

          //Set the joint type
          std::string jtype;
          if(tr.axis_(0)==1 && tr.axis_(1)==0 && tr.axis_(2)==0 && false==tr.is_rot_) { jtype = "px"; }
          else if(tr.axis_(0)==1 && tr.axis_(1)==0 && tr.axis_(2)==0 && true==tr.is_rot_) { jtype = "rx"; }
          else if(tr.axis_(0)==0 && tr.axis_(1)==1 && tr.axis_(2)==0 && false==tr.is_rot_) { jtype = "py"; }
          else if(tr.axis_(0)==0 && tr.axis_(1)==1 && tr.axis_(2)==0 && true==tr.is_rot_) { jtype = "ry"; }
          else if(tr.axis_(0)==0 && tr.axis_(1)==0 && tr.axis_(2)==1 && false==tr.is_rot_) { jtype = "pz"; }
          else if(tr.axis_(0)==0 && tr.axis_(1)==0 && tr.axis_(2)==1 && true==tr.is_rot_) { jtype = "rz"; }
          else { jtype = "rz"; }

          if(jtype == "px"){ lnk->joint_type_ = JOINT_TYPE_PRISMATIC_X;  }
          else if(jtype == "py"){ lnk->joint_type_ = JOINT_TYPE_PRISMATIC_Y;  }
          else if(jtype == "pz"){ lnk->joint_type_ = JOINT_TYPE_PRISMATIC_Z;  }
          else if(jtype == "rx"){ lnk->joint_type_ = JOINT_TYPE_REVOLUTE_X;  }
          else if(jtype == "ry"){ lnk->joint_type_ = JOINT_TYPE_REVOLUTE_Y;  }
          else if(jtype == "rz"){ lnk->joint_type_ = JOINT_TYPE_REVOLUTE_Z;  }
          else if(jtype == "sp"){ lnk->joint_type_ = JOINT_TYPE_SPHERICAL;  }
          else if(jtype == "sl"){ lnk->joint_type_ = JOINT_TYPE_SPLINE;  }
          else {throw(std::runtime_error("Error reading joint type"));}

#ifdef DEBUG
          std::cout<<"\nLnk "<<lnk->link_id_<<": "<<std::setw(20)<<lnk->name_
              <<", Jnt: "<<std::setw(40)<<lnk->joint_name_<<" ("<<lnk->joint_type_<<")"
              <<", Par: "<<std::setw(20)<<lnk->parent_name_<<" ("<<lnk->pos_in_parent_.transpose()
              <<", "<<lnk->ori_parent_quat_.x()
              <<" "<<lnk->ori_parent_quat_.y()
              <<" "<<lnk->ori_parent_quat_.z()
              <<" "<<lnk->ori_parent_quat_.w()<<")"
              <<", Com: "<<lnk->com_.transpose()
              <<", Mass: "<<lnk->mass_<<",\n Inertia: "<<lnk->inertia_<<", \nGr:";
          for(scl::sUInt i=0;i<lnk->graphics_obj_vec_.size();++i)
          { std::cout<<" "<<lnk->graphics_obj_vec_[i].file_name_; }
#endif
        }
      }

      /*******************OSIM FILE FORMAT HACK*******************
       *  At the last link, load in other stuff from the file....
       ***********************************************************/
      //Load the scalar properties
      if(NULL == arg_tiHndl_body.FirstChild("mass").ToElement())
      {
        std::string msg = "No mass. At body : " + body_name;
        throw(std::runtime_error(msg.c_str()));
      }
      else
      {
        std::stringstream ss(arg_tiHndl_body.FirstChild("mass").ToElement()->FirstChild()->Value());
        ss>>lnk->mass_;
        if(lnk->mass_ < 0.0001)
        { lnk->mass_ = 1.0; }//No massless links allowed
      }

      if(NULL == arg_tiHndl_body.FirstChild("mass_center").ToElement())
      {
        std::string msg = "No mass_center. At body : " + body_name;
        throw(std::runtime_error(msg.c_str()));
      }
      else
      {
        std::stringstream ss(arg_tiHndl_body.FirstChild("mass_center").ToElement()->FirstChild()->Value());
        ss>>lnk->com_(0); ss>>lnk->com_(1); ss>>lnk->com_(2);
      }

      if(NULL != arg_tiHndl_body.FirstChild("inertia").ToElement())
      {//Only one tag : Read in the entire inertia matrix.
        std::stringstream ss(arg_tiHndl_body.FirstChild("inertia").ToElement()->FirstChild()->Value());
        ss>>lnk->inertia_(0,0); ss>>lnk->inertia_(0,1); ss>>lnk->inertia_(0,2);
        ss>>lnk->inertia_(1,0); ss>>lnk->inertia_(1,1); ss>>lnk->inertia_(1,2);
        ss>>lnk->inertia_(2,0); ss>>lnk->inertia_(2,1); ss>>lnk->inertia_(2,2);
      }
      else
      {//Individual tags : More versioning
        lnk->inertia_ = Eigen::Matrix3d::Identity();//Reset the inertia and then read in the diagonal elements.
        if(NULL == arg_tiHndl_body.FirstChild("inertia_xx").ToElement())
        {
          std::string msg = "No inertia_xx. At body : " + body_name;
          throw(std::runtime_error(msg.c_str()));
        }
        else
        {
          std::stringstream ss(arg_tiHndl_body.FirstChild("inertia_xx").ToElement()->FirstChild()->Value());
          ss>>lnk->inertia_(0,0);
        }

        if(NULL == arg_tiHndl_body.FirstChild("inertia_yy").ToElement())
        {
          std::string msg = "No inertia_yy. At body : " + body_name;
          throw(std::runtime_error(msg.c_str()));
        }
        else
        {
          std::stringstream ss(arg_tiHndl_body.FirstChild("inertia_yy").ToElement()->FirstChild()->Value());
          ss>>lnk->inertia_(1,1);
        }

        if(NULL == arg_tiHndl_body.FirstChild("inertia_zz").ToElement())
        {
          std::string msg = "No inertia_zz. At body : " + body_name;
          throw(std::runtime_error(msg.c_str()));
        }
        else
        {
          std::stringstream ss(arg_tiHndl_body.FirstChild("inertia_zz").ToElement()->FirstChild()->Value());
          ss>>lnk->inertia_(2,2);
        }
      }

      if(NULL != arg_tiHndl_body.FirstChild("VisibleObject").ToElement())
      {//Check if there are any graphics files
        Eigen::Vector3d scale_fac; //Temporarily store the scale factors and use them to init the viz objects.
        if(NULL == arg_tiHndl_body.FirstChild("VisibleObject").FirstChild("scale_factors").ToElement())
        {
          std::cout<<"\nCParserOsimForOldFiles::readBody() : WARNING : No scale_factors. At body : " << body_name<<". Setting to 1,1,1.";
          scale_fac(0) = 1.0; scale_fac(1) = 1.0; scale_fac(2) = 1.0;
        }
        else
        {
          std::stringstream ss(arg_tiHndl_body.FirstChild("VisibleObject").FirstChild("scale_factors").ToElement()->FirstChild()->Value());
          ss>>scale_fac(0); ss>>scale_fac(1); ss>>scale_fac(2);
        }

        //Initialize the viz objects. Append a .obj to the .vtp because we will convert to obj (scl's format).
        if(NULL == arg_tiHndl_body.FirstChild("VisibleObject").FirstChild("geometry_files").ToElement())
        {
          std::string msg = "No geometry_files. At body : " + body_name;
          throw(std::runtime_error(msg.c_str()));
        }
        else
        {
          if(S_NULL!=arg_tiHndl_body.FirstChild("VisibleObject").FirstChild("geometry_files").FirstChild().Text())
          {//If there are any geometry files, use them
            std::stringstream ss(arg_tiHndl_body.FirstChild("VisibleObject").FirstChild("geometry_files").ToElement()->FirstChild()->Value());
            //Create all the graphics objects
            std::string sbuf;
            while (ss >> sbuf) //Reads stream till whitespace
            {
              if ((sbuf != "") && (sbuf != "NULL"))
              {
                SRigidBodyGraphics tmp_gr;
                tmp_gr.scaling_ = scale_fac;
                tmp_gr.file_name_ = "CHANGE_ME_TO_THE_GRAPHICS_DIR_"+sbuf+".obj";
                lnk->graphics_obj_vec_.push_back(tmp_gr);
              }
            }
          }
        }
      }
#ifdef DEBUG
      std::cout<<"\nLnk "<<lnk->link_id_<<": "<<std::setw(20)<<lnk->name_
          <<", Jnt: "<<std::setw(40)<<lnk->joint_name_<<" ("<<lnk->joint_type_<<")"
          <<", Par: "<<std::setw(20)<<lnk->parent_name_<<" ("<<lnk->pos_in_parent_.transpose()
          <<", "<<lnk->ori_parent_quat_.x()
          <<" "<<lnk->ori_parent_quat_.y()
          <<" "<<lnk->ori_parent_quat_.z()
          <<" "<<lnk->ori_parent_quat_.w()<<")"
          <<", Com: "<<lnk->com_.transpose()
          <<", Mass: "<<lnk->mass_<<",\n Inertia: "<<lnk->inertia_<<", \nGr:";
      for(scl::sUInt i=0;i<lnk->graphics_obj_vec_.size();++i)
      { std::cout<<" "<<lnk->graphics_obj_vec_[i].file_name_; }
#endif

      return true;
    }
    catch(std::exception& e)
    { std::cerr<<"\nCParserOsimForOldFiles::readBody("<<body_name<<") : "<<e.what();  }
    return false;
  }

  /** Reads a joint for a body.
   * Will be converted into none/multiple joints/links for scl.
   *
   * CustomJoint: Osim has multiple dofs per joint. These will be split
   * into multiple joints.
   *
   * WeldJoint: Osim uses 0dof weld joints. These will be eliminated.
   */
  bool CParserOsimForOldFiles::readJoint(
      const scl_tinyxml::TiXmlHandle& arg_tiHndl_jnt,
      SOsimJoint& arg_joint)
  {
    try
    {
      //Read the usual joint stuff name
      if(NULL == arg_tiHndl_jnt.ToElement()->Attribute("name")) //Unnamed link : Throw error
      { throw(std::runtime_error("Found an unnamed link. XML file is corrupt.")); }
      else
      { arg_joint.name_ = arg_tiHndl_jnt.ToElement()->Attribute("name"); }

      if(NULL == arg_tiHndl_jnt.FirstChild("parent_body").ToElement())
      {
        std::string s; s= "Found no parent at joint : " + arg_joint.name_;
        throw(std::runtime_error(s.c_str()));
      }
      else
      {
        std::stringstream ss(arg_tiHndl_jnt.FirstChild("parent_body").ToElement()->FirstChild()->Value());
        ss>>arg_joint.parent_name_;
      }

      if(NULL == arg_tiHndl_jnt.FirstChild("location_in_parent").ToElement())
      {
        std::string s;
        s= "Found no location_in_parent at joint : " + arg_joint.name_;
        throw(std::runtime_error(s.c_str()));
      }
      else
      {
        std::stringstream ss(arg_tiHndl_jnt.FirstChild("location_in_parent").ToElement()->FirstChild()->Value());
        ss>>arg_joint.pos_in_parent_(0); ss>>arg_joint.pos_in_parent_(1); ss>>arg_joint.pos_in_parent_(2);
      }

      if(NULL == arg_tiHndl_jnt.FirstChild("orientation_in_parent").ToElement())
      {
        std::string s;
        s= "Found no orientation_in_parent at joint : " + arg_joint.name_;
        throw(std::runtime_error(s.c_str()));
      }
      else
      {
        std::stringstream ss(arg_tiHndl_jnt.FirstChild("orientation_in_parent").ToElement()->FirstChild()->Value());
        Eigen::Vector3d tvec;
        ss>>tvec(0); ss>>tvec(1); ss>>tvec(2);
        Eigen::Matrix3d tm;
        tm = Eigen::AngleAxisd(tvec(0),Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(tvec(1),Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(tvec(2),Eigen::Vector3d::UnitZ());
        arg_joint.ori_in_parent_ = Eigen::Quaternion<scl::sFloat>(tm);
      }

      //Read in a coordinate set
      scl_tinyxml::TiXmlElement* tiElem_Coord;
      tiElem_Coord = arg_tiHndl_jnt.FirstChild("CoordinateSet").FirstChild("objects").FirstChild("Coordinate").ToElement();
      for(; tiElem_Coord; tiElem_Coord=tiElem_Coord->NextSiblingElement() )
      {
        scl_tinyxml::TiXmlHandle tiHndl_coord(tiElem_Coord);

        std::string coord_name;
        if(NULL == tiHndl_coord.ToElement()->Attribute("name")) //Unnamed link : Throw error
        { throw(std::runtime_error("Found an unnamed coordinate.")); }
        else
        { coord_name = tiHndl_coord.ToElement()->Attribute("name"); }

        SOsimJoint::SOsimCoordinate *tcoord = arg_joint.coordinates_.create(coord_name);
        if(S_NULL == tcoord) { throw(std::runtime_error("Could not allocate coordinate")); }

        //Set up a coordinate
        tcoord->coord_name_ = coord_name;

        if(NULL == tiHndl_coord.FirstChild("default_value").ToElement())
        { throw(std::runtime_error("No default_value found")); }
        else
        {
          std::stringstream ss(tiHndl_coord.FirstChild("default_value").ToElement()->FirstChild()->Value());
          ss>>tcoord->default_pos_;
        }

        if(NULL == tiHndl_coord.FirstChild("range").ToElement())
        { throw(std::runtime_error("No range found")); }
        else
        {
          std::stringstream ss(tiHndl_coord.FirstChild("range").ToElement()->FirstChild()->Value());
          ss>>tcoord->min_; ss>>tcoord->max_;
        }
      }

      //Read in a transform axis set
      scl_tinyxml::TiXmlElement* tiElem_Trax;
      tiElem_Trax = arg_tiHndl_jnt.FirstChild("TransformAxisSet").FirstChild("objects").FirstChild("TransformAxis").ToElement();
      for(; tiElem_Trax; tiElem_Trax=tiElem_Trax->NextSiblingElement() )
      {
        scl_tinyxml::TiXmlHandle tiHndl_trax(tiElem_Trax);

        std::string trax_name;
        if(NULL == tiHndl_trax.ToElement()->Attribute("name")) //Unnamed link : Throw error
        { throw(std::runtime_error("Found an unnamed coordinate.")); }
        else
        { trax_name = tiHndl_trax.ToElement()->Attribute("name"); }

        SOsimJoint::SOsimTransformAxis *ttrax = arg_joint.trf_axes_.create(trax_name);
        if(S_NULL == ttrax) { throw(std::runtime_error("Could not allocate transform axis")); }

        //Set up a transform axis
        ttrax->name_ = trax_name;

        if(NULL == tiHndl_trax.FirstChild("coordinate").ToElement())
        { throw(std::runtime_error("No coordinate found")); }
        else
        {
          std::stringstream ss(tiHndl_trax.FirstChild("coordinate").ToElement()->FirstChild()->Value());
          ss>>ttrax->coord_name_;
        }

        if(NULL == tiHndl_trax.FirstChild("axis").ToElement())
        { throw(std::runtime_error("No axis found")); }
        else
        {
          std::stringstream ss(tiHndl_trax.FirstChild("axis").ToElement()->FirstChild()->Value());
          ss>>ttrax->axis_(0); ss>>ttrax->axis_(1); ss>>ttrax->axis_(2);
        }

        if(NULL == tiHndl_trax.FirstChild("is_rotation").ToElement())
        { throw(std::runtime_error("No is_rotation found")); }
        else
        {
          std::string s(tiHndl_trax.FirstChild("is_rotation").ToElement()->FirstChild()->Value());
          s == "true" ? ttrax->is_rot_ = true : ttrax->is_rot_ = false;
        }
      }

      return true;
    }
    catch(std::exception& e)
    { std::cerr<<"\nCParserOsimForOldFiles::readJoint() : "<<e.what();  }
    return false;
  }

}
