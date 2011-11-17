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
/* \file CChaiGraphics.cpp
 *
 *  Created on: Aug 26, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CChaiGraphics.hpp"

#include <scl/Singletons.hpp>

#include <string>
#include <vector>
#include <stdexcept>

#include "chai3d.h"

#include "GL/freeglut.h"

namespace scl {

  const sFloat CHAI_SPHERE_RENDER_RADIUS = 0.0001; //m
  const sFloat CHAI_SPHERE_MUSC_VIA_RADIUS = 0.004; //m
  const sFloat CHAI_MUSC_THICKNESS = 3; //pixels

  sBool CChaiGraphics::initGraphics(
      const std::string & arg_graphics_name)
  {
    SDatabase* db=S_NULL;
    try
    {
      //Get a handle to the database
      db = CDatabase::getData();
      if(S_NULL == db)
      { throw(std::runtime_error("Database not initialized"));  }

      //Create a graphics instance (might have multiple graphics instances
      //for rendering different scenegraphs)
      if(S_NULL!=db->s_gui_.chai_data_.at(arg_graphics_name))
      {
        std::string err;
        err = "A graphics instance already exists with name="+arg_graphics_name;
        throw(std::runtime_error(err));
      }

      //Allocate a new dynamic graphics data structure on the pile.
      data_ = db->s_gui_.chai_data_.create(arg_graphics_name);
      if(S_NULL==data_)
      { throw(std::runtime_error("Couldn't create chai graphics data structure on the pile")); }

      //Name it
      data_->name_ = arg_graphics_name;

      //Create a new world : Don't worry about deallocating it.
      data_->chai_world_ = new cWorld();
      if(S_NULL == data_->chai_world_)
      { throw(std::runtime_error("Couldn't initialize chai world"));  }

      //Create a camera
      data_->chai_cam_ = new cCamera(data_->chai_world_);
      if(S_NULL == data_->chai_cam_)
      { throw(std::runtime_error("Couldn't create a chai camera")); }

      //Insert the camera into the scenegraph world
      data_->chai_world_->addChild(data_->chai_cam_);

      //Get the configuration data for the graphics from the database
      data_parsed_ = db->s_parser_.graphics_worlds_.at(arg_graphics_name);
      if(S_NULL==data_parsed_)
      {
        std::cout<<"\nCChaiGraphics::initGraphics() : Warning. Couldn't find the graphics' settings in the database. Setting to defaults.";

        //Set the background color (R,G,B) to black.
        data_->chai_world_->setBackgroundColor(0.0, 0.0, 0.0);

        data_->chai_cam_->set(cVector3d (5.0, 1.0, 1.0),    // camera position (eye)
            cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
            cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

        //Set the near and far clipping planes of the camera
        //Anything in front/behind these clipping planes will not be rendered
        data_->chai_cam_->setClippingPlanes(0.01, 10);

        // create a light source and attach it to the camera
        cLight* light = new cLight(data_->chai_world_);
        if(S_NULL==light)
        { throw(std::runtime_error("Couldn't add a light to the world")); }

        data_->chai_cam_->addChild(light);                   // attach light to camera
        light->setEnabled(true);                   // enable light source
        light->setPos(cVector3d( 2.0, 0.5, 1.0));  // position the light source
        light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam
      }
      else
      {//NOTE TODO: Populate from the database.
        //Set the background color (R,G,B) to black.
        data_->chai_world_->setBackgroundColor(0.0, 0.0, 0.0);

        cVector3d tmp1(data_parsed_->cam_pos_[0], data_parsed_->cam_pos_[1], data_parsed_->cam_pos_[2]);
        cVector3d tmp2(data_parsed_->cam_lookat_[0], data_parsed_->cam_lookat_[1], data_parsed_->cam_lookat_[2]);
        cVector3d tmp3(data_parsed_->cam_up_[0], data_parsed_->cam_up_[1], data_parsed_->cam_up_[2]);

#ifdef W_TESTING
        std::cout<<"\nCam Pos:"<<data_parsed_->cam_pos_[0]
                                                        <<" "<<data_parsed_->cam_pos_[1]<<" "<<data_parsed_->cam_pos_[2]<<std::flush;
        std::cout<<"\nCam Lookat:"<<data_parsed_->cam_lookat_[0]
                                                              <<" "<<data_parsed_->cam_lookat_[1]<<" "<< data_parsed_->cam_lookat_[2];
        std::cout<<"\nCam Up:"<<data_parsed_->cam_up_[0]
                                                      <<" "<<data_parsed_->cam_up_[1]<<" "<< data_parsed_->cam_up_[2];
        std::cout<<std::endl;
#endif

        data_->chai_cam_->set(tmp1,    // camera position (eye)
            tmp2,    // lookat position (target)
            tmp3);   // direction of the "up" vector

        //Set the near and far clipping planes of the camera
        //Anything in front/behind these clipping planes will not be rendered
        data_->chai_cam_->setClippingPlanes(
            data_parsed_->cam_clipping_dist_[0], data_parsed_->cam_clipping_dist_[1]);

        // create a light source and attach it to the camera
        std::vector<SGraphicsParsedData::SLight>::iterator it,ite;
        ite = data_parsed_->lights_.end();
        for(it = data_parsed_->lights_.begin();
            it!=ite; ++it)
        {
          cLight* light = new cLight(data_->chai_world_);
          if(S_NULL==light)
          { throw(std::runtime_error("Couldn't add a light to the world")); }

          data_->chai_cam_->addChild(light);                   // attach light to camera
          light->setEnabled(true);                   // enable light source

          cVector3d pos( it->pos_[0], it->pos_[1], it->pos_[2]);
          cVector3d lookat( it->lookat_[0], it->lookat_[1], it->lookat_[2]);
          cVector3d dir = lookat-pos;

#ifdef W_TESTING
          std::cout<<"\nLight Pos:"<<pos[0]<<" "<<pos[1]<<" "<<pos[2];
          std::cout<<"\nLight Lookat:"<<lookat[0]<<" "<<lookat[1]<<" "<<lookat[2];
          std::cout<<"\nLight Dir:"<<dir[0]<<" "<<dir[1]<<" "<<dir[2]<<std::flush;
          std::cout<<std::endl;
#endif

          light->setPos(pos);  // position the light source
          light->setDir(dir);  // define the direction of the light beam
        }
      }
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::initGraphics() : "<<ee.what();
      if(S_NULL!= data_->chai_cam_)
      {
        delete data_->chai_cam_;
        data_->chai_cam_ = S_NULL;
      }
      if(S_NULL!= data_->chai_world_)
      {
        delete data_->chai_world_;
        data_->chai_world_ = S_NULL;
      }
      if(S_NULL!=data_)
      {
        if(S_NULL!=db)
        {
          db->s_gui_.chai_data_.erase(data_);
          data_ = S_NULL;
        }
      }
      return false;
    }
    has_been_init_ = true;
    return true;
  }

  sBool CChaiGraphics::destroyGraphics()
  {
    try
    {
      //Get a handle to the database
      SDatabase* db = CDatabase::getData();
      if(S_NULL == db)
      { throw(std::runtime_error("Database not initialized"));  }

      if(S_NULL!=data_) // Data is created on the pile.
      {
        if(S_NULL!= data_->chai_world_)
        {
          delete data_->chai_world_;
          data_->chai_world_ = S_NULL;
        }
        else
        { throw(std::runtime_error("Invalid graphics state. Graphics data allocated but chai pointer is null."));  }

        db->s_gui_.chai_data_.erase(data_);
        data_ = S_NULL;
      }
      has_been_init_ = false;
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::destroyGraphics() : "<<ee.what();
      return false;
    }
    return true;
  }

  sBool CChaiGraphics::addRobotToRender(const std::string& arg_robot)
  {
    const SRobotLink * tmp_root_link = S_NULL;
    SGraphicsPhysicalLink* robot_brrep_root = S_NULL;
    try
    {
      if(!has_been_init_) { return false; }

      //1. Get the robot
      SDatabase* db = CDatabase::getData();
      if(S_NULL == db)
      { throw(std::runtime_error("Database not initialized"));  }

      SRobotParsedData * robdef; //A pointer to the robot's static config.
      robdef = db->s_parser_.robots_.at(arg_robot);

      if(arg_robot != robdef->name_)
      {
        throw(std::runtime_error(
            "The database is inconsistent. An entry's stored robot name doesn't match its (string) index name on the pile."
        ));
      }

      //2. Now obtain the robot root link's static (parser) data.
      tmp_root_link = robdef->robot_br_rep_.getRootNode();
      if(S_NULL == tmp_root_link)
      { throw(std::runtime_error("Found a robot without any links"));  }
      if(tmp_root_link->robot_name_ != arg_robot)
      { throw(std::runtime_error("Inconsistency detected : Root link of the robot says that it belongs to a different robot."));  }

      //3. Create a new robot tree on the graphics robot pile
      //(Ie. Create a (physics+graphics) branching-representation)
      sutil::CMappedTree<std::string, SGraphicsPhysicalLink>* rob_gr_brrep;
      rob_gr_brrep = data_->robots_rendered_.create(arg_robot);
      if(S_NULL==rob_gr_brrep)
      { throw(std::runtime_error("Couldn't create a (physics+graphics) representation for the robot on the pile"));  }

      //4. Create a new root node on the graphics robot pile
      robot_brrep_root = rob_gr_brrep->create(tmp_root_link->name_,true); //Special case : create root node
      if(S_NULL==robot_brrep_root)
      { throw(std::runtime_error("Couldn't create a ground root-node for the robot's graphics branching representation"));  }

      //5. Set its initialized graphics object and robot link (pointer to the parsed data for this link)
      robot_brrep_root->graphics_obj_ = S_NULL;
      robot_brrep_root->robot_link_ = tmp_root_link;
      robot_brrep_root->name_ = tmp_root_link->name_; //Have to set the name of this object

      //6. Set the graphics branching representation subtree (sets up this node's graphics and recurses into its children).
      if(false == addRobotLink(robot_brrep_root))
      { throw(std::runtime_error("Failed to add child robot-link"));  }

      //7. Set random rendering options (Defaults. Override later if you want.)
      robot_brrep_root->graphics_obj_->setShowFrame(false, false);
      robot_brrep_root->graphics_obj_->setFrameSize(0.15,0.4,true);
      robot_brrep_root->graphics_obj_->setShowTree(false,false);

      //7. Check for errors.
      if(S_NULL==robot_brrep_root->graphics_obj_)
      { throw(std::runtime_error("Failed to create chai graphics tree for robot"));  }

      //8. Add the constructed robot tree to the graphics (chai) world
      data_->chai_world_->addChild(robot_brrep_root->graphics_obj_);

#ifdef W_TESTING
      // Print some info
      printf("\nAdded a robot <%s> to graphics specification <%s>",
          robdef->name_.c_str(), data_->name_.c_str());
#endif
    }
    catch(std::exception& ee)
    {
      if(S_NULL!=robot_brrep_root->graphics_obj_)
      {
        //Destructor also deletes any children. So no worries about the child vector.
        delete robot_brrep_root->graphics_obj_;
        robot_brrep_root->graphics_obj_ = S_NULL;
      }
      std::cerr<<"\nCChaiGraphics::addRobotToRender() : "<<ee.what();
      return false;
    }
    return true;
  }

  /** Removes a robot's meshes from the graphics rendering environment.
   *
   * A robot is defined as:
   * 1. Anything whose dynamics are integrated by the physics simulator
   * 2. Any real world entity subject to the laws of physics */
  sBool CChaiGraphics::removeRobotFromRender(const std::string& arg_robot)
  {
    bool flag;
    try
    {
      if(!has_been_init_) { return false; }

      //1. Get the robot tree on the graphics robot pile
      sutil::CMappedTree<std::string, SGraphicsPhysicalLink>* rob_gr_brrep;
      rob_gr_brrep = data_->robots_rendered_.at(arg_robot);
      if(S_NULL==rob_gr_brrep)
      { throw(std::runtime_error("Couldn't find a graphics representation for the robot"));  }

      //2. Remove it from the chai graphics.
      SGraphicsPhysicalLink* gr_root = rob_gr_brrep->getRootNode();
      if(S_NULL == gr_root)
      { throw(std::runtime_error("Robot's graphics rendering root is NULL. Invalid data state, chai data might be corrupted."));  }

      flag = gr_root->graphics_obj_->removeFromGraph();
      if(false == flag)
      { throw(std::runtime_error("Could not remove robot's graphics from the chai graphics world. Invalid data state, chai data might be corrupted."));  }

      flag = data_->robots_rendered_.erase(arg_robot);
      if(false == flag)
      { throw(std::runtime_error("Could not remove robot's graphics from the mapped tree. Invalid data state, scl graphics data might be corrupted."));  }

#ifdef W_TESTING
      // Print some info
      printf("\nRemoved a robot <%s> from graphics specification <%s>",
          arg_robot.c_str(), data_->name_.c_str());
#endif
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::removeRobotFromRender() : "<<ee.what();
      return false;
    }
    return true;
  }

  sBool CChaiGraphics::addRobotLink(SGraphicsPhysicalLink* arg_link)
  {
    try
    {
      if(!has_been_init_) { return false; }

      //0.a. Check whether the passed argument is consist with what we expect
      if(S_NULL != arg_link->graphics_obj_)
      { throw(std::runtime_error("Error. Graphics object already initialized"));  }
      if(S_NULL == arg_link->robot_link_)
      { throw(std::runtime_error("Parser object pointer not initialized. Can't create graphics representation."));  }

      //0.b. Initialize the branching representation's required data

      //1. Create objects for each graphics mesh associated with the link
      if(0 < arg_link->robot_link_->graphics_obj_vec_.size())
      { // Attach one or more meshes
        //NOTE : Chai deletes its own data. So don't worry about the memory (leak).
        //1.a) Since we have meshes for this link, we will initialize a cMesh object
        //     to represent this link in chai's scenegraph
        arg_link->graphics_obj_ = new cMesh(data_->chai_world_);
        if(S_NULL == arg_link->graphics_obj_)
        { throw(std::runtime_error("Could not allocate a graphics object"));  }
        arg_link->graphics_obj_->setUseCulling(false,false);


        //Iterate over all the meshes and add them to the chai object
        std::vector<SRobotLinkGraphics>::const_iterator it,ite;
        ite = arg_link->robot_link_->graphics_obj_vec_.end();
        for(it = arg_link->robot_link_->graphics_obj_vec_.begin();
            it!=ite; ++it)
        {
          const SRobotLinkGraphics& lnk_gr = (*it);
          cMesh* tmp = new cMesh(data_->chai_world_);
          if(false == tmp->loadFromFile(lnk_gr.file_name_))
          {
            std::string err_str;
            err_str = "Couldn't find obj file: "+ lnk_gr.file_name_;
            throw(std::runtime_error(err_str.c_str()));
          }
          arg_link->graphics_obj_->addChild(tmp);

          // Set the object's position and orientation in its parent (fixed)
          tmp->setPos(lnk_gr.pos_in_parent_[0],
              lnk_gr.pos_in_parent_[1],
              lnk_gr.pos_in_parent_[2]);

          //Set the rotation in its parent
          Eigen::Matrix3d tmp_eigmat;
          cMatrix3d tmp_chaimat;
          Eigen::Quaternion<sFloat> tmp_quat(lnk_gr.ori_parent_quat_(3), lnk_gr.ori_parent_quat_(0),
              lnk_gr.ori_parent_quat_(1),lnk_gr.ori_parent_quat_(2));
          tmp_eigmat = tmp_quat.toRotationMatrix();

          tmp_chaimat.m[0][0] = tmp_eigmat(0,0);  tmp_chaimat.m[0][1] = tmp_eigmat(0,1);  tmp_chaimat.m[0][2] = tmp_eigmat(0,2);
          tmp_chaimat.m[1][0] = tmp_eigmat(1,0);  tmp_chaimat.m[1][1] = tmp_eigmat(1,1);  tmp_chaimat.m[1][2] = tmp_eigmat(1,2);
          tmp_chaimat.m[2][0] = tmp_eigmat(2,0);  tmp_chaimat.m[2][1] = tmp_eigmat(2,1);  tmp_chaimat.m[2][2] = tmp_eigmat(2,2);
          tmp->setRot(tmp_chaimat);

          cVector3d tmp_scale;
          tmp_scale[0] = lnk_gr.scaling_(0); tmp_scale[1] = lnk_gr.scaling_(1); tmp_scale[2] = lnk_gr.scaling_(2);
          tmp->scale(tmp_scale,true);
        }
      }
      else
      {
        //1.b) Since we have meshes for this link, we will initialize a sphere (default)
        //     to represent this link in chai's scenegraph
        arg_link->graphics_obj_ = new cShapeSphere(CHAI_SPHERE_RENDER_RADIUS);
      }

      //2. Now that the chai object's meshes (or lack thereof) have been parsed,
      //   we will set the object's position and orientation in its parent
      arg_link->graphics_obj_->setPos(arg_link->robot_link_->pos_in_parent_[0],
          arg_link->robot_link_->pos_in_parent_[1],
          arg_link->robot_link_->pos_in_parent_[2]);

      //Set the rotation in its parent
      Eigen::Matrix3d tmp_eigmat;
      cMatrix3d tmp_chaimat;
      tmp_eigmat = arg_link->robot_link_->ori_parent_quat_.toRotationMatrix();

      tmp_chaimat.m[0][0] = tmp_eigmat(0,0);  tmp_chaimat.m[0][1] = tmp_eigmat(0,1);  tmp_chaimat.m[0][2] = tmp_eigmat(0,2);
      tmp_chaimat.m[1][0] = tmp_eigmat(1,0);  tmp_chaimat.m[1][1] = tmp_eigmat(1,1);  tmp_chaimat.m[1][2] = tmp_eigmat(1,2);
      tmp_chaimat.m[2][0] = tmp_eigmat(2,0);  tmp_chaimat.m[2][1] = tmp_eigmat(2,1);  tmp_chaimat.m[2][2] = tmp_eigmat(2,2);
      arg_link->graphics_obj_->setRot(tmp_chaimat);

#ifdef W_TESTING
      //When testing, show the frames as well (gives a good idea of how things work).
      arg_link->graphics_obj_->setFrameSize(0.15,0.3,true);
      arg_link->graphics_obj_->setShowFrame(true,true);
#endif

      //3. Now initialize nodes on the graphics branching structure for the children
      sutil::CMappedTree<std::string, SGraphicsPhysicalLink>* rob_gr_brrep;
      rob_gr_brrep = data_->robots_rendered_.at(arg_link->robot_link_->robot_name_);

      //4. Create the children (recurse)
      std::vector<SRobotLink*>::const_iterator it,ite;
      ite = arg_link->robot_link_->child_addrs_.end();
      for(it = arg_link->robot_link_->child_addrs_.begin();
          it!=ite;++it)
      {
        //Create a non-root link in the branching structure
        SGraphicsPhysicalLink* child_link = rob_gr_brrep->create((*it)->name_,false);
        child_link->robot_link_ = (*it); //Set its data
        child_link->graphics_obj_ = S_NULL; //Set its data
        child_link->parent_name_ = arg_link->name_; //Set its name
        child_link->name_ = (*it)->name_; //Set its parent's name
        if(false==addRobotLink(child_link))
        { throw(std::runtime_error("Failed to add child robot-link"));  }

        //Set the children of the graphics object
        arg_link->graphics_obj_->addChild(child_link->graphics_obj_);
      }

#ifdef W_TESTING
      //Debugging information. To make sure we did the job correctly.
      printf("\n\n%s: Adding a child %s for parent link %s of robot %s \n Pos: %lf %lf %lf \n Rot: %lf %lf %lf %lf",
          data_->name_.c_str(),
          arg_link->robot_link_->name_.c_str(),
          arg_link->robot_link_->parent_name_.c_str(),
          arg_link->robot_link_->robot_name_.c_str(),
          arg_link->robot_link_->pos_in_parent_[0],
          arg_link->robot_link_->pos_in_parent_[1],
          arg_link->robot_link_->pos_in_parent_[2],
          arg_link->robot_link_->ori_parent_quat_.w(),
          arg_link->robot_link_->ori_parent_quat_.x(),
          arg_link->robot_link_->ori_parent_quat_.y(),
          arg_link->robot_link_->ori_parent_quat_.z());

      printf("\nQuaternion to Eigen rotation matrix:\n%lf %lf %lf \n%lf %lf %lf \n%lf %lf %lf ",
          tmp_eigmat(0,0),  tmp_eigmat(0,1),  tmp_eigmat(0,2),
          tmp_eigmat(1,0),  tmp_eigmat(1,1),  tmp_eigmat(1,2),
          tmp_eigmat(2,0),  tmp_eigmat(2,1),  tmp_eigmat(2,2));
#endif
    }
    catch(std::exception& ee)
    {
      if(S_NULL!=arg_link->graphics_obj_)
      {
        //Destructor also deletes any children. So no worries about the child vector.
        delete arg_link->graphics_obj_;
        arg_link->graphics_obj_ = S_NULL;
      }
      std::cerr<<"\nCChaiGraphics::addRobotLink() : Link="<<arg_link->robot_link_->name_<<", "<<ee.what();
      return false;
    }
    return true;
  }

  sBool CChaiGraphics::addMeshToRender(const std::string& arg_mesh_name,
      const std::string& arg_mesh_file, const Eigen::Vector3d& arg_pos,
      const Eigen::Matrix3d& arg_rot)
  {
    cMesh* tmp_chai_mesh = S_NULL;
    SGraphicsMesh* tmp_mesh_ds = S_NULL;
    try
    {
      if(!has_been_init_) { return false; }

      //1. Create a new chai mesh
      tmp_chai_mesh = new cMesh(data_->chai_world_);
      if(S_NULL == tmp_chai_mesh)
      { throw(std::runtime_error("Could not create a chai mesh."));  }

      //2. Load the mesh's graphics from a file.
      if(false == tmp_chai_mesh->loadFromFile(arg_mesh_file))
      {
        std::string err_str;
        err_str = "Couldn't load mesh graphics file: "+ arg_mesh_file;
        throw(std::runtime_error(err_str.c_str()));
      }

      //3. Initialize the mesh's position and orientation (relative to the origin)
      tmp_chai_mesh->setPos(arg_pos(0), arg_pos(1), arg_pos(2)); //Set position

      cMatrix3d tmp_chaimat;
      tmp_chaimat.m[0][0] = arg_rot(0,0);  tmp_chaimat.m[0][1] = arg_rot(0,1);  tmp_chaimat.m[0][2] = arg_rot(0,2);
      tmp_chaimat.m[1][0] = arg_rot(1,0);  tmp_chaimat.m[1][1] = arg_rot(1,1);  tmp_chaimat.m[1][2] = arg_rot(1,2);
      tmp_chaimat.m[2][0] = arg_rot(2,0);  tmp_chaimat.m[2][1] = arg_rot(2,1);  tmp_chaimat.m[2][2] = arg_rot(2,2);
      tmp_chai_mesh->setRot(tmp_chaimat); //Set rotation

      //4. Add the mesh to the world.
      data_->chai_world_->addChild(tmp_chai_mesh);

      //5. Create a new mesh data structure on the pile pointing to this chai object
      tmp_mesh_ds = data_->meshes_rendered_.create(arg_mesh_name);
      if(S_NULL == tmp_mesh_ds)
      { throw(std::runtime_error("Could not create a mesh data structure on the pile."));  }
      tmp_mesh_ds->graphics_obj_ = tmp_chai_mesh;
      tmp_mesh_ds->rotation_ = arg_rot;
      tmp_mesh_ds->translation_ = arg_pos;
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::addMeshToRender() : "<<ee.what();
      return false;
    }
    return true;
  }

  /** Removes a static mesh from the rendered scene. Indexed by its name.
   *
   * A mesh is defined as anything that DOESN"T obey the laws of
   * physics. It is merely rendered (possibly with collision etc). */
  sBool CChaiGraphics::removeMeshFromRender(const std::string& arg_mesh_name)
  {
    bool flag;
    try
    {
      if(!has_been_init_) { return false; }

      //1. Get mesh data structure pointing to the chai object
      SGraphicsMesh* tmp_mesh_ds = S_NULL;
      tmp_mesh_ds = data_->meshes_rendered_.at(arg_mesh_name);
      if(S_NULL == tmp_mesh_ds)
      { throw(std::runtime_error("Could not find the named mesh data structure."));  }

      cGenericObject* tmp_chai_mesh = tmp_mesh_ds->graphics_obj_;
      flag = tmp_chai_mesh->removeFromGraph();
      if(false == flag)
      { throw(std::runtime_error("Could not remove mesh from the chai graphics world. Invalid data state, chai data might be corrupted."));  }

      flag = data_->meshes_rendered_.erase(arg_mesh_name);
      if(false == flag)
      { throw(std::runtime_error("Could not remove mesh's data from the mapped tree. Invalid data state, scl graphics data might be corrupted."));  }
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::removeMeshFromRender() : "<<ee.what();
      return false;
    }
    return true;
  }



  sBool CChaiGraphics::addMusclesToRender(
      const std::string& arg_robot,
      const std::string& arg_msys,
      const sBool add_musc_via_points)
  {
    std::string musc_name("");
    try
    {
      if(!has_been_init_) { return false; }

      //1. Get the robot
      SDatabase* db = CDatabase::getData();
      if(S_NULL == db) { throw(std::runtime_error("Database not initialized"));  }

      sutil::CMappedTree<std::string, SGraphicsPhysicalLink>* rob_gr;
      rob_gr = data_->robots_rendered_.at(arg_robot);
      if(S_NULL==rob_gr)//Require an existing robot to render muscles.
      { throw(std::runtime_error("Couldn't find a (physics+graphics) representation for the robot on the pile"));  }

      SMuscleSystem *msys_db = db->s_parser_.muscle_systems_.at(arg_msys);
      if(S_NULL == msys_db) { throw(std::runtime_error("Could not find muscle system in the database. Did you pass the right name?"));  }

      SGraphicsMsys *msys_gr = data_->muscles_rendered_.create(arg_msys);
      if(S_NULL == db) { throw(std::runtime_error("Could not create muscle rendering data struct on the pile"));  }

      sutil::CMappedList<std::basic_string<char>, scl::SMuscle>::iterator it,ite;
      for(it = msys_db->muscles_.begin(), ite = msys_db->muscles_.end();
          it!=ite; ++it)
      {
        SMuscle &mus = *it;

        musc_name = mus.muscle_name_;//For debugging. Useful to know where it failed.

        SGraphicsMsys::SGraphicsMuscle gr_musc;//Initialize this and plug it into the msys

        std::vector<SMusclePoint>::iterator it,it2,ite;
        for(it = mus.points_.begin(), ite = mus.points_.end();
            it!=ite;++it)
        {
          it2 = it+1;

          SGraphicsPhysicalLink* gr_lnk = rob_gr->at((*it).parent_link_);
          if(S_NULL == gr_lnk)
          {
            std::cout<<"\nCChaiGraphics::addMusclesToRender("<<arg_msys<<") WARNING: Orphan muscle: "
                <<musc_name<<". Missing parent link: "<<(*it).parent_link_;
            continue;
          }

          SGraphicsMsys::SGraphicsMuscle::SGraphicsMusclePoint gr_mpt;//Initialize this and plug it into the muscle

          //Get the parent chai graphics object
          gr_mpt.graphics_parent_ = gr_lnk;

          gr_mpt.pos_ = new cVector3d();
          gr_mpt.pos_->x = (*it).point_(0);
          gr_mpt.pos_->y = (*it).point_(1);
          gr_mpt.pos_->z = (*it).point_(2);

          if(it2!=ite)
          {
            SGraphicsPhysicalLink* gr_lnk2 = rob_gr->at((*it2).parent_link_);
            if(S_NULL == gr_lnk2)
            {
              std::cout<<"\nCChaiGraphics::addMusclesToRender("<<arg_msys<<") WARNING: Orphan muscle upcoming : "
                  <<musc_name<<". Missing parent link: "<<(*it2).parent_link_;
              continue;
            }
            gr_mpt.graphics_parent_next_ = gr_lnk2;

            gr_mpt.pos_next_ = new cVector3d();
            gr_mpt.pos_next_->x = (*it2).point_(0);
            gr_mpt.pos_next_->y = (*it2).point_(1);
            gr_mpt.pos_next_->z = (*it2).point_(2);

            //Set up a line : The initial coordinates don't matter and will be immediately
            //updated with global coordinates of the skeletonwhile rendering.
            cShapeLine *tmp_l = new cShapeLine(*(gr_mpt.pos_),*(gr_mpt.pos_next_));

            //Set the color
            cColorf muscleColor(1,0,0,0.5);        //R,G,B,Alpha
            tmp_l->m_ColorPointA = muscleColor;
            tmp_l->m_ColorPointB = muscleColor;
            tmp_l->setShowEnabled(true,true);

            //Save the line's graphics object
            gr_mpt.graphics_via_line_ =  tmp_l;

            //Add it to the parent : We don't want it rotating around with its parent frame
            //because we use global positions and rotations (local don't work for multi
            //links if we don't know that the links are adjacent.
            this->data_->chai_world_->addChild(gr_mpt.graphics_via_line_);
          }

          if(add_musc_via_points)//Add via point rendering if required
          {
            gr_mpt.graphics_via_point_ = new cShapeSphere(CHAI_SPHERE_MUSC_VIA_RADIUS);
            gr_mpt.graphics_via_point_->setPos(*(gr_mpt.pos_));

            gr_mpt.graphics_parent_->graphics_obj_->addChild(gr_mpt.graphics_via_point_);
          }

          //Add the point to the muscle
          gr_musc.mpt_.push_back(gr_mpt);
        }

        //Add the muscle to the rendering system.
        msys_gr->msys_.push_back(gr_musc);

        //Get the next muscle
      }
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::addMusclesToRender("<<arg_msys<<") : "<<musc_name<<" : "<<ee.what();
      return false;
    }
    return true;
  }


  sBool CChaiGraphics::addSphereToRender(
        const std::string& arg_robot,
        const std::string& arg_link,
        const Eigen::Vector3d& arg_pos,
        const sFloat arg_size,
        cGenericObject** arg_ret_ptr)
  {
    try
    {
      if(!has_been_init_) { return false; }

      /** Render a sphere at the link's position */
      sutil::CMappedTree<std::string, scl::SGraphicsPhysicalLink> * rob_br = S_NULL;
      rob_br = data_->robots_rendered_.at(arg_robot);//Shortcut
      if(S_NULL == rob_br)
      { throw(std::runtime_error("Could not find robot"));  }

      if(S_NULL == rob_br->at(arg_link))
      { throw(std::runtime_error(std::string("Could not find robot's link: ")+arg_link));  }

      cGenericObject *op_gr = rob_br->at(arg_link)->graphics_obj_;
      if(S_NULL == op_gr)
      { throw(std::runtime_error("Could not find task's rendering obj"));  }

      //Create a new sphere and add it to the robot.
      cGenericObject *new_op_gr = S_NULL;
      new_op_gr = new cShapeSphere(arg_size);
      if(S_NULL == op_gr)
      { throw(std::runtime_error("Could not allocate new rendering object"));  }

      //Set its position in the parent frame
      new_op_gr->setPos(arg_pos(0),arg_pos(1),arg_pos(2));

      //Add it to the parent frame as a child
      op_gr->addChild(new_op_gr);

      if(S_NULL!=arg_ret_ptr)
      { *arg_ret_ptr = new_op_gr; }
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::addSphereToRender("<<arg_robot<<") : "<<arg_link<<" : "<<ee.what();
      return false;
    }
    return true;
  }

  sBool CChaiGraphics::addSphereToRender(
        const Eigen::Vector3d& arg_pos,
        cGenericObject*& arg_ret_ptr,
        const sFloat arg_size)
  {
    try
    {
      if(!has_been_init_) { return false; }

      //Create a new sphere and add it to the robot.
      cGenericObject *new_op_gr = new cShapeSphere(arg_size);
      if(S_NULL == new_op_gr)
      { throw(std::runtime_error("Could not allocate new rendering object"));  }

      //Set its position in the parent frame
      new_op_gr->setPos(arg_pos(0),arg_pos(1),arg_pos(2));

      //Add it to the parent frame as a child
      data_->chai_world_->addChild(new_op_gr);

      //Set the return pointer if passed, so that the caller can manipulate the sphere.
      arg_ret_ptr = new_op_gr;
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::addSphereToRender(no-robot) : global-frame : "<<ee.what();
      return false;
    }
    return true;
  }




  sBool CChaiGraphics::updateGraphics()
  {
    bool flag = true;
#ifdef W_TESTING
    try
    {
      if(S_NULL == data_) { throw(std::runtime_error("Graphics data structure not initialized"));  }
#endif
      if(!has_been_init_)
      { return false; }

      //1. Loop over all the robots and update their graphics:
      flag = flag && updateGraphicsForRobots();

      //3. Loop over all the musles and update their graphics
      flag = flag && updateGraphicsForMuscles();

      //2. Loop over all meshes and update their graphics
      flag = flag && updateGraphicsForMeshes();

      glViewport(0,0,data_->gl_width_,data_->gl_height_);
      data_->chai_cam_->renderView(data_->gl_width_,data_->gl_height_);//Finally : Use the camera to render the scene view.

#ifdef W_TESTING
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::updateGraphics() : "<<ee.what();
      return false;
    }
#endif
    return flag;
  }

  sBool CChaiGraphics::updateGraphicsForRobots()
  {
    try
    {
      //0. Get a copy of the database
      SDatabase* db = CDatabase::getData();
#ifdef W_TESTING
      if(S_NULL == db) { throw(std::runtime_error("Database not initialized"));  }
#endif

      //1. Loop over all the robots and update their graphics:
      sutil::CMappedList<std::string, sutil::CMappedTree<std::string, scl::SGraphicsPhysicalLink>
      >::iterator it,ite;
      for(it = data_->robots_rendered_.begin(), ite = data_->robots_rendered_.end();
          it!=ite; ++it)
      {
        //1.a. Get the robot's branching representation and its io buffer
        sutil::CMappedTree<std::string, SGraphicsPhysicalLink>& rob_brrep = *it;

        //NOTE TODO : This is a bit inefficient. Find a better way to do this.
        const SRobotIOData* rob_io;
        std::string tmp_rob_name = !it;
        rob_io = db->s_io_.io_data_.at_const(tmp_rob_name);
#ifdef W_TESTING
        if(S_NULL == rob_io) { throw(std::runtime_error("Robot's I/O database entry is not initialized"));  }
#endif

        //1.b. Now iterate over the robot's links and update their transformation (from the parent)
        sutil::CMappedTree<std::basic_string<char>, scl::SGraphicsPhysicalLink>::iterator itgr, itgre;
        for(itgr = rob_brrep.begin(), itgre = rob_brrep.end();
            itgr!=itgre; ++itgr)
        {
          //1.b.i. Obtain all the data structures.
          SGraphicsPhysicalLink& lnk = *itgr;

          const SRobotLink* lnk_robdata = lnk.robot_link_;
#ifdef W_TESTING
          if(S_NULL == lnk_robdata) { throw(std::runtime_error("Found an uninitialized pointer to a robot link in the graphics pile"));  }
#endif


          cGenericObject* lnk_grdata = lnk.graphics_obj_;
#ifdef W_TESTING
          if(S_NULL == lnk_grdata) { throw(std::runtime_error("Found an uninitialized pointer to a chai generic object in the graphics pile"));  }
#endif

          //1.b.ii. Obtain the joint angle/translation etc..
          sInt link_id = lnk_robdata->link_id_;
          sFloat q, dq, tau_measured;

          if(-1 == link_id)
          {//Ground node. Do not change.
            continue;
          }
#ifdef W_TESTING
          if(-1 > link_id) { throw(std::runtime_error("Found a link with an id less than 0"));  }
          if(link_id >= rob_io->sensors_.q_.size()) { throw(std::runtime_error("Found a link with an id greater than the max for this robot"));  }
#endif
          q = rob_io->sensors_.q_(link_id);
          dq = rob_io->sensors_.dq_(link_id);
          tau_measured = rob_io->sensors_.force_gc_measured_(link_id);

          //Compute the local translation and rotation for the link (from its parent).
          cVector3d tr;
          cMatrix3d rot;
          Eigen::Matrix3d tmp_rot(Eigen::Matrix3d::Identity());

          //Transformation depends on the joint type:
          switch(lnk_robdata->joint_type_)
          {
            case JOINT_TYPE_PRISMATIC_X:
              tr[0] = lnk_robdata->pos_in_parent_(0) + q;
              tr[1] = lnk_robdata->pos_in_parent_(1);
              tr[2] = lnk_robdata->pos_in_parent_(2);
              break;
            case JOINT_TYPE_PRISMATIC_Y:
              tr[0] = lnk_robdata->pos_in_parent_(0);
              tr[1] = lnk_robdata->pos_in_parent_(1) + q;
              tr[2] = lnk_robdata->pos_in_parent_(2);
              break;
            case JOINT_TYPE_PRISMATIC_Z:
              tr[0] = lnk_robdata->pos_in_parent_(0);
              tr[1] = lnk_robdata->pos_in_parent_(1);
              tr[2] = lnk_robdata->pos_in_parent_(2) + q;
              break;
            case JOINT_TYPE_REVOLUTE_X:
              tr[0] = lnk_robdata->pos_in_parent_(0);
              tr[1] = lnk_robdata->pos_in_parent_(1);
              tr[2] = lnk_robdata->pos_in_parent_(2);
              //Emulate euler angles with Eigen angle-axis
              tmp_rot = lnk_robdata->ori_parent_quat_.toRotationMatrix() * Eigen::AngleAxisd(q,Eigen::Vector3d::UnitX());
              break;
            case JOINT_TYPE_REVOLUTE_Y:
              tr[0] = lnk_robdata->pos_in_parent_(0);
              tr[1] = lnk_robdata->pos_in_parent_(1);
              tr[2] = lnk_robdata->pos_in_parent_(2);
              //Emulate euler angles with Eigen angle-axis
              tmp_rot = lnk_robdata->ori_parent_quat_.toRotationMatrix() * Eigen::AngleAxisd(q,Eigen::Vector3d::UnitY());
              break;
            case JOINT_TYPE_REVOLUTE_Z:
              tr[0] = lnk_robdata->pos_in_parent_(0);
              tr[1] = lnk_robdata->pos_in_parent_(1);
              tr[2] = lnk_robdata->pos_in_parent_(2);
              //Emulate euler angles with Eigen angle-axis
              tmp_rot = lnk_robdata->ori_parent_quat_.toRotationMatrix() * Eigen::AngleAxisd(q,Eigen::Vector3d::UnitZ());
              break;
            case JOINT_TYPE_SPHERICAL:
              //NOTE TODO : Implement support for spherical joints.
              tr.zero();
              tmp_rot = Eigen::Matrix3d::Identity();
              break;
            default:
              tr.zero();
              tmp_rot = Eigen::Matrix3d::Identity();
              break; //JOINT_TYPE_NOTASSIGNED
          }

          rot.m[0][0] = tmp_rot(0,0);  rot.m[0][1] = tmp_rot(0,1);  rot.m[0][2] = tmp_rot(0,2);
          rot.m[1][0] = tmp_rot(1,0);  rot.m[1][1] = tmp_rot(1,1);  rot.m[1][2] = tmp_rot(1,2);
          rot.m[2][0] = tmp_rot(2,0);  rot.m[2][1] = tmp_rot(2,1);  rot.m[2][2] = tmp_rot(2,2);

          //3. Initialize the mesh's position and orientation (relative to the parent)
          lnk_grdata->setPos(tr); //Set position
          lnk_grdata->setRot(rot); //Set rotation

          //Advance to the next link in the robot
        }

        //Advance to the next robot
      }
      return true;
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::updateGraphicsForRobots() : "<<ee.what();
      return false;
    }
    return true;
  }

  sBool CChaiGraphics::updateGraphicsForMuscles()
  {
    try
    {
      //The preliminaries
      glLineWidth(CHAI_MUSC_THICKNESS);                   // Muscle line size (pixels)

      // Recompute global positions : Required for the muscle points
      data_->chai_world_->computeGlobalPositions(true);

      //Loop over all the muscles and update their graphics:
      sutil::CMappedList<std::basic_string<char>, scl::SGraphicsMsys>::iterator it,ite;
      for(it = data_->muscles_rendered_.begin(), ite = data_->muscles_rendered_.end();
          it!=ite; ++it)
      {
        //Get a muscle system
        SGraphicsMsys& msys = *it;

        std::vector<SGraphicsMsys::SGraphicsMuscle>::iterator it,ite;
        for(it = msys.msys_.begin(), ite = msys.msys_.end();
            it!=ite; ++it)
        {//Loop over all the muscles in a system.
          std::vector<SGraphicsMsys::SGraphicsMuscle::SGraphicsMusclePoint>::iterator itmp,itmpe;
          for(itmp = (*it).mpt_.begin(), itmpe = (*it).mpt_.end();
              itmp!=itmpe; ++itmp)
          {//Loop over all the muscle connection points for a muscle
            if(S_NULL == itmp->graphics_via_line_)
              continue;//End point. No line segment

            SGraphicsMsys::SGraphicsMuscle::SGraphicsMusclePoint& mp = *itmp;//Tmp ref

            //Reposition the line's end points if the parent links have moved, which they will.
            cGenericObject *par = mp.graphics_parent_->graphics_obj_;
            cGenericObject *par2 = mp.graphics_parent_next_->graphics_obj_;
            cVector3d rotvec, rotvec2;
            cShapeLine* l = dynamic_cast<cShapeLine*>(itmp->graphics_via_line_);

            rotvec = par->getGlobalRot() * (*(*itmp).pos_);//Rotate the point to this frame.
            l->m_pointA = par->getGlobalPos() + rotvec; //Translate the rotated position vector from the frame's position.

            rotvec2 = par2->getGlobalRot() * (*(*itmp).pos_next_);//Rotate the point to this frame.
            l->m_pointB = par2->getGlobalPos() + rotvec2; //Translate the rotated position vector from the frame's position.
          }
        }

        //Advance to the next muscle
      }
    }
    catch(std::exception& ee)
    {
      std::cerr<<"\nCChaiGraphics::updateGraphicsForMuscles() : "<<ee.what();
      return false;
    }
    return true;
  }

}
