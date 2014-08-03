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
/* \file CRepCreator3d.cpp
 *
 *  Created on: Jun 21, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 *  Edited by: Kenji Hata <khata@stanford.edu>
 */

#include "CRepCreator3d.hpp"

#include <chai3d.h>
#include <dynamics3d.h>

#include <sutil/CMappedTree.hpp>

#include <vector>
#include <stdexcept>
#include <iostream>

using namespace std;
using namespace scl;

#define _R(i,j) R[(i)*4+(j)]

namespace scl_ext
{
  /**
   * Creates a 3d root structure out of a set of robot definitions
   *
   * Also assigns ids to the SRigidBody objects
   *
   * input : Robot name
   * output : 3dNodeRoot*
   *
   * Returns,
   * NULL : failure
   * Valid 3dNodeRoot pointer : success
   *
   * NOTE : Remember to dereference the vectors in 3dNodeRoot
   */
  cDynamicBase* CRepCreator3d::c3dRootRepCreator(const SRobotParsed& arg_robot)
  {
    const SRigidBody* tmp_root; // For parsing the database
    cDynamicBase *ret_3d_base;
    cDynamicBase *ret_3d_baseRobot;
    cDynObject* ret_3d_root;   // Returns a root node
    bool flag;

    try
    {
      //*******Step 1***********
      //Check if a valid representation was passed.
      //************************
      if (false == arg_robot.has_been_init_)
      { throw(std::runtime_error("Passed an uninitialized robot data structure"));  }

      //*******Step 2***********
      //Traverse the robotRoot's tree and construct a 3d tree structure
      //************************
      //Step 2a: Find the desired robot root.
      const sutil::CMappedTree<std::string, SRigidBody> & br = arg_robot.rb_tree_;
      tmp_root = br.getRootNodeConst();//The root node.
      if (tmp_root == NULL)
      { throw(std::runtime_error("Robot doesn't have valid root node"));}
      if(false == tmp_root->is_root_)
      { throw(std::runtime_error(
          "Robot's branching representation returns root with SRigidBody::is_root_==false."));}

      //Step 2b: Create the corresponding root 3d node
      //Root node's home frame translation and orientation
      chai3d::cWorld *cw = new chai3d::cWorld();
      cdw = new cDynamicWorld(cw);

      cw->addChild(cdw);

      cdw->setGravity(arg_robot.gravity_(0),arg_robot.gravity_(1),arg_robot.gravity_(2));

      Eigen::Quaternion<sFloat> tmp_quat(tmp_root->ori_parent_quat_.w(), tmp_root->ori_parent_quat_.x(), tmp_root->ori_parent_quat_.y(), tmp_root->ori_parent_quat_.z());
      Eigen::Matrix3d tmp_rot_mat = tmp_quat.toRotationMatrix();
      chai3d::cMatrix3d rot;
      rot(0,0) = tmp_rot_mat(0,0); rot(1,0) = tmp_rot_mat(1,0); rot(2,0) = tmp_rot_mat(2,0);
      rot(0,1) = tmp_rot_mat(0,1); rot(1,1) = tmp_rot_mat(1,1); rot(2,1) = tmp_rot_mat(2,1);
      rot(0,2) = tmp_rot_mat(0,2); rot(1,2) = tmp_rot_mat(1,2); rot(2,2) = tmp_rot_mat(2,2);
      chai3d::cVector3d pos = chai3d::cVector3d(tmp_root->pos_in_parent_[0],tmp_root->pos_in_parent_[1],tmp_root->pos_in_parent_[2]);

      ret_3d_base = cdw->newBaseObject(pos, rot);
      if (ret_3d_base == NULL)
      { throw(std::runtime_error("Can't create 3d base object"));}
      ret_3d_root = ret_3d_base->m_dynBaseObject;

      ret_3d_root->fixed(tmp_root->link_is_fixed_);
      ret_3d_root->scl_id=tmp_root->link_id_; //NOTE TODO What about the multiple robot case? Will all have rootId = -1
      ret_3d_root->name_ = tmp_root->name_;

      ret_3d_baseRobot = cdw->newBaseObject(pos, rot);
      if (ret_3d_baseRobot == NULL)
      { throw(std::runtime_error("Can't create 3d base object"));}
      ret_3d_root = ret_3d_baseRobot->m_dynBaseObject;

      ret_3d_root->fixed(tmp_root->link_is_fixed_);
      ret_3d_root->scl_id=tmp_root->link_id_; //NOTE TODO What about the multiple robot case? Will all have rootId = -1
      ret_3d_root->name_ = tmp_root->name_;

      std::cout << "ROOT NODE NAME: " << ret_3d_root->name_ << std::endl;

      // add the graphics file
      cDynamicMaterial * mat = new cDynamicMaterial();
      // mat->setEpsilon(.05);
      mat->setStaticFriction(0.1);
      mat->setDynamicFriction(0.1);

      cDynamicLink *tmp_link = ret_3d_baseRobot->newLink(mat);
      ret_3d_baseRobot->linkChild(tmp_link,chai3d::cVector3d(0,0,0), chai3d::cIdentity3d());

      ret_3d_baseRobot->setShowContactNormalForces(true);

      cDynamicLink *ret_3d_link = ret_3d_base->newLink(mat);
      double error = 1e-6;   // tolerance error for contact detection
      double radius = 1e-4;  // radius zone for contact detection
      chai3d::cMultiMesh* objectModel = new chai3d::cMultiMesh();
      ret_3d_link->addChild(objectModel);

      chai3d::cMatrix3d tmp_mat;
      chai3d::cVector3d tmp_pos;
      for(std::vector<SRigidBodyGraphics>::const_iterator it = tmp_root->graphics_obj_vec_.begin();
          it != tmp_root->graphics_obj_vec_.end(); ++it) {
        objectModel->loadFromFile((*it).file_name_);
        objectModel->scale((double)(*it).scaling_(0), true);

        tmp_pos = chai3d::cVector3d((double)(*it).pos_in_parent_[0],
            (double)(*it).pos_in_parent_[1],
            (double)(*it).pos_in_parent_[2]);
        //Set the rotation in its parent
        Eigen::Quaternion<sFloat> tmp_quat((*it).ori_parent_quat_(3), (*it).ori_parent_quat_(0),
            (*it).ori_parent_quat_(1),(*it).ori_parent_quat_(2));
        Eigen::Matrix3d tmp_rot_mat = tmp_quat.toRotationMatrix();
        tmp_mat(0,0) = tmp_rot_mat(0,0); tmp_mat(1,0) = tmp_rot_mat(1,0); tmp_mat(2,0) = tmp_rot_mat(2,0);
        tmp_mat(0,1) = tmp_rot_mat(0,1); tmp_mat(1,1) = tmp_rot_mat(1,1); tmp_mat(2,1) = tmp_rot_mat(2,1);
        tmp_mat(0,2) = tmp_rot_mat(0,2); tmp_mat(1,2) = tmp_rot_mat(1,2); tmp_mat(2,2) = tmp_rot_mat(2,2);

        objectModel->setUseDisplayList(true, true);
        objectModel->setUseCulling(false, true);
        objectModel->setLocalRot(tmp_mat);
        objectModel->setLocalPos(tmp_pos);
        objectModel->computeGlobalPositions(true);
      }
      ret_3d_link->setCollisionModel(objectModel);
      ret_3d_link->setImageModel(objectModel);
      ret_3d_link->buildCollisionHull(radius, error);
      ret_3d_base->linkChild(ret_3d_link,chai3d::cVector3d(0,0,0), chai3d::cIdentity3d());

      //Step 2c: Create child nodes
      flag = createChild3dNodes(tmp_root,NULL,ret_3d_baseRobot);
      if(false==flag)
      { throw(std::runtime_error("Can't create child nodes")); }

      //*******Step 3***********
      //Initialize 3dDynamics : Any other initialization.
      //************************
      //TODO:	 cDynamicsInitialize(ret_3d_root,0);
    }
    catch(std::exception& e)
    {
      std::cout<<"\nCRepCreator3d::3dRootRepCreator() : "<<e.what();
      delete ret_3d_root;
      delete ret_3d_base;
      return S_NULL;
    }

    //Return the root of the 3d tree
    return ret_3d_baseRobot; //Return the created 3d root structure
  }

  /**This function creates child 3d nodes for a given link in the robot's
   * definition. */
  bool CRepCreator3d::createChild3dNodes(
      const SRigidBody* arg_link, cDynamicLink* arg_parent, cDynamicBase* arg_base)
  {
    bool flag = true;
    std::vector<SRigidBody*>::const_iterator child_iter, child_iterE;
    //Traverse the list of child links and create a 3dNode for each one
    for (child_iter = arg_link->child_addrs_.begin(),
        child_iterE = arg_link->child_addrs_.end();
        child_iter != child_iterE;
        ++child_iter
    )
    {
      cDynamicLink* tmp_3d_node;
      tmp_3d_node = create3dNonRootNode((*child_iter), arg_parent,
          arg_base);
      if(S_NULL == tmp_3d_node)
      {
        printf("\nCRepCreator3d::createChild3dNodes() : Error. Couldn't create 3d node.");
        return false;
      }
#ifdef DEBUG
      else
      {
        printf("\nCRepCreator3d::createChild3dNodes() : Created 3d Node (%d) %s .",
            tmp_3d_node->m_dynObject->scl_id, tmp_3d_node->m_dynObject->name_.c_str());
      }
#endif
      flag = createChild3dNodes((*child_iter), tmp_3d_node, arg_base);
      if(false == flag)
      {
        printf("\nCRepCreator3d::createChild3dNodes() : Error. Couldn't create child node(s).");
        return false;
      }
    }
    return flag;
  }

  cDynamicLink* CRepCreator3d::
  create3dNonRootNode(const SRigidBody* arg_link,
      cDynamicLink* arg_parent_node, cDynamicBase *base_obj)
  {
    cDynamicLink *ret_3d_link;
    cDynObject *ret_3d_node = S_NULL; //Create a node (a link in a branching structure)
    cDynamicJoint *tmp_joint;

    try
    {
      if(S_NULL == arg_link) //No node passed. Can't initialize
      {throw(std::runtime_error("Passed an invalid link specification."));}

      if (S_NULL == arg_link->parent_addr_)
      {
        throw(std::runtime_error(
            "Tried to create a 3d non-root node with SRigidBody specification that doesn't have a parent."));
      }

      if (true == arg_link->is_root_)
      {
        throw(std::runtime_error(
            "Tried to create a 3d non-root node with a root node specification."));
      }

      if (0 > arg_link->link_id_)
      {
        throw(std::runtime_error(
            "Tried to create a 3d non-root node with an invalid (negative integer) id."));
      }

      //Non-root Node
      cDynamicMaterial * mat = new cDynamicMaterial();
      //mat->setEpsilon(.95);

      //SET FRICTION HERE!
      mat->setStaticFriction(0.1);
      mat->setDynamicFriction(0.1);
      ret_3d_link = base_obj->newLink(mat);

      base_obj->setShowContactNormalForces(true);

      ret_3d_node = ret_3d_link->m_dynObject;
      ret_3d_node->name_ = arg_link->name_;
      ret_3d_node->scl_id = arg_link->link_id_;

      if(S_NULL == ret_3d_node)
      { throw(std::runtime_error("Couldn't create a new 3d node."));  }

      //2. Set up COM frame of reference WRT the home frame
      //Only translates to the COM. The rotation is the same
      //as the home frame.
      chai3d::cVector3d com = chai3d::cVector3d((double)arg_link->com_[0], (double)arg_link->com_[1], (double)arg_link->com_[2]);
      chai3d::cVector3d in = chai3d::cVector3d(arg_link->inertia_(0,0), arg_link->inertia_(1,1), arg_link->inertia_(2,2));

      //3. Set up the node's physical link properties
      ret_3d_link->setMassProperties((double)arg_link->mass_, in, com);

      // Create new joint for the new child node
      switch(arg_link->joint_type_)
      {
        case JOINT_TYPE_PRISMATIC_X:
          tmp_joint = ret_3d_link->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_X);
          break;
        case JOINT_TYPE_PRISMATIC_Y:
          tmp_joint = ret_3d_link->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Y);
          break;
        case JOINT_TYPE_PRISMATIC_Z:
          tmp_joint = ret_3d_link->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);
          break;
        case JOINT_TYPE_REVOLUTE_X:
          tmp_joint = ret_3d_link->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_X);
          break;
        case JOINT_TYPE_REVOLUTE_Y:
          tmp_joint = ret_3d_link->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Y);
          break;
        case JOINT_TYPE_REVOLUTE_Z:
          tmp_joint = ret_3d_link->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Z);
          break;
        case JOINT_TYPE_SPHERICAL:
          tmp_joint = ret_3d_link->newJoint(DYN_JOINT_SPHERICAL);
          break;
          //case JOINT_TYPE_NONE:
          /* TODO: FIGURE OUT WHAT TO DO HERE */
          //tmp_joint = NULL;
          //break;
        default: break; //JOINT_TYPE_NOTASSIGNED
      }

      if(S_NULL == tmp_joint)
      { throw(std::runtime_error("Couldn't initialize 3d joint (Maybe an unrecognized joint type specification)."));  }

      tmp_joint->name_ = arg_link->joint_name_;
      tmp_joint->setDamping(0.0);

      double error = 0.0001;   // tolerance error for contact detection
      double radius = 0.0001;  // radius zone for contact detection
      chai3d::cMultiMesh* objectModel = new chai3d::cMultiMesh();
      for(std::vector<SRigidBodyGraphics>::const_iterator it = arg_link->graphics_obj_vec_.begin();
          it != arg_link->graphics_obj_vec_.end(); ++it) {
        if((*it).collision_type_){

          if((*it).class_ == (*it).GRAPHIC_TYPE_CUBOID){
            chai3d::cMesh* meshObject = objectModel->newMesh();
            cCreateBox(meshObject, (double)(*it).scaling_[0], (double)(*it).scaling_[1], (double)(*it).scaling_[2]);

          }else if((*it).class_ == (*it).GRAPHIC_TYPE_CYLINDER){
            chai3d::cMesh* meshObject = objectModel->newMesh();
            cCreateCylinder(meshObject, (double)(*it).scaling_[0], (double)(*it).scaling_[1]);

          }else if ((*it).class_ == (*it).GRAPHIC_TYPE_SPHERE){
            chai3d::cMesh* meshObject = objectModel->newMesh();
            cCreateSphere(meshObject, (double)(*it).scaling_[0]);

          }else if ((*it).class_ == (*it).GRAPHIC_TYPE_FILE_OBJ){
            //   if(false == cLoadFileOBJ(objectModel, (*it).file_name_))
            //	   cLoadFile3DS(objectModel, (*it).file_name_);
            objectModel->loadFromFile((*it).file_name_);
            objectModel->setLocalPos((double)(*it).pos_in_parent_[0],
                (double)(*it).pos_in_parent_[1],
                (double)(*it).pos_in_parent_[2]);
            //Set the rotation in its parent
            Eigen::Quaternion<sFloat> tmp_quat((*it).ori_parent_quat_(3), (*it).ori_parent_quat_(0),
                (*it).ori_parent_quat_(1),(*it).ori_parent_quat_(2));
            Eigen::Matrix3d tmp_rot_mat = tmp_quat.toRotationMatrix();
            chai3d::cMatrix3d tmp_mat;
            tmp_mat(0,0) = tmp_rot_mat(0,0); tmp_mat(1,0) = tmp_rot_mat(1,0); tmp_mat(2,0) = tmp_rot_mat(2,0);
            tmp_mat(0,1) = tmp_rot_mat(0,1); tmp_mat(1,1) = tmp_rot_mat(1,1); tmp_mat(2,1) = tmp_rot_mat(2,1);
            tmp_mat(0,2) = tmp_rot_mat(0,2); tmp_mat(1,2) = tmp_rot_mat(1,2); tmp_mat(2,2) = tmp_rot_mat(2,2);
            objectModel->setLocalRot(tmp_mat);
            objectModel->scale((double)(*it).scaling_(0), true);;
          }
          ret_3d_link->setCollisionModel(objectModel);
          ret_3d_link->setImageModel(objectModel);
          ret_3d_link->buildCollisionHull(radius, error);
        }
      }

      //NOTE : 3d : Home frame = Local frame wrt parent.
      // Home frame translation and orientation
      // set the link's position and rotation in parent
      Eigen::Quaternion<sFloat> tmp_quat(arg_link->ori_parent_quat_.w(), arg_link->ori_parent_quat_.x(), arg_link->ori_parent_quat_.y(), arg_link->ori_parent_quat_.z());
      Eigen::Matrix3d tmp_rot_mat = tmp_quat.toRotationMatrix();
      chai3d::cMatrix3d crot;
      crot(0,0) = tmp_rot_mat(0,0); crot(1,0) = tmp_rot_mat(1,0); crot(2,0) = tmp_rot_mat(2,0);
      crot(0,1) = tmp_rot_mat(0,1); crot(1,1) = tmp_rot_mat(1,1); crot(2,1) = tmp_rot_mat(2,1);
      crot(0,2) = tmp_rot_mat(0,2); crot(1,2) = tmp_rot_mat(1,2); crot(2,2) = tmp_rot_mat(2,2);
      chai3d::cVector3d cpos = chai3d::cVector3d(arg_link->pos_in_parent_[0],arg_link->pos_in_parent_[1],arg_link->pos_in_parent_[2]);
      if (arg_parent_node)
        arg_parent_node->linkChild(ret_3d_link, cpos, crot);
      else
        base_obj->linkChild(ret_3d_link,cpos,crot);
    }
    catch(std::exception& e){
      std::cout<<"\nCRepCreator3d::create3dNonRootNode("
          <<arg_link->robot_name_<<":"<<arg_link->name_<<") : "<<e.what();
      delete ret_3d_node;
      delete tmp_joint;
      return S_NULL;
    }
    return ret_3d_link;
  }

}
