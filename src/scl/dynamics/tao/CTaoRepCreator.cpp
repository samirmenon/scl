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
/* \file CTaoRepCreator.cpp
 *
 *  Created on: May 11, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */ 

#include <scl/dynamics/tao/CTaoRepCreator.hpp>

#include <scl/data_structs/SRobotParsedData.hpp>

#include <scl/dynamics/tao/tao/dynamics/taoDynamics.h>
#include <scl/dynamics/tao/tao/dynamics/taoJoint.h>
#include <scl/dynamics/tao/tao/utility/TaoDeMassProp.h>

#include <sutil/CMappedTree.hpp>

#include <vector>
#include <stdexcept>
#include <iostream>

using namespace std;

namespace scl {

  /**
   * Creates a tao root structure out of a set of robot definitions
   *
   * Also assigns ids to the SRobotLink objects
   *
   * input : Robot name
   * output : taoNodeRoot*
   *
   * Returns,
   * NULL : failure
   * Valid taoNodeRoot pointer : success
   *
   * NOTE : Remember to dereference the vectors in taoNodeRoot
   */
taoNodeRoot* CTaoRepCreator::taoRootRepCreator(const SRobotParsedData& arg_robot)
{
  const SRobotLink* tmp_root; // For parsing the database
  taoNodeRoot * ret_tao_root;  // Returns a root node
  bool flag;

  try
  {
    //*******Step 1***********
    //Check if a valid representation was passed.
    //************************
    if (false == arg_robot.has_been_init_)
    { throw(std::runtime_error("Passed an uninitialized robot data structure"));  }

    //*******Step 2***********
    //Traverse the robotRoot's tree and construct a tao tree structure
    //************************
    //Step 2a: Find the desired robot root.
    const sutil::CMappedTree<std::string, SRobotLink> & br = arg_robot.robot_br_rep_;
    tmp_root = br.getRootNodeConst();//The root node.
    if (tmp_root == NULL)
    { throw(std::runtime_error("Robot doesn't have valid root node"));}
    if(false == tmp_root->is_root_)
    { throw(std::runtime_error(
        "Robot's branching representation returns root with SRobotLink::is_root_==false."));}

    //Step 2b: Create the corresponding root tao node
    //Root node's home frame translation and orientation
    deFrame tmp_root_frame;
    tmp_root_frame.translation().set(tmp_root->pos_in_parent_[0],
        tmp_root->pos_in_parent_[1],
        tmp_root->pos_in_parent_[2]);

    tmp_root_frame.rotation().set(tmp_root->ori_parent_quat_.x(),
            tmp_root->ori_parent_quat_.y(),
            tmp_root->ori_parent_quat_.z(),
            tmp_root->ori_parent_quat_.w());

    ret_tao_root = new taoNodeRoot(tmp_root_frame); //Define a new tao root node to return
    if (ret_tao_root == NULL)
    { throw(std::runtime_error("Can't create tao root node"));}

    ret_tao_root->setIsFixed(tmp_root->link_is_fixed_); // Is the node fixed or not
    ret_tao_root->setID(tmp_root->link_id_); //NOTE TODO What about the multiple robot case? Will all have rootId = -1
    ret_tao_root->name_ = tmp_root->name_;

    //Step 2c:
    flag = createChildTaoNodes(tmp_root, (taoNode*) ret_tao_root);
    if(false==flag)
    { throw(std::runtime_error("Can't create child nodes")); }

    //*******Step 3***********
    //Initialize TaoDynamics
    //************************
    taoDynamics::initialize(ret_tao_root);
  }
  catch(std::exception& e)
  {
    std::cout<<"\nCTaoRepCreator::taoRootRepCreator() : "<<e.what();
    delete ret_tao_root;
    return S_NULL;
  }
  //Return the root of the tao tree
  return ret_tao_root; //Return the created tao root structure
}

/**This function creates child tao nodes for a given link in the robot's
 * definition. */
bool CTaoRepCreator::createChildTaoNodes(
    const SRobotLink* arg_link, taoNode* arg_parent)
{
  bool flag = true;
	std::vector<SRobotLink*>::const_iterator child_iter, child_iterE;
	//Traverse the list of child links and create a taoNode for each one
	for (child_iter = arg_link->child_addrs_.begin(),
	    child_iterE = arg_link->child_addrs_.end();
	    child_iter != child_iterE;
	    ++child_iter
	    )
	{
	  taoNode* tmp_tao_node;
	  tmp_tao_node = createTaoNonRootNode((*child_iter),
	      (const taoNode*) arg_parent);
	  if(S_NULL == tmp_tao_node)
	  {
	    printf("\nCTaoRepCreator::createChildTaoNodes() : Error. Couldn't create tao node.");
	    return false;
	  }
#ifdef DEBUG
	  else
	  {
	    printf("\nCTaoRepCreator::createChildTaoNodes() : Created Tao Node (%d) %s .",
	      tmp_tao_node->getID(), tmp_tao_node->name_.c_str());
	  }
#endif
	  flag = createChildTaoNodes((*child_iter), tmp_tao_node);
	  if(false == flag)
	  {
	    printf("\nCTaoRepCreator::createChildTaoNodes() : Error. Couldn't create child node(s).");
	    return false;
	  }
	}
	return flag;
}

/**
 * This function initializes a node in the tree being created
 *
 * Returns a taoNode*
 *
 * Sets the following parameters for the
 * new taoNode:
 * 1. Link to its parent
 * 2. Its frame of reference wrt its parent
 * 3. Its physical link properties (mass, inertia etc)
 * 4. The node's joint(s) -- typically each node represents <1 link,1 joint>
 */
taoNode* CTaoRepCreator::
createTaoNonRootNode(const SRobotLink* arg_link,
    const taoNode* arg_parent_node)
{
  taoNode* ret_tao_node = S_NULL; //Create a node (a link in a branching structure)
  taoJoint* tmp_joint = S_NULL; //Create a new joint (to connect it to a parent link).

  try
  {
    if(S_NULL == arg_link) //No node passed. Can't initialize
    {throw(std::runtime_error("Passed an invalid link specification."));}

    if (S_NULL == arg_link->parent_addr_)
    {
      throw(std::runtime_error(
          "Tried to create a tao non-root node with SRobotLink specification that doesn't have a parent."));
    }

    if (true == arg_link->is_root_)
    {
      throw(std::runtime_error(
        "Tried to create a tao non-root node with a root node specification."));
    }

    if (0 > arg_link->link_id_)
    {
      throw(std::runtime_error(
          "Tried to create a tao non-root node with an invalid (negative integer) id."));
    }

    //Non-root Node
    //NOTE : TAO : Home frame = Local frame wrt parent.
    //Home frame translation and orientation
    deFrame tmp_frame;
    tmp_frame.identity();
    tmp_frame.translation().set(arg_link->pos_in_parent_[0],
        arg_link->pos_in_parent_[1],
        arg_link->pos_in_parent_[2]);

    tmp_frame.rotation().set(arg_link->ori_parent_quat_.x(),
        arg_link->ori_parent_quat_.y(),
        arg_link->ori_parent_quat_.z(),
        arg_link->ori_parent_quat_.w());

    ret_tao_node = new taoNode((taoDNode*) arg_parent_node,
        &tmp_frame);

    ret_tao_node->name_ = arg_link->name_;
    ret_tao_node->setID(arg_link->link_id_);

    if(S_NULL == ret_tao_node)
    { throw(std::runtime_error("Couldn't create a new tao node."));  }

    //2. Set up COM frame of reference WRT the home frame
    //Only translates to the COM. The rotation is the same
    //as the home frame.
    tmp_frame.identity();
    tmp_frame.translation().set(arg_link->com_[0],
        arg_link->com_[1],arg_link->com_[2]);

    //3. Set up the node's physical link properties
    deMassProp tmp_m; //Init mass, and set it in the new tao child node
    tmp_m.inertia(arg_link->inertia_[0],
        arg_link->inertia_[1], arg_link->inertia_[2],
        &tmp_frame);
    tmp_m.mass(arg_link->mass_, &tmp_frame);
    tmp_m.get(ret_tao_node->mass(), ret_tao_node->center(),
        ret_tao_node->inertia());

    // Create new joint for the new child node
    switch(arg_link->joint_type_)
    {
    case JOINT_TYPE_PRISMATIC_X:
      tmp_joint = new taoJointPrismatic(TAO_AXIS_X);
      tmp_joint->setDVar(new taoVarDOF1);
      break;
    case JOINT_TYPE_PRISMATIC_Y:
      tmp_joint = new taoJointPrismatic(TAO_AXIS_Y);
      tmp_joint->setDVar(new taoVarDOF1);
      break;
    case JOINT_TYPE_PRISMATIC_Z:
      tmp_joint = new taoJointPrismatic(TAO_AXIS_Z);
      tmp_joint->setDVar(new taoVarDOF1);
      break;
    case JOINT_TYPE_REVOLUTE_X:
      tmp_joint = new taoJointRevolute(TAO_AXIS_X);
      tmp_joint->setDVar(new taoVarDOF1);
      break;
    case JOINT_TYPE_REVOLUTE_Y:
      tmp_joint = new taoJointRevolute(TAO_AXIS_Y);
      tmp_joint->setDVar(new taoVarDOF1);
      break;
    case JOINT_TYPE_REVOLUTE_Z:
      tmp_joint = new taoJointRevolute(TAO_AXIS_Z);
      tmp_joint->setDVar(new taoVarDOF1);
      break;
    case JOINT_TYPE_SPHERICAL:
      tmp_joint = new taoJointSpherical();
      tmp_joint->setDVar(new taoVarSpherical);
      break;
    default: break; //JOINT_TYPE_NOTASSIGNED
    }


    if(S_NULL == tmp_joint)
    { throw(std::runtime_error("Couldn't initialize tao joint (Maybe an unrecognized joint type specification)."));  }

    tmp_joint->name_ = arg_link->joint_name_;

    tmp_joint->reset(); //Reset the joint before it is added to the link
    tmp_joint->setDamping(0.0);
    //tmp_joint->setInertia(0.0);//NOTE TODO : WHY??? This seems wrong.

    ret_tao_node->addJoint(tmp_joint);// Add joint to new_child_node

    ret_tao_node->addABNode(); //Initialize geometry links
  }
  catch(std::exception& e)
  {
    std::cout<<"\nCTaoRepCreator::createTaoNonRootNode("
    <<arg_link->robot_name_<<":"<<arg_link->name_<<") : "<<e.what();
    delete ret_tao_node;
    delete tmp_joint;
    return S_NULL;
  }
  return ret_tao_node;
}

}
