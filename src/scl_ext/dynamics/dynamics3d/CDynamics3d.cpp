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
/* \file CDynamics3d.cpp
 *
 *  Created on: Jun 21, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 *  Edited by: Kenji Hata <khata@stanford.edu>
 */
#include "CDynamics3d.hpp"

#include <scl/data_structs/SRobotIO.hpp>
#include <scl/data_structs/SGcModel.hpp>

#include <scl_ext/dynamics/dynamics3d/CRepCreator3d.hpp>

#include <chai3d.h>
// Include the dynamics 3d library (if present).
#include <dynamics3d.h>

#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <math.h>

using namespace scl;


namespace scl_ext
{

  CDynamics3d::CDynamics3d()
  : robot_name_("(no robot yet)"),
    ndof_(0)
  {  c_base = NULL;}


  CDynamics3d::
  ~CDynamics3d()
  {  }

  typedef std::map<int, int> id_counter_t;

  static void c3d_collect_ids(cDynObject * node, id_counter_t & id_counter)
  {
    int const id(node->scl_id);
    id_counter_t::iterator idc(id_counter.find(id));
    if (id_counter.end() == idc) {
      id_counter.insert(std::make_pair(id, 1));
    }
    else {
      ++idc->second;
    }
    for (cDynObject * child(node->child()); 0 != child; child = child->sibling()) {
      c3d_collect_ids(child, id_counter);
    }
  }

  static bool c3d_consistency_check(cDynamicBase * base)
  {
    cDynObject *root = base->m_dynBaseObject;
    /*
      id_counter_t id_counter;
      for (cDynObject * node(root->child()); 0 != node; node = node->sibling()) {
        c3d_collect_ids(node, id_counter);
      }
      int expected_id(0);
      for (id_counter_t::const_iterator idc(id_counter.begin()); idc != id_counter.end(); ++idc, ++expected_id) {
        if (false ){//idc->first != expected_id) {
          std::cout << "scl::3d_consistency_check(): ID gap, expected "
              << expected_id << " but encountered " << idc->first << "\n";
          return false;
        }
        if (1 != idc->second) {
          std::cout << "scl::3d_consistency_check(): duplicate ID " << idc->first << "\n";
          return false;
        }
      }*/
    return true;
  }

  bool CDynamics3d::init(const SRobotParsed& arg_robot_data)
  {
    if (c_base){
      fprintf(stderr, "scl::CDynamics3d::init(): already initialized\n");
      return false;
    }
    if(false == arg_robot_data.has_been_init_){
      fprintf(stderr, "scl::CDynamics3d::init(): Passed an uninitialized robot\n");
      return false;
    }

    //Set all the robot parameters
    robot_name_ = arg_robot_data.name_;
    gravity_ = arg_robot_data.gravity_;

    // Initialize the 3d
    cDynamicBase * c3d_base(CRepCreator3d::c3dRootRepCreator(arg_robot_data));
    if (!c3d_base) {
      std::cout << "scl::CDynamics3d::init(`" << robot_name_
          << "'): scl::CRepCreator3d::chai3dRootRepCreator() failed [invalid robot name?]\n";
      return false;
    }
#ifdef DEBUG
    else
    {
      std::cout << "\nscl::CDynamics3d::init(`" << robot_name_
          << "'): scl::CRepCreator3d::c3dRootRepCreator() created a cDynBase*";
    }
    std::cout<<std::flush;
#endif
    if ( ! c3d_consistency_check(c3d_base)) {
      std::cout << "scl::CDynamics3d::init(`" << robot_name_
          << "'): consistency check failed on c3d root \n";
      return false;
    }
    c3d_base->m_dynamicWorld->setGravity((double)gravity_(0), (double)gravity_(1), (double)gravity_(2));

    //NOTE TODO Perhaps this was what TRY_TO_CONVERT_NAMES achieved
    sutil::CMappedTree<std::basic_string<char>, scl::SRigidBody>::const_iterator itbr, itbre;
    for(itbr = arg_robot_data.rb_tree_.begin(),
        itbre = arg_robot_data.rb_tree_.end();
        itbr!=itbre; ++itbr)
    {
      std::vector<cDynamicLink*>::iterator it, ite;
      it = c3d_base->m_dynamicLinks.begin();
      ite = c3d_base->m_dynamicLinks.end();

      for (/**/; it != ite; ++it)
      {
        const SRigidBody& l_ds = *itbr;
        if (l_ds.name_ == (*it)->m_dynObject->name_)
        {
          (*it)->name_   = itbr->name_;
          if((*it)->getJoint(0)){
            (*it)->getJoint(0)->name_  = itbr->joint_name_;
            (*it)->getJoint(0)->setJointLimits(itbr->joint_limit_lower_, itbr->joint_limit_upper_, 0.0001);
          }
          break;
        }
      }
    }

    // manually calculates ndof (3 dof per spherical, 1 per prismatic/revolute)
    ndof_ = 0;
    for(size_t i(0); i<c3d_base->m_dynamicJoints.size(); ++i){
      if(c3d_base->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL)
        ndof_+=3;
      else
        ndof_++;
    }
    c_base = c3d_base;
    has_been_init_ = true;



    return true;
  }

  const void* CDynamics3d::getIdForLink(std::string arg_link_name)
  {
    for(size_t ii(0); ii<c_base->m_dynamicLinks.size(); ++ii){
      if(arg_link_name == c_base->m_dynamicLinks[ii]->name_){
        return c_base->m_dynamicLinks[ii]->m_dynObject;
      }
    }
    return 0;
  }

  sBool CDynamics3d::integrate(SRobotIO& arg_inputs, const sFloat arg_time_interval)
  {
    //1. Integrate the dynamics.

    //2. Return the integrated state by copying it into the IO data struct
    for (size_t ii(0); ii < c_base->m_dynamicJoints.size(); ++ii)
    {
      cDynJoint *cj = c_base->m_dynamicJoints[ii]->m_dynJoint;
      sInt io_ds_idx = cj->object()->scl_id;
      /*if (cj->type() == CDYN_SPHERICAL){
			cDynQuaternion q = cDynQuaternion(arg_inputs.sensors_.q2_(io_ds_idx), arg_inputs.sensors_.q3_(io_ds_idx),
											arg_inputs.sensors_.q4_(io_ds_idx),arg_inputs.sensors_.q_(io_ds_idx));
			cDynVector3 dq = cDynVector3(arg_inputs.sensors_.dq_(io_ds_idx), arg_inputs.sensors_.dq2_(io_ds_idx), arg_inputs.sensors_.dq3_(io_ds_idx));
			cDynVector3 ddq = cDynVector3(0,0,0);

			// q's init zeroes out the quat, which bugs out the dynamics engine.
			if(q[0]!=0 || q[1]!=0 || q[2]!=0 || q[3]!=0)
				cj->sq(q);
			cj->sv(dq);
			cj->sa(ddq);

		}else{*/
      cj->q((double)arg_inputs.sensors_.q_(io_ds_idx));
      cj->v((double)arg_inputs.sensors_.dq_(io_ds_idx));
      cj->a(0);
      cj->torque((double) arg_inputs.actuators_.force_gc_commanded_(io_ds_idx));
      //}
    }

    // integrates the dynamics
    sFloat tstep = arg_time_interval;
    c_base->m_dynamicWorld->computeGlobalPositions(true);
    c_base->m_dynamicWorld->updateDynamics(tstep);
    /*cDynamicContactList* cl = c_base->m_dynamicContacts;
	for(size_t i(0); i<cl->getNumContacts(); ++i){
		std::cout<<cl->getContact(i)->m_dynamicLink->name_ <<std::endl;
	}
	std::cout<<"\n"<<std::endl;*/

    // pulls the information back into SCL
    for (size_t ii(0); ii < c_base->m_dynamicJoints.size(); ++ii)
    {
      cDynJoint *cj = c_base->m_dynamicJoints[ii]->m_dynJoint;
      sInt io_ds_idx = cj->object()->scl_id;
      /*if (cj->type() == CDYN_SPHERICAL){
			double w = cj->sq()[3];
			double x = cj->sq()[0];
			double y = cj->sq()[1];
			double z = cj->sq()[2];
			arg_inputs.sensors_.q_(io_ds_idx) = w;
			arg_inputs.sensors_.q2_(io_ds_idx) = x;
			arg_inputs.sensors_.q3_(io_ds_idx) = y;
			arg_inputs.sensors_.q4_(io_ds_idx) = z;
			arg_inputs.sensors_.dq_(io_ds_idx) = cj->sv()[0];
			arg_inputs.sensors_.dq2_(io_ds_idx) = cj->sv()[1];
			arg_inputs.sensors_.dq3_(io_ds_idx) = cj->sv()[2];
			arg_inputs.sensors_.ddq_(io_ds_idx) = cj->sa()[0];
			arg_inputs.sensors_.ddq_(io_ds_idx) = cj->sa()[1];
			arg_inputs.sensors_.ddq_(io_ds_idx) = cj->sa()[2];
			//arg_inputs.sensors_.force_gc_measured_(io_ds_idx) = cj->torqueSpherical().magnitude();

		}else{*/
      arg_inputs.sensors_.q_(io_ds_idx) = cj->q();
      arg_inputs.sensors_.dq_(io_ds_idx) = cj->v();
      arg_inputs.sensors_.ddq_(io_ds_idx) = cj->a();
      arg_inputs.sensors_.force_gc_measured_(io_ds_idx) = cj->torque();
      //	}
    }



    return true;
  }

  /**
   * Gets the robot's kinetic energy
   */

  sFloat CDynamics3d::computeEnergyKinetic(sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree, const Eigen::VectorXd& arg_q, const Eigen::VectorXd& arg_dq)
  {
    cDynObject *root = c_base->m_dynBaseObject;
    return cDynamicsKineticEnergy(root);
  }

  /**
   * Gets the robot's potential energy
   */
  sFloat CDynamics3d::computeEnergyPotential(sutil::CMappedTree<std::string, SRigidBodyDyn> &arg_tree, const Eigen::VectorXd& arg_q)
  {
    //TODO: Fix me
    cDynObject *root = c_base->m_dynBaseObject;
    cDynVector3 gravity = cDynVector3(gravity_[0], gravity_[1], gravity_[2]);
    return cDynamicsPotentialEnergy(root, &gravity);
  }
}
