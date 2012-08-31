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
/* \file CTaoDynamics.cpp
 *
 *  Created on: Aug 23, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CTaoDynamics.hpp"

#include <scl/data_structs/SRobotIOData.hpp>

#include <scl/dynamics/tao/CTaoRepCreator.hpp>
#include <scl/dynamics/tao/jspace/tao_util.hpp>
#include <scl/dynamics/tao/jspace/Model.hpp>
#include <scl/control/data_structs/SGcModel.hpp>

#include <scl/dynamics/tao/tao/dynamics/taoNode.h>
#include <scl/dynamics/tao/tao/dynamics/taoJoint.h>
#include <scl/dynamics/tao/tao/dynamics/taoDynamics.h>

#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>

#ifdef W_QNX
extern "C" {
#include <string.h> // for qnx
}
#endif

namespace scl
{

  CTaoDynamics::
  CTaoDynamics()
  : robot_name_("(no robot yet)"),
    model_(S_NULL),
    ndof_(0)
  {
  }


  CTaoDynamics::
  ~CTaoDynamics()
  {
    if(S_NULL!=model_)
    { delete model_;  }
  }


  typedef std::map<int, int> id_counter_t;

  static void tao_collect_ids(taoDNode * node, id_counter_t & id_counter)
  {
    int const id(node->getID());
    id_counter_t::iterator idc(id_counter.find(id));
    if (id_counter.end() == idc) {
      id_counter.insert(std::make_pair(id, 1));
    }
    else {
      ++idc->second;
    }
    for (taoDNode * child(node->getDChild()); 0 != child; child = child->getDSibling()) {
      tao_collect_ids(child, id_counter);
    }
  }


  static bool tao_consistency_check(taoNodeRoot * root)
  {
    if (root->getID() != -1) {
      std::cout << "scl::tao_consistency_check(): root has ID " << root->getID() << " instead of -1\n";
      return false;
    }
    id_counter_t id_counter;
    for (taoDNode * node(root->getDChild()); 0 != node; node = node->getDSibling()) {
      tao_collect_ids(node, id_counter);
    }
    int expected_id(0);
    //NOTE TODO: XXXX  : Is this really necessary?
    for (id_counter_t::const_iterator idc(id_counter.begin()); idc != id_counter.end(); ++idc, ++expected_id) {
      if (idc->first != expected_id) {
        std::cout << "scl::tao_consistency_check(): ID gap, expected "
            << expected_id << " but encountered " << idc->first << "\n";
        return false;
      }
      if (1 != idc->second) {
        std::cout << "scl::tao_consistency_check(): duplicate ID " << idc->first << "\n";
        return false;
      }
    }
    return true;
  }


  bool CTaoDynamics::
  init(const SRobotParsedData& arg_robot_data)
  {
    if (model_) {
      fprintf(stderr, "scl::CTaoDynamics::init(): already initialized\n");
      return false;
    }

    if(false == arg_robot_data.has_been_init_){
      fprintf(stderr, "scl::CTaoDynamics::init(): Passed an uninitialized robot\n");
      return false;
    }

    //Set all the robot parameters
    robot_name_ = arg_robot_data.name_;
    gravity_ = arg_robot_data.gravity_;

    //Initialize the tao tree
    taoNodeRoot * kgm_root(scl::CTaoRepCreator::taoRootRepCreator(arg_robot_data));
    if ( ! kgm_root) {
      std::cout << "scl::CTaoDynamics::init(`" << robot_name_
          << "'): scl::CTaoRepCreator::taoRootRepCreator() failed [invalid robot name?]\n";
      return false;
    }
#ifdef W_TESTING
    else
    {
      std::cout << "\nscl::CTaoDynamics::init(`" << robot_name_
          << "'): scl::CTaoRepCreator::taoRootRepCreator() created a kgm tree";
    }
    std::cout<<std::flush;
#endif

    if ( ! tao_consistency_check(kgm_root)) {
      std::cout << "scl::CTaoDynamics::init(`" << robot_name_
          << "'): consistency check failed on TAO tree\n";
      return false;
    }

    // Parse it again to get a second copy. Extra cost only incurred
    // at init time, so that should be acceptable, although it would
    // be cleaner to have TAO tree deep copy functionality somewhere.
    taoNodeRoot * cc_root;
    cc_root = scl::CTaoRepCreator::taoRootRepCreator(arg_robot_data);
    if ( ! cc_root) {
      std::cout << "scl::CTaoDynamics::init(`" << robot_name_
          << "'): scl::CTaoRepCreator::taoRootRepCreator(cc) failed [invalid robot name?]\n";
      return false;
    }
#ifdef W_TESTING
    else
    {
      std::cout << "\nscl::CTaoDynamics::init(`" << robot_name_
          << "'): scl::CTaoRepCreator::taoRootRepCreator() created a cc tree";
    }
    std::cout<<std::flush;
#endif

    jspace::tao_tree_info_s * kgm_tree(jspace::create_bare_tao_tree_info(kgm_root));
    jspace::tao_tree_info_s * cc_tree(jspace::create_bare_tao_tree_info(cc_root));

    //NOTE TODO Perhaps this was what TRY_TO_CONVERT_NAMES achieved
    sutil::CMappedTree<std::basic_string<char>, scl::SRobotLink>::const_iterator itbr, itbre;
    for(itbr = arg_robot_data.robot_br_rep_.begin(),
        itbre = arg_robot_data.robot_br_rep_.end();
        itbr!=itbre; ++itbr)
    {
      std::vector<jspace::STaoNodeInfo>::iterator it, ite, icc;

      //Name the stuff in the kgm tree
      it = kgm_tree->info.begin();
      icc = cc_tree->info.begin();
      ite = kgm_tree->info.end();
      for (/**/; it != ite; ++it, ++icc)
      {
        const SRobotLink& l_ds = *itbr;
        if (l_ds.name_ == it->node->name_)
        {
          it->link_name   = itbr->name_;
          it->joint_name  = itbr->joint_name_;
          it->limit_lower = itbr->joint_limit_lower_;
          it->limit_upper = itbr->joint_limit_upper_;
          icc->link_name  = itbr->name_;
          icc->joint_name = itbr->joint_name_;
          icc->limit_lower = itbr->joint_limit_lower_;
          icc->limit_upper = itbr->joint_limit_upper_;
          break;
        }
      }
    }

    model_ = new jspace::Model();
    model_->init(kgm_tree, cc_tree, gravity_(0), gravity_(1), gravity_(2), 0);
    ndof_ = model_->getNDOF();
    state_.init(ndof_, ndof_, 0);

    has_been_init_ = true;
    return true;
  }


  bool CTaoDynamics::
  updateModelMatrices(SRobotSensorData const * sensor_data,
      SGcModel * js_model)
  {
    if ( ! sensor_data) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): sensor_data is NULL\n");
      return false;
    }
    if ( ! js_model) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): js_model is NULL\n");
      return false;
    }
    if ( ! model_) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): not initialized\n");
      return false;
    }

    // NOTE TODO: jspace should probably use Eigen already in
    // jspace::State (instead of std::vector<double>)

    // NOTE : If you get a segfault here, you probably need to finish
    // implementing/using registerTaoDynamics() in DbRegisterFunctions.cpp
    size_t const npos(sensor_data->q_.rows());

    state_.position_.resize(npos);
    memcpy(&state_.position_[0], sensor_data->q_.data(), npos * sizeof(double));
    size_t const nvel(sensor_data->dq_.rows());
    state_.velocity_.resize(nvel);
    memcpy(&state_.velocity_[0], sensor_data->dq_.data(), nvel * sizeof(double));
    state_.force_.setZero(npos);
    model_->update(state_);

    if ( ! model_->getMassInertia(js_model->A_)) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): model_->getMassInertia() failed\n");
      return false;
    }
    if ( ! model_->getInverseMassInertia(js_model->Ainv_)) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): model_->getInverseMassInertia() failed\n");
      return false;
    }
    if ( ! model_->getCoriolisCentrifugal(js_model->b_)) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): model_->getCoriolisCentrifugal() failed\n");
      return false;
    }
    if ( ! model_->getGravity(js_model->g_)) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): model_->getGravity() failed\n");
      return false;
    }

//    std::vector<Eigen::MatrixXd>::iterator it, ite;
//    for(it = js_model->J_com_vec_.begin(), ite = js_model->J_com_vec_.end();
//        it!=ite;++it)
//    {
//      if ( ! calculateJacobian(js_model->b_)) {
//        fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): model_->getCoriolisCentrifugal() failed\n");
//        return false;
//      }
//    }

    return true;
  }


  const void* CTaoDynamics::getIdForLink(std::string arg_link_name)
  {
    taoDNode * tmp = model_->getNodeByName(arg_link_name);
    return static_cast<const void*>(tmp);
  }

  sBool CTaoDynamics::calculateTransformationMatrix(
      const void* arg_link_id,
      Eigen::Affine3d& arg_T)
  {
    const taoDNode * link_addr;
    link_addr = static_cast<const taoDNode *>(arg_link_id);
    bool flag = model_->getGlobalFrame(link_addr,arg_T);
    return flag;
  }

  sBool CTaoDynamics::calculateJacobian(
      const void* arg_link_id,
      const Eigen::VectorXd& arg_pos_global,
      Eigen::MatrixXd& arg_J)
  {
    const taoDNode * link_addr;
    link_addr = static_cast<const taoDNode *>(arg_link_id);
    bool flag = model_->computeJacobian(link_addr,arg_pos_global,arg_J);
    return flag;
  }

  sBool CTaoDynamics::integrate(SRobotIOData& arg_inputs, const sFloat arg_time_interval)
  {
    //NOTE TODO : Implement this for applying external contact forces.

    //1. Compute how the applied forces influence the joint torques.
    // NOTE : This assumes that the supplied Jacobian is valid.
    /*
     * NOTE : The following is experimental. Uncomment to test
    arg_inputs.actuators_.forces_.resetIterator();
    while(S_NULL!= arg_inputs.actuators_.forces_.iterator_)
    {
      scl::SForce* tmp_f = arg_inputs.actuators_.forces_.iterator_->data_;

      arg_inputs.actuators_.force_gc_commanded_ += tmp_f->J_.transpose() * tmp_f->force_;
    }
     */

    //We will integrate the cc tree (arbit. We could integrate the kgm tree as well).
    jspace::tao_tree_info_s * tao_tree = model_->_getCCTree();

    //Uses the cc tree to integrate the dynamics.
    for (size_t ii(0); ii < ndof_; ++ii)
    {
      sInt io_ds_idx;
      io_ds_idx = tao_tree->info[ii].node->getID();

      sFloat q, dq, tau;
      q = arg_inputs.sensors_.q_(io_ds_idx);
      dq = arg_inputs.sensors_.dq_(io_ds_idx);
      tau = arg_inputs.actuators_.force_gc_commanded_(io_ds_idx);

      taoJoint * joint(tao_tree->info[ii].node->getJointList());
      joint->setQ(&q);
      joint->setDQ(&dq);
      joint->zeroDDQ();
      joint->setTau(&tau);//Set the joint torques to be applied.
    }

    //NOTE TODO: Inefficiency due to tao.
    deVector3 gravity;
    gravity[0] = gravity_(0);
    gravity[1] = gravity_(1);
    gravity[2] = gravity_(2);// m/s^2

    sFloat tstep = arg_time_interval;

    taoDynamics::fwdDynamics(tao_tree->root, &gravity);
    taoDynamics::integrate(tao_tree->root, tstep);
    taoDynamics::updateTransformation(tao_tree->root);

    //Uses the cc tree to integrate the dynamics.
    for (size_t ii(0); ii < ndof_; ++ii)
    {
      sFloat q, dq, ddq, tau;
      taoJoint * joint(tao_tree->info[ii].node->getJointList());
      joint->getQ(&q);
      joint->getDQ(&dq);
      joint->getDDQ(&ddq);
      joint->getTau(&tau);

      sInt io_ds_idx;
      io_ds_idx = tao_tree->info[ii].node->getID();
      arg_inputs.sensors_.q_(io_ds_idx) = q;
      arg_inputs.sensors_.dq_(io_ds_idx) = dq;
      arg_inputs.sensors_.ddq_(io_ds_idx) = ddq;
      arg_inputs.sensors_.force_gc_measured_(io_ds_idx) = tau;
    }

    return true;
  }

  /***
   * Gets the robot's kinetic energy
   */
  sFloat CTaoDynamics::getKineticEnergy()
  {
    jspace::tao_tree_info_s * tao_tree = model_->_getCCTree();
    sFloat ke = taoDynamics::kineticEnergy(tao_tree->root);
    return ke;
  }

  /***
   * Gets the robot's potential energy
   */
  sFloat CTaoDynamics::getPotentialEnergy()
  {
    deVector3 gravity(0,0,-9.81);
    jspace::tao_tree_info_s * tao_tree = model_->_getCCTree();
    sFloat pe = taoDynamics::potentialEnergy(tao_tree->root, &gravity);
    return pe;
  }

}
