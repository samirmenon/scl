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
  CTaoDynamics::CTaoDynamics()
  : robot_name_("(no robot yet)"),
    tao_tree_q_root_(NULL), tao_tree_q_dq_root_(NULL),
    model_(S_NULL),
    ndof_(0)
  {
  }

  CTaoDynamics::~CTaoDynamics()
  {
    if(S_NULL!=model_)
    { delete model_;  }
  }

  static void tao_collect_ids(taoDNode * node, std::map<int, int> & id_counter)
  {
    int const id(node->getID());
    std::map<int, int>::iterator idc(id_counter.find(id));
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
    std::map<int, int> id_counter;
    for (taoDNode * node(root->getDChild()); 0 != node; node = node->getDSibling()) {
      tao_collect_ids(node, id_counter);
    }
    int expected_id(0);
    //NOTE TODO: XXXX  : Is this really necessary?
    for (std::map<int, int>::const_iterator idc(id_counter.begin()); idc != id_counter.end(); ++idc, ++expected_id) {
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
    try
    {
      has_been_init_ = false;
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
      tao_tree_q_root_ = scl::CTaoRepCreator::taoRootRepCreator(arg_robot_data);
      if ( ! tao_tree_q_root_) {
        std::cout << "scl::CTaoDynamics::init(`" << robot_name_
            << "'): scl::CTaoRepCreator::taoRootRepCreator() failed [invalid robot name?]\n";
        return false;
      }

      if ( ! tao_consistency_check(tao_tree_q_root_)) {
        std::cout << "scl::CTaoDynamics::init(`" << robot_name_
            << "'): consistency check failed on TAO tree\n";
        return false;
      }

      // Parse it again to get a second copy. Extra cost only incurred
      // at init time, so that should be acceptable, although it would
      // be cleaner to have TAO tree deep copy functionality somewhere.
      tao_tree_q_dq_root_ = scl::CTaoRepCreator::taoRootRepCreator(arg_robot_data);
      if ( ! tao_tree_q_dq_root_) {
        std::cout << "scl::CTaoDynamics::init(`" << robot_name_
            << "'): scl::CTaoRepCreator::taoRootRepCreator(cc) failed [invalid robot name?]\n";
        return false;
      }

      jspace::STaoTreeInfo * js_q_tree(jspace::create_bare_tao_tree_info(tao_tree_q_root_));
      jspace::STaoTreeInfo * js_q_dq_tree(jspace::create_bare_tao_tree_info(tao_tree_q_dq_root_));

      //NOTE TODO Perhaps this was what TRY_TO_CONVERT_NAMES achieved
      sutil::CMappedTree<std::basic_string<char>, scl::SRigidBody>::const_iterator itbr, itbre;
      for(itbr = arg_robot_data.robot_br_rep_.begin(),
          itbre = arg_robot_data.robot_br_rep_.end();
          itbr!=itbre; ++itbr)
      {
        std::vector<jspace::STaoNodeInfo>::iterator it, ite, icc;

        //Name the stuff in the kgm tree
        it = js_q_tree->info.begin();
        icc = js_q_dq_tree->info.begin();
        ite = js_q_tree->info.end();
        for (/**/; it != ite; ++it, ++icc)
        {
          const SRigidBody& l_ds = *itbr;
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
      model_->init(js_q_tree, js_q_dq_tree, gravity_(0), gravity_(1), gravity_(2), 0);
      ndof_ = model_->getNDOF();
      state_.init(ndof_, ndof_, 0);

      has_been_init_ = true;
    }
    catch(std::exception& e)
    { std::cerr<<"\nCTaoDynamics::init() : Error : "<<e.what();  }

    return has_been_init_;
  }


  bool CTaoDynamics::
  computeGCModel(SRobotSensorData const * arg_sensor_data,
      SGcModel * arg_gc_model)
  {
    if ( ! arg_sensor_data) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): sensor_data is NULL\n");
      return false;
    }
    if ( ! arg_gc_model) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): arg_gc_model is NULL\n");
      return false;
    }
    if ( ! model_) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): not initialized\n");
      return false;
    }

    // NOTE : If you get a segfault here, you probably need to finish
    // implementing/using registerTaoDynamics() in DbRegisterFunctions.cpp

    // Set generalized coordinates
    state_.position_ = arg_sensor_data->q_;
    size_t const npos(arg_sensor_data->q_.rows());
    arg_gc_model->q_  = arg_sensor_data->q_;

    // Set generalized velocities.
    state_.velocity_ = arg_sensor_data->dq_;
    arg_gc_model->dq_  = arg_sensor_data->dq_;

    // No external forces on the system
    state_.force_.setZero(npos);

    // Update the model based on the system's state.
    model_->update(state_);

    if ( ! model_->getMassInertia(arg_gc_model->A_)) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): model_->getMassInertia() failed\n");
      return false;
    }
    if ( ! model_->getInverseMassInertia(arg_gc_model->Ainv_)) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): model_->getInverseMassInertia() failed\n");
      return false;
    }
    if ( ! model_->getCoriolisCentrifugal(arg_gc_model->b_)) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): model_->getCoriolisCentrifugal() failed\n");
      return false;
    }
    if ( ! model_->getGravity(arg_gc_model->g_)) {
      fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): model_->getGravity() failed\n");
      return false;
    }

    bool flag;
    arg_gc_model->pos_com_.setZero(3);
    Eigen::Vector3d tmp_lnk_com;
    sutil::CMappedTree<std::string, SRigidBodyDyn>::iterator it, ite;
    for(it = arg_gc_model->link_ds_.begin(), ite = arg_gc_model->link_ds_.end(); it!=ite;++it)
    {
      // No matrices need to be computed for the root nodes (those are fixed, non-dynamic).
      if(it->link_ds_->is_root_)
      { continue; }

      if(NULL == it->link_dynamic_id_)
      {
        fprintf(stdout, "scl::CTaoDynamics::updateModelMatrices(): Com link dynamic id not initialized for '%s'. Binding to tao dynamics.\n", it->name_.c_str());
        it->link_dynamic_id_ = getIdForLink(it->name_);
        if(NULL == it->link_dynamic_id_)
        {
          fprintf(stdout, "scl::CTaoDynamics::updateModelMatrices(): Error : Could not generate Tao dynamics ID for '%s'.\n");
          return false;
        }
      }
      flag = computeTransform_Depracated(it->link_dynamic_id_,it->T_o_lnk_);
      if(false == flag) {
        fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): Error : Com transformation matrix computation failed\n");
        return false;
      }

      tmp_lnk_com = it->T_o_lnk_ * it->link_ds_->com_;
      tmp_lnk_com *= it->link_ds_->mass_;
      arg_gc_model->pos_com_ += tmp_lnk_com;
      if ( ! computeJacobian_Depracated(it->link_dynamic_id_, tmp_lnk_com ,it->J_com_)) {
        fprintf(stderr, "scl::CTaoDynamics::updateModelMatrices(): Error : Could not compute the com Jacobian at link : %s\n",it->name_.c_str());
        return false;
      }
    }

    arg_gc_model->pos_com_ = arg_gc_model->pos_com_.array() / arg_gc_model->mass_;

    return true;
  }


  const void* CTaoDynamics::getIdForLink(std::string arg_link_name)
  {
    taoDNode * tmp = model_->getNodeByName(arg_link_name);
    return static_cast<const void*>(tmp);
  }

  sBool CTaoDynamics::computeTransform_Depracated(
      const void* arg_link_id,
      Eigen::Affine3d& arg_T)
  {
    taoDNode * tao_node;
    tao_node = static_cast<taoDNode *>(const_cast<void*>(arg_link_id));

    if (NULL == tao_node)
    { return false; }

    // Update the link frame in tao.
    tao_node->updateFrame();

    deFrame const * tao_frame(tao_node->frameGlobal());
    deQuaternion const & tao_quat(tao_frame->rotation());
    deVector3 const & tao_trans(tao_frame->translation());

    // Initialize the affine tranform as a pure translation.
    arg_T = Eigen::Translation3d(tao_trans[0], tao_trans[1], tao_trans[2]);

    // Then multiply the rotation part.
    // Note: Eigen::Quaternion(w, x, y, z) asks for w first, (but stores it as xyzw)
    // deQuaternion(qx, qy, qz, qw) asks for w last and stores it as xyzw.
    arg_T *= Eigen::Quaternion<double>(/*w*/tao_quat[3/*w*/], /*x*/tao_quat[0], /*y*/tao_quat[1], /*z*/tao_quat[2]);

    return true;
  }

  sBool CTaoDynamics::computeJacobian_Depracated(
      const void* arg_link_id,
      const Eigen::VectorXd& arg_pos_global,
      Eigen::MatrixXd& arg_J)
  {
    taoDNode * tmp_node;
    tmp_node = static_cast<taoDNode *>(const_cast<void*>(arg_link_id));

    if (NULL == tmp_node) { return false;  }

    // Zero the Jacobian
    arg_J = Eigen::MatrixXd::Zero(6, ndof_);

    //Update all the transformation matrices etc.
    taoDynamics::updateTransformation(tao_tree_q_root_);
    taoDynamics::globalJacobian(tao_tree_q_root_);

    deVector6 tmp_J_col; //Compute the Jacobian col by col.
    while(false == tmp_node->isRoot())
    {//Iterate over all nodes
      // Get the joint
      taoJoint* tmp_j = tmp_node->getJointList();
      while(NULL!=tmp_j)
      {//Iterate over all joints, get a J column for each
        tmp_j->getJgColumns(&tmp_J_col);

        // NOTE TODO : Need a neater way to get this joint's id
        int col_id = tmp_node->getID();

        // Fill in the col of the Jacobian
        for (int irow=0; irow < 6; ++irow)
        { arg_J(irow, col_id) = tmp_J_col.elementAt(irow); }

        // NOTE TODO: v_op = v_trans + v_rot. But v_rot isn't accounted for in tao. Why???
        // So, d(v_glob)/dqi += d(ω_joint)/dqi x r_glob
        // dx = J * dq
        // J_col_i = [dx/dqi dy/dqi dz/dqi dωx/dqi dωy/dqi dωz/dqi]'
        const Eigen::VectorXd &r = arg_pos_global;
        double wx = tmp_J_col.elementAt(3), wy = tmp_J_col.elementAt(4), wz = tmp_J_col.elementAt(5);
        // Cross prod: wx/qi ry K - wx/qi rz J - wy/qi rx K + wy/qi rz I + wz/qi rx J - wz/qi ry I
        //           = wy/qi rz I - wz/qi ry I + wz/qi rx J - wx/qi rz J + wx/qi ry K - wy/qi rx K
        arg_J(0, col_id) += wy * r(2) - wz * r(1) ;
        arg_J(1, col_id) += wz * r(0) - wx * r(2) ;
        arg_J(2, col_id) += wx * r(1) - wy * r(0);

        // NOTE TODO : Fix this later. Should loop over all joint dofs
        break;
        tmp_j = tmp_j->getNext(); //Move to next joint. Terminate when over.
      }
      tmp_node = tmp_node->getDParent(); //Move to next parent. Terminate at root.
    }
    return true;
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
    jspace::STaoTreeInfo * tao_tree = model_->_getCCTree();

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

  /**
   * Gets the robot's kinetic energy
   */
  sFloat CTaoDynamics::getKineticEnergy()
  {
    sFloat ke = taoDynamics::kineticEnergy(tao_tree_q_dq_root_);
    return ke;
  }

  /**
   * Gets the robot's potential energy
   */
  sFloat CTaoDynamics::getPotentialEnergy()
  {
    if (gravity_.norm()<0.0001)
    { return 0; }

    deVector3 gravity(gravity_(0),gravity_(1),gravity_(2));
    sFloat pe = taoDynamics::potentialEnergy(tao_tree_q_dq_root_, &gravity);
    return pe;
  }

  /** TODO : This dynamics engine implementation should be stateless.
   * This present system is because the intermediate jspace implementation
   * has an extra data layer. Should be cleaned up in the future. */
  sBool CTaoDynamics::setGeneralizedCoordinates(Eigen::VectorXd &arg_q)
  {
    if(false == has_been_init_){return false; }

    //We will integrate the kgm tree (arbit. We could integrate the kgm tree as well).
    jspace::STaoTreeInfo * tao_tree = model_->_getKGMTree();

    //Uses the cc tree to integrate the dynamics.
    for (size_t ii(0); ii < ndof_; ++ii)
    {
      sInt io_ds_idx;
      io_ds_idx = tao_tree->info[ii].node->getID();

      sFloat q;
      q = arg_q(io_ds_idx);

      taoJoint * joint(tao_tree->info[ii].node->getJointList());
      joint->setQ(&q);
    }

    //We will integrate the cc tree (arbit. We could integrate the kgm tree as well).
    jspace::STaoTreeInfo * tao_tree_cc = model_->_getCCTree();

    //Uses the cc tree to integrate the dynamics.
    for (size_t ii(0); ii < ndof_; ++ii)
    {
      sInt io_ds_idx;
      io_ds_idx = tao_tree_cc->info[ii].node->getID();

      sFloat q;
      q = arg_q(io_ds_idx);

      taoJoint * joint(tao_tree_cc->info[ii].node->getJointList());
      joint->setQ(&q);
    }

    return true;
  }

  /** TODO : This dynamics engine implementation should be stateless.
   * This present system is because the intermediate jspace implementation
   * has an extra data layer. Should be cleaned up in the future. */
  sBool CTaoDynamics::setGeneralizedVelocities(Eigen::VectorXd &arg_dq)
  {
    if(false == has_been_init_){return false; }

    //We will integrate the cc tree (arbit. We could integrate the kgm tree as well).
    jspace::STaoTreeInfo * tao_tree = model_->_getKGMTree();

    //Uses the cc tree to integrate the dynamics.
    for (size_t ii(0); ii < ndof_; ++ii)
    {
      sInt io_ds_idx;
      io_ds_idx = tao_tree->info[ii].node->getID();

      sFloat dq;
      dq = arg_dq(io_ds_idx);

      taoJoint * joint(tao_tree->info[ii].node->getJointList());
      joint->setDQ(&dq);
    }

    //We will integrate the cc tree (arbit. We could integrate the kgm tree as well).
    jspace::STaoTreeInfo * tao_tree_cc = model_->_getCCTree();

    //Uses the cc tree to integrate the dynamics.
    for (size_t ii(0); ii < ndof_; ++ii)
    {
      sInt io_ds_idx;
      io_ds_idx = tao_tree_cc->info[ii].node->getID();

      sFloat dq;
      dq = arg_dq(io_ds_idx);

      taoJoint * joint(tao_tree_cc->info[ii].node->getJointList());
      joint->setDQ(&dq);
    }
    return true;
  }

  /** Sets the external generalized forces
   *
   * TODO : This dynamics engine implementation should be stateless.
   * This present system is because the intermediate jspace implementation
   * has an extra data layer. Should be cleaned up in the future. */
  sBool CTaoDynamics::setGeneralizedForces(Eigen::VectorXd &arg_fgc)
  {
    if(false == has_been_init_){return false; }

    //We will integrate the cc tree (arbit. We could integrate the kgm tree as well).
    jspace::STaoTreeInfo * tao_tree = model_->_getKGMTree();

    //Uses the cc tree to integrate the dynamics.
    for (size_t ii(0); ii < ndof_; ++ii)
    {
      sInt io_ds_idx;
      io_ds_idx = tao_tree->info[ii].node->getID();

      sFloat tau;
      tau = arg_fgc(io_ds_idx);

      taoJoint * joint(tao_tree->info[ii].node->getJointList());
      joint->setTau(&tau);//Set the joint torques to be applied.
    }

    //We will integrate the cc tree (arbit. We could integrate the kgm tree as well).
    jspace::STaoTreeInfo * tao_tree_cc = model_->_getCCTree();

    //Uses the cc tree to integrate the dynamics.
    for (size_t ii(0); ii < ndof_; ++ii)
    {
      sInt io_ds_idx;
      io_ds_idx = tao_tree_cc->info[ii].node->getID();

      sFloat tau;
      tau = arg_fgc(io_ds_idx);

      taoJoint * joint(tao_tree_cc->info[ii].node->getJointList());
      joint->setTau(&tau);//Set the joint torques to be applied.
    }
    return true;
  }

}
