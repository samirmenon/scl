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
/* \file test_dynamics.cpp
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "test_dynamics.hpp"

#include <sutil/CSystemClock.hpp>

#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CSclParser.hpp>
#include <scl/util/DatabaseUtils.hpp>

//Tao Dynamics
#include <scl/dynamics/tao/CTaoDynamics.hpp>
#include <scl/dynamics/tao/CTaoRepCreator.hpp>
#include <scl/dynamics/analytic/CDynamicsAnalyticRPP.hpp>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <cmath>
#include <stdio.h>
#include <math.h>

#include <Eigen/Dense>

using namespace scl;
using namespace std;

namespace scl_test
{

  /**
   * Tests the performance of the tao dynamics engine:
   *
   * Reads in a toy robot specification and lets it fall under gravity
   * with full dynamics.
   */
  void test_dynamics(int id, const std::string &file_name)
  {
    scl::CTaoDynamics * dynamics = S_NULL;
    scl::sUInt r_id=0;
    bool flag;

    try
    {
      //0. Create vars
      long long i; //Counters
      long long imax; //Counter limits
      sClock t1,t2; //Clocks: pre and post

      //Test database
      scl::SDatabase * db = scl::CDatabase::getData();
      if(S_NULL==db)
      { throw(std::runtime_error("Database not initialized."));  }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized database"<<std::flush;  }

      db->dir_specs_ = scl::CDatabase::getData()->cwd_ + std::string("../../specs/");

      //0. Parse the file for robots
      std::string tmp_infile;
      tmp_infile = scl::CDatabase::getData()->cwd_+ file_name;
      std::cout<<"\nTest Result ("<<r_id++<<")  Opening file : "<<tmp_infile;

      scl_parser::CSclParser tmp_lparser;

      //1 Create robot from a file specification (And register it with the db)
      std::vector<std::string> rob_names;
      flag = tmp_lparser.listRobotsInFile(tmp_infile,rob_names);
      if(false == flag)
      { throw(std::runtime_error("Could not read a list of robots from the file"));  }

      std::string robot_name; robot_name = rob_names.at(0);
      flag = scl_registry::parseRobot(tmp_infile, robot_name, &tmp_lparser);
      if(false == flag)
      { throw(std::runtime_error("Could not register robot with the database"));  }
      else
      {
        std::cout<<"\nTest Result ("<<r_id++<<")  Created a robot "
            <<robot_name<<" on the pile"<<std::flush;
      }

#ifdef DEBUG
      std::cout<<"\nPrinting parsed robot "
          <<db->s_parser_.robots_.at(robot_name)->name_;
      scl_util::printRobotLinkTree(*( db->s_parser_.robots_.at(robot_name)->robot_br_rep_.getRootNode()),0);
#endif

      //Initialize the dynamics computational object
      dynamics = new scl::CTaoDynamics();
      if (S_NULL==dynamics)
      { throw(std::runtime_error("Failed to allocate memory for tao dynamics."));  }

      flag = dynamics->init(* db->s_parser_.robots_.at(robot_name));
      if (false==flag) { throw(std::runtime_error("Failed to initialize tao dynamics."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized tao dynamics for the robot.";  }

      SRobotIOData * io_ds;
      io_ds = scl::CDatabase::getData()->s_io_.io_data_.at(robot_name);
      if(S_NULL == io_ds)
      { throw(std::runtime_error("Could not find the robot's I/O data structure in the database"));  }

      scl::sFloat ke[2], pe[2];
      flag = dynamics->integrate((*io_ds), scl::CDatabase::getData()->sim_dt_); //Need to integrate once to flush the state
      if(false == flag)
      { throw(std::runtime_error("Could not integrate with the dynamics engine"));  }

      ke[0] = dynamics->getKineticEnergy(); //Now you can get the energies
      pe[0] = dynamics->getPotentialEnergy();
      if(0.001 > fabs(pe[0]))
      { throw(std::runtime_error("Zero potential energy at start."));  }

      imax = 200;
      sFloat tstep = 0.0001;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        flag = dynamics->integrate((*io_ds), scl::CDatabase::getData()->sim_dt_);
      }
      t2 = sutil::CSystemClock::getSysTime();

      ke[1] = dynamics->getKineticEnergy(); //Now you can get the energies
      if(0.001 > fabs(ke[1]))
      { throw(std::runtime_error("Zero kinetic energy after dynamics simulation."));  }
      pe[1] = dynamics->getPotentialEnergy();
      if(0.001 > fabs(pe[1]))
      { throw(std::runtime_error("Zero potential energy after dynamics simulation."));  }

      scl::sFloat energy_err = ((ke[1]+pe[1]) - (ke[0]+pe[0]))/(ke[0]+pe[0]);
      std::cout<<"\nTest Result ("<<r_id++<<") Initial Energy : "<<(ke[0]+pe[0])
                             <<". Final Energy : "<<(ke[1]+pe[1])<<". Error : "<<energy_err;
      std::cout<<"\nTest Result ("<<r_id++<<") Total Simulated Time : "<<((double)imax)*tstep <<" sec";
      std::cout<<"\nTest Result ("<<r_id++<<") Simulation Took Time : "<<t2-t1 <<" sec";

      //Now stress test the setup -- Only in release mode.
#ifndef DEBUG
      ke[0] = dynamics->getKineticEnergy(); //Now you can get the energies
      pe[0] = dynamics->getPotentialEnergy();

      imax = 20000;
      tstep = 0.0001;
      t1 = sutil::CSystemClock::getSysTime();
      for(i=0; i<imax; ++i)
      {
        flag = dynamics->integrate((*io_ds), scl::CDatabase::getData()->sim_dt_);
      }
      t2 = sutil::CSystemClock::getSysTime();

      ke[1] = dynamics->getKineticEnergy(); //Now you can get the energies
      pe[1] = dynamics->getPotentialEnergy();
      energy_err = ((ke[1]+pe[1]) - (ke[0]+pe[0]))/(ke[0]+pe[0]);
      std::cout<<"\nTest Result ("<<r_id++<<") Initial Energy : "<<(ke[0]+pe[0])
                                       <<". Final Energy : "<<(ke[1]+pe[1])<<". Error : "<<energy_err;
      std::cout<<"\nTest Result ("<<r_id++<<") Total Simulated Time : "<<((double)imax)*tstep <<" sec";
      std::cout<<"\nTest Result ("<<r_id++<<") Simulation Took Time : "<<t2-t1 <<" sec";
#endif

      //Delete stuff
      if(S_NULL!= dynamics)  {  delete dynamics; dynamics = S_NULL; }

      std::cout<<"\nTest #"<<id<<" : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" : Failed.";

      if(S_NULL!= dynamics)  {  delete dynamics; dynamics = S_NULL; }
    }
  }

  /** Tests the performance of analytical dynamics implementations in scl. */
  void test_dynamics_tao_vs_analytic_rpp(int id)
  {
    scl::CTaoDynamics * dynamics = S_NULL;
    scl::sUInt r_id=0;
    bool flag;

    const double test_precision = 0.00001;

    try
    {
      //0. Create vars
      long long i; //Counters
      long long imax; //Counter limits
      sClock t1,t2; //Clocks: pre and post

      //Test database
      scl::SDatabase * db = scl::CDatabase::getData();
      if(S_NULL==db)
      { throw(std::runtime_error("Database not initialized."));  }
      else
      { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized database"<<std::flush;  }

      db->dir_specs_ = scl::CDatabase::getData()->cwd_ + std::string("../../specs/");

      //0. Parse the file for robots
      std::string tmp_infile;
      tmp_infile = db->dir_specs_ + "Bot-RPP/Bot-RPPCfg.xml";
      std::cout<<"\nTest Result ("<<r_id++<<")  Opening file : "<<tmp_infile;

      scl_parser::CSclParser tmp_lparser;

      //1 Create robot from the file specification (And register it with the db)
      std::string robot_name = "rppbot";
      flag = scl_registry::parseRobot(tmp_infile, robot_name, &tmp_lparser);
      if(false == flag)
      { throw(std::runtime_error("Could not register robot with the database"));  }
      else
      {
        std::cout<<"\nTest Result ("<<r_id++<<")  Created a robot "
            <<robot_name<<" on the pile"<<std::flush;
      }

      // Check the robot was parsed.
      scl::SRobotParsedData *rob_ds = db->s_parser_.robots_.at(robot_name);
      if(NULL == rob_ds)
      { throw(std::runtime_error("Could not find registered robot in the database"));  }

      // IF the robot wasn't sorted, issue a warning and set present order as sorted
      std::vector<std::string> tmp_sort_order;
      flag = rob_ds->robot_br_rep_.sort_get_order(tmp_sort_order);
      if(false == flag)
      {
        std::cout<<"\nWARNING : Robot branching representation is not sorted by default. Sorting. Robot = "<<rob_ds->name_;

        // Get the present node ordering.
        sutil::CMappedTree<std::string, SRigidBody>::const_iterator it,ite;
        for(it = rob_ds->robot_br_rep_.begin(), ite = rob_ds->robot_br_rep_.end();
                  it!=ite; ++it)
        { tmp_sort_order.push_back(it->name_); }

        // Sort it.
        flag = rob_ds->robot_br_rep_.sort(tmp_sort_order);
        if(false == flag)
        { throw(std::runtime_error("Could not sort unsorted robot branching representation."));  }
      }

#ifdef DEBUG
      std::cout<<"\nPrinting parsed robot "<<rob_ds->name_;
      scl_util::printRobotLinkTree( *(rob_ds->robot_br_rep_.getRootNode()), 0);
#endif

      //*********** Create the dynamics computational object *************
      dynamics = new scl::CTaoDynamics();
      if (S_NULL==dynamics)
      { throw(std::runtime_error("Failed to allocate memory for tao dynamics."));  }

      // Initialize the dynamics computational object
      flag = dynamics->init(*rob_ds);
      if (false==flag) { throw(std::runtime_error("Failed to initialize tao dynamics."));  }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Initialized tao dynamics for the robot.";  }

      //*********** Create and intialize the analytic dynamics computational object *************
      scl::CDynamicsAnalyticRPP dyn_anlyt;
      flag = dyn_anlyt.init(*rob_ds);
      if (false==flag) { throw(std::runtime_error("Failed to initialize analytic dynamics."));  }

      SRobotIOData * io_ds;
      io_ds = scl::CDatabase::getData()->s_io_.io_data_.at(robot_name);
      if(S_NULL == io_ds)
      { throw(std::runtime_error("Could not find the robot's I/O data structure in the database"));  }

      //******************* Now test the actual implementation ******************
      io_ds->sensors_.q_.setZero(3);
      io_ds->sensors_.dq_.setZero(3);
      io_ds->sensors_.ddq_.setZero(3);

      dynamics->integrate(*io_ds, 0.001); // Sets the dynamic state. Also sets q to some nonzero value.

      // *********************************************************************************************************
      //                              Set up the robot's dynamics data struct
      // *********************************************************************************************************
      // These are used to compute the full dynamics model later in the code..
      scl::SGcModel rob_gc_model;
      flag = rob_gc_model.init(*rob_ds);
      if(true == flag)
      { throw(std::runtime_error("Could not create a dynamic object for the robot"));  }

      // *********************************************************************************************************
      //                                   Test Transformation Matrices
      // *********************************************************************************************************
      // Set up variables.
      std::string link_name;
      Eigen::Affine3d Ttao, Tanlyt;

      // Loop through all links and get transformations.
      sutil::CMappedTree<std::string, SRigidBody>::const_iterator it,ite;
      for(it = rob_ds->robot_br_rep_.begin(), ite = rob_ds->robot_br_rep_.end();
          it!=ite; ++it)
      {
        // Test : Link 0
        link_name = it->name_;

        // Skip the root node (all matrices are zero).
        if(it->is_root_) { continue; }

        flag = dynamics->calculateTransformationMatrix(dynamics->getIdForLink(link_name),Ttao);
        if (false==flag) { throw(std::runtime_error("Failed to compute tao transformation matrix."));  }

        flag = dyn_anlyt.calculateTransformationMatrix(io_ds->sensors_.q_, dyn_anlyt.getIdForLink(link_name),
            dyn_anlyt.getIdForLink("root"), Tanlyt);
        if (false==flag) { throw(std::runtime_error(std::string("Failed to compute analytic transformation matrix at: ") + link_name));  }

        for(int i=0; i<4 && flag; i++)
          for(int j=0; j<4 && flag; j++)
          { flag = flag && (fabs(Ttao.matrix()(i,j) - Tanlyt.matrix()(i,j))<test_precision);  }

        if (false==flag)
        {
          std::cout<<"\nTao transform Org->"<<link_name<<":\n"<<Ttao.matrix();
          std::cout<<"\nAnalytic transform Org->"<<link_name<<":\n"<<Tanlyt.matrix();
          throw(std::runtime_error("Tao and analytic transformation matrices don't match."));
        }
        else { std::cout<<"\nTest Result ("<<r_id++<<")  Analytic and tao transformations match for zero position : "<<link_name;  }

#ifdef DEBUG
        std::cout<<"\nTao transform Org->"<<link_name<<":\n"<<Ttao.matrix();
        std::cout<<"\nAnalytic transform Org->"<<link_name<<":\n"<<Tanlyt.matrix();
#endif
      }

      // *********************************************************************************************************
      //                               Test Transformation Matrix for a range of GCs
      // *********************************************************************************************************
      double gcstep=0.1;
      for (double a=-3.14;a<3.14;a+=gcstep)
        for (double b=-3.14;b<3.14;b+=gcstep)
          for (double c=-3.14;c<3.14;c+=gcstep)
          {
            io_ds->sensors_.q_(0) = a;
            io_ds->sensors_.q_(1) = b;
            io_ds->sensors_.q_(2) = c;
            flag = dynamics->setGeneralizedCoordinates(io_ds->sensors_.q_);
            if (false==flag)
            {
              std::cout<<"\nGeneralized Coordinates: "<<io_ds->sensors_.q_.transpose();
              throw(std::runtime_error("Failed to update tao's internal state.."));
            }

            // Loop through all links and get transformations.
            for(it = rob_ds->robot_br_rep_.begin(), ite = rob_ds->robot_br_rep_.end();
                it!=ite; ++it)
            {
              // Test : Link 0
              link_name = it->name_;

              // Skip the root node (all matrices are zero).
              if(it->is_root_) { continue; }

              flag = dynamics->calculateTransformationMatrix(dynamics->getIdForLink(link_name),Ttao);
              if (false==flag) { throw(std::runtime_error("Failed to compute tao transformation matrix."));  }

              flag = dyn_anlyt.calculateTransformationMatrix(io_ds->sensors_.q_, dyn_anlyt.getIdForLink(link_name),
                  dyn_anlyt.getIdForLink("root"), Tanlyt);
              if (false==flag) { throw(std::runtime_error(std::string("Failed to compute analytic transformation matrix at: ") + link_name));  }

              for(int i=0; i<4 && flag; i++)
                for(int j=0; j<4 && flag; j++)
                { flag = flag && (fabs(Ttao.matrix()(i,j) - Tanlyt.matrix()(i,j))<test_precision); }

              if (false==flag)
              {
                std::cout<<"\nGeneralized Coordinates: "<<io_ds->sensors_.q_.transpose();
                std::cout<<"\nTao transform Org->"<<link_name<<":\n"<<Ttao.matrix();
                std::cout<<"\nAnalytic transform Org->"<<link_name<<":\n"<<Tanlyt.matrix();
                throw(std::runtime_error("Tao and analytic transformation matrices don't match."));
              }
            }
          }
      std::cout<<"\nTest Result ("<<r_id++<<")  Analytic and tao transformations match for all links and gcs [-pi,pi]";

      // *********************************************************************************************************
      //                                         Test Generalized Inertia Matrix
      // *********************************************************************************************************
      // Set up variables.
      Eigen::MatrixXd Mgc_tao, Mgc_anlyt;

      flag = dynamics->updateModelMatrices(&(io_ds->sensors_),&rob_gc_model);
      if (false==flag) { throw(std::runtime_error("Failed to compute tao model matrices (for generalized inertia)."));  }

      Mgc_tao = rob_gc_model.A_;

      flag = dyn_anlyt.computeMgc(io_ds->sensors_.q_, Mgc_anlyt);
      if (false==flag) { throw(std::runtime_error("Failed to compute analytic generalized inertia."));  }

      for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
        { flag = flag && (fabs(Mgc_tao(i,j) - Mgc_anlyt(i,j))<test_precision); }

      if (false==flag)
      {
        std::cout<<"\nTao Mgc:\n"<<Mgc_tao;
        std::cout<<"\nAnalytic Mgc:\n"<<Mgc_anlyt;
        throw(std::runtime_error("Tao and analytic Generalized Inertias don't match."));
      }
      else { std::cout<<"\nTest Result ("<<r_id++<<")  Analytic and tao Generalized Inertias match for zero position";  }

#ifdef DEBUG
      std::cout<<"\nTao Mgc:\n"<<Mgc_tao;
      std::cout<<"\nAnalytic Mgc:\n"<<Mgc_anlyt;
#endif

      // *********************************************************************************************************
      //                          Test Generalized Inertia Matrix for a range of GCs
      // *********************************************************************************************************
      for (double a=-3.14;a<3.14;a+=gcstep)
        for (double b=-3.14;b<3.14;b+=gcstep)
          for (double c=-3.14;c<3.14;c+=gcstep)
          {
            io_ds->sensors_.q_(0) = a;
            io_ds->sensors_.q_(1) = b;
            io_ds->sensors_.q_(2) = c;

            flag = dynamics->updateModelMatrices(&(io_ds->sensors_),&rob_gc_model);
            if (false==flag) { throw(std::runtime_error("Failed to compute tao model matrices (for generalized inertia)."));  }

            Mgc_tao = rob_gc_model.A_;

            flag = dyn_anlyt.computeMgc(io_ds->sensors_.q_, Mgc_anlyt);
            if (false==flag) { throw(std::runtime_error("Failed to compute analytic generalized inertia."));  }

            for(int i=0; i<3; i++)
              for(int j=0; j<3; j++)
              { flag = flag && (fabs(Mgc_tao(i,j) - Mgc_anlyt(i,j))<test_precision); }

            if (false==flag)
            {
              std::cout<<"\nGeneralized Coordinates: "<<io_ds->sensors_.q_.transpose();
              std::cout<<"\nTao Mgc:\n"<<Mgc_tao;
              std::cout<<"\nAnalytic Mgc:\n"<<Mgc_anlyt;
              throw(std::runtime_error("Tao and analytic Generalized Inertias don't match."));
            }
          }
      std::cout<<"\nTest Result ("<<r_id++<<")  Analytic and tao Generalized Inertias match for all gcs [-pi, pi]";

      // *********************************************************************************************************
      //                                         Test Com Jacobians
      // *********************************************************************************************************
      // Set up variables.
      Eigen::MatrixXd Jcom_tao, Jcom_anlyt;
      Eigen::VectorXd pos;
      pos.setZero(3);

      for(it = rob_ds->robot_br_rep_.begin(), ite = rob_ds->robot_br_rep_.end();
          it!=ite; ++it)
      {
        // Test : Link 0
        link_name = it->name_;

        // Skip the root node (all matrices are zero).
        if(it->is_root_) { continue; }

        pos = it->com_;
        flag = dynamics->calculateJacobian(dynamics->getIdForLink(link_name),pos,Jcom_tao);
        if (false==flag) { throw(std::runtime_error("Failed to compute tao com Jacobian."));  }

        flag = dyn_anlyt.computeJcom(io_ds->sensors_.q_, dyn_anlyt.getIdForLink(link_name), Jcom_anlyt);
        if (false==flag) { throw(std::runtime_error("Failed to compute analytic com Jacobian."));  }

        for(int i=0; i<3; i++)
          for(int j=0; j<3; j++)
          { flag = flag && fabs(Jcom_tao(i,j) - Jcom_anlyt(i,j)) < test_precision; }

        if (false==flag)
        {
          std::cout<<"\nTao Jcom_"<<link_name<<":\n"<<Jcom_tao;
          std::cout<<"\nAnalytic Jcom_"<<link_name<<":\n"<<Jcom_anlyt;
          throw(std::runtime_error("Tao and analytic Jacobians don't match."));
        }
        else { std::cout<<"\nTest Result ("<<r_id++<<")  Analytic and tao com Jacobians match for zero position : "<<link_name;  }

#ifdef DEBUG
        std::cout<<"\nTao Jcom_"<<link_name<<":\n"<<Jcom_tao;
        std::cout<<"\nAnalytic Jcom_"<<link_name<<":\n"<<Jcom_anlyt;
#endif
      }

      // ********************************************************************************************************
      //Delete stuff
      if(S_NULL!= dynamics)  {  delete dynamics; dynamics = S_NULL; }

      std::cout<<"\nTest #"<<id<<" : Succeeded.";
    }
    catch (std::exception& ee)
    {
      std::cout<<"\nTest Result ("<<r_id++<<") : "<<ee.what();
      std::cout<<"\nTest #"<<id<<" : Failed.";

      if(S_NULL!= dynamics)  {  delete dynamics; dynamics = S_NULL; }
    }
  }
}

