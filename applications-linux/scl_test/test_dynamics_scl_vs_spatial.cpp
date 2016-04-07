/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/*
 * test_dynamics_scl_vs_spatial.cpp
 *
 *  Created on: Jun 12, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author : Nayan Singhal <singhalnayan91@gmail.com>
 *              Brains in Silicon Lab,
 *              Stanford University.
 *
 *  Edited by: Samir Menon <smenon@stanford.edu>
 */
#include "test_dynamics_scl_vs_spatial.hpp"
#include <scl_ext/dynamics/scl_spatial/CDynamicsSclSpatial.hpp>
#include <sutil/CSystemClock.hpp>

#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/util/DatabaseUtils.hpp>

#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>

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
using namespace scl_ext;


namespace scl_test
{

  bool checkMatrix(Eigen::VectorXd & arg_value , Eigen::VectorXd & arg_check_value)
  {
    sFloat precision = 0.001;
    for(sInt row = 0 ; row < arg_value.rows(); row++ )
    {
      for(sInt col = 0 ; col < arg_value.cols() ; col++ )
      {
        if(!(((arg_value(row,col) - precision ) < arg_check_value(row,col) ) && ((arg_value(row,col) + precision ) > arg_check_value(row,col))))
        {
          return false;
        }

      }
    }

    return true;
  }

  void test_dynamics_scl_vs_spatial(int id){

    scl::sUInt test_id = 1;
    scl::CDynamicsScl dynamics;
    scl::sUInt r_id=0;
    bool flag;

    //0. Create vars
    long long i; //Counters
    long long imax; //Counter limits

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

    scl::CParserScl tmp_lparser;

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
    scl::SRobotParsed *rob_ds = db->s_parser_.robots_.at(robot_name);
    if(NULL == rob_ds)
    { throw(std::runtime_error("Could not find registered robot in the database"));  }

    // IF the robot wasn't sorted, issue a warning and set present order as sorted
    std::vector<std::string> tmp_sort_order;
    flag = rob_ds->rb_tree_.sort_get_order(tmp_sort_order);
    if(false == flag)
    {
      std::cout<<"\nWARNING : Robot branching representation is not sorted by default. Sorting. Robot = "<<rob_ds->name_;

      // Get the present node ordering.
      sutil::CMappedTree<std::string, SRigidBody>::const_iterator it,ite;
      for(it = rob_ds->rb_tree_.begin(), ite = rob_ds->rb_tree_.end();
          it!=ite; ++it)
      { tmp_sort_order.push_back(it->name_); }

      // Sort it.
      flag = rob_ds->rb_tree_.sort(tmp_sort_order);
      if(false == flag)
      { throw(std::runtime_error("Could not sort unsorted robot branching representation."));  }
    }

#ifdef DEBUG
    std::cout<<"\nPrinting parsed robot "<<rob_ds->name_;
    scl_util::printRobotLinkTree( *(rob_ds->rb_tree_.getRootNode()), 0);
#endif

    CDynamicsScl* dyn_scl = new scl::CDynamicsScl();
    flag = dyn_scl->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
    if(false == flag) { throw(std::runtime_error("Could not initialize dynamics algorithms"));  }

    CDynamicsTao* dyn_tao = new scl::CDynamicsTao();
    flag = dyn_tao->init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
    if(false == flag) { throw(std::runtime_error("Could not initialize physics simulator"));  }

    SRobotIO * io_ds;
    io_ds = scl::CDatabase::getData()->s_io_.io_data_.at(robot_name);
    if(S_NULL == io_ds)
    { throw(std::runtime_error("Could not find the robot's I/O data structure in the database"));  }

    //******************* Now test the actual implementation ******************
    io_ds->sensors_.q_.setZero(3);
    io_ds->sensors_.dq_.setZero(3);
    io_ds->sensors_.ddq_.setZero(3);

    // *********************************************************************************************************
    //                              Set up the robot's dynamics data struct
    // *********************************************************************************************************
    // These are used to compute the full dynamics model later in the code..

    scl::SGcModel rob_gc_model;
    flag = rob_gc_model.init(*rob_ds);
    if(false == flag)
    { throw(std::runtime_error("Could not create a dynamic object for the robot"));  }

    imax = 200;
    for(i=0; i<imax; ++i)
    {
      flag = dyn_tao->integrate((*io_ds), scl::CDatabase::getData()->sim_dt_);  //calculating joint acceleration
    }

    CDynamicsSclSpatial test;
    Eigen::VectorXd ret_fgc , ret_ddq;

    flag = test.init(* scl::CDatabase::getData()->s_parser_.robots_.at(robot_name));
    if(false == flag) { throw(std::runtime_error("Could not initialize spatial dynamics"));  }

    //Test 1 : Testing Articulated Body Algorithm vs scl
    if (false == test.forwardDynamicsABA(io_ds, &rob_gc_model , ret_ddq))
    { throw(std::runtime_error("Failed to calculate joint Acceleration using Articulated Body Algorithm "));  }
    {
      if(true == checkMatrix(ret_fgc, io_ds->sensors_.ddq_))
      {
        std::cout<<"\nTest Result ("<<r_id++<<") Spatial ABA and Scl Acceleration match";
      }
      else
      {
        throw(std::runtime_error("Failed: Spatial ABA and Scl Acceleration don't match"));
      }
    }

    //Test 2 : Testing Composite Rigid Body Algorithm vs scl
    if (false == test.forwardDynamicsCRBA(io_ds, &rob_gc_model , ret_ddq))
    { throw(std::runtime_error("Failed to calculate joint Acceleration using Composite Rigid Body Algorithm"));  }
    {
      if(true == checkMatrix(ret_fgc, io_ds->sensors_.ddq_))
      {
        std::cout<<"\nTest Result ("<<r_id++<<") Spatial CRBA and Scl Acceleration match";
      }
      else
      {
        throw(std::runtime_error("Failed: Spatial CRBA and Scl Acceleration don't match"));
      }
    }

    //Test 3 : Testing Newton Euler Algorithm vs scl
    io_ds->sensors_.ddq_(0) = 1.4;   //setting joint acceleration of link 1 to zero
    for(i=0; i<imax; ++i)
    {
    	 flag = dyn_tao->integrate((*io_ds), scl::CDatabase::getData()->sim_dt_);  //calculating joint acceleration
    }

    //NOTE TODOD : I don't know what this test is doing. Please fix this....
//    if (false == test.inverseDynamicsNER(io_ds , &rob_gc_model , ret_fgc))
//    { throw(std::runtime_error("Failed to calculate joint Acceleration using Newton Euler Algorithm Algorithm"));  }
//
//    if(true == checkMatrix(ret_fgc, io_ds->sensors_.force_gc_measured_))
//    {
//      std::cout<<"\nTest Result ("<<r_id++<<") Spatial and Scl torque match";
//      std::cout<<"\n TAO : "<<io_ds->sensors_.force_gc_measured_.transpose();
//      std::cout<<"\n NER : "<<ret_fgc.transpose();
//    }
//    else
//    {
//      throw(std::runtime_error("Failed: Spatial and Scl torque don't match"));
//    }

    std::cout<<"\nTest #"<<id<<" : Succeeded.";
  }

} /* namespace scl_test */
