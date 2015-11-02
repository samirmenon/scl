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
/* \file GenericPrintables.hpp
 *
 *  Created on: Sep 19, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef GENERICPRINTABLES_HPP_
#define GENERICPRINTABLES_HPP_

#include <scl/data_structs/SRobotIO.hpp>
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/control/data_structs/SControllerBase.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>
#include <scl/control/task/data_structs/SControllerMultiTask.hpp>

#include <scl/Singletons.hpp>

#include <sutil/CRegisteredPrintables.hpp>

#include <stdexcept>
#include <iomanip>

#ifdef DEBUG
#include <cassert>
#endif

/* **************************************************************************
 *                         Starting namespace sutil
 *
 * These are a set of printable object definitions for objects in the database
 * Creating separate printable objects enables accessing the objects's values
 * through callbacks
 * Directly accessing the objects might be hard for certain situations
 * (Like from another process or a gui) and callbacks are good there.
 * ************************************************************************* */
namespace sutil
{
  template <>
  void printToStream<scl::SRobotParsed>(
      std::ostream& ostr,
      const scl::SRobotParsed& arg_data
  )
  {
    ostr<<"\n Name: "<<arg_data.name_;
    ostr<<"("<<arg_data.getType()<<")";
    ostr<<"\n  Dof: "<<arg_data.dof_;
    ostr<<"\n  Jlim_max : "<<arg_data.gc_pos_limit_max_.transpose();
    ostr<<"\n  Jlim_min : "<<arg_data.gc_pos_limit_min_.transpose();
    ostr<<"\n Actuator_force_max : "<<arg_data.actuator_forces_max_.transpose();
    ostr<<"\n Actuator_force_min : "<<arg_data.actuator_forces_min_.transpose();
    ostr<<"\n Flag_gc_pos_limits : "<<arg_data.flag_apply_gc_pos_limits_;
    ostr<<"\n Flag_gc_damping : "<<arg_data.flag_apply_gc_damping_;
    ostr<<"\n Flag_force_limits : "<<arg_data.flag_apply_actuator_force_limits_;
    ostr<<"\n Flag_vel_limits : "<<arg_data.flag_apply_actuator_vel_limits_;
    ostr<<"\n Flag_accel_limits : "<<arg_data.flag_apply_actuator_acc_limits_;
    ostr<<std::endl;
  }

  template <>
  void printToStream<scl::SRobotIO>(
      std::ostream& ostr,
      const scl::SRobotIO& arg_data
  )
  {
    ostr<<"\n Name: "<<arg_data.name_;
    ostr<<"("<<arg_data.getType()<<")";
    ostr<<"\nDof: "<<arg_data.dof_;
    ostr<<"\nHas been init: "<<arg_data.has_been_init_;
    ostr<<"\n  q: "<<arg_data.sensors_.q_.transpose();
    ostr<<"\n dq: "<<arg_data.sensors_.dq_.transpose();
    ostr<<"\nddq: "<<arg_data.sensors_.ddq_.transpose();
    ostr<<"\nMeasured force: "<<arg_data.sensors_.force_gc_measured_.transpose();
    ostr<<"\n Command force: "<<arg_data.actuators_.force_gc_commanded_.transpose();
    //NOTE TODO : Implement a loop to print the external forces.
    //ostr<<"\nMeasured force: "<<arg_data.sensors_.forces_external_;
    ostr<<std::endl;
  }

  template <>
  void printToStream<scl::SRigidBody>(
      std::ostream& ostr,
      const scl::SRigidBody& arg_data
  )
  {
    ostr<<"\n Name: "<<arg_data.name_;
    ostr<<"("<<arg_data.getType()<<")";
    ostr<<"\n Init   : "<<arg_data.has_been_init_;
    ostr<<"\n Id     : "<<arg_data.link_id_;
    ostr<<"\n Parent : "<<arg_data.parent_name_;
    ostr<<"\n Robot  : "<<arg_data.robot_name_;
    ostr<<"\n Joint  : "<<arg_data.joint_name_;
    ostr<<"\n Jtype  : "<<arg_data.joint_type_;
    ostr<<"\n Jlim   : ["<<arg_data.joint_limit_lower_<<", "<<arg_data.joint_limit_upper_<<"]";
    ostr<<"\n Com    : "<<arg_data.com_.transpose();
    ostr<<"\n Inertia: "<<arg_data.inertia_;
    ostr<<"\n PosPar : "<<arg_data.pos_in_parent_.transpose();
    ostr<<"\n OriPar : [w xyz] ["<<arg_data.ori_parent_quat_.w()
        <<", "<<arg_data.ori_parent_quat_.x()
        <<", "<<arg_data.ori_parent_quat_.y()
        <<", "<<arg_data.ori_parent_quat_.z()<<"]";
    ostr<<std::endl;
  }

  template <>
  void printToStream<scl::STaskBase>(
      std::ostream& ostr,
      const scl::STaskBase& arg_data
  )
  {
    ostr<<"\n Name: "<<arg_data.name_;
    ostr<<"("<<arg_data.getType()<<")";
    ostr<<". Parent("<<arg_data.parent_controller_->name_<<")";
    ostr<<"\n Init/Active : "<<arg_data.has_been_init_<<"/"<<arg_data.has_been_activated_;
    ostr<<"\n Priority    : "<<arg_data.priority_;
    ostr<<"\n F_task : "<<arg_data.force_task_.transpose();
    ostr<<"\n F_gc   : "<<arg_data.force_gc_.transpose();
    ostr<<std::endl;
  }

  template <>
  void printToStream<scl::STaskOpPos>(
      std::ostream& ostr,
      const scl::STaskOpPos& arg_data
  )
  {
    ostr<<"\n Name: "<<arg_data.name_;
    ostr<<"("<<arg_data.getType()<<")";
    ostr<<". Parent("<<arg_data.parent_controller_->name_<<")";
    ostr<<"\n Init/Active : "<<arg_data.has_been_init_<<"/"<<arg_data.has_been_activated_;
    ostr<<"\n Priority    : "<<arg_data.priority_;
    ostr<<"\n X      : "<<arg_data.x_;
    ostr<<"\n Xgoal  : "<<arg_data.x_goal_;
    ostr<<"\n F_task : "<<arg_data.force_task_.transpose();
    ostr<<"\n F_gc   : "<<arg_data.force_gc_.transpose();
    ostr<<std::endl;
  }

  template <>
  void printToStream<scl::SControllerMultiTask>(
      std::ostream& ostr,
      const scl::SControllerMultiTask& arg_data
  )
  {
    ostr<<"\n Name: "<<arg_data.name_;
    ostr<<"("<<arg_data.getType()<<")";
    ostr<<"\n Init      : "<<arg_data.has_been_init_;
    ostr<<std::endl;
    unsigned int i=0;
    sutil::CMappedMultiLevelList<std::string, scl::STaskBase*>::const_iterator it,ite;
    for(it = arg_data.tasks_.begin(), ite = arg_data.tasks_.end();
        it!=ite; ++it)
    {
#ifdef DEBUG
      assert(NULL != *it);
#endif
      const scl::STaskBase& task = **it;
      ostr<<"\n === Task #"<<i<<":"; i++;
      printToStream<scl::STaskBase>(ostr, task);
    }
  }

  template <>
  void printToStream<Eigen::Vector3d >(
      std::ostream& ostr,
      const Eigen::Vector3d& arg_eigvec
  )
  { ostr<<arg_eigvec.transpose()<<std::flush; }
}

/* **************************************************************************
 *                         Starting namespace scl
 *
 * Here we define callback adders for generic objects (like robots)
 * ************************************************************************* */
namespace scl
{
  /** This adds an object to the printable registry. That allows pretty-printing
   * this object from the command line shell...
   *
   * NOTE : This function may throw an exception (not typical SCL) */
  template <class T>
  bool printableAddObject(const T & arg_obj);

  /** This is to just add an object and member objects
   * NOTE : This function may throw an exception (not typical SCL) */
  template <>
  bool printableAddObject<scl::SRobotParsed>(const scl::SRobotParsed & arg_obj)
  {
    //NOTE: The robot's parsed and IO data structures are usually indexed by
    // the robot name. So we add "Parsed" here at the end to distinguish between
    // the. (Feel free to change this; you may pick your own convention).
    bool flag = sutil::printables::add(arg_obj.name_+std::string("Parsed"),arg_obj);
    if(false == flag)
    {throw(std::runtime_error(std::string("Could not add a printable: Robot: ")+
        arg_obj.name_));  }
    else
    { std::cout<<"\n"<<std::setw(10)<< "Printable: Robot: "<<std::setw(51)<<arg_obj.name_+std::string("Parsed"); }

    //Also add the links
    sutil::CMappedTree<std::string, scl::SRigidBody>::const_iterator itl, itle;
    for(itl = arg_obj.rb_tree_.begin(), itle = arg_obj.rb_tree_.end();
        itl!=itle; ++itl)
    {
      const scl::SRigidBody& l = *itl;
      flag = sutil::printables::add(l.name_,l);
      if(false == flag)
      {throw(std::runtime_error(std::string("Could not add a printable: Robot: ")+
          arg_obj.name_+". Link: "+l.name_));  }
      else
      { std::cout<<"\n"<<std::setw(10)<<"Printable: "<<arg_obj.name_<<" "<<l.getType()<<std::setw(40)<<l.name_; }
    }
    return true;
  }

  /** This is to just add an object and member objects */
  template <>
  bool printableAddObject<scl::SRobotIO>(const scl::SRobotIO & arg_obj)
  {
    //NOTE: The robot's parsed and IO data structures are usually indexed by
    // the robot name. Also see SRobotParsed
    bool flag = sutil::printables::add(arg_obj.name_,arg_obj);
    if(false == flag)
    {throw(std::runtime_error(std::string("Could not add a printable: Robot: ")+
        arg_obj.name_));  }
    else
    { std::cout<<"\n"<<std::setw(10)<<"Printable: Robot: "<<std::setw(51)<<arg_obj.name_; }
    return true;
  }

  bool addRobotPrintables()
  {
    bool flag;
    try
    {
      std::cout<<"\n\n*** Robot printables ***"
          <<"\n\nscl>> print <printable name>"
          <<"\n\n"<<std::setw(10)<< "Printable information"<<std::setw(45)<<"Printable name\n";

      //Add all the robot links as printables.
      sutil::CMappedList<std::string,scl::SRobotParsed>::iterator it,ite;
      for(it = scl::CDatabase::getData()->s_parser_.robots_.begin(),
          ite = scl::CDatabase::getData()->s_parser_.robots_.end();
          it!=ite;++it)
      { printableAddObject<scl::SRobotParsed>(*it); }

      //Add all the IO data structures for all the robots
      sutil::CMappedList<std::string, scl::SRobotIO>::iterator iti,itie;
      for(iti = scl::CDatabase::getData()->s_io_.io_data_.begin(),
          itie = scl::CDatabase::getData()->s_io_.io_data_.end();
          iti!=itie; ++iti)
      { printableAddObject<scl::SRobotIO>(*iti); }

      //Add all the task controller data structures for all the robots
      sutil::CMappedPointerList<std::string, scl::SControllerBase,true>::const_iterator itct,itcte;
      for(itct = scl::CDatabase::getData()->s_controller_.controllers_.begin(),
          itcte = scl::CDatabase::getData()->s_controller_.controllers_.end();
          itct!=itcte; ++itct)
      {
        const scl::SControllerMultiTask* ctrl = dynamic_cast<scl::SControllerMultiTask*>(*itct);
        if(S_NULL == ctrl)
        { continue;  } //Move on. Not a task controller.
        flag = sutil::printables::add(ctrl->name_,*ctrl);
        if(false == flag)
        {throw(std::runtime_error(std::string("Could not add a printable: Task Controller: ")+
            ctrl->name_));  }
        else
        { std::cout<<"\n"<<std::setw(10)<<"Printable: Task Controller: "<<std::setw(41)<<ctrl->name_; }
      }

      //Add all the task controller data structures for all the robots
      sutil::CMappedPointerList<std::string, scl::STaskBase,true>::const_iterator itctt,itctte;
      for(itctt = scl::CDatabase::getData()->s_controller_.tasks_.begin(),
          itctte = scl::CDatabase::getData()->s_controller_.tasks_.end();
          itctt!=itctte; ++itctt)
      {
        const scl::STaskBase* task = *itctt;
#ifdef DEBUG
        assert(S_NULL != task);
#else
        if(S_NULL == task)
        { continue;  } //Move on. Not a task.
#endif
        flag = sutil::printables::add(task->name_,*task);
        if(false == flag)
        {throw(std::runtime_error(std::string("Could not add a printable: Task Controller: ")+
            task->name_));  }
        else
        { std::cout<<"\n"<<std::setw(10)<<"Printable: Task: "<<std::setw(52)<<task->name_; }
      }

      //Add the UI points
      for(unsigned int i=0;i<SCL_NUM_UI_POINTS;++i)
      {
        std::stringstream ss; ss<<"ui"<<i;
        std::string name(ss.str());
        flag = sutil::printables::add(name,scl::CDatabase::getData()->s_gui_.ui_point_[i]);
        if(false == flag)
        {throw(std::runtime_error(std::string("Could not add a printable: ")+name));  }
        else
        { std::cout<<"\n"<<std::setw(10)<<"Printable: UI-point "<<std::setw(49)<<name; }
      }
      std::cout<<std::endl<<std::endl;
    }
    catch(std::exception &e)
    {
      std::cout<<"addRobotPrintables() : Error: "<<e.what();
      return false;
    }
    return true;
  }
}
#endif /* GENERICPRINTABLES_HPP_ */
