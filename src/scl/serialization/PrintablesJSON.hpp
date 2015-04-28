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
/* \file PrintablesJSON.hpp
 *
 *  Created on: Apr 28, 2015
 *
 *  Copyright (C) 2015
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef PRINTABLES_JSON_HPP_
#define PRINTABLES_JSON_HPP_

#include <scl/serialization/SerializationJSON.hpp>

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
  /** ************************* JSON CALL ******************** */
  template <>
  void printToStream<scl::SRobotParsed>(
      std::ostream& ostr,
      const scl::SRobotParsed& arg_data
  )
  {
    Json::Value val;
    bool flag = scl::serializeToJSON<scl::SRobotParsed>(arg_data, val);
    if(flag){
      ostr<<val.toStyledString();
      ostr<<std::endl;
    }
    else
      ostr<<"\nERROR : Could not print JSON for parsed robot: "<< arg_data.name_<<std::endl;
  }

  /** ************************* JSON CALL ******************** */
  template <>
  void printToStream<scl::SRobotIO>(
      std::ostream& ostr,
      const scl::SRobotIO& arg_data
  )
  {
    Json::Value val;
    bool flag = scl::serializeToJSON<scl::SRobotIO>(arg_data, val);
    if(flag){
      ostr<<val.toStyledString();
      ostr<<std::endl;
    }
    else
      ostr<<"\nERROR : Could not print JSON for robot io: "<< arg_data.name_<<std::endl;
  }

  /** ************************* JSON CALL ******************** */
  template <>
  void printToStream<scl::SRigidBody>(
      std::ostream& ostr,
      const scl::SRigidBody& arg_data
  )
  {
    Json::Value val;
    bool flag = scl::serializeToJSON<scl::SRigidBody>(arg_data, val);
    if(flag){
      ostr<<val.toStyledString();
      ostr<<std::endl;
    }
    else
      ostr<<"\nERROR : Could not print JSON for rigid body: "<< arg_data.name_<<std::endl;
  }

  /** ************************* JSON CALL ******************** */
  template <>
  void printToStream<scl::STaskBase>(
      std::ostream& ostr,
      const scl::STaskBase& arg_data
  )
  {
    Json::Value val;
    bool flag = scl::serializeToJSON<scl::STaskBase>(arg_data, val);
    if(flag){
      ostr<<val.toStyledString();
      ostr<<std::endl;
    }
    else
      ostr<<"\nERROR : Could not print JSON for control task: "<< arg_data.name_<<std::endl;
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

  template <>
  bool printableAddObject<scl::SDatabase>(const scl::SDatabase & arg_obj)
  {
    bool flag;
    try
    {
      std::cout<<"\n\n*** Robot printables ***"
          <<"\n\nscl>> print <printable name>"
          <<"\n\n"<<std::setw(10)<< "Printable information"<<std::setw(45)<<"Printable name\n";

      //Add all the robot links as printables.
      sutil::CMappedList<std::string,scl::SRobotParsed>::const_iterator it,ite;
      for(it = arg_obj.s_parser_.robots_.begin(),
          ite = arg_obj.s_parser_.robots_.end();
          it!=ite;++it)
      { printableAddObject<scl::SRobotParsed>(*it); }

      //Add all the IO data structures for all the robots
      sutil::CMappedList<std::string, scl::SRobotIO>::const_iterator iti,itie;
      for(iti = arg_obj.s_io_.io_data_.begin(),
          itie = arg_obj.s_io_.io_data_.end();
          iti!=itie; ++iti)
      { printableAddObject<scl::SRobotIO>(*iti); }

      //Add all the task controller data structures for all the robots
      sutil::CMappedPointerList<std::string, scl::SControllerBase,true>::const_iterator itct,itcte;
      for(itct = arg_obj.s_controller_.controllers_.begin(),
          itcte = arg_obj.s_controller_.controllers_.end();
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
      for(itctt = arg_obj.s_controller_.tasks_.begin(),
          itctte = arg_obj.s_controller_.tasks_.end();
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
        flag = sutil::printables::add(name,arg_obj.s_gui_.ui_point_[i]);
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
#endif /* PRINTABLES_JSON_HPP_ */
