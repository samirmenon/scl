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
/* \file GenericPrintables.cpp
 *
 *  Created on: Aug 19, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "GenericPrintables.hpp"


#include <scl/data_structs/SDatabase.hpp>
#include <scl/serialization/PrintablesJSON.hpp>
#include <scl/Singletons.hpp>

#include <sutil/CRegisteredPrintables.hpp>

#include <stdexcept>
#include <iomanip>

#ifdef DEBUG
#include <cassert>
#endif

/* **************************************************************************
 *                         Starting namespace scl
 *
 * Here we define callback adders for generic objects (like robots)
 * ************************************************************************* */
namespace scl
{
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
        flag = sutil::printables::add(ctrl->gc_model_->name_,*(ctrl->gc_model_));
        if(false == flag)
        {throw(std::runtime_error(std::string("Could not add a printable: GcModel: ")+
            ctrl->gc_model_->name_));  }
        else
        { std::cout<<"\n"<<std::setw(10)<<"Printable: GcModel: "<<std::setw(41)<<ctrl->gc_model_->name_; }
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
        // This is the special code for adding an op pos task
        if(task->getType() == "STaskOpPos")
        {
          const scl::STaskOpPos* task2 = dynamic_cast<const scl::STaskOpPos*>(task);

          if(NULL == task2)
          {throw(std::runtime_error(std::string("Could not add a printable: ")+task->name_));  }

          const scl::STaskOpPos &tt = *task2;

          flag = sutil::printables::add(task->name_,tt);
        }
        else if(task->getType() == "STaskGc")
        {
          const scl::STaskGc* task2 = dynamic_cast<const scl::STaskGc*>(task);

          if(NULL == task2)
          {throw(std::runtime_error(std::string("Could not add a printable: ")+task->name_));  }

          const scl::STaskGc &tt = *task2;

          flag = sutil::printables::add(task->name_,tt);
        }
        else
        { flag = sutil::printables::add(task->name_,*task); }
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


