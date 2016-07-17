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
/* \file Init.cpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include <scl/Init.hpp>

namespace scl
{
  namespace init
  {
    /** Dynamic typing helper functions for different scl types
     *  Registers the native dynamic types */
    scl::sBool registerNativeDynamicTypes()
    {
      bool flag;
      try
      {
        /** *****************************************************************************************************
         *                                        Controllers
         * ***************************************************************************************************** */
        sutil::CDynamicType<std::string,scl::CControllerGc> typeCGc(std::string("CControllerGc"));
        flag = typeCGc.registerType();
        if(false == flag) {throw(std::runtime_error("CControllerGc"));}

        sutil::CDynamicType<std::string,scl::CControllerMultiTask> typeCTask(std::string("CControllerMultiTask"));
        flag = typeCTask.registerType();
        if(false == flag) {throw(std::runtime_error("CControllerMultiTask"));}

        /** *****************************************************************************************************
         *                                        Control Tasks
         * ***************************************************************************************************** */
        sutil::CDynamicType<std::string,scl::CTaskOpPos> typeCTaskOpPos(std::string("CTaskOpPos"));
        flag = typeCTaskOpPos.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskOpPos"));}

        sutil::CDynamicType<std::string,scl::STaskOpPos> typeSTaskOpPos(std::string("STaskOpPos"));
        flag = typeSTaskOpPos.registerType();
        if(false == flag) {throw(std::runtime_error("STaskOpPos"));}

        sutil::CDynamicType<std::string,scl::CTaskOpPosPIDA1OrderInfTime> typeCTaskOpPosPIDA1OrderInfTime(std::string("CTaskOpPosPIDA1OrderInfTime"));
        flag = typeCTaskOpPosPIDA1OrderInfTime.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskOpPosPIDA1OrderInfTime"));}

        sutil::CDynamicType<std::string,scl::STaskOpPosPIDA1OrderInfTime> typeSTaskOpPosPIDA1OrderInfTime(std::string("STaskOpPosPIDA1OrderInfTime"));
        flag = typeSTaskOpPosPIDA1OrderInfTime.registerType();
        if(false == flag) {throw(std::runtime_error("STaskOpPosPIDA1OrderInfTime"));}

        sutil::CDynamicType<std::string,scl::CTaskOpPosNoGravity> typeCTaskOpPosNoGravity(std::string("CTaskOpPosNoGravity"));
        flag = typeCTaskOpPosNoGravity.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskOpPosNoGravity"));}

        sutil::CDynamicType<std::string,scl::STaskOpPosNoGravity> typeSTaskOpPosNoGravity(std::string("STaskOpPosNoGravity"));
        flag = typeSTaskOpPosNoGravity.registerType();
        if(false == flag) {throw(std::runtime_error("STaskOpPosNoGravity"));}

        sutil::CDynamicType<std::string,scl::CTaskNullSpaceDamping> typeCTaskNullSpaceDamping(std::string("CTaskNullSpaceDamping"));
        flag = typeCTaskNullSpaceDamping.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskNullSpaceDamping"));}

        sutil::CDynamicType<std::string,scl::STaskNullSpaceDamping> typeSTaskNullSpaceDamping(std::string("STaskNullSpaceDamping"));
        flag = typeSTaskNullSpaceDamping.registerType();
        if(false == flag) {throw(std::runtime_error("STaskNullSpaceDamping"));}

        sutil::CDynamicType<std::string,scl::CTaskGc> typeCTaskGc(std::string("CTaskGc"));
        flag = typeCTaskGc.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskGc"));}

        sutil::CDynamicType<std::string,scl::STaskGc> typeSTaskGc(std::string("STaskGc"));
        flag = typeSTaskGc.registerType();
        if(false == flag) {throw(std::runtime_error("STaskGc"));}

        sutil::CDynamicType<std::string,scl::CTaskGcLimitCentering> typeCTaskGcLimitCentering(std::string("CTaskGcLimitCentering"));
        flag = typeCTaskGcLimitCentering.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskGcLimitCentering"));}

        sutil::CDynamicType<std::string,scl::STaskGcLimitCentering> typeSTaskGcLimitCentering(std::string("STaskGcLimitCentering"));
        flag = typeSTaskGcLimitCentering.registerType();
        if(false == flag) {throw(std::runtime_error("STaskGcLimitCentering"));}

        sutil::CDynamicType<std::string,scl::CTaskGcSet> typeCTaskGcSet(std::string("CTaskGcSet"));
        flag = typeCTaskGcSet.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskGcSet"));}

        sutil::CDynamicType<std::string,scl::STaskGcSet> typeSTaskGcSet(std::string("STaskGcSet"));
        flag = typeSTaskGcSet.registerType();
        if(false == flag) {throw(std::runtime_error("STaskGcSet"));}

        sutil::CDynamicType<std::string,scl::CTaskComPos> typeCTaskComPos(std::string("CTaskComPos"));
        flag = typeCTaskComPos.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskComPos"));}

        sutil::CDynamicType<std::string,scl::STaskComPos> typeSTaskComPos(std::string("STaskComPos"));
        flag = typeSTaskComPos.registerType();
        if(false == flag) {throw(std::runtime_error("STaskComPos"));}

        sutil::CDynamicType<std::string,scl::CTaskConstraintPlane> typeCTaskConstraintPlane(std::string("CTaskConstraintPlane"));
        flag = typeCTaskConstraintPlane.registerType();
        if(false == flag) {throw(std::runtime_error("CTaskConstraintPlane"));}

        sutil::CDynamicType<std::string,scl::STaskConstraintPlane> typeSTaskConstraintPlane(std::string("STaskConstraintPlane"));
        flag = typeSTaskConstraintPlane.registerType();
        if(false == flag) {throw(std::runtime_error("STaskConstraintPlane"));}

        /** *****************************************************************************************************
         *                                        Actuator Sets
         * ***************************************************************************************************** */
        sutil::CDynamicType<std::string,scl::SActuatorSetMuscleParsed> typeSActuatorSetMuscleParsed(std::string("SActuatorSetMuscleParsed"));
        flag = typeSActuatorSetMuscleParsed.registerType();
        if(false == flag) {throw(std::runtime_error("SActuatorSetMuscleParsed"));}

        sutil::CDynamicType<std::string,scl::SActuatorSetMuscle> typeSActuatorSetMuscle(std::string("SActuatorSetMuscle"));
        flag = typeSActuatorSetMuscle.registerType();
        if(false == flag) {throw(std::runtime_error("SActuatorSetMuscle"));}

        sutil::CDynamicType<std::string,scl::CActuatorSetMuscle> typeCActuatorSetMuscle(std::string("CActuatorSetMuscle"));
        flag = typeCActuatorSetMuscle.registerType();
        if(false == flag) {throw(std::runtime_error("CActuatorSetMuscle"));}
      }
      catch (std::exception& e)
      {
        std::cout<<"\nregisterNativeDynamicTypes() : Could not register type : "<<e.what();
        return false;
      }
      return true;
    }
  }
}
