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
/* \file scl.hpp
 *
 *  Created on: Nov 14, 2013
 *
 *  Copyright (C) 2013
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SCL_HPP_
#define SCL_HPP_

#include <scl/DataTypes.hpp>
#include <scl/Singletons.hpp>

#include <scl/haptics/CHapticsBase.hpp>
#include <scl/haptics/chai/CHapticsChai.hpp>

#include <scl/util/DatabaseUtils.hpp>
#include <scl/util/FileFunctions.hpp>
#include <scl/util/HelperFunctions.hpp>
#include <scl/util/RobotMath.hpp>

#include <scl/robot/CRobotApp.hpp>
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/robot/GenericCallbacks.hpp>
#include <scl/robot/data_structs/SRobot.hpp>
#include <scl/robot/CRobot.hpp>
#include <scl/robot/GenericPrintables.hpp>

#include <scl/actuation/CActuatorBase.hpp>
#include <scl/actuation/CActuatorSetBase.hpp>
#include <scl/actuation/data_structs/SActuatorSetBase.hpp>
#include <scl/actuation/muscles/CActuatorMuscle.hpp>
#include <scl/actuation/muscles/CActuatorSetMuscle.hpp>
#include <scl/actuation/muscles/data_structs/SActuatorMuscle.hpp>
#include <scl/actuation/muscles/data_structs/SActuatorSetMuscle.hpp>

#include <scl/control/CControllerBase.hpp>
#include <scl/control/gc/data_structs/SControllerGc.hpp>
#include <scl/control/gc/CControllerGc.hpp>
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/control/task/tasks/CTaskComPos.hpp>
#include <scl/control/task/tasks/CTaskNULL.hpp>
#include <scl/control/task/tasks/CTaskOpPosNoGravity.hpp>
#include <scl/control/task/tasks/CTaskGcLimitCentering.hpp>
#include <scl/control/task/tasks/CTaskGc.hpp>
#include <scl/control/task/tasks/CTaskNullSpaceDamping.hpp>
#include <scl/control/task/tasks/CTaskGcSet.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcSet.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPosNoGravity.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcLimitCentering.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPosPIDA1OrderInfTime.hpp>
#include <scl/control/task/tasks/data_structs/STaskComPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskGc.hpp>
#include <scl/control/task/tasks/data_structs/STaskNullSpaceDamping.hpp>
#include <scl/control/task/tasks/CTaskOpPosPIDA1OrderInfTime.hpp>
#include <scl/control/task/CServo.hpp>
#include <scl/control/task/CNonControlTaskBase.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>
#include <scl/control/task/data_structs/SServo.hpp>
#include <scl/control/task/data_structs/SControllerMultiTask.hpp>
#include <scl/control/task/data_structs/SNonControlTaskBase.hpp>
#include <scl/control/task/CTaskBase.hpp>
#include <scl/control/data_structs/SControllerBase.hpp>

#include <scl/parser/saiparser/CParserSai.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/parser/osimparser/CParserOsim.hpp>
#include <scl/parser/osimparser/CParserOsimForOldFiles.hpp>
#include <scl/parser/CParserBase.hpp>

#include <scl/sensing/CSensorBase.hpp>
#include <scl/sensing/CSensorSetBase.hpp>

#include <scl/data_structs/SRigidBodyDyn.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/data_structs/SRobotParsed.hpp>
#include <scl/data_structs/SRigidBody.hpp>
#include <scl/data_structs/SGraphicsParsed.hpp>
#include <scl/data_structs/SMuscleSetParsed.hpp>
#include <scl/data_structs/SDatabase.hpp>
#include <scl/data_structs/SRobotIO.hpp>
#include <scl/data_structs/SObject.hpp>
#include <scl/data_structs/SForce.hpp>

#include <scl/dynamics/CDynamicsBase.hpp>
#include <scl/dynamics/CDynamicsAnalyticBase.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/analytic/CDynamicsAnalyticRPP.hpp>

#include <scl/graphics/CGraphicsBase.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/data_structs/SGraphicsChai.hpp>
#include <scl/graphics/chai/data_structs/SGraphicsChaiRigidBody.hpp>
#include <scl/graphics/chai/data_structs/SGraphicsChaiMuscleSet.hpp>


#endif /* SCL_HPP_ */
