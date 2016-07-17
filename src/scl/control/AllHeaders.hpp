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
/* \file AllHeaders.hpp
 *
 *  Created on: Jul 16, 2016
 *
 *  Copyright (C) 2016
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef SRC_SCL_CONTROL_ALLHEADERS_HPP_
#define SRC_SCL_CONTROL_ALLHEADERS_HPP_

// NOTE : This header file does not include the *Base classes/structs. Those
//        are assumed to be included in the appropriate C* classes (they are
//        inevitably used while defining functions).

/** GC Controller */
#include <scl/control/gc/CControllerGc.hpp>
#include <scl/control/gc/data_structs/SControllerGc.hpp>

/** Multi-Task Controller : Base control formulation */
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/CServo.hpp>
#include <scl/control/task/data_structs/SControllerMultiTask.hpp>
#include <scl/control/task/data_structs/SServo.hpp>

/** Multi-Task Controller : The actual tasks */
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>

#include <scl/control/task/tasks/CTaskOpPosPIDA1OrderInfTime.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPosPIDA1OrderInfTime.hpp>

#include <scl/control/task/tasks/CTaskOpPosNoGravity.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPosNoGravity.hpp>

#include <scl/control/task/tasks/CTaskGc.hpp>
#include <scl/control/task/tasks/data_structs/STaskGc.hpp>

#include <scl/control/task/tasks/CTaskGcLimitCentering.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcLimitCentering.hpp>

#include <scl/control/task/tasks/CTaskGcSet.hpp>
#include <scl/control/task/tasks/data_structs/STaskGcSet.hpp>

#include <scl/control/task/tasks/CTaskNullSpaceDamping.hpp>
#include <scl/control/task/tasks/data_structs/STaskNullSpaceDamping.hpp>

#include <scl/control/task/tasks/CTaskComPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskComPos.hpp>

#include <scl/control/task/tasks/CTaskConstraintPlane.hpp>
#include <scl/control/task/tasks/data_structs/STaskConstraintPlane.hpp>


#endif /* SRC_SCL_CONTROL_ALLHEADERS_HPP_ */
