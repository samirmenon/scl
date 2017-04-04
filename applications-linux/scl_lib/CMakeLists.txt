cmake_minimum_required(VERSION 2.6)

#Name the project
project(scl)

#Set Include directories
SET(SCL_INC_DIR ../../src/scl)

SET(SCLEXT_INC_DIR ../../src/scl_ext)

#Set 3rd party dependencies
#Eigen Includes
SET(EIGEN_INC_DIR ../../3rdparty/eigen)

#Tinyxml includes
SET(TIXML_INC_DIR ../../3rdparty/tinyxml)

#sUtil Includes
SET(SUTIL_INC_DIR ../../3rdparty/sUtil/src)

#Set all include directories
INCLUDE_DIRECTORIES(../../src/ ${EIGEN_INC_DIR}	    						
                    ${SUTIL_INC_DIR} ${TIXML_INC_DIR} 
                    ${SCLEXT_INC_DIR})

#Set all the base sources required for the library
SET(SCL_BASE_SRC ${SCL_INC_DIR}/Init.cpp
   )

#Set all the sources required for the library
SET(DB_SRC ${SCL_INC_DIR}/data_structs/SRobotIO.cpp
           ${SCL_INC_DIR}/data_structs/SRigidBody.cpp
           ${SCL_INC_DIR}/data_structs/SGcModel.cpp
           ${SCL_INC_DIR}/data_structs/SActuatorSetMuscleParsed.cpp
           ${SCL_INC_DIR}/data_structs/DataStructQueryFunctions.cpp
   )

SET(ACT_SRC ${SCL_INC_DIR}/actuation/muscles/CActuatorMuscle.cpp
            ${SCL_INC_DIR}/actuation/muscles/CActuatorSetMuscle.cpp
   )

SET(UTIL_SRC ${SCL_INC_DIR}/util/HelperFunctions.cpp
             ${SCL_INC_DIR}/util/FileFunctions.cpp
             ${SCL_INC_DIR}/util/RobotMath.cpp
             ${SCL_INC_DIR}/util/CmdLineArgReader.cpp
             ${SCL_INC_DIR}/util/EigenExtensions.cpp
   )

SET(ROBOT_SRC ${SCL_INC_DIR}/robot/CRobot.cpp
              ${SCL_INC_DIR}/robot/data_structs/SRobot.cpp
              ${SCL_INC_DIR}/robot/DbRegisterFunctions.cpp
   )
   
SET(SHELL_SRC ${SCL_INC_DIR}/shell/ConsoleShell.cpp)

SET(CTR_SRC ${SCL_INC_DIR}/control/data_structs/SControllerBase.cpp
            ${SCL_INC_DIR}/control/gc/data_structs/SControllerGc.cpp
            ${SCL_INC_DIR}/control/gc/CControllerGc.cpp
            ${SCL_INC_DIR}/control/task/data_structs/SServo.cpp
            ${SCL_INC_DIR}/control/task/data_structs/STaskBase.cpp
            ${SCL_INC_DIR}/control/task/data_structs/SNonControlTaskBase.cpp
            ${SCL_INC_DIR}/control/task/data_structs/SControllerMultiTask.cpp
            ${SCL_INC_DIR}/control/task/CServo.cpp
            ${SCL_INC_DIR}/control/task/CControllerMultiTask.cpp
   )
   
SET(CTR_TASKS_SRC ${SCL_INC_DIR}/control/task/tasks/CTaskNULL.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskOpPos.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskOpPos.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskOpPosPIDA1OrderInfTime.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskOpPosPIDA1OrderInfTime.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskNullSpaceDamping.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskNullSpaceDamping.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskGc.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskGc.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskGcLimitCentering.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskGcLimitCentering.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskGcSet.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskGcSet.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskComPos.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskComPos.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskConstraintPlane.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskConstraintPlane.cpp
   )
   
   
SET(CTR2_TASKS_SRC ${SCL_INC_DIR}/control/tasks/CTaskOpPos.cpp
                  ${SCL_INC_DIR}/control/tasks/data_structs/STaskOpPos.cpp
                  ${SCL_INC_DIR}/control/tasks/data_structs/STaskBase.cpp
   )
   
# This needs some work. Should add basic trajectory generation support.
SET(CTR_TRAJ_SRC 
   )

SET(DYN_ANALYTIC_SRC ${SCL_INC_DIR}/dynamics/analytic/CDynamicsAnalyticRPP.cpp
   )
   
SET(DYN_SCL_SRC ${SCL_INC_DIR}/dynamics/scl/CDynamicsScl.cpp
                ${SCLEXT_INC_DIR}/dynamics/scl_spatial/CDynamicsSclSpatial.cpp
                ${SCLEXT_INC_DIR}/dynamics/scl_spatial/CDynamicsSclSpatialMath.cpp
   )
   
SET(SCL_PARSER_SRC ${SCL_INC_DIR}/parser/sclparser/CParserScl.cpp
                     ${SCL_INC_DIR}/parser/sclparser/tixml_parser/CParserSclTiXml.cpp
   )
   
SET(SCL_IO_SRC ${SCL_INC_DIR}/io/CIORedis.cpp
   )

SET(SCL_SERIALIZATION_SRC ${SCL_INC_DIR}/serialization/SerializationJSON.cpp
   )
   
SET(SCL_CALLBACKS_SRC ${SCL_INC_DIR}/callbacks/GenericCallbacks.cpp
                      ${SCL_INC_DIR}/callbacks/PrintablesJSON.cpp
   )

SET(OSIM_PARSER_SRC ${SCL_INC_DIR}/parser/osimparser/CParserOsim.cpp
                    ${SCL_INC_DIR}/parser/osimparser/CParserOsimForOldFiles.cpp)

#Set tinyxml stuff
SET(TIXML_SRC ${TIXML_INC_DIR}/scl_tinyxml/scl_tinystr.cpp
            ${TIXML_INC_DIR}/scl_tinyxml/scl_tinyxml.cpp
            ${TIXML_INC_DIR}/scl_tinyxml/scl_tinyxmlerror.cpp
            ${TIXML_INC_DIR}/scl_tinyxml/scl_tinyxmlparser.cpp
   )
ADD_DEFINITIONS(-DTIXML_USE_STL -std=c++11)

SET(ALLSRC ${SCL_BASE_SRC} ${DB_SRC} ${ACT_SRC} ${UTIL_SRC} ${ROBOT_SRC} ${SHELL_SRC}
           ${CTR_SRC} ${CTR_TASKS_SRC} ${CTR2_TASKS_SRC} ${CTR_TRAJ_SRC} 
           ${DYN_ANALYTIC_SRC} ${DYN_SCL_SRC}
           ${SCL_PARSER_SRC} ${OSIM_PARSER_SRC}
           ${SCL_IO_SRC} ${SCL_SERIALIZATION_SRC} ${SCL_CALLBACKS_SRC}
           ${TIXML_SRC})

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  #Add debug definitions
  ADD_DEFINITIONS( -DASSERT=assert -DDEBUG=1)

  # Disable vectorization on 32 bit DEBUG mode. Doesn't work yet. Eigen problem
  #NOTE : Checks for 32 vs. 64 bit machine.
  if ( "${CMAKE_SIZEOF_VOID_P}" EQUAL "8" )
    SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -Wno-unused-parameter -ggdb -O0 -pg -fopenmp")
  elseif( "${CMAKE_SIZEOF_VOID_P}" EQUAL "4" )
    SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -Wno-unused-parameter -ggdb -O0 -pg -fopenmp -DEIGEN_DONT_ALIGN_STATICALLY")
  endif()
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

IF(CMAKE_BUILD_TYPE MATCHES Release)
  #Add release definitions
  ADD_DEFINITIONS( -DEIGEN_NO_DEBUG )
  SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -fopenmp")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

#Define the library to be built
add_library(scl STATIC ${ALLSRC})

#Link the library with the 3rd party libs
target_link_libraries(scl ${SCL_3PARTY_LIBS})
