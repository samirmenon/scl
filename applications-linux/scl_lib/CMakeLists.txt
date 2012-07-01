cmake_minimum_required(VERSION 2.6)

#Name the project
project(scl)

#Set Include directories
SET(SCL_INC_DIR ../../src/scl)

#Set 3rd party dependencies
#Eigen Includes
SET(EIGEN_INC_DIR ../../3rdparty/eigen3.0.5)

#sUtil Includes
SET(SUTIL_INC_DIR ../../3rdparty/sUtil/src)

SET(TAO_INC_DIR ${SCL_INC_DIR}/dynamics/tao)


INCLUDE_DIRECTORIES(../../src/ ${SCL_INC_DIR}
                    ${EIGEN_INC_DIR} ${TAO_INC_DIR}		    						
                    ${SUTIL_INC_DIR} )

#Set all the sources required for the library
SET(DB_SRC ${SCL_INC_DIR}/data_structs/SRobotIOData.cpp
           ${SCL_INC_DIR}/data_structs/SRobotLink.cpp
           ${SCL_INC_DIR}/data_structs/SMuscleSystem.cpp
   )

SET(UTIL_SRC ${SCL_INC_DIR}/util/DatabaseUtils.cpp
             ${SCL_INC_DIR}/util/HelperFunctions.cpp
             ${SCL_INC_DIR}/util/FileFunctions.cpp
   )

SET(ROBOT_SRC ${SCL_INC_DIR}/robot/CRobot.cpp
              ${SCL_INC_DIR}/robot/data_structs/SRobot.cpp
              ${SCL_INC_DIR}/robot/DbRegisterFunctions.cpp
   )

SET(CTR_SRC ${SCL_INC_DIR}/control/data_structs/SControllerBase.cpp
            ${SCL_INC_DIR}/control/data_structs/SGcModel.cpp
            ${SCL_INC_DIR}/control/gc/data_structs/SGcController.cpp
            ${SCL_INC_DIR}/control/gc/CGcController.cpp
            ${SCL_INC_DIR}/control/task/data_structs/SServo.cpp
            ${SCL_INC_DIR}/control/task/data_structs/STaskBase.cpp
            ${SCL_INC_DIR}/control/task/data_structs/SNonControlTaskBase.cpp
            ${SCL_INC_DIR}/control/task/data_structs/STaskController.cpp
            ${SCL_INC_DIR}/control/task/CServo.cpp
            ${SCL_INC_DIR}/control/task/CTaskController.cpp
   )
   
SET(CTR_TASKS_SRC ${SCL_INC_DIR}/control/task/tasks/CTaskNULL.cpp
                  ${SCL_INC_DIR}/control/task/tasks/COpPointTask.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/SOpPointTask.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskNullSpaceDamping.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskNullSpaceDamping.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskGc.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskGc.cpp
                  ${SCL_INC_DIR}/control/task/tasks/CTaskGcSet.cpp
                  ${SCL_INC_DIR}/control/task/tasks/data_structs/STaskGcSet.cpp
   )
   
SET(CTR_TRAJ_SRC ${SCL_INC_DIR}/control/trajectory/OTG/TypeIOTG.cpp
                 ${SCL_INC_DIR}/control/trajectory/OTG/TypeIOTGDecision.cpp
                 ${SCL_INC_DIR}/control/trajectory/OTG/TypeIOTGMath.cpp
                 ${SCL_INC_DIR}/control/trajectory/OTG/TypeIOTGPolynomial.cpp
                 ${SCL_INC_DIR}/control/trajectory/OTG/TypeIOTGProfiles.cpp
   )

SET(TAO_SRC ${SCL_INC_DIR}/dynamics/tao/tao/dynamics/taoCNode.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/dynamics/taoABDynamics.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/dynamics/taoABJoint.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/dynamics/taoWorld.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/dynamics/taoGroup.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/dynamics/taoDynamics.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/dynamics/taoJoint.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/dynamics/taoNode.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/dynamics/taoABNode.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/utility/TaoDeMassProp.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/utility/TaoDeLogger.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/matrix/TaoDeQuaternionf.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/matrix/TaoDeMatrix6.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/matrix/TaoDeVector6.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/matrix/TaoDeMatrix3f.cpp
            ${SCL_INC_DIR}/dynamics/tao/tao/matrix/TaoDeTransform.cpp
            ${SCL_INC_DIR}/dynamics/tao/jspace/vector_util.cpp
            ${SCL_INC_DIR}/dynamics/tao/jspace/Status.cpp
            ${SCL_INC_DIR}/dynamics/tao/jspace/State.cpp
            ${SCL_INC_DIR}/dynamics/tao/jspace/Model.cpp
            ${SCL_INC_DIR}/dynamics/tao/jspace/wrap_eigen.cpp
            ${SCL_INC_DIR}/dynamics/tao/jspace/tao_util.cpp
            ${SCL_INC_DIR}/dynamics/tao/jspace/tao_dump.cpp
            ${SCL_INC_DIR}/dynamics/tao/jspace/strutil.cpp
            ${SCL_INC_DIR}/dynamics/tao/CTaoDynamics.cpp
            ${SCL_INC_DIR}/dynamics/tao/CTaoRepCreator.cpp
   )

SET(LOTUS_PARSER_SRC ${SCL_INC_DIR}/parser/lotusparser/CLotusParser.cpp
                     ${SCL_INC_DIR}/parser/lotusparser/tixml_parser/CLotusTiXmlParser.cpp
   )

SET(SAI_PARSER_SRC ${SCL_INC_DIR}/parser/saiparser/CSaiParser.cpp)

SET(OSIM_PARSER_SRC ${SCL_INC_DIR}/parser/osimparser/COsimParser.cpp
                    ${SCL_INC_DIR}/parser/osimparser/COsimParserForOldFiles.cpp)

#Set tinyxml stuff
SET(TIXML_SRC ${SCL_INC_DIR}/parser/scl_tinyxml/scl_tinystr.cpp
            ${SCL_INC_DIR}/parser/scl_tinyxml/scl_tinyxml.cpp
            ${SCL_INC_DIR}/parser/scl_tinyxml/scl_tinyxmlerror.cpp
            ${SCL_INC_DIR}/parser/scl_tinyxml/scl_tinyxmlparser.cpp
   )
ADD_DEFINITIONS(-DTIXML_USE_STL)

SET(ALLSRC ${DB_SRC} ${UTIL_SRC} ${ROBOT_SRC} 
           ${CTR_SRC} ${CTR_TASKS_SRC} ${CTR_TRAJ_SRC} 
           ${TAO_SRC} 
					 ${LOTUS_PARSER_SRC} ${SAI_PARSER_SRC} ${OSIM_PARSER_SRC} ${TIXML_SRC})

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  #Add debug definitions
  ADD_DEFINITIONS( -DASSERT=assert -DDEBUG=1)
  SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -ggdb -O0 -pg -DW_TESTING -fopenmp")
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
