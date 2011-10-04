cmake_minimum_required(VERSION 2.6)

#Name the project
project(scl)

#Set Include directories
SET(WBC_INC_DIR ../../src/scl)

#Set 3rd party dependencies
#Eigen Includes
SET(EIGEN_INC_DIR ../../3rdparty/eigen3.0.2)

#sUtil Includes
SET(SUTIL_INC_DIR ../../3rdparty/sUtil/src)

SET(TAO_INC_DIR ${WBC_INC_DIR}/dynamics/tao)


INCLUDE_DIRECTORIES(../../src/ ${WBC_INC_DIR}
                    ${EIGEN_INC_DIR} ${TAO_INC_DIR}		    						
                    ${SUTIL_INC_DIR} )

#Set all the sources required for the library
SET(DB_SRC ${WBC_INC_DIR}/data_structs/SRobotIOData.cpp
           ${WBC_INC_DIR}/data_structs/SRobotLink.cpp
           ${WBC_INC_DIR}/data_structs/SMuscleSystem.cpp
   )

SET(UTIL_SRC ${WBC_INC_DIR}/util/DatabaseUtils.cpp
             ${WBC_INC_DIR}/util/HelperFunctions.cpp
             ${WBC_INC_DIR}/util/FileFunctions.cpp
   )

SET(ROBOT_SRC ${WBC_INC_DIR}/robot/CRobot.cpp
              ${WBC_INC_DIR}/robot/data_structs/SRobot.cpp
              ${WBC_INC_DIR}/robot/DbRegisterFunctions.cpp
   )

SET(CTR_SRC ${WBC_INC_DIR}/control/data_structs/SControllerBase.cpp
            ${WBC_INC_DIR}/control/data_structs/SGcModel.cpp
            ${WBC_INC_DIR}/control/gc/data_structs/SGcController.cpp
            ${WBC_INC_DIR}/control/gc/CGcController.cpp
            ${WBC_INC_DIR}/control/task/data_structs/SServo.cpp
            ${WBC_INC_DIR}/control/task/data_structs/STaskBase.cpp
            ${WBC_INC_DIR}/control/task/data_structs/STaskController.cpp
            ${WBC_INC_DIR}/control/task/CServo.cpp
            ${WBC_INC_DIR}/control/task/CTaskController.cpp
   )
   
SET(CTR_TASKS_SRC ${WBC_INC_DIR}/control/task/tasks/CTaskNULL.cpp
                  ${WBC_INC_DIR}/control/task/tasks/COpPointTask.cpp
                  ${WBC_INC_DIR}/control/task/tasks/data_structs/SOpPointTask.cpp
                  ${WBC_INC_DIR}/control/task/tasks/CTaskNullSpaceDamping.cpp
                  ${WBC_INC_DIR}/control/task/tasks/data_structs/STaskNullSpaceDamping.cpp
                  ${WBC_INC_DIR}/control/task/tasks/CTaskGc.cpp
                  ${WBC_INC_DIR}/control/task/tasks/data_structs/STaskGc.cpp
                  ${WBC_INC_DIR}/control/task/tasks/CTaskGcSet.cpp
                  ${WBC_INC_DIR}/control/task/tasks/data_structs/STaskGcSet.cpp
   )
   
SET(CTR_TRAJ_SRC ${WBC_INC_DIR}/control/trajectory/OTG/TypeIOTG.cpp
                 ${WBC_INC_DIR}/control/trajectory/OTG/TypeIOTGDecision.cpp
                 ${WBC_INC_DIR}/control/trajectory/OTG/TypeIOTGMath.cpp
                 ${WBC_INC_DIR}/control/trajectory/OTG/TypeIOTGPolynomial.cpp
                 ${WBC_INC_DIR}/control/trajectory/OTG/TypeIOTGProfiles.cpp
   )

SET(TAO_SRC ${WBC_INC_DIR}/dynamics/tao/tao/dynamics/taoCNode.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/dynamics/taoABDynamics.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/dynamics/taoABJoint.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/dynamics/taoWorld.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/dynamics/taoGroup.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/dynamics/taoDynamics.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/dynamics/taoJoint.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/dynamics/taoNode.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/dynamics/taoABNode.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/utility/TaoDeMassProp.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/utility/TaoDeLogger.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/matrix/TaoDeQuaternionf.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/matrix/TaoDeMatrix6.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/matrix/TaoDeVector6.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/matrix/TaoDeMatrix3f.cpp
            ${WBC_INC_DIR}/dynamics/tao/tao/matrix/TaoDeTransform.cpp
            ${WBC_INC_DIR}/dynamics/tao/jspace/vector_util.cpp
            ${WBC_INC_DIR}/dynamics/tao/jspace/Status.cpp
            ${WBC_INC_DIR}/dynamics/tao/jspace/State.cpp
            ${WBC_INC_DIR}/dynamics/tao/jspace/Model.cpp
            ${WBC_INC_DIR}/dynamics/tao/jspace/wrap_eigen.cpp
            ${WBC_INC_DIR}/dynamics/tao/jspace/tao_util.cpp
            ${WBC_INC_DIR}/dynamics/tao/jspace/tao_dump.cpp
            ${WBC_INC_DIR}/dynamics/tao/jspace/strutil.cpp
            ${WBC_INC_DIR}/dynamics/tao/CTaoDynamics.cpp
            ${WBC_INC_DIR}/dynamics/tao/CTaoRepCreator.cpp
   )

SET(LOTUS_PARSER_SRC ${WBC_INC_DIR}/parser/lotusparser/CLotusParser.cpp
                     ${WBC_INC_DIR}/parser/lotusparser/tixml_parser/CLotusTiXmlParser.cpp
   )

SET(SAI_PARSER_SRC ${WBC_INC_DIR}/parser/saiparser/CSaiParser.cpp)

SET(OSIM_PARSER_SRC ${WBC_INC_DIR}/parser/osimparser/COsimParser.cpp
                    ${WBC_INC_DIR}/parser/osimparser/COsimParserForOldFiles.cpp)

#Set tinyxml stuff
SET(TIXML_SRC ${WBC_INC_DIR}/parser/scl_tinyxml/scl_tinystr.cpp
            ${WBC_INC_DIR}/parser/scl_tinyxml/scl_tinyxml.cpp
            ${WBC_INC_DIR}/parser/scl_tinyxml/scl_tinyxmlerror.cpp
            ${WBC_INC_DIR}/parser/scl_tinyxml/scl_tinyxmlparser.cpp
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
add_library(scl SHARED ${ALLSRC})

#Link the library with the 3rd party libs
target_link_libraries(scl ${WBC_3PARTY_LIBS})
