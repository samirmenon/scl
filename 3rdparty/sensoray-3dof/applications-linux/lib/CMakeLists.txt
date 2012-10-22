cmake_minimum_required(VERSION 2.0)

#Name the project
project(sensoray-3dof)

#Set Include directories
SET(SENSORAY_BASE_DIR ../../)

SET(SENSORAY_INC_DIR ${SENSORAY_BASE_DIR}src/sensoray)
SET(SENSORAY_COMMON_INC_DIR ${SENSORAY_BASE_DIR}src/sensoray/common)
SET(SENSORAY_LINUX_INC_DIR ${SENSORAY_BASE_DIR}src/sensoray/linux)

INCLUDE_DIRECTORIES(${SENSORAY_INC_DIR} ${SENSORAY_COMMON_INC_DIR} ${SENSORAY_LINUX_INC_DIR})

#Set all the sources required for the library
SET(SENSORAY_SRC ${SENSORAY_INC_DIR}/common/s26core.c
             ${SENSORAY_INC_DIR}/linux/s26mod.c
             ${SENSORAY_INC_DIR}/CSensoray3DofIO.cpp)

SET(SENSORAY_ALL_SRC ${SENSORAY_SRC}) 

######################END OF SRC DEFS##############

ADD_DEFINITIONS(-Wall -m32 -DOSTYPE_LINUX -DENDIAN_LITTLE -fPIC)

#Set the build mode to debug by default
IF(CMAKE_BUILD_TYPE MATCHES Debug)
  #Add debug definitions
  ADD_DEFINITIONS(-DASSERT=assert -DDEBUG=1)
  SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -ggdb -O0 -g -pg")
  SET(CMAKE_C_FLAGS_DEBUG "-Wall -ggdb -O0 -g -pg")
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

IF(CMAKE_BUILD_TYPE MATCHES Release)
  #Add release definitions
  SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")
  SET(CMAKE_C_FLAGS_RELEASE "-Wall -O3")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

#Define the libraries to be built (cmake sources + 3rdparty libs)
ADD_LIBRARY(sensoray-3dof STATIC ${SENSORAY_ALL_SRC})
