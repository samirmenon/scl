cmake_minimum_required(VERSION 2.0)

#Name the project
project(sensoray-3dof)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

#Set Include directories
SET(SENSORAY_BASE_DIR ../../)

SET(SENSORAY_INC_DIR ${SENSORAY_BASE_DIR}src/sensoray)
SET(SENSORAY_COMMON_INC_DIR ${SENSORAY_BASE_DIR}src/sensoray/common)
SET(SENSORAY_LINUX_INC_DIR ${SENSORAY_BASE_DIR}src/sensoray/linux)

INCLUDE_DIRECTORIES(${SENSORAY_INC_DIR} ${SENSORAY_COMMON_INC_DIR} ${SENSORAY_LINUX_INC_DIR})

#Set all the sources required for the library
SET(SENSORAY_ALL_SRC ${SENSORAY_INC_DIR}/common/s26core.c
             ${SENSORAY_INC_DIR}/linux/s26mod.c)

######################END OF SRC DEFS##############
# NOTE : cmake makes gcc use -rdynamic by default. Doing this diables it.
#SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
#SET(CMAKE_LINK_C_FLAGS "")

#Add debug definitions
IF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(CMAKE_C_FLAGS_DEBUG "-Wall -m32 -D OSTYPE_LINUX -D ENDIAN_LITTLE -ggdb -O0 -g -pg")
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

#Add release definitions
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(CMAKE_C_FLAGS_RELEASE "-Wall -m32 -D OSTYPE_LINUX -D ENDIAN_LITTLE -O3")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

#Define the libraries to be built (cmake sources + 3rdparty libs)
ADD_LIBRARY(sensoray-3dof STATIC ${SENSORAY_ALL_SRC})
