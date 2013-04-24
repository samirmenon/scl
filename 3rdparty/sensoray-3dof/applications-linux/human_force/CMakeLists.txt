cmake_minimum_required(VERSION 2.0)

#Name the project
project(human_force)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

#Set Include directories
SET(SENSORAY_BFR_DEMO_DIR ./)
SET(SENSORAY_BASE_DIR ../../)

SET(SENSORAY_INC_DIR ${SENSORAY_BASE_DIR}src/)
SET(SENSORAY_COMMON_INC_DIR ${SENSORAY_INC_DIR}/sensoray/common)
SET(SENSORAY_LINUX_INC_DIR ${SENSORAY_INC_DIR}/sensoray/linux)

SET(SUTIL_INC_DIR ${SENSORAY_BASE_DIR}../sUtil/src/)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the 3rdParty lib#################

#Set scl base directory
SET(SCL_BASE_DIR ../../)

###(a) Scl controller
SET(SCL_INC_DIR ${SCL_BASE_DIR}src/scl/)
SET(SCL_INC_DIR_BASE ${SCL_BASE_DIR}src/)
SET(TAO_INC_DIR ${SCL_INC_DIR}dynamics/tao/)
ADD_DEFINITIONS(-DTIXML_USE_STL)

###(b) Eigen
SET(EIGEN_INC_DIR ${SCL_BASE_DIR}3rdparty/eigen3.1.2/)

###(c) Chai3D scenegraph
SET(CHAI_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/chai3d/)
ADD_DEFINITIONS(-D_LINUX -DLINUX)

### (d) sUtil code
SET(SUTIL_INC_DIR ${SCL_BASE_DIR}3rdparty/sUtil/src/)


INCLUDE_DIRECTORIES(${SENSORAY_INC_DIR} ${SENSORAY_COMMON_INC_DIR} ${SENSORAY_LINUX_INC_DIR} ${SUTIL_INC_DIR})

#Set all the sources required for the library
SET(SENSORAY_ALL_SRC ${SENSORAY_BFR_DEMO_DIR}/human_force_main.cpp
                     ${SENSORAY_INC_DIR}/sensoray/CSensoray3DofIODriver.cpp)

######################END OF SRC DEFS##############
#Add debug definitions
IF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -m32 -ggdb -O0 -g -pg")
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

#Add release definitions
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -m32 -O3")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

#Define the libraries to be built (cmake sources + 3rdparty libs)
ADD_EXECUTABLE(human_force ${SENSORAY_ALL_SRC})

###############SPECIAL CODE TO FIND AND LINK SENSORAY's LIB DIR ######################
find_library( SENSORAY_LIBRARY_DEBUG NAMES sensoray-3dof
            PATHS   ${SENSORAY_BASE_DIR}/applications-linux/lib/
            PATH_SUFFIXES debug )

find_library( SENSORAY_LIBRARY_RELEASE NAMES sensoray-3dof
            PATHS   ${SENSORAY_BASE_DIR}/applications-linux/lib/
            PATH_SUFFIXES release )

SET( SENSORAY_LIBRARY debug     ${SENSORAY_LIBRARY_DEBUG}
              optimized ${SENSORAY_LIBRARY_RELEASE} )

target_link_libraries(human_force ${SENSORAY_LIBRARY})

