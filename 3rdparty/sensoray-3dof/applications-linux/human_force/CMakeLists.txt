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
SET(SCL_BASE_DIR ../../../../)

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


INCLUDE_DIRECTORIES(${SENSORAY_INC_DIR} ${SENSORAY_COMMON_INC_DIR} ${SENSORAY_LINUX_INC_DIR} ${SUTIL_INC_DIR}
${SCL_INC_DIR_BASE} ${SCL_INC_DIR} ${TAO_INC_DIR} ${EIGEN_INC_DIR} ${CHAI_INC_DIR} ${} )

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



###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( SCL_LIBRARY_DEBUG NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES debug )

find_library( SCL_LIBRARY_RELEASE NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES release )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(human_force ${SCL_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK CHAI's LIB DIR ######################
# NOTE : This is only to detect 32 vs 64 bit code.
# The else is not really an else, just a continuation of the if. CMake syntax.
# The bfr is supported only on 32 bit systems. Else use the Force Dim devices.
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  # 64 bit..
  set( CHAI_LIB_PATH ${CHAI_INC_DIR}../lib_haptics/ )
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  # 32 bit..
  set( CHAI_LIB_PATH ${CHAI_INC_DIR}../lib_bfr/ )
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_LIB_PATH}
            PATH_SUFFIXES debug )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_LIB_PATH}
            PATH_SUFFIXES release )

SET( CHAI_LIBRARY debug     ${CHAI_LIBRARY_DEBUG}
              optimized ${CHAI_LIBRARY_RELEASE} )

target_link_libraries(human_force ${CHAI_LIBRARY})


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


###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(human_force gomp GL GLU glut ncurses usb-1.0 pthread rt pci z)