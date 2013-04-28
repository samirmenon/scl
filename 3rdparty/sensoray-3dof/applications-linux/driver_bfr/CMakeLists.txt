cmake_minimum_required(VERSION 2.0)

#Name the project
project(bfr_driver_test)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

#Set Include directories
SET(SENSORAY_BFR_DEMO_DIR ./)
SET(SENSORAY_BASE_DIR ../../)

SET(SENSORAY_INC_DIR ${SENSORAY_BASE_DIR}src/)
SET(SENSORAY_COMMON_INC_DIR ${SENSORAY_INC_DIR}/sensoray/common)
SET(SENSORAY_LINUX_INC_DIR ${SENSORAY_INC_DIR}/sensoray/linux)

SET(SUTIL_INC_DIR ${SENSORAY_BASE_DIR}../sUtil/src/)

INCLUDE_DIRECTORIES(${SENSORAY_INC_DIR} ${SENSORAY_COMMON_INC_DIR} ${SENSORAY_LINUX_INC_DIR} ${SUTIL_INC_DIR})

#Set all the sources required for the library
SET(SENSORAY_ALL_SRC ${SENSORAY_BFR_DEMO_DIR}/bfr_driver_test.cpp
                     ${SENSORAY_INC_DIR}/sensoray/CSensoray3DofIODriver.cpp)

SET(SENSORAY_FX_SRC ${SENSORAY_BFR_DEMO_DIR}/bfr_driver_test_fx.cpp
                     ${SENSORAY_INC_DIR}/sensoray/CSensoray3DofIODriver.cpp)

SET(SENSORAY_RAW_SRC ${SENSORAY_BFR_DEMO_DIR}/bfr_driver_test_raw_input.cpp
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
ADD_EXECUTABLE(bfr_driver_test ${SENSORAY_ALL_SRC})
ADD_EXECUTABLE(bfr_driver_fx ${SENSORAY_FX_SRC})
ADD_EXECUTABLE(bfr_driver_raw_input ${SENSORAY_RAW_SRC})


###############SPECIAL CODE TO FIND AND LINK SENSORAY's LIB DIR ######################
find_library( SENSORAY_LIBRARY_DEBUG NAMES sensoray-3dof
            PATHS   ${SENSORAY_BASE_DIR}/applications-linux/lib/
            PATH_SUFFIXES debug )

find_library( SENSORAY_LIBRARY_RELEASE NAMES sensoray-3dof
            PATHS   ${SENSORAY_BASE_DIR}/applications-linux/lib/
            PATH_SUFFIXES release )

SET( SENSORAY_LIBRARY debug     ${SENSORAY_LIBRARY_DEBUG}
              optimized ${SENSORAY_LIBRARY_RELEASE} )

target_link_libraries(bfr_driver_test ${SENSORAY_LIBRARY})
target_link_libraries(bfr_driver_fx ${SENSORAY_LIBRARY})
target_link_libraries(bfr_driver_raw_input ${SENSORAY_LIBRARY})


