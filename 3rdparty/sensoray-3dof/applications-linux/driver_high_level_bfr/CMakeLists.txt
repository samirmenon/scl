cmake_minimum_required(VERSION 2.0)

#Name the project
project(bfr_driver_hl_test)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

#Set Include directories
SET(SENSORAY_BFR_DEMO_DIR ./)
SET(SENSORAY_BASE_DIR ../../)

SET(SENSORAY_INC_DIR ${SENSORAY_BASE_DIR}src/)
SET(SENSORAY_COMMON_INC_DIR ${SENSORAY_INC_DIR}/sensoray/common)
SET(SENSORAY_LINUX_INC_DIR ${SENSORAY_INC_DIR}/sensoray/linux)

SET(SUTIL_INC_DIR ${SENSORAY_BASE_DIR}../sUtil/src/)

SET(EIGEN_INC_DIR ${SENSORAY_BASE_DIR}../eigen3.1.2/)

INCLUDE_DIRECTORIES(${SENSORAY_INC_DIR} ${SENSORAY_COMMON_INC_DIR} ${SENSORAY_LINUX_INC_DIR} ${SUTIL_INC_DIR} ${EIGEN_INC_DIR})

###############SPECIAL CODE TO SET CONSTEXPR FOR OLD GCC ######################
EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
STRING(REGEX MATCHALL "[0-9]+" GCC_VERSION_COMPONENTS ${GCC_VERSION})
list(GET GCC_VERSION_COMPONENTS 0 GCC_MAJOR)
list(GET GCC_VERSION_COMPONENTS 1 GCC_MINOR)

ADD_DEFINITIONS(-DGCC_${GCC_MAJOR}_${GCC_MINOR})

###############################################################################

#Set all the sources required for the library
SET(BFR_FX_TEST_ALL_SRC ${SENSORAY_BFR_DEMO_DIR}/bfr_driver_hl_fx_test.cpp
                     ${SENSORAY_INC_DIR}/sensoray/CSensoray3DofIODriver.cpp
                     ${SENSORAY_INC_DIR}/sensoray/CBfrDriver.cpp)
                     
SET(BFR_FGC_TEST_ALL_SRC ${SENSORAY_BFR_DEMO_DIR}/bfr_driver_hl_fgc_test.cpp
                     ${SENSORAY_INC_DIR}/sensoray/CSensoray3DofIODriver.cpp
                     ${SENSORAY_INC_DIR}/sensoray/CBfrDriver.cpp)

SET(BFR_ENC_TEST_ALL_SRC ${SENSORAY_BFR_DEMO_DIR}/bfr_driver_hl_encoder_test.cpp
                     ${SENSORAY_INC_DIR}/sensoray/CSensoray3DofIODriver.cpp
                     ${SENSORAY_INC_DIR}/sensoray/CBfrDriver.cpp)


######################END OF SRC DEFS##############
#Add debug definitions
IF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -m32 -ggdb -O0 -g -pg -DDEBUG -std=c++0x")
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

#Add release definitions
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -m32 -O3 -std=c++0x")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

#Define the libraries to be built (cmake sources + 3rdparty libs)
ADD_EXECUTABLE(bfr_driver_hl_fx_test ${BFR_FX_TEST_ALL_SRC})
ADD_EXECUTABLE(bfr_driver_hl_fgc_test ${BFR_FGC_TEST_ALL_SRC})
ADD_EXECUTABLE(bfr_driver_hl_enc_test ${BFR_ENC_TEST_ALL_SRC})


###############SPECIAL CODE TO FIND AND LINK SENSORAY's LIB DIR ######################
find_library( SENSORAY_LIBRARY_DEBUG NAMES sensoray-3dof
            PATHS   ${SENSORAY_BASE_DIR}/applications-linux/lib/
            PATH_SUFFIXES debug )

find_library( SENSORAY_LIBRARY_RELEASE NAMES sensoray-3dof
            PATHS   ${SENSORAY_BASE_DIR}/applications-linux/lib/
            PATH_SUFFIXES release )

SET( SENSORAY_LIBRARY debug     ${SENSORAY_LIBRARY_DEBUG}
              optimized ${SENSORAY_LIBRARY_RELEASE} )

target_link_libraries(bfr_driver_hl_fx_test ${SENSORAY_LIBRARY})
target_link_libraries(bfr_driver_hl_fgc_test ${SENSORAY_LIBRARY})
target_link_libraries(bfr_driver_hl_enc_test ${SENSORAY_LIBRARY})
