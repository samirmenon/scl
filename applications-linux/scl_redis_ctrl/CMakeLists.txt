################Initialize the Cmake Defaults#################

cmake_minimum_required(VERSION 2.6)

#Name the project
project(scl_redis_app)

#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

################Initialize the 3rdParty lib#################

#Set scl base directory
SET(SCL_BASE_DIR ../../)

###(a) Scl controller
SET(SCL_INC_DIR ${SCL_BASE_DIR}src/scl/)
SET(SCL_INC_DIR_BASE ${SCL_BASE_DIR}src/)
ADD_DEFINITIONS(-DTIXML_USE_STL)

###(b) Eigen
SET(EIGEN_INC_DIR ${SCL_BASE_DIR}3rdparty/eigen/)

### (c) sUtil code
SET(SUTIL_INC_DIR ${SCL_BASE_DIR}3rdparty/sUtil/src/)

### (d) scl_tinyxml (parser)
SET(TIXML_INC_DIR ../../3rdparty/tinyxml)

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${SCL_INC_DIR_BASE} ${EIGEN_INC_DIR} ${SUTIL_INC_DIR} ${TIXML_INC_DIR}) 

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++11 -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11 -DNDEBUG")

#Set all the sources required for the library
SET(redis_BASE_DIR ${SCL_BASE_DIR}/applications-linux/scl_redis_ctrl/)

SET(ALL_SRC ${redis_BASE_DIR}/scl_redis_ctrl_main.cpp)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(scl_redis_ctrl ${ALL_SRC})

###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_INC_DIR}/../build_cmake
            PATH_SUFFIXES build_dbg )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_INC_DIR}/../build_cmake
            PATH_SUFFIXES build_rel )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(scl_redis_ctrl ${SCL_LIBRARY})

###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(scl_redis_ctrl hiredis jsoncpp)
