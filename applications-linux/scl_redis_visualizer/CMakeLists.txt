################Initialize the Cmake Defaults#################
cmake_minimum_required(VERSION 2.6)

#Name the project
project(scl_redis_visualizer_app)

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

###(c) Chai3D scenegraph
SET(CHAI_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d.git/src/)
ADD_DEFINITIONS(-D_LINUX -DLINUX)

### (d) sUtil code
SET(SUTIL_INC_DIR ${SCL_BASE_DIR}3rdparty/sUtil/src/)

### (e) scl_tinyxml (parser)
SET(TIXML_INC_DIR ../../3rdparty/tinyxml)

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${SCL_INC_DIR_BASE} ${EIGEN_INC_DIR} ${CHAI_INC_DIR} ${SUTIL_INC_DIR} ${TIXML_INC_DIR}) 

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++11 -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11 -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(TUTE3_BASE_DIR ${SCL_BASE_DIR}/applications-linux/scl_redis_visualizer/)

#These are the source files that will be compiled.
SET(ALL_SRC ${TUTE3_BASE_DIR}/scl_redis_visualizer_main.cpp
            ${SCL_INC_DIR}/graphics/chai/ChaiGlutHandlers.cpp)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(scl_redis_visualizer ${ALL_SRC})

###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( SCL_LIBRARY_DEBUG NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES debug )

find_library( SCL_LIBRARY_RELEASE NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES release )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(scl_redis_visualizer ${SCL_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK CHAI's LIB DIR ######################
find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_INC_DIR}/../build_cmake
            PATH_SUFFIXES build_dbg )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_INC_DIR}/../build_cmake
            PATH_SUFFIXES build_rel )

target_link_libraries(scl_redis_visualizer ${CHAI_LIBRARY})


###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(scl_redis_visualizer gomp GL GLU GLEW glut ncurses rt dl hiredis jsoncpp)
