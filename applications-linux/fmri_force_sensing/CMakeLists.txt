################Initialize the Cmake Defaults#################

cmake_minimum_required(VERSION 2.6)

#Name the project
project(fmri_force_sensing)

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
SET(TAO_INC_DIR ${SCL_INC_DIR}dynamics/tao/)
ADD_DEFINITIONS(-DTIXML_USE_STL)

###(b) Eigen
SET(EIGEN_INC_DIR ${SCL_BASE_DIR}3rdparty/eigen3.1.2/)

###(c) Chai3D scenhapticraph
SET(CHAI_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/chai3d/)
SET(CHAI_LIB_DIR ${CHAI_INC_DIR}../lib/)
ADD_DEFINITIONS(-D_LINUX -DLINUX)

### (d) sUtil code
SET(SUTIL_INC_DIR ${SCL_BASE_DIR}3rdparty/sUtil/src/)

###(d) Cdhd
SET(CDHD_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/DHD/include)

###(e) CGEL
SET(CGEL_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/GEL/src)

###(f) CGIFLIB
SET(CGIFLIB_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/giflib/include)

###(g) CLIB3DS
SET(CLIB3DS_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/lib3ds/include)

###(h) CLIBJPEG
SET(CLIBJPEG_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/libjpeg/include)

###(i) CLIBPNG
SET(CLIBPNG_INC_DIR ${SCL_BASE_DIR}3rdparty/chai3d-3.0/libpng/include)

###(j) Sensoray
#Add Sensoray drivers
SET(SENSORAY_INC_DIR ${SCL_BASE_DIR}3rdparty/sensoray-3dof/src/)
SET(SENSORAY_COMMON_INC_DIR ${SENSORAY_INC_DIR}/sensoray/common)
SET(SENSORAY_LINUX_INC_DIR ${SENSORAY_INC_DIR}/sensoray/linux)

###(k) rs232 (Serial port communication)
SET(RS232_INC_DIR ${SCL_BASE_DIR}3rdparty/rs232)

################Initialize the executable#################
#Set the include directories
INCLUDE_DIRECTORIES(${SCL_INC_DIR} ${SCL_INC_DIR_BASE} ${TAO_INC_DIR} ${EIGEN_INC_DIR} ${CHAI_INC_DIR} ${SUTIL_INC_DIR}
${CDHD_INC_DIR} ${CGEL_INC_DIR} ${CGIFLIB_INC_DIR} ${CLIB3DS_INC_DIR} ${CLIBJPEG_INC_DIR} ${CLIBPNG_INC_DIR}
${SENSORAY_INC_DIR} ${SENSORAY_COMMON_INC_DIR} ${SENSORAY_LINUX_INC_DIR}
${RS232_INC_DIR})

#Set the compilation flags
# NOTE : This is only to detect 32 vs 64 bit code.
# The else is not really an else, just a continuation of the if. CMake syntax.
# The bfr is supported only on 32 bit systems. Else use the Force Dim devices.
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  # 64 bit..
  SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp")
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  # 32 bit..
  SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp -DC_ENABLE_BFR_DEVICE_SUPPORT")
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

#Set all the sources required for the library
SET(EG_BASE_DIR ${SCL_BASE_DIR}/applications-linux/fmri_force_sensing/)

SET(ALL_SRC ${EG_BASE_DIR}fmri_force_sensing_main.cpp
            ${SCL_INC_DIR}/graphics/chai/CChaiGraphics.cpp 
            ${SCL_INC_DIR}/graphics/chai/ChaiGlutHandlers.cpp
            ${SENSORAY_INC_DIR}/sensoray/CSensoray3DofIODriver.cpp
            ${SENSORAY_INC_DIR}/sensoray/CBfrDriver.cpp
            ${RS232_INC_DIR}/rs232.c)

#Set the executable to be built and its required linked libraries (the ones in the /usr/lib dir)
add_executable(fmri_force_sensing ${ALL_SRC})

###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( SCL_LIBRARY_DEBUG NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES debug )

find_library( SCL_LIBRARY_RELEASE NAMES scl
            PATHS   ${SCL_BASE_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES release )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(fmri_force_sensing ${SCL_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK CHAI's LIB DIR ######################
find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_LIB_DIR}
            PATH_SUFFIXES debug )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_LIB_DIR}
            PATH_SUFFIXES release )

SET( CHAI_LIBRARY debug     ${CHAI_LIBRARY_DEBUG}
              optimized ${CHAI_LIBRARY_RELEASE} )

target_link_libraries(fmri_force_sensing ${CHAI_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK IMAGE LIB DIR ######################
find_library( IMG_LIBRARY_DEBUG NAMES imageformats
            PATHS   ${CHAI_LIB_DIR}
            PATH_SUFFIXES debug )

find_library( IMG_LIBRARY_RELEASE NAMES imageformats
            PATHS   ${CHAI_LIB_DIR}
            PATH_SUFFIXES release )

SET( IMG_LIBRARY debug     ${IMG_LIBRARY_DEBUG}
              optimized ${IMG_LIBRARY_RELEASE} )

target_link_libraries(fmri_force_sensing ${IMG_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK SENSORAY LIB DIR ######################
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  # 64 bit..
  message(WARNING "NOTE : HAPTICS WILL NOT WORK!!! NON-HAPTIC DEBUG ONLY! Sensoray drivers only work on 32 bit systems." )
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

# 32 bit..
find_library( CSENSORAY_LIBRARY_DEBUG NAMES sensoray-3dof
            PATHS   ${SENSORAY_INC_DIR}../applications-linux/lib/
            PATH_SUFFIXES debug )

find_library( CSENSORAY_LIBRARY_RELEASE NAMES sensoray-3dof
            PATHS   ${SENSORAY_INC_DIR}../applications-linux/lib/
            PATH_SUFFIXES release )

SET( CSENSORAY_LIBRARY debug     ${CSENSORAY_LIBRARY_DEBUG}
              optimized ${CSENSORAY_LIBRARY_RELEASE} )

target_link_libraries(fmri_force_sensing ${CSENSORAY_LIBRARY})


###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(fmri_force_sensing gomp GL GLU glut ncurses usb-1.0 pthread rt pci z)

