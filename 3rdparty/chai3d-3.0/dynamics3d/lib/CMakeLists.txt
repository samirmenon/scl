# A convenience file for compiling dynamics3d.
# This does not include access to any dynamics3d code. Merely simplifies
# your build process IF you get the dynamics3d code from elsewhere.
cmake_minimum_required(VERSION 2.0)

#Name the project
project(dynamics3d)

#Set Include directories
SET(CHAI_BASE_DIR ../../)

SET(CHAI_INC_DIR ../../chai3d)
SET(DHD_INC_DIR ../../DHD/include)
SET(GEL_INC_DIR ../../GEL/src)
SET(GIF_INC_DIR ../../giflib/include)
SET(3DS_INC_DIR ../../lib3ds/include)
SET(JPEG_INC_DIR ../../libjpeg/include)
SET(PNG_INC_DIR ../../libpng/include)

SET(DYN3D_INC_DIR ../src)

SET(EIGEN_INC_DIR ../../../eigen3.1.2)

INCLUDE_DIRECTORIES(${CHAI_INC_DIR} ${DHD_INC_DIR} ${GEL_INC_DIR}
                    ${GIF_INC_DIR} ${3DS_INC_DIR} ${JPEG_INC_DIR} 
                    ${PNG_INC_DIR} ${EIGEN_INC_DIR} ${DYN3D_INC_DIR})

#Set all the sources required for the library
SET(DYN3D_SRC ${DYN3D_INC_DIR}/dynamics/CDynABJoint.cpp
              ${DYN3D_INC_DIR}/dynamics/CDynABDynamics.cpp
              ${DYN3D_INC_DIR}/dynamics/CDynDynamics.cpp
              ${DYN3D_INC_DIR}/dynamics/CDynABNode.cpp
              ${DYN3D_INC_DIR}/matrix/CDynVector6.cpp
              ${DYN3D_INC_DIR}/matrix/CDynMatrix6.cpp
	      ${DYN3D_INC_DIR}/matrix/CDynMatrix3f.h
              ${DYN3D_INC_DIR}/var/CDynJointVar.cpp
              ${DYN3D_INC_DIR}/var/CDynState.cpp
              ${DYN3D_INC_DIR}/var/CDynJointSphereVar.cpp
              ${DYN3D_INC_DIR}/var/CDynInterpolate.cpp
              ${DYN3D_INC_DIR}/var/CDynJointSphereInterval.cpp
              ${DYN3D_INC_DIR}/var/CDynJointInterval.cpp
              ${DYN3D_INC_DIR}/extras/CDynHull.cpp
              ${DYN3D_INC_DIR}/object/CDynJointList.cpp
              ${DYN3D_INC_DIR}/object/CDynJoint.cpp
              ${DYN3D_INC_DIR}/object/CDynMaterial.cpp
              ${DYN3D_INC_DIR}/object/CDynMass.cpp
              ${DYN3D_INC_DIR}/object/CDynGeometry.cpp
              ${DYN3D_INC_DIR}/object/CDynPrimitive.cpp
              ${DYN3D_INC_DIR}/object/CDynObject.cpp
              ${DYN3D_INC_DIR}/object/CDynForce.cpp
              ${DYN3D_INC_DIR}/global/CDynGlobal.cpp
              ${DYN3D_INC_DIR}/node/CDynContactPoint.cpp
              ${DYN3D_INC_DIR}/node/CDynConstraintList.cpp
              ${DYN3D_INC_DIR}/node/CDynFrictionRecord.cpp
              ${DYN3D_INC_DIR}/node/CDynSphericalConstraint.cpp
              ${DYN3D_INC_DIR}/node/CDynPinConstraint.cpp
              ${DYN3D_INC_DIR}/node/CDynRayCast.cpp
              ${DYN3D_INC_DIR}/node/CDynWorld.cpp
              ${DYN3D_INC_DIR}/node/CDynAttractConstraint.cpp
              ${DYN3D_INC_DIR}/node/CDynContactList.cpp
              ${DYN3D_INC_DIR}/node/CDynTrackConstraint.cpp
              ${DYN3D_INC_DIR}/node/CDynBound.cpp
              ${DYN3D_INC_DIR}/node/CDynCollisionCheckRecord.cpp
              ${DYN3D_INC_DIR}/node/CDynCollision.cpp
              ${DYN3D_INC_DIR}/node/CDynContactPointCache.cpp
              ${DYN3D_INC_DIR}/node/CDynJointLimit.cpp
              ${DYN3D_INC_DIR}/node/CDynContact.cpp
              ${DYN3D_INC_DIR}/node/CDynBaseNode.cpp
              ${DYN3D_INC_DIR}/node/CDynPrimPair.cpp
              ${DYN3D_INC_DIR}/node/CDynConstraint.cpp
              ${DYN3D_INC_DIR}/distance/CDynHierarchy.cpp
              ${DYN3D_INC_DIR}/distance/CDynCover.cpp
              ${DYN3D_INC_DIR}/distance/CDynBSphere.cpp
              ${DYN3D_INC_DIR}/distance/CDynDist.cpp
              ${DYN3D_INC_DIR}/distance/CDynPrim.cpp
              ${DYN3D_INC_DIR}/chai3d/CDynamicLink.cpp
              ${DYN3D_INC_DIR}/chai3d/CDynamicBase.cpp
              ${DYN3D_INC_DIR}/chai3d/CDynamicJoint.cpp
              ${DYN3D_INC_DIR}/chai3d/CDynamicWorld.cpp
              ${DYN3D_INC_DIR}/chai3d/CDynamicContact.cpp
              ${DYN3D_INC_DIR}/chai3d/CDynamicMaterial.cpp
              ${DYN3D_INC_DIR}/chai3d/CDynamicWorld.cpp~
              ${DYN3D_INC_DIR}/utility/CDynTimeEvent.cpp
              ${DYN3D_INC_DIR}/utility/CDynLogger.cpp
              ${DYN3D_INC_DIR}/utility/CDynQHeader.cpp
              ${DYN3D_INC_DIR}/utility/CDynError.cpp
            )

######################END OF SRC DEFS##############

ADD_DEFINITIONS(-D_LINUX -DLINUX -DdDOUBLE)
ADD_DEFINITIONS(-lpthread -lrt -lpci -lz -lusb-1.0
				-lGL -lGLU -lGLEW -lglut -fPIC)


#Set the build mode to debug by default
IF(CMAKE_BUILD_TYPE MATCHES Debug)
  #Add debug definitions
  ADD_DEFINITIONS(-DASSERT=assert -DDEBUG=1)
  SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-overflow=5 -Wextra -ggdb -O0 -g -pg")
  SET(CMAKE_C_FLAGS_DEBUG "-DHAVE_STDARG_H -DHAVE_SYS_TYPES_H -DHAVE_UNISTD_H -DHAVE_CONFIG_H -Wall -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-overflow=5 -Wextra -ggdb -O0 -g -pg")
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

IF(CMAKE_BUILD_TYPE MATCHES Release)
  #Add release definitions
  SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")
  SET(CMAKE_C_FLAGS_RELEASE "-DHAVE_STDARG_H -DHAVE_SYS_TYPES_H -DHAVE_UNISTD_H -DHAVE_CONFIG_H -Wall -O3")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

#Define the libraries to be built (cmake sources + 3rdparty libs)
ADD_LIBRARY(dynamics3d SHARED ${DYN3D_SRC})

# If requires linking: 
# target_link_libraries(dynamics3d imageformats)

###############SPECIAL CODE TO FIND AND LINK DHD's LIB (Coz not open source) ######################
#NOTE : For a 32 bit compile (or other machine arch), please change lin-x86_64 to lin-i686 (or other arch setting)
if ( "${CMAKE_SIZEOF_VOID_P}" EQUAL "8" )
       set(DHD_LIB_TYPE "lin-x86_64")
elseif( "${CMAKE_SIZEOF_VOID_P}" EQUAL "4" )
       set(DHD_LIB_TYPE "lin-i686")
endif()

# Note : The DHD library is not open source so we can't use
# the debugger.
find_library( DHD_LIBRARY NAMES dhd
            PATHS   ${CHAI_BASE_DIR}/DHD/lib/
            PATH_SUFFIXES ${DHD_LIB_TYPE} )

target_link_libraries(dynamics3d ${DHD_LIBRARY})
