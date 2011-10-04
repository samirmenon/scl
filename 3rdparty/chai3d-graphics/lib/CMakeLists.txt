cmake_minimum_required(VERSION 2.0)

#Name the project
project(wbc)

#Set Include directory
SET(CHAI_TOP_DIR ..)
SET(CHAI_INC_DIR ../chai)
INCLUDE_DIRECTORIES(${CHAI_INC_DIR})

#Set all the sources required for the library
SET(CHAI_SRC ${CHAI_INC_DIR}/display/CViewport.cpp
            ${CHAI_INC_DIR}/widgets/CBitmap.cpp
            ${CHAI_INC_DIR}/widgets/CFont.cpp
            ${CHAI_INC_DIR}/widgets/CLabel.cpp
            ${CHAI_INC_DIR}/forces/CInteractionBasics.cpp
            ${CHAI_INC_DIR}/forces/CProxyPointForceAlgo.cpp
            ${CHAI_INC_DIR}/forces/CGenericPointForceAlgo.cpp
            ${CHAI_INC_DIR}/forces/CPotentialFieldForceAlgo.cpp
            ${CHAI_INC_DIR}/tools/CGenericTool.cpp
            ${CHAI_INC_DIR}/tools/CGeneric3dofPointer.cpp
            ${CHAI_INC_DIR}/timers/CThread.cpp
            ${CHAI_INC_DIR}/timers/CPrecisionClock.cpp
            ${CHAI_INC_DIR}/devices/CGenericDevice.cpp
            ${CHAI_INC_DIR}/devices/CGenericHapticDevice.cpp
            ${CHAI_INC_DIR}/effects/CGenericEffect.cpp
            ${CHAI_INC_DIR}/effects/CEffectVibration.cpp
            ${CHAI_INC_DIR}/effects/CEffectStickSlip.cpp
            ${CHAI_INC_DIR}/effects/CEffectViscosity.cpp
            ${CHAI_INC_DIR}/effects/CEffectMagnet.cpp
            ${CHAI_INC_DIR}/effects/CEffectSurface.cpp
            ${CHAI_INC_DIR}/graphics/CColor.cpp
            ${CHAI_INC_DIR}/graphics/CMaterial.cpp
            ${CHAI_INC_DIR}/graphics/CGenericTexture.cpp
            ${CHAI_INC_DIR}/graphics/CTriangle.cpp
            ${CHAI_INC_DIR}/graphics/CTexture2D.cpp
            ${CHAI_INC_DIR}/graphics/CVertex.cpp
            ${CHAI_INC_DIR}/graphics/CDraw3D.cpp
            ${CHAI_INC_DIR}/graphics/CMacrosGL.cpp
            ${CHAI_INC_DIR}/scenegraph/CMesh.cpp
            ${CHAI_INC_DIR}/scenegraph/CShapeTorus.cpp
            ${CHAI_INC_DIR}/scenegraph/CShapeLine.cpp
            ${CHAI_INC_DIR}/scenegraph/CShapeSphere.cpp
            ${CHAI_INC_DIR}/scenegraph/CWorld.cpp
            ${CHAI_INC_DIR}/scenegraph/CGenericObject.cpp
            ${CHAI_INC_DIR}/scenegraph/CLight.cpp
            ${CHAI_INC_DIR}/scenegraph/CCamera.cpp
            ${CHAI_INC_DIR}/extras/CExtras.cpp
            ${CHAI_INC_DIR}/collisions/CCollisionBasics.cpp
            ${CHAI_INC_DIR}/collisions/CCollisionAABBTree.cpp
            ${CHAI_INC_DIR}/collisions/CCollisionSpheresGeometry.cpp
            ${CHAI_INC_DIR}/collisions/CGenericCollision.cpp
            ${CHAI_INC_DIR}/collisions/CCollisionAABBBox.cpp
            ${CHAI_INC_DIR}/collisions/CCollisionAABB.cpp
            ${CHAI_INC_DIR}/collisions/CCollisionBrute.cpp
            ${CHAI_INC_DIR}/collisions/CCollisionSpheres.cpp
            ${CHAI_INC_DIR}/math/CMatrix3d.cpp
            ${CHAI_INC_DIR}/math/CString.cpp
            ${CHAI_INC_DIR}/math/CQuaternion.cpp
            ${CHAI_INC_DIR}/math/CMaths.cpp
            ${CHAI_INC_DIR}/math/CVector3d.cpp
            ${CHAI_INC_DIR}/files/CFileLoaderOBJ.cpp
            ${CHAI_INC_DIR}/files/CFileLoader3DS.cpp
            ${CHAI_INC_DIR}/files/CMeshLoader.cpp
            ${CHAI_INC_DIR}/files/CFileLoaderBMP.cpp
            ${CHAI_INC_DIR}/files/CImageLoader.cpp
            ${CHAI_INC_DIR}/files/CFileLoaderTGA.cpp
            )

SET(ALLSRC ${CHAI_SRC})

######################END OF SRC DEFS##############

#IF(UNIX MATCHES TRUE)
ADD_DEFINITIONS(-D_LINUX -DdDOUBLE)
ADD_DEFINITIONS(-lpthread -lrt -lpci -lz -lusb 
		-lGL -lGLU -lglut)
#ENDIF(UNIX MATCHES TRUE)


#Set the build mode to debug by default
#SET(CMAKE_BUILD_TYPE Debug)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  #Add debug definitions
  ADD_DEFINITIONS(-DASSERT=assert -DDEBUG=1)
  SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -ggdb -O0 -g -pg")
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

IF(CMAKE_BUILD_TYPE MATCHES Release)
  #Add release definitions
  SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

#Define the library to be built
ADD_LIBRARY(chai3d SHARED ${ALLSRC} ${ALL_LIBS})
