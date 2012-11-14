cmake_minimum_required(VERSION 2.0)

#Name the project
project(chai3d)

#Set Include directories
SET(CHAI_BASE_DIR ..)

SET(CHAI_INC_DIR ../chai3d)
SET(DHD_INC_DIR ../DHD/include)
SET(GEL_INC_DIR ../GEL/src)
SET(GIF_INC_DIR ../giflib/include)
SET(3DS_INC_DIR ../lib3ds/include)
SET(JPEG_INC_DIR ../libjpeg/include)
SET(PNG_INC_DIR ../libpng/include)

SET(EIGEN_INC_DIR ../../eigen3.1.2)

INCLUDE_DIRECTORIES(${CHAI_INC_DIR} ${DHD_INC_DIR} ${GEL_INC_DIR}
                    ${GIF_INC_DIR} ${3DS_INC_DIR} ${JPEG_INC_DIR} 
                    ${PNG_INC_DIR} ${EIGEN_INC_DIR})

#Set all the sources required for the library
SET(CHAI_SRC ${CHAI_INC_DIR}/graphics/CFont.cpp
             ${CHAI_INC_DIR}/graphics/CDraw3D.cpp
             ${CHAI_INC_DIR}/graphics/CColor.cpp
             ${CHAI_INC_DIR}/graphics/CImage.cpp
             ${CHAI_INC_DIR}/graphics/CFog.cpp
             ${CHAI_INC_DIR}/graphics/CTriangle.cpp
             ${CHAI_INC_DIR}/graphics/CEdge.cpp
             ${CHAI_INC_DIR}/graphics/CVertex.cpp
             ${CHAI_INC_DIR}/graphics/GLee.cpp
             ${CHAI_INC_DIR}/graphics/CDisplayList.cpp
             ${CHAI_INC_DIR}/graphics/CPrimitives.cpp
             ${CHAI_INC_DIR}/files/CFileImageJPG.cpp
             ${CHAI_INC_DIR}/files/CFileModelOBJ.cpp
             ${CHAI_INC_DIR}/files/CFileImagePNG.cpp
             ${CHAI_INC_DIR}/files/CFileImageRAW.cpp
             ${CHAI_INC_DIR}/files/CFileImagePPM.cpp
             ${CHAI_INC_DIR}/files/CFileImageBMP.cpp
             ${CHAI_INC_DIR}/files/CFileModel3DS.cpp
             ${CHAI_INC_DIR}/files/CFileImageGIF.cpp
             ${CHAI_INC_DIR}/forces/CGenericForceAlgorithm.cpp
             ${CHAI_INC_DIR}/forces/CAlgorithmPotentialField.cpp
             ${CHAI_INC_DIR}/forces/CInteractionBasics.cpp
             ${CHAI_INC_DIR}/forces/CAlgorithmFingerProxy.cpp
             ${CHAI_INC_DIR}/world/CWorld.cpp
             ${CHAI_INC_DIR}/world/CGenericObject.cpp
             ${CHAI_INC_DIR}/world/CShapeSphere.cpp
             ${CHAI_INC_DIR}/world/CMesh.cpp
             ${CHAI_INC_DIR}/world/CMultiMesh.cpp
             ${CHAI_INC_DIR}/world/CShapeBox.cpp
             ${CHAI_INC_DIR}/world/CShapeTorus.cpp
						 ${CHAI_INC_DIR}/world/CShapeBeltedEllipsoid.cpp
             ${CHAI_INC_DIR}/world/CShapeCylinder.cpp
             ${CHAI_INC_DIR}/world/CShapeLine.cpp
             ${CHAI_INC_DIR}/system/CGlobals.cpp
             ${CHAI_INC_DIR}/system/CString.cpp
             ${CHAI_INC_DIR}/system/CMutex.cpp
             ${CHAI_INC_DIR}/system/CThread.cpp
             ${CHAI_INC_DIR}/timers/CFrequencyCounter.cpp
             ${CHAI_INC_DIR}/timers/CPrecisionClock.cpp
             ${CHAI_INC_DIR}/effects/CEffectSurface.cpp
             ${CHAI_INC_DIR}/effects/CEffectVibration.cpp
             ${CHAI_INC_DIR}/effects/CEffectPotentialField.cpp
             ${CHAI_INC_DIR}/effects/CEffectViscosity.cpp
             ${CHAI_INC_DIR}/effects/CEffectStickSlip.cpp
             ${CHAI_INC_DIR}/effects/CEffectMagnet.cpp
             ${CHAI_INC_DIR}/effects/CGenericEffect.cpp
             ${CHAI_INC_DIR}/display/CViewport.cpp
             ${CHAI_INC_DIR}/display/CCamera.cpp
             ${CHAI_INC_DIR}/widgets/CPanel.cpp
             ${CHAI_INC_DIR}/widgets/CBackground.cpp
             ${CHAI_INC_DIR}/widgets/CLevel.cpp
             ${CHAI_INC_DIR}/widgets/CDial.cpp
             ${CHAI_INC_DIR}/widgets/CScope.cpp
             ${CHAI_INC_DIR}/widgets/CBitmap.cpp
             ${CHAI_INC_DIR}/widgets/CGenericWidget.cpp
             ${CHAI_INC_DIR}/widgets/CLabel.cpp
             ${CHAI_INC_DIR}/devices/CPhantomDevices.cpp
             ${CHAI_INC_DIR}/devices/CHapticDeviceHandler.cpp
             ${CHAI_INC_DIR}/devices/CMyCustomDevice.cpp
             ${CHAI_INC_DIR}/devices/CVirtualDevice.cpp
             ${CHAI_INC_DIR}/devices/CDeltaDevices.cpp
             ${CHAI_INC_DIR}/devices/CFalconDevice.cpp
             ${CHAI_INC_DIR}/devices/CGenericDevice.cpp
             ${CHAI_INC_DIR}/devices/CGenericHapticDevice.cpp
             ${CHAI_INC_DIR}/shaders/CShader.cpp
             ${CHAI_INC_DIR}/shaders/CShaderProgram.cpp
             ${CHAI_INC_DIR}/collisions/CCollisionAABB.cpp
             ${CHAI_INC_DIR}/collisions/CCollisionBrute.cpp
             ${CHAI_INC_DIR}/collisions/CCollisionAABBTree.cpp
             ${CHAI_INC_DIR}/collisions/CGenericCollision.cpp
             ${CHAI_INC_DIR}/collisions/CCollisionAABBBox.cpp
             ${CHAI_INC_DIR}/collisions/CCollisionBasics.cpp
             ${CHAI_INC_DIR}/materials/CTexture1d.cpp
             ${CHAI_INC_DIR}/materials/CMaterial.cpp
             ${CHAI_INC_DIR}/materials/CGenericTexture.cpp
             ${CHAI_INC_DIR}/materials/CTexture2d.cpp
             ${CHAI_INC_DIR}/lighting/CShadowMap.cpp
             ${CHAI_INC_DIR}/lighting/CGenericLight.cpp
             ${CHAI_INC_DIR}/lighting/CSpotLight.cpp
             ${CHAI_INC_DIR}/lighting/CDirectionalLight.cpp
             ${CHAI_INC_DIR}/lighting/CPositionalLight.cpp
             ${CHAI_INC_DIR}/tools/CGenericTool.cpp
             ${CHAI_INC_DIR}/tools/CToolGripper.cpp
             ${CHAI_INC_DIR}/tools/CHapticPoint.cpp
             ${CHAI_INC_DIR}/tools/CToolCursor.cpp
            )

SET(GEL_SRC ${GEL_INC_DIR}/CGELSkeletonNode.cpp
            ${GEL_INC_DIR}/CGELLinearSpring.cpp
            ${GEL_INC_DIR}/CGELWorld.cpp
            ${GEL_INC_DIR}/CGELMesh.cpp
            ${GEL_INC_DIR}/CGELSkeletonLink.cpp
            ${GEL_INC_DIR}/CGELVertex.cpp
            ${GEL_INC_DIR}/CGELMassParticle.cpp
           )

SET(GIF_SRC ${GIF_INC_DIR}/../src/dev2gif.c  
            ${GIF_INC_DIR}/../src/dgif_lib.c  
            ${GIF_INC_DIR}/../src/egif_lib.c  
            ${GIF_INC_DIR}/../src/getarg.c  
            ${GIF_INC_DIR}/../src/gifalloc.c  
            ${GIF_INC_DIR}/../src/gif_err.c  
            ${GIF_INC_DIR}/../src/gif_font.c  
            ${GIF_INC_DIR}/../src/gif_hash.c  
            ${GIF_INC_DIR}/../src/qprintf.c  
            ${GIF_INC_DIR}/../src/quantize.c
           )

SET(3DS_SRC ${3DS_INC_DIR}/../src/lib3ds_atmosphere.c
            ${3DS_INC_DIR}/../src/lib3ds_chunk.c
            ${3DS_INC_DIR}/../src/lib3ds_io.c
            ${3DS_INC_DIR}/../src/lib3ds_math.c
            ${3DS_INC_DIR}/../src/lib3ds_node.c
            ${3DS_INC_DIR}/../src/lib3ds_track.c
            ${3DS_INC_DIR}/../src/lib3ds_viewport.c
            ${3DS_INC_DIR}/../src/lib3ds_background.c
            ${3DS_INC_DIR}/../src/lib3ds_chunktable.c
            ${3DS_INC_DIR}/../src/lib3ds_light.c
            ${3DS_INC_DIR}/../src/lib3ds_matrix.c
            ${3DS_INC_DIR}/../src/lib3ds_quat.c
            ${3DS_INC_DIR}/../src/lib3ds_util.c
            ${3DS_INC_DIR}/../src/lib3ds_camera.c
            ${3DS_INC_DIR}/../src/lib3ds_file.c
            ${3DS_INC_DIR}/../src/lib3ds_material.c
            ${3DS_INC_DIR}/../src/lib3ds_mesh.c
            ${3DS_INC_DIR}/../src/lib3ds_shadow.c
            ${3DS_INC_DIR}/../src/lib3ds_vector.c
           )

SET(JPEG_SRC ${JPEG_INC_DIR}/../src/jaricom.c
             ${JPEG_INC_DIR}/../src/jcapimin.c
             ${JPEG_INC_DIR}/../src/jcapistd.c
             ${JPEG_INC_DIR}/../src/jcarith.c
             ${JPEG_INC_DIR}/../src/jccoefct.c
             ${JPEG_INC_DIR}/../src/jccolor.c
             ${JPEG_INC_DIR}/../src/jcdctmgr.c
             ${JPEG_INC_DIR}/../src/jchuff.c
             ${JPEG_INC_DIR}/../src/jcinit.c
             ${JPEG_INC_DIR}/../src/jcmainct.c
             ${JPEG_INC_DIR}/../src/jcmarker.c
             ${JPEG_INC_DIR}/../src/jcmaster.c
             ${JPEG_INC_DIR}/../src/jcomapi.c
             ${JPEG_INC_DIR}/../src/jcparam.c
             ${JPEG_INC_DIR}/../src/jcprepct.c
             ${JPEG_INC_DIR}/../src/jcsample.c
             ${JPEG_INC_DIR}/../src/jctrans.c
             ${JPEG_INC_DIR}/../src/jdapimin.c
             ${JPEG_INC_DIR}/../src/jdapistd.c
             ${JPEG_INC_DIR}/../src/jdarith.c
             ${JPEG_INC_DIR}/../src/jdatadst.c
             ${JPEG_INC_DIR}/../src/jdatasrc.c
             ${JPEG_INC_DIR}/../src/jdcoefct.c
             ${JPEG_INC_DIR}/../src/jdcolor.c
             ${JPEG_INC_DIR}/../src/jddctmgr.c
             ${JPEG_INC_DIR}/../src/jdhuff.c
             ${JPEG_INC_DIR}/../src/jdinput.c
             ${JPEG_INC_DIR}/../src/jdmainct.c
             ${JPEG_INC_DIR}/../src/jdmarker.c
             ${JPEG_INC_DIR}/../src/jdmaster.c
             ${JPEG_INC_DIR}/../src/jdmerge.c
             ${JPEG_INC_DIR}/../src/jdpostct.c
             ${JPEG_INC_DIR}/../src/jdsample.c
             ${JPEG_INC_DIR}/../src/jdtrans.c
             ${JPEG_INC_DIR}/../src/jerror.c
             ${JPEG_INC_DIR}/../src/jfdctflt.c
             ${JPEG_INC_DIR}/../src/jfdctfst.c
             ${JPEG_INC_DIR}/../src/jfdctint.c
             ${JPEG_INC_DIR}/../src/jidctflt.c
             ${JPEG_INC_DIR}/../src/jidctfst.c
             ${JPEG_INC_DIR}/../src/jidctint.c
             ${JPEG_INC_DIR}/../src/jmemansi.c
             ${JPEG_INC_DIR}/../src/jmemmgr.c
             ${JPEG_INC_DIR}/../src/jmemnobs.c
             ${JPEG_INC_DIR}/../src/jquant1.c
             ${JPEG_INC_DIR}/../src/jquant2.c
             ${JPEG_INC_DIR}/../src/jutils.c
            )

SET(PNG_SRC ${PNG_INC_DIR}/../src/adler32.c
            ${PNG_INC_DIR}/../src/compress.c
            ${PNG_INC_DIR}/../src/crc32.c
            ${PNG_INC_DIR}/../src/deflate.c
            ${PNG_INC_DIR}/../src/gzclose.c
            ${PNG_INC_DIR}/../src/gzlib.c
            ${PNG_INC_DIR}/../src/gzread.c
            ${PNG_INC_DIR}/../src/gzwrite.c
            ${PNG_INC_DIR}/../src/infback.c
            ${PNG_INC_DIR}/../src/inffast.c
            ${PNG_INC_DIR}/../src/inflate.c
            ${PNG_INC_DIR}/../src/inftrees.c
            ${PNG_INC_DIR}/../src/png.c
            ${PNG_INC_DIR}/../src/pngerror.c
            ${PNG_INC_DIR}/../src/pngget.c
            ${PNG_INC_DIR}/../src/pngmem.c
            ${PNG_INC_DIR}/../src/pngpread.c
            ${PNG_INC_DIR}/../src/pngread.c
            ${PNG_INC_DIR}/../src/pngrio.c
            ${PNG_INC_DIR}/../src/pngrtran.c
            ${PNG_INC_DIR}/../src/pngrutil.c
            ${PNG_INC_DIR}/../src/pngset.c
            ${PNG_INC_DIR}/../src/pngtest.c
            ${PNG_INC_DIR}/../src/pngtrans.c
            ${PNG_INC_DIR}/../src/pngwio.c
            ${PNG_INC_DIR}/../src/pngwrite.c
            ${PNG_INC_DIR}/../src/pngwtran.c
            ${PNG_INC_DIR}/../src/pngwutil.c
            ${PNG_INC_DIR}/../src/trees.c
            ${PNG_INC_DIR}/../src/uncompr.c
            ${PNG_INC_DIR}/../src/zutil.c
           )

SET(CHAI_ALL_SRC ${CHAI_SRC} ${GEL_SRC}) 
SET(GR_LIBS_ALL_SRC ${GIF_SRC} ${3DS_SRC} ${JPEG_SRC} ${PNG_SRC})

######################END OF SRC DEFS##############

ADD_DEFINITIONS(-D_LINUX -DLINUX -DdDOUBLE -DC_ENABLE_DELTA_DEVICE_SUPPORT)
ADD_DEFINITIONS(-lpthread -lrt -lpci -lz -lusb-1.0
		-lGL -lGLU -lglut -fPIC)


#Set the build mode to debug by default
IF(CMAKE_BUILD_TYPE MATCHES Debug)
  #Add debug definitions
  ADD_DEFINITIONS(-DASSERT=assert -DDEBUG=1)
	#SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-overflow=5 -Wextra -ggdb -O0 -g -pg")
	#SET(CMAKE_C_FLAGS_DEBUG "-DHAVE_STDARG_H -DHAVE_SYS_TYPES_H -DHAVE_UNISTD_H -DHAVE_CONFIG_H -Wall -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-overflow=5 -Wextra -ggdb -O0 -g -pg")
  SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wpointer-arith -Wcast-qual -Wstrict-overflow=5 -Wextra -ggdb -O0 -g -pg")
  SET(CMAKE_C_FLAGS_DEBUG "-DHAVE_STDARG_H -DHAVE_SYS_TYPES_H -DHAVE_UNISTD_H -DHAVE_CONFIG_H -Wall -Wpointer-arith -Wcast-qual -Wstrict-overflow=5 -ggdb -O0 -g -pg")
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

IF(CMAKE_BUILD_TYPE MATCHES Release)
  #Add release definitions
  SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")
  SET(CMAKE_C_FLAGS_RELEASE "-DHAVE_STDARG_H -DHAVE_SYS_TYPES_H -DHAVE_UNISTD_H -DHAVE_CONFIG_H -Wall -O3")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

#Make sure the generated makefile is not shortened
SET(CMAKE_VERBOSE_MAKEFILE ON)

#Define the libraries to be built (cmake sources + 3rdparty libs)
ADD_LIBRARY(chai3d STATIC ${CHAI_ALL_SRC})
ADD_LIBRARY(imageformats STATIC ${GR_LIBS_ALL_SRC})

target_link_libraries(chai3d imageformats)

###############SPECIAL CODE TO FIND AND LINK DHD's LIB (Coz not open source) ######################
#NOTE : For a 32 bit compile (or other machine arch), please change lin-x86_64 to lin-i686 (or other arch setting)
find_library( DHD_LIBRARY NAMES dhd
            PATHS   ${CHAI_BASE_DIR}/DHD/lib/
            PATH_SUFFIXES lin-x86_64 )

target_link_libraries(chai3d ${DHD_LIBRARY})
