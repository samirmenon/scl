# $Author : Samir Menon
# Stanford University. 
# $Date May 7, 2010

############################# COMPILER OPTIONS ############################
CXX = g++
SYS_ARCH = $(shell echo `uname -m`)

############################# SPECIFY ALL LIBS ############################
#scl lib
WBC_INC_DIR = ../../src/scl/
#NOTE TODO: THIS IS MESSY. TAO shouldn't need its own include dir.
WBC_TAO_INC_DIR = ../../src/scl/dynamics/tao/
WBC_DBG_LIB_DIR = -L../scl_lib/debug/ -Wl,--rpath=../scl_lib/debug/
WBC_FIN_LIB_DIR = -L../scl_lib/release/ -Wl,--rpath=../scl_lib/release/
WBC_LIBS = -lscl -lgomp
WBC_DEFS_DBG = -DTIXML_USE_STL -DASSERT=assert -DDEBUG=1 -DTESTING -DW_TESTING=1 -fopenmp
WBC_DEFS_FIN = -DTIXML_USE_STL -fopenmp

#3rd party lib : Eigen (local install)
EIGEN_INC_DIR = ../../3rdparty/eigen3.0.2/

#3rd party lib : sUtil (local install)
SUTIL_INC_DIR = ../../3rdparty/sUtil/src/

#3rd party lib : chai (local install)
CHAI_INC_DIR = ../../3rdparty/chai3d-graphics/chai/
CHAI_DBG_LIB_DIR = -L$(CHAI_INC_DIR)../lib/debug/ -Wl,--rpath=$(CHAI_INC_DIR)../lib/debug/
CHAI_FIN_LIB_DIR = -L$(CHAI_INC_DIR)../lib/release/ -Wl,--rpath=$(CHAI_INC_DIR)../lib/release/
CHAI_LIB_DBG = -lchai3d
CHAI_LIB_FIN = -lchai3d
CHAI_DEFS = -D_LINUX \
					  -lpthread -lrt -lGL -lGLU -lglut
					  
############################# INCLUDE AND LINK OPTIONS ############################
#Collate all the include directiories
INCLUDEDIRS = -I../../src/ -I$(WBC_INC_DIR) \
              -I$(WBC_TAO_INC_DIR) \
              -I$(CHAI_INC_DIR) \
              -I$(EIGEN_INC_DIR) \
              -I$(SUTIL_INC_DIR)

#Combine all libs to be linked
###Debug
LINK_DBG_DIRS = $(WBC_DBG_LIB_DIR) $(CHAI_DBG_LIB_DIR)
LINK_LIBS_DBG = $(WBC_LIBS) $(CHAI_LIB_DBG)

###Final (Release)
LINK_FIN_DIRS = $(WBC_FIN_LIB_DIR) $(CHAI_FIN_LIB_DIR)
LINK_LIBS_FIN = $(WBC_LIBS) $(CHAI_LIB_FIN)

#Combine other defs
DEFS_DBG = $(WBC_DEFS_DBG) $(CHAI_DEFS)
DEFS_FIN = $(WBC_DEFS_FIN) $(CHAI_DEFS)

############################# Compile options ############################
DBGFLAGS = -Wall -ggdb -pg -O0 \
						$(INCLUDEDIRS) \
           -fPIC $(LINK_DBG_DIRS) $(LINK_LIBS_DBG) \
           $(DEFS_DBG)
FINALFLAGS = -Wall -O3 $(INCLUDEDIRS) \
           -fPIC $(LINK_FIN_DIRS) $(LINK_LIBS_FIN) \
           $(DEFS_FIN) 

############################ SOURCE FILES ############################

#Finalize all library dependencies for different simulation types
SAIMAT_SRC = ./saimatrix/SAIMatrix3.cpp \
             ./saimatrix/SAITransform.cpp \
             ./saimatrix/SAILapack.cpp \
             ./saimatrix/SAIMatrix6.cpp \
             ./saimatrix/SAIQuaternion.cpp \
             ./saimatrix/SAIMatrix.cpp \
             ./saimatrix/SAIVector.cpp \
             ./saimatrix/SAIVector3.cpp \
             ./saimatrix/SAIVector6.cpp

GRAPHICS_SRC = $(WBC_INC_DIR)graphics/chai/CChaiGraphics.cpp \
               $(WBC_INC_DIR)graphics/chai/ChaiGlutHandlers.cpp

#THIRDPARTYSRC = $(SAIMAT_SRC)
ALLSRC = $(THIRDPARTYSRC) $(GRAPHICS_SRC) scl_test.cpp 

############################ BUILD SPECIFICATIONS ############################
TARGET = scl_test

#Build all simulations
.PHONY: all
all:
	$(CXX) $(ALLSRC) -o $(TARGET) $(DBGFLAGS)

.PHONY: release
release:
	$(CXX) $(ALLSRC) -o $(TARGET) $(FINALFLAGS)

#Clean up options
.PHONY : clean
clean: 
	@rm -f *.o $(TARGET) gmon.out

#WARNING : DELETE ALL SOURCE AND HEADER FILES.
#IMP: UPDATE FROM REPO AFTER USING THIS TO GET FILES BACK.
.PHONY : srcclean	
srcclean:
	@rm -f *.o $(TARGET) $(ALLSRC) gmon.out
