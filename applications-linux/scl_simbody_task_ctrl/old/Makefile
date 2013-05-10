# $Author : Samir Menon
# Stanford University. 
# $Date Jul 25, 2012

############################# COMPILER OPTIONS ############################
CXX = g++
SYS_ARCH = $(shell echo `uname -m`)

############################# SPECIFY ALL LIBS ############################
#scl lib
SCL_INC_DIR = ../../src/scl/
#NOTE TODO: THIS IS MESSY. TAO shouldn't need its own include dir.
SCL_TAO_INC_DIR = ../../src/scl/dynamics/tao/
SCL_DBG_LIB_DIR = -L../scl_lib/debug/ -Wl,--rpath=../scl_lib/debug/
SCL_FIN_LIB_DIR = -L../scl_lib/release/ -Wl,--rpath=../scl_lib/release/
SCL_LIBS = -lscl
SCL_DEFS_DBG = -fopenmp -lgomp -DTIXML_USE_STL -DASSERT=assert -DDEBUG=1 -DTESTING -DW_TESTING=1
SCL_DEFS_FIN = -fopenmp -lgomp -DTIXML_USE_STL -DW_THREADING_ON

#3rd party lib : Eigen (local install)
EIGEN_INC_DIR = ../../3rdparty/eigen3.1.2/

#3rd party lib : sUtil (local install)
SUTIL_INC_DIR = ../../3rdparty/sUtil/src/

#3rd party lib : chai (local install)
CHAI_INC_DIR = ../../3rdparty/chai3d-3.0/chai3d/
CHAI_DBG_LIB_DIR = -L$(CHAI_INC_DIR)../lib/debug/ -Wl,--rpath=$(CHAI_INC_DIR)../lib/debug/
CHAI_FIN_LIB_DIR = -L$(CHAI_INC_DIR)../lib/release/ -Wl,--rpath=$(CHAI_INC_DIR)../lib/release/
CHAI_LIB_DBG = -lchai3d
CHAI_LIB_FIN = -lchai3d
CHAI_DEFS = -D_LINUX -DLINUX \
					  -lpthread -lrt -lGL -lGLU -lglut

#3rd party lib : simbody (local install)
SIMBODY_BASE_DIR = ../../3rdparty/simbody.svn/
SIMBODY_INC_DIR1 = $(SIMBODY_BASE_DIR)Simbody/include/ 
SIMBODY_INC_DIR2 = $(SIMBODY_BASE_DIR)SimTKcommon/include/ 
SIMBODY_INC_DIR3 = $(SIMBODY_BASE_DIR)SimTKmath/include/
SIMBODY_INC_DIR4 = $(SIMBODY_BASE_DIR)SimTKcommon/BigMatrix/include/
SIMBODY_INC_DIR5 = $(SIMBODY_BASE_DIR)SimTKcommon/Geometry/include/
SIMBODY_INC_DIR6 = $(SIMBODY_BASE_DIR)SimTKcommon/Mechanics/include/
SIMBODY_INC_DIR7 = $(SIMBODY_BASE_DIR)SimTKcommon/Polynomial/include/
SIMBODY_INC_DIR8 = $(SIMBODY_BASE_DIR)SimTKcommon/Random/include/
SIMBODY_INC_DIR9 = $(SIMBODY_BASE_DIR)SimTKcommon/Scalar/include/
SIMBODY_INC_DIR10 = $(SIMBODY_BASE_DIR)SimTKcommon/Simulation/include/
SIMBODY_INC_DIR11 = $(SIMBODY_BASE_DIR)SimTKcommon/SmallMatrix/include/
SIMBODY_INC_DIR12 = $(SIMBODY_BASE_DIR)SimTKmath/Geometry/include/
SIMBODY_INC_DIR13 = $(SIMBODY_BASE_DIR)SimTKmath/Integrators/include/
SIMBODY_INC_DIR14 = $(SIMBODY_BASE_DIR)SimTKmath/LinearAlgebra/include/
SIMBODY_INC_DIR15 = $(SIMBODY_BASE_DIR)SimTKmath/Optimizers/include/
SIMBODY_INC_DIR16 = $(SIMBODY_BASE_DIR)Simbody/Visualizer/include/
SIMBODY_LIB_DIR = -L$(SIMBODY_BASE_DIR)build/ -Wl,--rpath=$(SIMBODY_BASE_DIR)build/
SIMBODY_LIBS = -lgfortran -lSimTKAtlas -lSimTKcommon -lSimTKlapack -lSimTKmath -lSimTKsimbody
					  
############################# INCLUDE AND LINK OPTIONS ############################
#Collate all the include directiories
INCLUDEDIRS = -I../../src/ -I$(SCL_INC_DIR) \
              -I$(SCL_TAO_INC_DIR) \
              -I$(CHAI_INC_DIR) -I$(EIGEN_INC_DIR) \
              -I$(SUTIL_INC_DIR) \
              -I$(SIMBODY_INC_DIR1) -I$(SIMBODY_INC_DIR2) -I$(SIMBODY_INC_DIR3) -I$(SIMBODY_INC_DIR4) \
              -I$(SIMBODY_INC_DIR5) -I$(SIMBODY_INC_DIR6) -I$(SIMBODY_INC_DIR7) -I$(SIMBODY_INC_DIR8) \
              -I$(SIMBODY_INC_DIR9) -I$(SIMBODY_INC_DIR10) -I$(SIMBODY_INC_DIR11) -I$(SIMBODY_INC_DIR12) \
              -I$(SIMBODY_INC_DIR13) -I$(SIMBODY_INC_DIR14) -I$(SIMBODY_INC_DIR15) -I$(SIMBODY_INC_DIR16)  

#Combine all libs to be linked
###Debug
LINK_DBG_DIRS = $(SCL_DBG_LIB_DIR) $(CHAI_DBG_LIB_DIR) $(SIMBODY_LIB_DIR)
LINK_LIBS_DBG = $(SCL_LIBS) $(CHAI_LIB_DBG) $(SIMBODY_LIBS)

###Final (Release)
LINK_FIN_DIRS = $(SCL_FIN_LIB_DIR) $(CHAI_FIN_LIB_DIR) $(SIMBODY_LIB_DIR)
LINK_LIBS_FIN = $(SCL_LIBS) $(CHAI_LIB_FIN) $(SIMBODY_LIBS)

#Combine other defs
DEFS_DBG = $(SCL_DEFS_DBG) $(CHAI_DEFS)
DEFS_FIN = $(SCL_DEFS_FIN) $(CHAI_DEFS)

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
GRAPHICS_SRC = $(SCL_INC_DIR)graphics/chai/CChaiGraphics.cpp \
               $(SCL_INC_DIR)graphics/chai/ChaiGlutHandlers.cpp \
               $(SCL_INC_DIR)dynamics/simbody/CSimbodyDynamics.cpp

#THIRDPARTYSRC = $(SAIMAT_SRC)
ALLSRC = $(THIRDPARTYSRC) $(GRAPHICS_SRC) simbody_task_main.cpp

############################ BUILD SPECIFICATIONS ############################
TARGET = scl_simbody_task_ctrl

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
