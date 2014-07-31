# $Author : Samir Menon, Stanford University.
# $Date Jul 29, 2014

############################# SPECIFY ALL LIBS/DIRS Etc. ############################
#scl lib
SCL_INC_DIR = ../../src/scl/
SCL_TAO_INC_DIR = ../../src/scl/dynamics/tao/

SCL_DBG_LIB_DIR = -L../scl_lib/debug/ -Wl,--rpath=../scl_lib/debug/
SCL_FIN_LIB_DIR = -L../scl_lib/release/ -Wl,--rpath=../scl_lib/release/

SCL_LIBS = -lscl -fopenmp -lgomp 

SCL_SCL_DEFS_DBG = -DTIXML_USE_STL -DASSERT=assert -DDEBUG=1
SCL_SCL_DEFS_FIN = -DTIXML_USE_STL

#3rd party lib : Eigen (local install)
EIGEN_INC_DIR = ../../3rdparty/eigen/

#3rd party lib : sUtil (local install)
SUTIL_INC_DIR = ../../3rdparty/sUtil/src/
					  
############################# INCLUDE AND LINK OPTIONS ############################
#Collate all the include directiories
INCLUDEDIRS = -I../../src/ -I$(SCL_INC_DIR) -I$(SCL_TAO_INC_DIR) -I$(EIGEN_INC_DIR) -I$(SUTIL_INC_DIR)

############################# Compile options ############################
DBGFLAGS = -Wall -ggdb -pg -O0 $(INCLUDEDIRS) -fPIC $(SCL_DBG_LIB_DIR) $(SCL_LIBS) $(SCL_DEFS_DBG)
FINALFLAGS = -Wall -O3 $(INCLUDEDIRS) -fPIC $(SCL_FIN_LIB_DIR) $(SCL_LIBS) $(SCL_DEFS_FIN) 

############################ BUILD SPECIFICATIONS ############################
# The final application ot be built
TARGET = scl_tutorial1_xml_kinematics_dynamics

#Build all simulations
.PHONY: all
all: 
	g++ scl_tutorial1_xml_kinematics_dynamics.cpp -o $(TARGET) $(DBGFLAGS)

.PHONY: release 
release: 
	g++ scl_tutorial1_xml_kinematics_dynamics.cpp -o $(TARGET) $(FINALFLAGS) 

#Clean up options
.PHONY : clean
clean: 
	@rm -f *.o $(TARGET) gmon.out *~
