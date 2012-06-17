# $Author : Samir Menon
# Stanford University. 
# $Date May 7, 2010

############################# COMPILER OPTIONS ############################
CXX = g++
SYS_ARCH = $(shell echo `uname -m`)

############################# SPECIFY ALL LIBS ############################
#scl lib
WBC_INC_DIR = ../../src/scl/
WBC_DBG_LIB_DIR = -L../scl_lib/debug/ -Wl,--rpath=../scl_lib/debug/
WBC_FIN_LIB_DIR = -L../scl_lib/release/ -Wl,--rpath=../scl_lib/release/
WBC_LIBS = -lscl
WBC_DEFS_DBG = -DTIXML_USE_STL -DASSERT=assert -DDEBUG=1 -DTESTING -DW_TESTING=1
WBC_DEFS_FIN = -DTIXML_USE_STL

#3rd party lib : Eigen (local install)
EIGEN_INC_DIR = ../../3rdparty/eigen3.0.5/

#3rd party lib : sUtil (local install)
SUTIL_INC_DIR = ../../3rdparty/sUtil/src/

############################# INCLUDE AND LINK OPTIONS ############################
#Collate all the include directiories
INCLUDEDIRS = -I../../src/ -I$(WBC_INC_DIR) \
              -I$(EIGEN_INC_DIR) \
              -I$(SUTIL_INC_DIR)

#Combine all libs to be linked
###Debug
LINK_DBG_DIRS = $(WBC_DBG_LIB_DIR) 
LINK_LIBS_DBG = $(WBC_LIBS) 

###Final (Release)
LINK_FIN_DIRS = $(WBC_FIN_LIB_DIR) 
LINK_LIBS_FIN = $(WBC_LIBS)

#Combine other defs
DEFS_DBG = $(WBC_DEFS_DBG)
DEFS_FIN = $(WBC_DEFS_FIN)

############################# Compile options ############################
DBGFLAGS = -Wall -ggdb -pg -O0 \
           $(INCLUDEDIRS) \
           -fPIC $(LINK_DBG_DIRS) $(LINK_LIBS_DBG) \
           $(DEFS_DBG)
FINALFLAGS = -Wall -O3 $(INCLUDEDIRS) \
           -fPIC $(LINK_FIN_DIRS) $(LINK_LIBS_FIN) \
           $(DEFS_FIN) 

############################ SOURCE FILES ############################

ALLSRC = $(THIRDPARTYSRC) scl_file_converter.cpp

############################ BUILD SPECIFICATIONS ############################
TARGET = scl_file_converter

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
