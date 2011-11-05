# $Author : Samir Menon
# Stanford University. 
# $Date May 7, 2010

############################# COMPILER OPTIONS ############################
CXX = g++
SYS_ARCH = $(shell echo `uname -m`)

############################# SPECIFY ALL LIBS ############################
#3rd party lib : Eigen (local install)
EIGEN_INC_DIR = ../../3rdparty/eigen3.0.2/

#3rd party lib : sUtil (local install)
SUTIL_INC_DIR = ../../3rdparty/sUtil/src/

#3rd party lib : Socket (local install)
SOCK_INC_DIR = ../../3rdparty/practical-socket/
					  
############################# INCLUDE AND LINK OPTIONS ############################
#Collate all the include directiories
INCLUDEDIRS = -I$(EIGEN_INC_DIR) \
              -I$(SUTIL_INC_DIR) \
              -I$(SOCK_INC_DIR)

############################# Compile options ############################
DBGFLAGS = -Wall -ggdb -pg -O0 \
						$(INCLUDEDIRS) \
           -fPIC $(LINK_DBG_DIRS) $(LINK_LIBS_DBG) \
           $(DEFS_DBG)
FINALFLAGS = -Wall -O3 $(INCLUDEDIRS) \
           -fPIC $(LINK_FIN_DIRS) $(LINK_LIBS_FIN) \
           $(DEFS_FIN) 

############################ SOURCE FILES ############################
ALLSRC = scl_nw_conn.cpp $(SOCK_INC_DIR)PracticalSocket.cpp

############################ BUILD SPECIFICATIONS ############################
TARGET = nw_conn

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
