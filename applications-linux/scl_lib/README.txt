Using dynamic linking:

Example : Build main.cpp linked with scl classes

> g++ -I../../src/ -c -Wall -fPIC -o main.o main.cpp
> g++ -L../../lib/<debug/final> -Wl,--rpath=../../lib/<debug/final> -lscl -o main main.o

################
Important flags:
################
-fPIC
-L<libpath>
-Wl,--rpath=<libpath>


#For compiling with chai using cmake

#Chai Includes + GL/GLUT includes+libs
SET(CHAI_INC_DIR  ../../3rdparty/chai3d-2.0.0-linx64/src)
ADD_DEFINITIONS(-D_LINUX)

#---> Sets ${GLUT_LIBRARY} -- Reqd by chai
FIND_PACKAGE(GLUT)
IF(GLUT_FOUND)
   SET(CHAI_INC_DIR ${CHAI_INC_DIR} ${GLUT_INCLUDE_DIR})
   SET(WBC_3PARTY_LIBS ${WBC_3PARTY_LIBS} 
                       ${GLUT_LIBRARY}
      )
   MESSAGE("Adding Glut " ${GLUT_INCLUDE_DIR} " " ${GLUT_LIBRARY})
ELSE(GLUT_FOUND)
   MESSAGE("Missing Glut")
ENDIF(GLUT_FOUND)

#Sets ${OPENGL_LIBRARY} -- Reqd by chai
FIND_PACKAGE(OpenGL)
IF(OPENGL_FOUND)
   SET(CHAI_INC_DIR ${CHAI_INC_DIR} ${OPENGL_INCLUDE_DIR})   
   SET(WBC_3PARTY_LIBS ${WBC_3PARTY_LIBS} 
                       ${OPENGL_LIBRARY}
      )
   MESSAGE("Adding Opengl " ${OPENGL_INCLUDE_DIR} " " ${OPENGL_LIBRARY})
ELSE(OPENGL_FOUND)
   MESSAGE("Missing Opengl")
ENDIF(OPENGL_FOUND)
