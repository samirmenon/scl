#!/bin/bash

# ***********************
# First we go to the scl base dir and set up submodules
cd ../../ 
git submodule init && git submodule sync && git submodule init && git submodule update 

# ***********************
# Now we compile chai (the graphics lib)
cd 3rdparty/chai3d.git/ 
#rm -rf build_* 
sh make_everything.sh

# ***********************
# Now we compile the otg
cd ../../otgtypeii/Linux/
sh make_everything.sh

# ***********************
# Now we compile scl lib
cd ../../../applications-linux/scl_lib 
#rm -rf build_* 
sh make_debug.sh 
sh make_release.sh 
