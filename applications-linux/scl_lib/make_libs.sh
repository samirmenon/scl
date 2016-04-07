#!/bin/bash

# ***********************
# First we go to the scl base dir and set up submodules
cd ../../ 
git submodule init && git submodule sync && git submodule init && git submodule update 

# ***********************
# Now we compile chai (the graphics lib)
# NOTE : We compile chai twice. This is a shared lib (works better for faster compiles for non-haptic apps)
cd 3rdparty/chai3d-3.0/lib/ 
#rm -rf build_* 
sh make_debug.sh 
sh make_release.sh 
# NOTE : We compile chai twice. This is a static lib (works better for linking with dhd for haptics)
cd ../lib_haptics/ 
#rm -rf build_* 
sh make_debug.sh 
sh make_release.sh 

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
