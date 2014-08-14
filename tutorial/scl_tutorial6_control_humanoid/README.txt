This is an example app. You need to compile (make) it and then run it.
This tutorial will show you how to start using a controller.

This app shows you how to set up a basic humanoid robot and control it using
some default trajectories or a user specified goal position..

1. Compile with the CMake system
#Debug mode (adds lots of debugging symbols to the application)
$ mkdir -p build_dbg
$ cd build_dbg
$ cmake .. -DCMAKE_BUILD_TYPE=Debug
$ make -j
$ cp -rf scl_tutorial6_control_humanoid ..
or simply run a script that does all that
$ sh make_dbg.sh

#Release (fast; no debugging information)
$ mkdir -p build_rel
$ cd build_rel
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ make -j
$ cp -rf scl_tutorial6_control_humanoid ..
or simply run a script that does all that
$ sh make_rel.sh

2. Run
$ ./scl_tutorial6_control_humanoid

Use the mouse to move the camere around:
rotate camera : l-click + drag
zoom camera : r-click + drag
move camera : l-click+ctrl+drag
