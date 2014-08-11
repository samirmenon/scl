This is an example app. You need to compile (make) it and then run it.
This tutorial will show you how to start using a controller.

1. Compile with the CMake system
#Debug mode (adds lots of debugging symbols to the application)
$ mkdir -p build_dbg
$ cd build_dbg
$ cmake .. -DCMAKE_BUILD_TYPE=Debug
$ make -j
$ cp -rf scl_tutorial5_control_multi_task ..
or simply run a script that does all that
$ sh make_dbg.sh

#Release (fast; no debugging information)
$ mkdir -p build_rel
$ cd build_rel
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ make -j
$ cp -rf scl_tutorial5_control_multi_task ..
or simply run a script that does all that
$ sh make_rel.sh

2. Run
$ ./scl_tutorial5_control_multi_task

Use the mouse to move the camere around:
rotate camera : l-click + drag
zoom camera : r-click + drag
move camera : l-click+ctrl+drag
