This is an example app. You need to compile (make) it and then run it.
Compared to the earlier two tutorials, we now have quite a lot of code
to deal with. It usually takes quite long to compile the code so we have
two options
(a) Make a complex Makefile that checks for files that have been recently
updated, and then recompiles only the updated files (do it yourself).
(b) Start using CMake, which does this for you (we'll do this instead).

1. Compile with the CMake system
#Debug mode (adds lots of debugging symbols to the application)
$ mkdir -p build_dbg
$ cd build_dbg
$ cmake .. -DCMAKE_BUILD_TYPE=Debug
$ make -j
$ cp -rf scl_tutorial3_graphics_physics_cmake ..
or simply run a script that does all that
$ sh make_dbg.sh

#Release (fast; no debugging information)
$ mkdir -p build_rel
$ cd build_rel
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ make -j
$ cp -rf scl_tutorial3_graphics_physics_cmake ..
or simply run a script that does all that
$ sh make_rel.sh

2. Run
$ ./scl_tutorial3_graphics_physics_cmake

Use the mouse to move the camere around:
rotate camera : l-click + drag
zoom camera : r-click + drag
move camera : l-click+ctrl+drag
