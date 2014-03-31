2014-03-26: Edited by SM

1. This app requires the dynamics3d engine. Please contact Francois Conti
for a copy. Once you have it, place the sources in this folder:
<scl-base-dir>/3rdparty/chai3d-3.0/dynamics3d/src

2. Next, please compile the sources using the CMake file provided:
$ cd ../lib
$ sh make_release.sh && sh make_debug.sh

Now you are ready to use the app.

3. This is an example app to demonstrate dynamics. Compile it by running CMake:
#Debug
$ sh make_dbg.sh
#Release
$ sh make_rel.sh

Run it:
$ ./scl_dynamics_dyn3d ../../specs/Puma/PumaCfg.xml
