cd ../../ &&
git submodule init && git submodule update &&
#Build the sensoray driver library.
cd 3rdparty/sensoray-3dof/applications-linux/lib/ &&
#rm -rf build_* &&
sh make_debug.sh &&
sh make_release.sh &&
cd ../../../../ &&
# NOTE : We compile chai twice. This is a shared lib (works better for faster compiles for non-haptic apps)
cd 3rdparty/chai3d-3.0/lib/ &&
#rm -rf build_* &&
sh make_debug.sh &&
sh make_release.sh &&
# NOTE : We compile chai thrice. This is a static lib (works better for linking with dhd for haptics)
cd ../lib_haptics/ &&
#rm -rf build_* &&
sh make_debug.sh &&
sh make_release.sh &&
# NOTE : We compile chai thrice. This is a static lib with the sensoray drivers integrated
cd ../lib_bfr/ &&
#rm -rf build_* &&
sh make_debug.sh &&
sh make_release.sh &&
cd ../../yaml-cpp-0.3.0/ &&
#rm -rf build &&
mkdir -p build &&
cd build &&
cmake .. &&
make -j8 &&
cd ../../../applications-linux/scl_lib &&
#rm -rf build_* &&
sh make_debug.sh &&
sh make_release.sh &&
cd ../scl_test && 
#rm -rf build_* &&
sh make_debug.sh &&
sh make_release.sh &&
cd ../fmri_xyz_localizer && sh make_dbg.sh && sh make_rel.sh &&
cd ../scl_dynamics && make release -j8 &&
cd ../scl_gc_ctrl && make release -j8 &&
cd ../scl_task_ctrl && make release -j8 &&
cd ../scl_example_ctrl && sh make_dbg.sh && sh make_rel.sh &&
cd ../scl_file_converter && make release -j8 &&
cd ../scl_haptic_ctrl && sh make_dbg.sh && sh make_rel.sh &&
cd ../scl_lib
