cd ../../ &&
git submodule init && git submodule update &&
cd 3rdparty/chai3d-graphics/lib/ &&
#rm build_* -rf &&
sh make_debug.sh &&
sh make_release.sh &&
cd ../../yaml-cpp-0.2.7/ &&
#rm build -rf &&
mkdir -p build &&
cd build &&
cmake .. &&
make -j8 &&
cd ../../../applications-linux/scl_lib &&
#rm build_* -rf &&
sh make_debug.sh &&
sh make_release.sh &&
cd ../scl_test && 
#rm build_* -rf &&
sh make_debug.sh &&
sh make_release.sh &&
cd ../scl_dynamics && make release -j8 &&
cd ../scl_gc_ctrl && make release -j8 &&
cd ../scl_task_ctrl && make release -j8 &&
cd ../scl_example_ctrl && sh make_dbg.sh && sh make_rel.sh &&
cd ../scl_file_converter && make release -j8 &&
cd ../scl_lib
