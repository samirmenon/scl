mkdir -p build_dbg &&
cd build_dbg &&
cmake .. -DCMAKE_BUILD_TYPE=Debug &&
make -j8 &&
cp -rf scl_tutorial3_graphics_physics_cmake ../ &&
cd ..
