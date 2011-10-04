mkdir -p build_dbg &&
cd build_dbg &&
cmake .. -DCMAKE_BUILD_TYPE=Debug &&
make -j2 &&
cp -rf scl_eg_ctrl ../ &&
cd ..
