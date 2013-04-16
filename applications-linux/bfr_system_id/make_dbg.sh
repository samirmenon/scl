mkdir -p build_dbg &&
cd build_dbg &&
cmake .. -DCMAKE_BUILD_TYPE=Debug &&
make -j8 &&
cp -rf bfr_system_id ../ &&
cp -rf bfr_nw_system_id ../ &&
cd ..
