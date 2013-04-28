mkdir -p build_dbg &&
cd build_dbg &&
cmake .. -DCMAKE_BUILD_TYPE=Debug &&
make -j8 &&
cp -rf bfr_driver_test ../ &&
cp -rf bfr_driver_fx ../ &&
cp -rf bfr_driver_raw_input ../ &&
cd ..
