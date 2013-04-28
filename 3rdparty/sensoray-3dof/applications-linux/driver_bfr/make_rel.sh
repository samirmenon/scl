mkdir -p build_rel &&
cd build_rel &&
cmake .. -DCMAKE_BUILD_TYPE=Release &&
make -j8 &&
cp -rf bfr_driver_test ../ &&
cp -rf bfr_driver_fx ../ &&
cp -rf bfr_driver_raw_input ../ &&
cd ..
