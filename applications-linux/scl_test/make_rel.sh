mkdir -p build_rel &&
cd build_rel &&
cmake .. -DCMAKE_BUILD_TYPE=Release &&
make -j8 &&
cp -rf scl_test ../ &&
cd ..
