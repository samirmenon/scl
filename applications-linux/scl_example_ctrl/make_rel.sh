mkdir -p build_rel &&
cd build_rel &&
cmake .. -DCMAKE_BUILD_TYPE=Release &&
make -j2 &&
cp -rf scl_eg_ctrl ../ &&
cd ..
