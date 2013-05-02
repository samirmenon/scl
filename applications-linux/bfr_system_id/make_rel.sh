mkdir -p build_rel &&
cd build_rel &&
cmake .. -DCMAKE_BUILD_TYPE=Release &&
make -j8 &&
cp -rf bfr_system_id ../ &&
cp -rf bfr_nw_system_id ../ &&
cp -rf bfr_force_system_id ../ &&
cp -rf bfr_force_system_id_v2 ../ &&
cd ..
