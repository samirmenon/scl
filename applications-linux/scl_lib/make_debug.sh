rm debug -rf &&
mkdir debug &&
mkdir -p build_dbg &&
cd build_dbg &&
cmake .. -DCMAKE_BUILD_TYPE=Debug &&
make -j2 &&
mv lib* ../debug/ &&
cd ..
