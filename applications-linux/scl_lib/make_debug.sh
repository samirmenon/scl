rm -rf debug &&
mkdir debug &&
mkdir -p build_dbg &&
cd build_dbg &&
cmake .. -DCMAKE_BUILD_TYPE=Debug &&
make -j8 &&
mv lib* ../debug/ &&
cd ..
