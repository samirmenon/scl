rm debug -rf &&
mkdir -p debug &&
mkdir -p build_dbg &&
cd build_dbg &&
cmake .. -DCMAKE_BUILD_TYPE=Debug &&
make &&
mv lib* ../debug/ &&
cd ..
