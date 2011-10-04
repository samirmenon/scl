rm release -rf &&
mkdir -p release &&
mkdir -p build_rel &&
cd build_rel &&
cmake .. -DCMAKE_BUILD_TYPE=Release &&
make -j2 &&
mv lib* ../release/ &&
cd ..
