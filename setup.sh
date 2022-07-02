#!/bin/bash

# Setup prebuild zstd library:
cd external/zstd

mkdir -p prefix

cd build/cmake
mkdir -p build
cd build
cd cmake ..

cmake -DCMAKE_INSTALL_PREFIX=../../../prefix ..

make install -j

# Additional configurations: