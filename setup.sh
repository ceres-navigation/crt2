#!/bin/bash

# Setup python:
sudo apt-get install python3-dev -y

cd external

# Setup prebuild zstd library:
cd zstd
mkdir -p prefix
cd build/cmake
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=../../../prefix ..
make install -j
cd ../../../../

# Setup oneTBB library:
cd oneTBB
mkdir -p prefix
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=../prefix -DTBB_TEST=OFF ..
cmake --build .
cmake --install .
cd ../../