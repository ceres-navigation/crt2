# Setup ZSTD Dependency:
In a directory above this one (so that you will have `zstd/` and `crt2/` in the same directory) run:
```
git clone https://github.com/facebook/zstd.git

cd zstd
mkdir prefix

cd build/cmake
mkdir build
cd build
cd cmake ..
make

cmake -DCMAKE_INSTALL_PREFIX=../../../prefix ..
```