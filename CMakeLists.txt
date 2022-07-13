cmake_minimum_required(VERSION 3.9)

# add_subdirectory(external/pybind11)
# pybind11_add_module(_rt src/pybind.cpp)

project(rt CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) 

# Uncomment these lines to compile for debugging:
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "-g")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -mtune=native -march=native -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fno-math-errno -fno-signed-zeros -fno-trapping-math -freciprocal-math -fno-rounding-math -fno-signaling-nans -fexcess-precision=fast")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include_directories(include)
include_directories(external/tinyobjloader)
include_directories(external/zstd/lib)
include_directories(external/oneTBB/include)
include_directories(external/lodepng)

add_subdirectory(external)
add_subdirectory(lib)
add_subdirectory(src)