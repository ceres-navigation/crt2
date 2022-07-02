cmake_minimum_required(VERSION 3.9)
link_directories(../zstd/prefix/lib)

project(ceres-rt CXX)

set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(zstd PATHS ../zstd/prefix)

set(CMAKE_CXX_FLAGS "-Wall -mtune=native -march=native -g")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fno-math-errno -fno-signed-zeros -fno-trapping-math -freciprocal-math -fno-rounding-math -fno-signaling-nans -fexcess-precision=fast")

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(${PROJECT_NAME}
  src/prim.cpp
  src/loader.cpp
  src/bvh.cpp
  src/tlas.cpp
  src/treeas.cpp
  src/tiled.cpp
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE PUBLIC zstd)

# find_package(pybind11 CONFIG)
# pybind11_add_module(ceres_rt pybind.cpp)

# target_link_libraries(${PROJECT_NAME} PRIVATE PUBLIC lib bvh crt model_loaders materials)

include_directories(include)

include_directories(../zstd/lib)