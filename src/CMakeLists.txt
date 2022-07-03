# find_package(pybind11 CONFIG)
# pybind11_add_module(_crt pybind.cpp)
link_directories(${CMAKE_SOURCE_DIR}/external/zstd/prefix/lib)
find_package(zstd PATHS ${CMAKE_SOURCE_DIR}/external/zstd/prefix)

# Build the convert_obj tool:
add_executable(convert_obj convert_obj.cpp)
target_link_libraries(convert_obj PRIVATE PUBLIC zstd tinyobjloader geometry)

add_executable(convert_objs convert_objs.cpp)
target_link_libraries(convert_objs PRIVATE PUBLIC zstd tinyobjloader geometry)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/external)
include_directories(${CMAKE_SOURCE_DIR}/external/zstd/lib)