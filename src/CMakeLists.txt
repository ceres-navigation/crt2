link_directories(${CMAKE_SOURCE_DIR}/external/zstd/prefix/lib)
find_package(zstd PATHS ${CMAKE_SOURCE_DIR}/external/zstd/prefix)

link_directories(${CMAKE_SOURCE_DIR}/external/oneTBB/prefix/lib)
find_package(tbb PATHS ${CMAKE_SOURCE_DIR}/external/oneTBB/prefix)

# Build the batch convert objs tool:
add_executable(convert_objs convert_objs.cpp)
target_link_libraries(convert_objs PRIVATE PUBLIC zstd tinyobjloader geometry tbb)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/external/oneTBB/include)