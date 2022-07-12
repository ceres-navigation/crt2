link_directories(${CMAKE_SOURCE_DIR}/external/zstd/prefix/lib)
find_package(zstd PATHS ${CMAKE_SOURCE_DIR}/external/zstd/prefix)

link_directories(${CMAKE_SOURCE_DIR}/external/oneTBB/prefix/lib)

# Build the batch convert objs tool:
# add_executable(convert_objs convert_objs.cpp)
# target_link_libraries(convert_objs PUBLIC zstd tinyobjloader tbb rt)

# Add the rendering tool:
add_executable(vira vira.cpp)
target_link_libraries(vira PUBLIC zstd tinyobjloader tbb lodepng rt)