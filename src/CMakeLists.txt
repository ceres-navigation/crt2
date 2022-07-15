link_directories(${CMAKE_SOURCE_DIR}/external/zstd/prefix/lib)
find_package(zstd PATHS ${CMAKE_SOURCE_DIR}/external/zstd/prefix)

link_directories(${CMAKE_SOURCE_DIR}/external/oneTBB/prefix/lib)

# Build the batch convert objs tool:
add_executable(convert_objs convert_objs.cpp)
target_link_libraries(convert_objs PUBLIC zstd tinyobjloader tbb vira)

# Add the rendering tool:
add_executable(render_test render_test.cpp)
target_link_libraries(render_test PUBLIC zstd tinyobjloader tbb lodepng vira)