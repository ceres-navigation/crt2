link_directories(${CMAKE_SOURCE_DIR}/external/zstd/prefix/lib)
find_package(zstd PATHS ${CMAKE_SOURCE_DIR}/external/zstd/prefix)

link_directories(${CMAKE_SOURCE_DIR}/external/oneTBB/prefix/lib)

# Build the batch convert objs tool:
add_executable(convert_objs convert_objs.cpp)
target_link_libraries(convert_objs PUBLIC zstd tinyobjloader tbb rt)

# Add the rendering tool:
add_executable(render render.cpp)
target_link_libraries(render PUBLIC zstd tinyobjloader tbb rt)

# Add the rendering tool:
add_executable(jacco jacco.cpp)
target_link_libraries(jacco PUBLIC zstd tinyobjloader tbb rt)