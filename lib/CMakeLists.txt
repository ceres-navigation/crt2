link_directories(${CMAKE_SOURCE_DIR}/external/zstd/prefix/lib)
find_package(zstd PATHS ${CMAKE_SOURCE_DIR}/external/zstd/prefix)

add_library(
    geometry
    geometry.cpp
)

target_link_libraries(geometry PRIVATE PUBLIC zstd)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/external/tinyobjloader)
include_directories(${CMAKE_SOURCE_DIR}/external/zstd/lib)

set_target_properties(geometry PROPERTIES LINKER_LANGUAGE CXX)