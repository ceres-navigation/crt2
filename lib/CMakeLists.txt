link_directories(${CMAKE_SOURCE_DIR}/external/zstd/prefix/lib)
find_package(zstd PATHS ${CMAKE_SOURCE_DIR}/external/zstd/prefix)

add_library(
    rt
    scene.cpp
    geometry.cpp
    rigid_body.cpp
    bvh.cpp
    group.cpp

    cameras/simple_camera.cpp

    graph_nodes/blas_node.cpp

    lights/point_light.cpp

    primitives/ray.cpp
    primitives/aabb.cpp
    primitives/triangle.cpp

    sensors/simple_sensor.cpp

    vector_math/vector.cpp
    vector_math/rotation.cpp
)

target_link_libraries(rt PRIVATE PUBLIC zstd)

set_target_properties(rt PROPERTIES LINKER_LANGUAGE CXX)