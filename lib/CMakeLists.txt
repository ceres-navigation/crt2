link_directories(${CMAKE_SOURCE_DIR}/external/zstd/prefix/lib)
find_package(zstd PATHS ${CMAKE_SOURCE_DIR}/external/zstd/prefix)

add_library(
    vira
    
    scene.cpp

    acceleration/bvh.cpp

    cameras/simple_camera.cpp

    lights/point_light.cpp

    path_tracing/backward.cpp

    primitives/geometry.cpp
    primitives/ray.cpp
    primitives/aabb.cpp
    primitives/triangle.cpp

    sensors/simple_sensor.cpp

    utils/rigid_body.cpp
    utils/vector.cpp
    utils/rotation.cpp
)

target_link_libraries(vira PRIVATE PUBLIC zstd)

set_target_properties(vira PROPERTIES LINKER_LANGUAGE CXX)