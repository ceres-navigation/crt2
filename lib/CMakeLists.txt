link_directories(${CMAKE_SOURCE_DIR}/external/zstd/prefix/lib)
find_package(zstd PATHS ${CMAKE_SOURCE_DIR}/external/zstd/prefix)

add_library(
    vira
    
    scene.cpp
    geometry.cpp
    binary_file.cpp

    acceleration/bvh.cpp

    cameras/simple_camera.cpp

    lights/point_light.cpp

    materials/lambertian.cpp
    materials/sampling.cpp

    path_tracing/backward.cpp

    physics/spectral_radiance.cpp

    primitives/ray.cpp
    primitives/aabb.cpp
    primitives/triangle.cpp

    sensors/simple_sensor.cpp

    utils/rigid_body.cpp
    utils/vector.cpp
    utils/rotation.cpp
    utils/euler_angles.cpp
)

target_link_libraries(vira PRIVATE PUBLIC zstd)

set_target_properties(vira PROPERTIES LINKER_LANGUAGE CXX)