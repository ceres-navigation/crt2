#include <iostream>
#include <random>
#include <fstream> // Needed for writing ppm image

#include "vector_math/vector.hpp"
#include "sensors/simple_sensor.hpp"

#include "cameras/camera.hpp"
#include "cameras/simple_camera.hpp"

#include "lights/light.hpp"
#include "lights/point_light.hpp"

#include "primitives/triangle.hpp"
#include "primitives/aabb.hpp"

#include "acceleration/bvh.hpp"

#include "geometry.hpp"

#include <chrono>
#include <vector>

using Scalar = float;

int main(){
    // Define sensor:
    Scalar resolution[2] = {640,640};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-5);

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Declare some variables:
    auto start = std::chrono::high_resolution_clock::now();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    // Load geometry:
    std::cout << "Reading Geometry...\n";
    start = std::chrono::high_resolution_clock::now();
    Geometry<Scalar> geometry("cube");
    geometry.read_obj("../suzanne.obj");
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "    " << geometry.num_triangles << " triangles loaded in " << duration.count() << " milliseconds\n";

    // Build BVH:
    std::cout << "Building BVH...\n";
    start = std::chrono::high_resolution_clock::now();
    geometry.build_bvh();
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "    BVH built in " << duration.count() << " milliseconds\n";

    // Render:
    std::cout << "Rendering with " << sensor.get_resolution_h()*sensor.get_resolution_v() << " rays...\n";
    start = std::chrono::high_resolution_clock::now();
    uint8_t  image[(int) sensor.get_resolution_h()][(int) sensor.get_resolution_v()] = {0};
    for( int u = 0; u < sensor.get_resolution_h(); u++) {
        for (int v = 0; v < sensor.get_resolution_v(); v++){
            auto ray = camera.pixel_to_ray(u,v);
            geometry.intersect( ray );

            // Format an output image:
            if (ray.hit.t < std::numeric_limits<Scalar>::max()) {
                image[u][v] = 255;
            }
        }
    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "    Rendered in " << duration.count() << " milliseconds\n";
    std::string filename = "frame_bvh.ppm";
    std::ofstream ppm2(filename, std::ios::binary);
    ppm2 << "P5\n" << sensor.get_resolution_h() << ' ' << sensor.get_resolution_v() << "\n255\n";
    ppm2.write(reinterpret_cast<char *>(image), sensor.get_resolution_h() * sensor.get_resolution_v() * sizeof(uint8_t));
    ppm2.flush();


    return 0;
}