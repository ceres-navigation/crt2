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

#include "geometry.hpp"

#include <chrono>
#include <vector>

// triangle count
#define N 64

using Scalar = float;

struct BVHNode {
    Vector3<Scalar> aabbMin, aabbMax;     // 24 bytes
    uint leftChild, rightChild;  // 8 bytes
    bool isLeaf;                 // 4 bytes
    uint firstPrim, primCount;
};


int main(){
    // Define sensor:
    Scalar resolution[2] = {640,640};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-15);

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Load geometry and construct BVH:
    // Geometry<Scalar> geometry("cube");
    // // geometry.read_obj("../suzanne.obj");
    // geometry.read_obj("../cube.obj");
    // // geometry.read_obj("../random.obj");
    // Triangle<Scalar>* triangles = geometry.triangles;

    // Generate random triangles:
    std::default_random_engine generator;
    std::uniform_real_distribution<Scalar> distribution(0.0,1.0);
    Triangle<Scalar> tri[N];
    // application init; gets called once at startup
    for (int i = 0; i < N; i++)
    {
        Vector3<Scalar> r0( distribution(generator), distribution(generator), distribution(generator) );
        Vector3<Scalar> r1( distribution(generator), distribution(generator), distribution(generator) );
        Vector3<Scalar> r2( distribution(generator), distribution(generator), distribution(generator) );
        tri[i].vertex0 = r0 * (Scalar) 9 - Vector3<Scalar>( 5 );
        tri[i].vertex1 = tri[i].vertex0 + r1;
        tri[i].vertex2 = tri[i].vertex0 + r2;
    }

    // Ray trace:
    std::cout << "Beginning render...\n";
    auto start = std::chrono::high_resolution_clock::now();
    uint8_t  image[(int) sensor.get_resolution_h()][(int) sensor.get_resolution_v()] = {0};
    auto res_h = sensor.get_resolution_h();
    auto res_v = sensor.get_resolution_v();

    for( int u = 0; u < res_h; u++) {
        for (int v = 0; v < res_v; v++){
            auto ray = camera.pixel_to_ray(u,v);
            for (int i = 0; i < N; i++){
                intersect_triangle( ray, tri[i] );

                // Format an output image:
                if (ray.hit.t < std::numeric_limits<Scalar>::max()) {
                    image[u][v] = 255;
                }
            }
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Naive Duration: " << duration.count() << " milliseconds\n";

    // Write image to file:
    std::string filename = "frame_naive.ppm";
    std::ofstream ppm(filename, std::ios::binary);
    ppm << "P5\n" << sensor.get_resolution_h() << ' ' << sensor.get_resolution_v() << "\n255\n";
    ppm.write(reinterpret_cast<char *>(image), sensor.get_resolution_h() * sensor.get_resolution_v() * sizeof(uint8_t));
    ppm.flush();

    // Build BVH:


    return 0;
}