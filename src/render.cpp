#include <vector>

#include "vector_math/vector.hpp"
#include "sensors/simple_sensor.hpp"

#include "cameras/camera.hpp"
#include "cameras/simple_camera.hpp"

#include "lights/light.hpp"
#include "lights/point_light.hpp"

#include "primitives/triangle.hpp"

#include <iostream>

// TODO: Make this configurable:
using Scalar = double;


#include <random> // Needed for generating random numbers

#include <fstream> // Needed for writing ppm image

// Helper function to print vector3 (if needed):
void print_vector(Vector3<Scalar> v){
    std::cout << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]\n";
}

void render(Camera<Scalar> &camera, Light<Scalar> &light, Triangle<Scalar> *triangles, uint32_t num_triangles){
    //Simple ray tracing:
    auto res_h = camera.sensor->get_resolution_h();
    auto res_v = camera.sensor->get_resolution_v();

    uint8_t  image[(int) res_h][(int) res_v] = {0};

    // Loop through all pixels:
    for (int u = 0; u < res_h; u++){
        for (int v = 0; v < res_v; v++){

            // Generate a ray:
            Ray<Scalar> ray = camera.pixel_to_ray(u,v);

            // Trace ray against all triangles:
            for (uint32_t i = 0; i < num_triangles; i++){
                intersect_triangle<Scalar>(ray, triangles[i]);
            }

            // Format an output image:
            if (ray.hit.t < std::numeric_limits<Scalar>::max()) {
                image[u][v] = 255;
            }
        }
    }

    // Write image to file:
    std::string filename = "frame.ppm";
    std::ofstream ppm(filename, std::ios::binary);
    ppm << "P5\n" << res_h << ' ' << res_v << "\n255\n";
    ppm.write(reinterpret_cast<char *>(image), res_h * res_v * sizeof(uint8_t));
    ppm.flush();
}

int main(){
    // Define sensor:
    Scalar resolution[2] = {1000,1000};
    Scalar size[2] = {30,30};
    SimpleSensor<Scalar> sensor(resolution, size);

    // Define camera:
    SimpleCamera<Scalar> camera(30, sensor, true);
    camera.set_position(0,0,-18);

    // Define a simple light:
    PointLight<Scalar> light(1);

    // Define some random triangles:
    uint32_t num_triangles = 100;
    auto triangles = new Triangle<Scalar>[num_triangles];

    // Setup random number generator:
    std::random_device dev;
    std::default_random_engine rng;
    std::uniform_real_distribution<double> dist(-1.0,1.0);

    for (uint32_t i = 0; i < num_triangles; i++){
        auto translation = Vector3<Scalar>(10*dist(rng),10*dist(rng),0);
        triangles[i].vertex0 = Vector3<Scalar>(dist(rng),dist(rng),dist(rng)) + translation;
        triangles[i].vertex1 = Vector3<Scalar>(dist(rng),dist(rng),dist(rng)) + translation;
        triangles[i].vertex2 = Vector3<Scalar>(dist(rng),dist(rng),dist(rng)) + translation;
    }

    // Ray trace:
    render(camera, light, triangles, num_triangles);
    
    // Heap allocation requires deletion when finished:
    delete [] triangles;

    return 0;
}